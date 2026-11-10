#!/bin/bash
# A bundle of MowBite OS for mowbite-host on the Tron from an arm64 image of docker/os.Dockerfile (a loaded image or
# the .tar of docker save): <out dir>/mowbite-os-<revision>.tar.gz with <revision>/install, planner, tools (the safety
# measurement for mowbite-measure), revision and bundle.json (format 2). serial is the commit time of the image's
# commit and base the rootfs version it needs, both read from this repo at that commit; api from the schema in the
# image. Unsigned; the owner signs it with sign-bundle.sh.
set -euo pipefail
if [ $# -ne 2 ]; then
    echo "usage: $0 <image or image.tar> <out dir>" >&2
    exit 2
fi
image=$1
out=$2
if [ -f "$image" ]; then
    image=$(docker load -i "$image" | sed -n 's/^Loaded image: //p' | tail -1)
fi
label() { docker image inspect -f "{{ index .Config.Labels \"$1\" }}" "$image"; }
arch=$(docker image inspect -f '{{ .Architecture }}' "$image")
commit=$(label org.opencontainers.image.revision)
min_app=$(label mowbite.min_app)
if [ "$arch" != arm64 ] || ! [[ $commit =~ ^[0-9a-f]{40}$ ]] || [ -z "$min_app" ]; then
    echo "$image: architecture '$arch', revision '$commit', min_app '$min_app'; needs arm64, a commit and min_app" >&2
    exit 1
fi
revision=${commit:0:12}
repo=$(dirname "$(readlink -f "$0")")
if ! serial=$(git -C "$repo" show -s --format=%ct "$commit" 2> /dev/null) ||
   ! base=$(git -C "$repo" show "$commit:platform/tron/rootfs/files/usr/lib/mowbite/base" 2> /dev/null); then
    echo "commit $commit or its usr/lib/mowbite/base is not in the repo at $repo: fetch it first" >&2
    exit 1
fi
work=$(mktemp -d "$out/.bundle.XXXXXX")
container=
cleanup() {
    [ -z "$container" ] || docker rm "$container" > /dev/null
    rm -rf "$work"
}
trap cleanup EXIT
container=$(docker create --platform linux/arm64 "$image" true)
mkdir "$work/$revision"
docker cp -q "$container:/ws/install" "$work/$revision/install"
docker cp -q "$container:/opt/mowbite_planner" "$work/$revision/planner"
docker cp -q "$container:/ws/tools" "$work/$revision/tools"
docker cp -q "$container:/opt/runtime-packages.txt" "$work/runtime-packages.txt"
printf '%s\n' "$commit" > "$work/$revision/revision"
python3 -I - "$work" "$revision" "$commit" "$min_app" "$serial" "$base" > "$work/$revision/bundle.json" <<'PY'
import json, sys, time
work, revision, commit, min_app, serial, base = sys.argv[1:]
packages = sorted({line.strip() for line in open(f"{work}/runtime-packages.txt") if line.strip()})
with open(f"{work}/{revision}/install/mowbite_api/share/mowbite_api/api.schema.json") as f:
    api = json.load(f)["x-api"]
print(json.dumps({"format": 2, "component": "os", "revision": revision, "commit": commit, "serial": int(serial),
                  "date": int(time.time()), "base": int(base.strip()), "api": api, "min_app": min_app,
                  "runtime_packages": packages}, indent=1))
PY
bundle="$out/mowbite-os-$revision.tar.gz"
tar -C "$work" --owner=0 --group=0 --numeric-owner --sort=name --mode=go-w -czf "$bundle.tmp" "$revision"
mv "$bundle.tmp" "$bundle"
echo "$bundle $(( $(stat -c %s "$bundle") / 1000000 )) MB sha256 $(sha256sum "$bundle" | cut -d' ' -f1)"
