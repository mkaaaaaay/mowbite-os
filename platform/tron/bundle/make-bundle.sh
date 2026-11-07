#!/bin/bash
# A bundle of MowBite OS for mowbite-update on the Tron from an arm64 image of docker/os.Dockerfile (a loaded image or
# the .tar of docker save): <out dir>/mowbite-os-<revision>.tar.gz with <revision>/install, planner, revision and
# bundle.json. Unsigned; the owner signs it: ssh-keygen -Y sign -f <key> -n mowbite-bundle <bundle>
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
docker cp -q "$container:/opt/runtime-packages.txt" "$work/runtime-packages.txt"
printf '%s\n' "$commit" > "$work/$revision/revision"
python3 -I - "$work/runtime-packages.txt" "$revision" "$commit" "$min_app" > "$work/$revision/bundle.json" <<'PY'
import json, sys
packages = sorted({line.strip() for line in open(sys.argv[1]) if line.strip()})
print(json.dumps({"format": 1, "revision": sys.argv[2], "commit": sys.argv[3], "min_app": sys.argv[4],
                  "runtime_packages": packages}, indent=1))
PY
bundle="$out/mowbite-os-$revision.tar.gz"
tar -C "$work" --owner=0 --group=0 --numeric-owner --sort=name --mode=go-w -czf "$bundle.tmp" "$revision"
mv "$bundle.tmp" "$bundle"
echo "$bundle $(( $(stat -c %s "$bundle") / 1000000 )) MB sha256 $(sha256sum "$bundle" | cut -d' ' -f1)"
