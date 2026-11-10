#!/bin/bash
# A bundle of MowBite OS for mowbite-host on the Tron from an arm64 image of docker/os.Dockerfile (a loaded image or
# the .tar of docker save): <out dir>/mowbite-os-<revision>.tar.gz with <revision>/install, planner, app (when the
# image has one, readable for mowbite-web), tools (the safety measurement for mowbite-measure), revision and
# bundle.json (format 2). serial is the commit time of the image's commit and base the rootfs version it needs, both
# read from this repo at that commit; from the image api (its schema), planner_files_version (data_schema.json of
# mowbite_bridge), planner_interface (contract.json of its planner, none without one) and bundled, what it brings of
# planner and app. Unsigned; the owner signs it with sign-bundle.sh.
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
here=$(dirname "$(readlink -f "$0")")
if ! serial=$(git -C "$here" show -s --format=%ct "$commit" 2> /dev/null) ||
   ! base=$(git -C "$here" show "$commit:platform/tron/rootfs/files/usr/lib/mowbite/base" 2> /dev/null); then
    echo "commit $commit or its usr/lib/mowbite/base is not in the repo at $here: fetch it first" >&2
    exit 1
fi
source "$here/pack.sh"
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
docker cp -q "$container:/opt/mowbite_app" "$work/app"
if [ -f "$work/app/app.json" ]; then
    mv "$work/app" "$work/$revision/app"
fi
docker cp -q "$container:/ws/tools" "$work/$revision/tools"
docker cp -q "$container:/opt/runtime-packages.txt" "$work/runtime-packages.txt"
printf '%s\n' "$commit" > "$work/$revision/revision"
fields=$(python3 -I - "$HOST_LIB" "$work" "$revision" "$commit" "$min_app" "$serial" "$base" <<'PY'
import json, os, sys
host, work, revision, commit, min_app, serial, base = sys.argv[1:]
sys.path.insert(0, os.path.join(host, "bundle"))
import app_export, planner_meta
tree = f"{work}/{revision}"
packages = sorted({line.strip() for line in open(f"{work}/runtime-packages.txt") if line.strip()})
with open(f"{tree}/install/mowbite_api/share/mowbite_api/api.schema.json") as f:
    api = json.load(f)["x-api"]
with open(f"{tree}/install/mowbite_bridge/share/mowbite_bridge/data_schema.json") as f:
    table = json.load(f)
meta = {"commit": commit, "serial": int(serial), "base": int(base.strip()), "api": api, "min_app": min_app,
        "runtime_packages": packages,
        "planner_files_version": table["planner_files_version"][str(table["schema"])]}
planner = planner_meta.read(f"{tree}/planner")
if "interface" in planner:
    meta["planner_interface"] = planner["interface"]
with open(f"{tree}/planner/source.json") as f:
    source = json.load(f)
meta["bundled"] = {"planner": {"revision": source["commit"][:12], "version": planner["version"],
                               "serial": source["serial"]}}
if os.path.isdir(f"{tree}/app"):
    app = app_export.meta(f"{tree}/app")
    meta["bundled"]["app"] = {"revision": app["commit"][:12], "version": app["version"], "serial": app["serial"]}
print(json.dumps(meta))
PY
)
bundle_json os "$fields" > "$work/$revision/bundle.json"
pack "$work/$revision" "$out"
