#!/bin/bash
# A bundle of the MowBite Planner for mowbite-host from a commit of its repo: <out dir>/mowbite-planner-<revision>.tar.gz
# with <revision>/planner (the colcon installation of mowbite_planner alone, local_setup.bash at its top; the
# interfaces stay those of the OS) and bundle.json (format 2). Built in a throwaway container of ROS_IMAGE without
# network; serial is the commit time, version, interface, files_version and requires come from the package
# (host/bundle/planner_meta.py, contract.json required). No machine code in it. Unsigned; sign it with sign-bundle.sh.
set -euo pipefail
if [ $# -ne 3 ]; then
    echo "usage: $0 <planner repo> <commit> <out dir>" >&2
    exit 2
fi
repo=$(readlink -f "$1")
out=$3
image=${ROS_IMAGE:-ros:jazzy-ros-base}
source "$(dirname "$(readlink -f "$0")")/pack.sh"
if ! commit=$(git -C "$repo" rev-parse --verify -q "$2^{commit}") ||
   ! serial=$(git -C "$repo" show -s --format=%ct "$commit"); then
    echo "$2 is no commit of $repo" >&2
    exit 1
fi
revision=${commit:0:12}
work=$(mktemp -d "$out/.bundle.XXXXXX")
trap 'rm -rf "$work"' EXIT
mkdir "$work/$revision"
docker run --rm --network none --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$repo:/src:ro" -v "$work/$revision:/out" "$image" bash -ec '
    git init -q /tmp/planner && cd /tmp/planner
    git -c safe.directory="*" fetch -q --depth 1 /src "$1" && git checkout -q FETCH_HEAD
    source /opt/ros/jazzy/setup.bash
    colcon --log-base /tmp/log build --base-paths ros2/mowbite_planner --build-base /tmp/build \
        --install-base /out/planner > /tmp/build.log 2>&1 || { tail -30 /tmp/build.log; exit 1; }' _ "$commit"
python3 -I - "$HOST_LIB" "$work/$revision" <<'PY'
import sys
sys.path.insert(0, sys.argv[1])
from mowbite_host import bundle
machine = bundle.elf_files(sys.argv[2])
if machine:
    sys.exit("machine code in the planner: " + ", ".join(machine[:10]))
PY
meta=$(python3 -I "$HOST_LIB/bundle/planner_meta.py" "$work/$revision/planner" --contract)
python3 -I -c '
import json, sys, time
meta, revision, commit, serial = json.loads(sys.argv[1]), *sys.argv[2:]
print(json.dumps({"format": 2, "component": "planner", "revision": revision, "commit": commit, "serial": int(serial),
                  "date": int(time.time()), **meta}, indent=1))
' "$meta" "$revision" "$commit" "$serial" > "$work/$revision/bundle.json"
chmod -R a+rX "$work/$revision"
pack "$work" "$revision" planner "$out"
