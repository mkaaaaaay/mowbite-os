#!/bin/bash
# A bundle of the MowBite Planner for mowbite-host from a commit of its repo: <out dir>/mowbite-planner-<revision>.tar.gz
# with <revision>/planner (the colcon installation of mowbite_planner and of mowbite_planner_os, the node of MowBite
# OS, local_setup.bash at its top; the interfaces stay those of the OS) and bundle.json. Built in a throwaway
# container of ROS_IMAGE without network; serial is the commit time, version, interface, files_version and requires
# come from the package (host/bundle/planner_meta.py, contract.json required). Unsigned; sign it with sign-bundle.sh.
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
work=$(mktemp -d "$out/.bundle.XXXXXX")
trap 'rm -rf "$work"' EXIT
mkdir "$work/tree"
docker run --rm --network none --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$repo:/src:ro" -v "$work/tree:/out" "$image" bash -ec '
    git init -q /tmp/planner && cd /tmp/planner
    git -c safe.directory="*" fetch -q --depth 1 /src "$1" && git checkout -q FETCH_HEAD
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    colcon --log-base /tmp/log build --base-paths ros2/mowbite_planner os/mowbite_planner_os --build-base /tmp/build \
        --install-base /out/planner > /tmp/build.log 2>&1 || { tail -30 /tmp/build.log; exit 1; }' _ "$commit"
meta=$(python3 -I "$HOST_LIB/bundle/planner_meta.py" "$work/tree/planner")
bundle_json planner "$meta" "{\"commit\": \"$commit\", \"serial\": $serial}" > "$work/tree/bundle.json"
pack "$work/tree" "$out"
