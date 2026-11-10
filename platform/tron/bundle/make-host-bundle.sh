#!/bin/bash
# A bundle of mowbite-host from a commit of this repo: <out dir>/mowbite-host-<revision>.tar.gz with
# <revision>/mowbite_host and bundle.json (serial the commit time, base the rootfs version of then, which brings the
# mowbite_boot it runs under). Unsigned; sign it with sign-bundle.sh.
set -euo pipefail
if [ $# -ne 2 ]; then
    echo "usage: $0 <commit> <out dir>" >&2
    exit 2
fi
out=$2
source "$(dirname "$(readlink -f "$0")")/pack.sh"
work=$(mktemp -d "$out/.bundle.XXXXXX")
trap 'rm -rf "$work"' EXIT
host_files "$1" "$work/tree"
pack "$work/tree" "$out"
