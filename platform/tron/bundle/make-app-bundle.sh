#!/bin/bash
# A bundle of the MowBite app for mowbite-host from the web export of its release (MowBite-web.zip):
# <out dir>/mowbite-app-<revision>.tar.gz with <revision>/app (the export as host/bundle/app_export.py checks and
# unpacks it, readable for mowbite-web) and bundle.json. revision, commit, serial, version and api come from
# app.json of the export; whether it goes with the OS of a mower its host decides. Unsigned; sign it with
# sign-bundle.sh.
set -euo pipefail
if [ $# -ne 2 ]; then
    echo "usage: $0 <MowBite-web.zip> <out dir>" >&2
    exit 2
fi
zip=$1
out=$2
source "$(dirname "$(readlink -f "$0")")/pack.sh"
work=$(mktemp -d "$out/.bundle.XXXXXX")
trap 'rm -rf "$work"' EXIT
mkdir "$work/tree"
python3 -I "$HOST_LIB/bundle/app_export.py" export "$zip" "$work/tree/app"
meta=$(python3 -I "$HOST_LIB/bundle/app_export.py" meta "$work/tree/app")
bundle_json app "$meta" > "$work/tree/bundle.json"
pack "$work/tree" "$out"
