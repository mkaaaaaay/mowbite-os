#!/bin/bash
# A bundle of the MowBite app for mowbite-host from the web export of its release (MowBite-web.zip):
# <out dir>/mowbite-app-<revision>.tar.gz with <revision>/app (the export as host/bundle/app_export.py checks and
# unpacks it, readable for mowbite-web) and bundle.json (format 2). revision, commit, serial, version and api come from
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
python3 -I "$HOST_LIB/bundle/app_export.py" export "$zip" "$work/app"
meta=$(python3 -I "$HOST_LIB/bundle/app_export.py" meta "$work/app")
revision=$(python3 -I -c 'import json, sys; print(json.loads(sys.argv[1])["commit"][:12])' "$meta")
mkdir "$work/$revision"
mv "$work/app" "$work/$revision/app"
python3 -I -c '
import json, sys, time
meta, revision = json.loads(sys.argv[1]), sys.argv[2]
print(json.dumps({"format": 2, "component": "app", "revision": revision, "date": int(time.time()), **meta}, indent=1))
' "$meta" "$revision" > "$work/$revision/bundle.json"
chmod 644 "$work/$revision/bundle.json"
pack "$work" "$revision" app "$out"
