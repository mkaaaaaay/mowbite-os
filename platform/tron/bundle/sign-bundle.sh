#!/bin/bash
# Signs a bundle of make-bundle.sh for mowbite-host: shows what it holds and the test gate result of its commit,
# refuses a serial not newer than the last one signed for its component, asks, signs (namespace of mowbite-host)
# and checks the signature against allowed_signers of the rootfs.
#   sign-bundle.sh [--yes] <bundle.tar.gz> <private key>
# --yes signs without asking, only when the gate result of its commit is there and ok (for publishing after a merge).
# The last serial per component and a log of what was signed stay in $MOWBITE_SIGN_STATE (default
# ~/.local/state/mowbite-sign). The gate result is <commit, first 7 digits or more>.json in $MOWBITE_GATE (a folder or
# host:folder, as start.sh of the test gate writes it); without MOWBITE_GATE it is not looked up.
set -euo pipefail
yes=false
if [ "${1:-}" = --yes ]; then
    yes=true
    shift
fi
if [ $# -ne 2 ]; then
    echo "usage: $0 [--yes] <bundle.tar.gz> <private key>" >&2
    exit 2
fi
bundle=$1
key=$2
here=$(dirname "$(readlink -f "$0")")
source "$here/pack.sh"
signers=${MOWBITE_SIGNERS:-$here/../rootfs/files/etc/mowbite/allowed_signers}
state=${MOWBITE_SIGN_STATE:-$HOME/.local/state/mowbite-sign}

# component revision commit serial namespace on the first line, then the lines to show; the checks of mowbite-host.
read_meta() {
    python3 -I - "$HOST_LIB" "$1" <<'PY'
import sys, time
sys.path.insert(0, sys.argv[1])
from mowbite_host import Refused, bundle
from mowbite_host.compat import span_text
try:
    meta, _ = bundle.read_meta(sys.argv[2])
except Refused as why:
    sys.exit(f"{why.reason}: {why}")
if meta["component"] not in bundle.MAX_BUNDLE:
    sys.exit(f"unknown component {meta['component']}")
print(meta["component"], meta["revision"], meta.get("commit", ""), meta["serial"], bundle.namespace(meta["component"]))
day = lambda t: time.strftime("%Y-%m-%d %H:%M", time.localtime(t)) if isinstance(t, int) else "-"
print(f"  component {meta['component']}, revision {meta['revision']}, commit {meta.get('commit', '-')}")
print(f"  serial {meta['serial']} (commit time {day(meta['serial'])}), built {day(meta.get('date'))}")
if meta["component"] == "os":
    print(f"  platforms {', '.join(meta.get('platforms', ['any']))}")
    print(f"  base {meta.get('base', 0)}, api {meta.get('api', '-')} break {meta.get('api_break', '-')}, "
          f"min_app {meta.get('min_app', '-')}, "
          f"{len(meta.get('runtime_packages', []))} runtime packages")
    print(f"  planner_interface {meta.get('planner_interface', '-')}, "
          f"planner_files_version {meta.get('planner_files_version', '-')}")
    for name, entry in sorted(meta.get("bundled", {}).items()):
        print(f"  brings {name} {entry.get('version')} ({entry.get('revision')}, serial {entry.get('serial')})")
elif meta["component"] == "planner":
    print(f"  version {meta['version']}, interface {meta['interface']}, files_version {meta['files_version']}, "
          f"requires {', '.join(f'{k} {v}' for k, v in sorted(meta.get('requires', {}).items())) or '-'}")
elif meta["component"] == "app":
    print(f"  version {meta['version']}, api {span_text(meta['api'])}")
else:
    print(f"  base {meta.get('base', 0)}")
PY
}

meta=$(read_meta "$bundle")
read -r component revision commit serial namespace <<< "$(head -1 <<< "$meta")"
echo "$bundle"
tail -n +2 <<< "$meta"
echo "  $(( $(stat -c %s "$bundle") / 1000000 )) MB, sha256 $(sha256sum "$bundle" | cut -d' ' -f1)"

last=$(cat "$state/$component.serial" 2> /dev/null || echo 0)
if [ "$serial" -le "$last" ]; then
    echo "refused: serial $serial is not newer than $last, the last one signed for $component" >&2
    exit 1
fi

# gate_dir <ls|cat> [file]: lists the folder of gate results or reads one file of it.
gate_dir() {
    if [[ $MOWBITE_GATE == *:* ]]; then
        ssh "${MOWBITE_GATE%%:*}" "$1" "${MOWBITE_GATE#*:}${2:+/$2}"
    else
        "$1" "$MOWBITE_GATE${2:+/$2}"
    fi 2> /dev/null || true
}

if [ -n "${MOWBITE_GATE:-}" ] && [ -n "$commit" ]; then
    # The test gate names the file as git abbreviates the commit: the longest name that begins the commit counts.
    file=$(gate_dir ls | awk -v commit="$commit" '
        /^[0-9a-f]+\.json$/ {
            digits = length($0) - 5
            if (digits >= 7 && digits > best && substr($0, 1, digits) == substr(commit, 1, digits)) {
                best = digits
                name = $0
            }
        }
        END { print name }')
    gate=
    [ -z "$file" ] || gate=$(gate_dir cat "$file")
    if [ -z "$gate" ]; then
        echo "  test gate: NO RESULT for ${commit:0:7}"
    else
        python3 -I - "$gate" "$commit" <<'PY' && gate_ok=true
import json, sys
gate = json.loads(sys.argv[1])
noted = gate.get("commit")
if not isinstance(noted, str) or len(noted) < 7 or not sys.argv[2].startswith(noted):
    print(f"  test gate: NO RESULT, the file is of commit {noted}")
    sys.exit(1)
print(f"  test gate: {'ok' if gate.get('ok') else 'NOT OK'}, updated {gate.get('updated')}")
for name, step in sorted(gate.get("steps", {}).items()):
    print(f"    {'ok  ' if step.get('ok') else 'FAIL'} {name} ({step.get('at')}, {step.get('res')})")
sys.exit(0 if gate.get("ok") is True else 1)
PY
    fi
else
    echo "  test gate: not looked up (MOWBITE_GATE unset)"
fi

if $yes; then
    [ "${gate_ok:-false}" = true ] || { echo "not signed: --yes needs an ok gate result of the commit" >&2; exit 1; }
else
    read -r -p "Sign $revision for $component? [y/N] " answer
    [ "$answer" = y ] || { echo "not signed"; exit 1; }
fi
rm -f "$bundle.sig"
ssh-keygen -q -Y sign -f "$key" -n "$namespace" "$bundle"
ssh-keygen -q -Y verify -f "$signers" -I mowbite -n "$namespace" -s "$bundle.sig" < "$bundle" > /dev/null || {
    echo "the signature does not hold for allowed_signers of the rootfs: $bundle.sig is no use to a mower" >&2
    exit 1
}
mkdir -p "$state"
printf '%s\n' "$serial" > "$state/$component.serial.tmp"
mv "$state/$component.serial.tmp" "$state/$component.serial"
printf '%s %s %s %s %s\n' "$(date +%FT%T)" "$component" "$revision" "$serial" \
    "$(sha256sum "$bundle" | cut -d' ' -f1)" >> "$state/signed.log"
echo "signed: $bundle.sig"
