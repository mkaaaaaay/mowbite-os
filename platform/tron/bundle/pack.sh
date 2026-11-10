# Sourced by the make-*-bundle.sh scripts, sign-bundle.sh and make-image.sh.
HOST_LIB=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../../../host

# bundle_json <component> <JSON object>...: bundle.json in the format of mowbite-host with the fields of the objects,
# revision from commit and date now.
bundle_json() {
    python3 -I -c '
import json, sys, time
sys.path.insert(0, sys.argv[1])
from mowbite_host import bundle
fields = {}
for text in sys.argv[3:]:
    fields.update(json.loads(text))
print(json.dumps({"format": bundle.FORMAT, "component": sys.argv[2], "revision": fields["commit"][:12],
                  "date": int(time.time()), **fields}, indent=1))' "$HOST_LIB" "$@"
}

# host_files <commit> <dir>: mowbite_host of this repo at commit and its bundle.json into dir, the layout of an
# installation of the host and of its seed in the rootfs; serial is the commit time, base the rootfs version of then.
host_files() {
    local commit serial base repo
    repo=$(git -C "$HOST_LIB" rev-parse --show-toplevel)
    if ! commit=$(git -C "$repo" rev-parse --verify -q "$1^{commit}") ||
       ! serial=$(git -C "$repo" show -s --format=%ct "$commit") ||
       ! base=$(git -C "$repo" show "$commit:platform/tron/rootfs/files/usr/lib/mowbite/base"); then
        echo "$1 is no commit of $repo with usr/lib/mowbite/base" >&2
        return 1
    fi
    mkdir -p "$2"
    git -C "$repo" archive "$commit" host/mowbite_host | tar -x -C "$2" --strip-components=1
    bundle_json host "{\"commit\": \"$commit\", \"serial\": $serial, \"base\": $base}" > "$2/bundle.json"
}

# pack <tree> <out dir>: <out dir>/mowbite-<component>-<revision>.tar.gz of the tree and its bundle.json as
# <revision>/, owned by root, writable only by it, readable by all; refused unless mowbite-host reads it, takes its
# size and, but for the OS, finds no machine code in it.
pack() {
    local tree=$1 out=$2 component revision bundle
    read -r component revision < <(python3 -I -c 'import json, sys
meta = json.load(open(sys.argv[1]))
print(meta["component"], meta["revision"])' "$tree/bundle.json")
    if [ "$(basename "$tree")" != "$revision" ]; then
        mv "$tree" "$(dirname "$tree")/$revision"
        tree=$(dirname "$tree")/$revision
    fi
    bundle="$out/mowbite-$component-$revision.tar.gz"
    chmod -R a+rX "$tree"
    tar -C "$(dirname "$tree")" --owner=0 --group=0 --numeric-owner --sort=name --mode=go-w -czf "$bundle.tmp" \
        "$revision"
    if ! python3 -I - "$HOST_LIB" "$bundle.tmp" "$tree" <<'PY'
import os, sys
sys.path.insert(0, sys.argv[1])
from mowbite_host import Refused, bundle
try:
    meta, _ = bundle.read_meta(sys.argv[2])
except Refused as why:
    sys.exit(f"{why.reason}: {why}")
size, limit = os.path.getsize(sys.argv[2]), bundle.MAX_BUNDLE.get(meta["component"], 0)
if size > limit:
    sys.exit(f"the bundle has {size // 1000000} MB, mowbite-host takes at most {limit // 1000000} MB of it")
machine = bundle.elf_files(sys.argv[3]) if meta["component"] != "os" else []
if machine:
    sys.exit("machine code in it: " + ", ".join(machine[:10]))
PY
    then
        rm -f "$bundle.tmp"
        return 1
    fi
    mv "$bundle.tmp" "$bundle"
    echo "$bundle $(($(stat -c %s "$bundle") / 1000000)) MB sha256 $(sha256sum "$bundle" | cut -d' ' -f1)"
}
