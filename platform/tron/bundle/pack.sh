# Sourced by make-bundle.sh, make-app-bundle.sh and make-planner-bundle.sh.
HOST_LIB=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../../../host

# pack <work dir> <revision> <component> <out dir>: <out dir>/mowbite-<component>-<revision>.tar.gz of
# <work dir>/<revision>, owned by root, writable only by it, readable as the files are; refused above the size
# mowbite-host takes for the component.
pack() {
    local work=$1 revision=$2 component=$3 out=$4
    local bundle="$out/mowbite-$component-$revision.tar.gz" limit size
    limit=$(python3 -I -c 'import sys; sys.path.insert(0, sys.argv[1]); from mowbite_host import bundle
print(bundle.MAX_BUNDLE[sys.argv[2]])' "$HOST_LIB" "$component")
    chmod 755 "$work/$revision"
    tar -C "$work" --owner=0 --group=0 --numeric-owner --sort=name --mode=go-w -czf "$bundle.tmp" "$revision"
    size=$(stat -c %s "$bundle.tmp")
    if [ "$size" -gt "$limit" ]; then
        rm -f "$bundle.tmp"
        echo "the bundle has $((size / 1000000)) MB, mowbite-host takes at most $((limit / 1000000)) MB of $component" >&2
        return 1
    fi
    mv "$bundle.tmp" "$bundle"
    echo "$bundle $((size / 1000000)) MB sha256 $(sha256sum "$bundle" | cut -d' ' -f1)"
}
