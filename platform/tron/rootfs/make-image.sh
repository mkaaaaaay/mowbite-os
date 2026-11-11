#!/bin/bash
# The rootfs image of one Tron: the base tar of build-base.sh, files/ of the repo over it, the vendor parts
# (vendor-files.txt) and the WLAN profiles from the owner's own rootfs backup, the SSH host keys, the key for root and
# optionally the password hash for root.
# ext4 with only the features of the vendor rootfs (kernel 5.10), exactly the size of the rootfs partition. The image
# holds vendor files and secrets: it stays in the owner's encrypted storage, never in a repo, an upload or a release.
#   make-image.sh <base .tar> <vendor rootfs dir> <host key dir> <root key .pub> <new .img> [root password hash]
# "-" for the vendor rootfs or the host keys leaves them out (probe image for QEMU, keys made on the first boot).
# Prints no secrets; work files lie next to the image and are removed at the end.
set -euo pipefail
BASE=${1:?base .tar of build-base.sh}
VENDOR=${2:?directory with the vendor rootfs}
HOSTKEYS=${3:?directory with ssh_host_*_key and .pub}
ROOTKEY=${4:?public key for root}
OUT=${5:?new .img}
# One line from "openssl passwd -6"; without it root has no password.
ROOTHASH=${6:-}

HERE=$(cd "$(dirname "$0")" && pwd)
SIZE=17179869184
FEATURES=none,has_journal,ext_attr,resize_inode,dir_index,filetype,extent,64bit,flex_bg,sparse_super,large_file,huge_file,dir_nlink,extra_isize,metadata_csum
MKE2FS=/usr/sbin/mke2fs

test ! -e "$OUT" || { echo "$OUT exists" >&2; exit 1; }
test -f "$BASE" && test -f "$ROOTKEY"
test "$VENDOR" = - || test -d "$VENDOR"
test "$HOSTKEYS" = - || test -d "$HOSTKEYS"
grep -q '^ssh-' "$ROOTKEY" || { echo "$ROOTKEY is no public key" >&2; exit 1; }
if [ -n "$ROOTHASH" ]; then
    test "$(wc -l < "$ROOTHASH")" = 1 && grep -qE '^\$(6|y)\$[^:[:space:]]+$' "$ROOTHASH" \
        || { echo "$ROOTHASH is no sha512 or yescrypt hash" >&2; exit 1; }
fi

umask 077
work=$(mktemp -d "$(dirname "$OUT")/.make-image.XXXXXX")
trap 'rm -rf "$work"' EXIT

# Paths of the base tar without the leading ./ for the check of missing parent directories.
tar -tf "$BASE" | sed 's|^\./||; s|/$||' | sort -u > "$work/base.list"

# files/ of the repo go over the base; the base must come from the same packages.txt and build-base.sh and hold no
# file that files/ no longer has.
"$HERE/base-inputs.sh" > "$work/inputs.now"
tar -xOf "$BASE" ./usr/lib/mowbite/base-inputs > "$work/inputs.base" 2>/dev/null \
    || { echo "$BASE has no base-inputs, build a new base" >&2; exit 1; }
if ! cmp -s <(grep -v '^file ' "$work/inputs.base") <(grep -v '^file ' "$work/inputs.now") \
    || [ -n "$(LC_ALL=C comm -23 <(grep '^file ' "$work/inputs.base") <(grep '^file ' "$work/inputs.now"))" ]; then
    echo "$BASE is out of date for packages.txt, build-base.sh or a removal under files/, build a new base" >&2
    exit 1
fi
tar -C "$HERE/files" -cf "$work/files.tar" --owner=0 --group=0 --numeric-owner --mode=go-w .
# The broker of MowBite OS with the configuration the Compose stack of the other platforms uses.
tar -C "$HERE/../../../docker/mower" -rf "$work/files.tar" --owner=0 --group=0 --numeric-owner --mode=go-w \
    --transform 's|^mosquitto.conf$|./etc/mosquitto/mosquitto.conf|' mosquitto.conf
# The seed of mowbite-host (its library at the commit checked out, with bundle.json as an installation has it) and
# mowbite_boot, which chooses between the seed and an installed one; the seed 0755 and 0644 despite the umask.
source "$HERE/../bundle/pack.sh"
host_files HEAD "$work/host/usr/lib/mowbite-host"
chmod -R u=rwX,go=rX "$work/host/usr/lib/mowbite-host"
install -D -m 644 "$HERE/../../../host/boot/mowbite_boot.py" "$work/host/usr/lib/mowbite-boot/mowbite_boot.py"
tar -C "$work/host" -rf "$work/files.tar" --owner=0 --group=0 --numeric-owner --mode=go-w --exclude=__pycache__ \
    ./usr/lib/mowbite-host ./usr/lib/mowbite-boot
# The web server of the MowBite App, from web/ of the repo; readable for its user despite the umask.
install -d -m 755 "$work/web/usr/lib/mowbite-web"
install -m 644 "$HERE/../../../web/mowbite_web.py" "$work/web/usr/lib/mowbite-web/"
tar -C "$work/web" -rf "$work/files.tar" --owner=0 --group=0 --numeric-owner --mode=go-w ./usr/lib/mowbite-web

items=()
parents=()
if [ "$VENDOR" != - ]; then
    mapfile -t items < <(sed 's/#.*//; /^[[:space:]]*$/d' "$HERE/vendor-files.txt")
fi
for item in "${items[@]}"; do
    test -e "$VENDOR/$item" || test -L "$VENDOR/$item" || { echo "missing in the vendor rootfs: $item" >&2; exit 1; }
    dir=$(dirname "$item")
    while [ "$dir" != . ]; do
        grep -qxF "$dir" "$work/base.list" || parents+=("./$dir")
        dir=$(dirname "$dir")
    done
done
if [ ${#parents[@]} -gt 0 ]; then
    mapfile -t parents < <(printf '%s\n' "${parents[@]}" | sort -u)
fi
# Owned by root, no write access for group and others (the extracted backup belongs to the user).
if [ ${#items[@]} -gt 0 ]; then
    tar -C "$VENDOR" -cf "$work/vendor.tar" --owner=0 --group=0 --numeric-owner --mode=go-w \
        --no-recursion "${parents[@]}" --recursion "${items[@]/#/./}"
fi

# Host keys, key for root and WLAN profiles; the directories they go to exist in the base, apart from /root/.ssh.
mkdir -p "$work/p/etc/ssh" "$work/p/root/.ssh" "$work/p/etc/NetworkManager/system-connections"
private=(./root/.ssh ./root/.ssh/authorized_keys)
for k in "$HOSTKEYS"/ssh_host_*_key; do
    test "$HOSTKEYS" != - || break
    n=$(basename "$k")
    install -m 600 "$k" "$work/p/etc/ssh/$n"
    install -m 644 "$k.pub" "$work/p/etc/ssh/$n.pub"
    private+=("./etc/ssh/$n" "./etc/ssh/$n.pub")
done
install -m 600 "$ROOTKEY" "$work/p/root/.ssh/authorized_keys"
chmod 700 "$work/p/root/.ssh"
for c in "$VENDOR"/etc/NetworkManager/system-connections/*; do
    test "$VENDOR" != - && test -f "$c" || continue
    n=$(basename "$c")
    # No name servers over IPv6: router advertisements would bypass the resolver that DHCPv4 hands out.
    awk '/^\[/ { sec = $0 }
        sec == "[ipv6]" && /^ignore-auto-dns=/ { next }
        { print }
        $0 == "[ipv6]" { print "ignore-auto-dns=true"; seen = 1 }
        END { if (!seen) print "[ipv6]\nmethod=auto\nignore-auto-dns=true" }' "$c" \
        > "$work/p/etc/NetworkManager/system-connections/$n"
    private+=("./etc/NetworkManager/system-connections/$n")
done
grep -qxF etc/NetworkManager/system-connections "$work/base.list"
grep -qxF root "$work/base.list"
tar -C "$work/p" -cf "$work/private.tar" --owner=0 --group=0 --numeric-owner --no-recursion "${private[@]}"
# The password hash goes into root's line of the base's shadow, which keeps its group (shadow) and mode.
if [ -n "$ROOTHASH" ]; then
    mkdir -p "$work/s"
    tar -C "$work/s" -xf "$BASE" ./etc/shadow
    group=$(tar -tvf "$BASE" --numeric-owner ./etc/shadow | awk '{ split($2, o, "/"); print o[2] }')
    awk -F: -v OFS=: 'NR == FNR { h = $0; next } $1 == "root" { $2 = h; n++ } { print } END { exit n != 1 }' \
        "$ROOTHASH" "$work/s/etc/shadow" > "$work/s/etc/shadow.new"
    mv "$work/s/etc/shadow.new" "$work/s/etc/shadow"
    tar -C "$work/s" -cf "$work/shadow.tar" --owner=0 --group="$group" --numeric-owner --mode=0640 ./etc/shadow
    tar -Af "$work/private.tar" "$work/shadow.tar"
fi

cp "$BASE" "$work/all.tar"
tar -Af "$work/all.tar" "$work/files.tar"
if [ -e "$work/vendor.tar" ]; then
    tar -Af "$work/all.tar" "$work/vendor.tar"
fi
tar -Af "$work/all.tar" "$work/private.tar"

truncate -s "$SIZE" "$OUT"
"$MKE2FS" -q -F -t ext4 -O "$FEATURES" -b 4096 -I 256 -E root_owner=0:0 -d "$work/all.tar" "$OUT" $((SIZE / 4096))
test "$(stat -c %s "$OUT")" -eq "$SIZE"
echo "image $OUT: ${#items[@]} vendor items, ${#parents[@]} new parent directories," \
     "$(( ${#private[@]} - 2 )) host key and WLAN files, root password: $([ -n "$ROOTHASH" ] && echo set || echo none)"
