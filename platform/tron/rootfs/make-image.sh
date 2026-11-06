#!/bin/bash
# The rootfs image of one Tron: the base tar of build-base.sh plus the vendor parts (vendor-files.txt) and the WLAN
# profiles from the owner's own rootfs backup, the SSH host keys and the key for root. ext4 with only the features of
# the vendor rootfs (kernel 5.10), exactly the size of the rootfs partition. The image holds vendor files and secrets:
# it stays in the owner's encrypted storage, never in a repo, an upload or a release.
#   make-image.sh <base .tar> <vendor rootfs dir> <host key dir> <root key .pub> <new .img>
# "-" for the vendor rootfs or the host keys leaves them out (probe image for QEMU, keys made on the first boot).
# Prints no secrets; work files lie next to the image and are removed at the end.
set -euo pipefail
BASE=${1:?base .tar of build-base.sh}
VENDOR=${2:?directory with the vendor rootfs}
HOSTKEYS=${3:?directory with ssh_host_*_key and .pub}
ROOTKEY=${4:?public key for root}
OUT=${5:?new .img}

HERE=$(cd "$(dirname "$0")" && pwd)
SIZE=17179869184
FEATURES=none,has_journal,ext_attr,resize_inode,dir_index,filetype,extent,64bit,flex_bg,sparse_super,large_file,huge_file,dir_nlink,extra_isize,metadata_csum
MKE2FS=/usr/sbin/mke2fs

test ! -e "$OUT" || { echo "$OUT exists" >&2; exit 1; }
test -f "$BASE" && test -f "$ROOTKEY"
test "$VENDOR" = - || test -d "$VENDOR"
test "$HOSTKEYS" = - || test -d "$HOSTKEYS"
grep -q '^ssh-' "$ROOTKEY" || { echo "$ROOTKEY is no public key" >&2; exit 1; }

umask 077
work=$(mktemp -d "$(dirname "$OUT")/.make-image.XXXXXX")
trap 'rm -rf "$work"' EXIT

# Paths of the base tar without the leading ./ for the check of missing parent directories.
tar -tf "$BASE" | sed 's|^\./||; s|/$||' | sort -u > "$work/base.list"

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
    install -m 600 "$c" "$work/p/etc/NetworkManager/system-connections/$n"
    private+=("./etc/NetworkManager/system-connections/$n")
done
grep -qxF etc/NetworkManager/system-connections "$work/base.list"
grep -qxF root "$work/base.list"
tar -C "$work/p" -cf "$work/private.tar" --owner=0 --group=0 --numeric-owner --no-recursion "${private[@]}"

cp "$BASE" "$work/all.tar"
if [ -e "$work/vendor.tar" ]; then
    tar -Af "$work/all.tar" "$work/vendor.tar"
fi
tar -Af "$work/all.tar" "$work/private.tar"

truncate -s "$SIZE" "$OUT"
"$MKE2FS" -q -F -t ext4 -O "$FEATURES" -b 4096 -I 256 -E root_owner=0:0 -d "$work/all.tar" "$OUT" $((SIZE / 4096))
test "$(stat -c %s "$OUT")" -eq "$SIZE"
echo "image $OUT: ${#items[@]} vendor items, ${#parents[@]} new parent directories," \
     "$(( ${#private[@]} - 2 )) host key and WLAN files"
