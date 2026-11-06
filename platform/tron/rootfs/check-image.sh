#!/bin/bash
# Offline checks of an image of make-image.sh before it goes near a mower: size, e2fsck, the same ext4 features as
# the vendor rootfs, the boot essentials, owner and mode of the key files, and that every program and library finds
# its libraries under the image's own loader (qemu-aarch64). Prints no file contents.
#   check-image.sh <.img> <vendor rootfs .img> <new work dir, removed at the end>
# Result: one PASS or FAIL line per check; exit 1 on any FAIL.
set -uo pipefail
IMG=${1:?.img of make-image.sh}
REF=${2:?vendor rootfs .img}
WORK=${3:?new work dir}
DEBUGFS=/usr/sbin/debugfs
SIZE=17179869184

test ! -e "$WORK" || { echo "$WORK exists" >&2; exit 1; }
mkdir -p "$WORK"
trap 'rm -rf "$WORK"' EXIT
fail=0
result() { if [ "$2" = 0 ]; then echo "PASS $1"; else echo "FAIL $1"; fail=1; fi; }

test "$(stat -c %s "$IMG")" -eq "$SIZE"; result "size $SIZE B" $?
/usr/sbin/e2fsck -fn "$IMG" >"$WORK/e2fsck.txt" 2>&1; result "e2fsck clean" $?
features() { /usr/sbin/dumpe2fs -h "$1" 2>/dev/null | sed -n 's/^Filesystem features: *//p'; }
test "$(features "$IMG")" = "$(features "$REF")"; result "ext4 features as the vendor rootfs" $?

stat_of() { "$DEBUGFS" -c -R "stat $1" "$IMG" 2>/dev/null | sed -n 's/.*Mode: *\([0-7]*\).*/\1/p; s/^User: *\([0-9]*\) *Group: *\([0-9]*\).*/\1:\2/p' | paste -sd' '; }
test "$("$DEBUGFS" -c -R 'stat /usr/sbin/init' "$IMG" 2>/dev/null | grep -c 'Fast link dest: "../lib/systemd/systemd"')" = 1
result "/sbin/init is systemd" $?
for f in /etc/mowbite/probezeit /usr/lib/mowbite/probezeit /usr/lib/mowbite/tron-hwinit /etc/fstab \
         /etc/systemd/system/sysinit.target.wants/mowbite-probezeit.service \
         /etc/systemd/system/timers.target.wants/mowbite-probezeit-check.timer; do
    "$DEBUGFS" -c -R "stat $f" "$IMG" 2>/dev/null | grep -q '^Inode:'; result "present: $f" $?
done
test "$(stat_of /root/.ssh)" = "0700 0:0"; result "/root/.ssh 0700 root" $?
test "$(stat_of /root/.ssh/authorized_keys)" = "0600 0:0"; result "authorized_keys 0600 root" $?
for d in /etc/ssh /etc/NetworkManager/system-connections; do
    "$DEBUGFS" -c -R "ls -p $d" "$IMG" 2>/dev/null | awk -F/ -v d="$d" '
        $6 != "" && $6 != "." && $6 != ".." && ($6 ~ /_key$/ || d ~ /system-connections/) {
            n++; if ($3 != "100600" || $4 != 0 || $5 != 0) bad++ }
        END { printf "%s: %d secret files, %d not 0600 root\n", d, n, bad; exit bad > 0 }'
    result "secret files in $d 0600 root" $?
done

# Every program and library of the image against the image's loader and libraries.
"$DEBUGFS" -c -R "rdump / $WORK" "$IMG" >/dev/null 2>&1
TREE=$WORK
LIBPATH=/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/aarch64-linux-gnu
python3 -I - "$TREE" > "$WORK/elf.list" <<'EOF'
import os, struct, sys
root = sys.argv[1]
for top in ("usr", "opt", "etc", "vendor"):
    for d, _, files in os.walk(os.path.join(root, top)):
        for f in files:
            p = os.path.join(d, f)
            if os.path.islink(p) or not os.path.isfile(p):
                continue
            with open(p, "rb") as h:
                head = h.read(20)
            # Executables and shared objects for aarch64; kernel modules are relocatable and skipped.
            if head[:4] == b"\x7fELF" and len(head) == 20 and struct.unpack("<HH", head[16:20]) in ((2, 183), (3, 183)):
                print(p)
EOF
# The loader has to resolve a known program, otherwise an empty list below proves nothing.
qemu-aarch64 -L "$TREE" "$TREE/usr/lib/ld-linux-aarch64.so.1" --list "$TREE/usr/bin/ls" 2>&1 | grep -q 'libc.so.6 => '
result "loader of the image resolves libraries" $?
: > "$WORK/missing.txt"
while read -r elf; do
    qemu-aarch64 -L "$TREE" "$TREE/usr/lib/ld-linux-aarch64.so.1" --library-path "$LIBPATH" --list "$elf" 2>&1 \
        | sed -n "s|^\(.*not found\)|${elf#$TREE}: \1|p" >> "$WORK/missing.txt"
done < "$WORK/elf.list"
echo "$(wc -l < "$WORK/elf.list") ELF files checked"
test ! -s "$WORK/missing.txt"; result "all libraries found" $?
sed 's/^/  /' "$WORK/missing.txt" | head -40
exit $fail
