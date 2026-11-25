#!/bin/bash
# Offline checks of an image of make-image.sh before it goes near a mower: size, e2fsck, the same ext4 features as
# the vendor rootfs, the boot essentials, files/ of the repo unchanged, panic, key and sleep settings, owner and mode
# of the key files, and that every program and library finds its libraries under the image's own loader
# (qemu-aarch64). Prints no file contents.
#   check-image.sh <.img> <vendor rootfs .img> <new work dir, removed at the end>
# Result: one PASS or FAIL line per check; exit 1 on any FAIL.
set -uo pipefail
IMG=${1:?.img of make-image.sh}
REF=${2:?vendor rootfs .img}
WORK=${3:?new work dir}
HERE=$(cd "$(dirname "$0")" && pwd)
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
         /etc/systemd/system/timers.target.wants/mowbite-probezeit-check.timer /userdata /meta \
         /etc/locale.conf /etc/NetworkManager/conf.d/20-unused-wifi.conf /usr/bin/fuser \
         /etc/systemd/system/sysinit.target.wants/mowbite-crash.service \
         /etc/systemd/system/multi-user.target.wants/mowbite-tron-audio.service /usr/lib/mowbite/tron-audio \
         /usr/lib/mowbite/tron-stereo-ae /usr/lib/udev/rules.d/60-mowbite-stereo.rules \
         /etc/systemd/system/multi-user.target.wants/mowbite-tron-stereo-gain.service \
         /usr/lib/udev/rules.d/60-mowbite-rear.rules \
         /etc/systemd/system/sysinit.target.wants/systemd-pstore.service /usr/lib/mowbite/vendor-files \
         /usr/lib/mowbite/setup-ap /etc/systemd/system/multi-user.target.wants/mowbite-setup-ap.service \
         /usr/lib/mowbite/wifi-gesture \
         /usr/lib/mowbite/bluesleep-lock /etc/systemd/system/sysinit.target.wants/mowbite-bluesleep-lock.service; do
    "$DEBUGFS" -c -R "stat $f" "$IMG" 2>/dev/null | grep -q '^Inode:'; result "present: $f" $?
done
for t in motd-news apt-daily apt-daily-upgrade; do
    ! "$DEBUGFS" -c -R "stat /etc/systemd/system/timers.target.wants/$t.timer" "$IMG" 2>/dev/null | grep -q '^Inode:'
    result "off: $t.timer" $?
done
test "$(stat_of /usr/lib/mowbite/crash)" = "0755 0:0"; result "crash script 0755 root" $?
# The hardware stays off until the setup: the defaults say so and the settings of the image change nothing.
"$DEBUGFS" -c -R "cat /usr/lib/mowbite/os.defaults" "$IMG" 2>/dev/null | grep -qx 'MOWBITE_HARDWARE=false'
result "os.defaults: MOWBITE_HARDWARE=false" $?
"$DEBUGFS" -c -R "stat /etc/mowbite/os.conf" "$IMG" 2>/dev/null | grep -q '^Inode:' \
    && ! "$DEBUGFS" -c -R "cat /etc/mowbite/os.conf" "$IMG" 2>/dev/null | grep -qv '^[[:space:]]*\(#\|$\)'
result "os.conf: no active setting" $?
# Classifies the password fields without printing them: root hashed or locked, every other account locked.
"$DEBUGFS" -c -R "cat /etc/shadow" "$IMG" 2>/dev/null | awk -F: '
    $1 == "root" { root = $2 ~ /^\$(6|y)\$/ ? "set" : $2 ~ /^[!*]/ ? "none" : "bad"; next }
    $2 !~ /^[!*]/ { others++ }
    END { printf "root password: %s, other accounts with a password: %d\n", root, others
          exit !((root == "set" || root == "none") && !others) }'
result "root password hashed or locked, no other password" $?
test "$(stat_of /etc/shadow)" = "0640 0:42"; result "/etc/shadow 0640 root:shadow" $?
if [ -e "$IMG.json" ]; then
    # Base image of the installer: no key for root, the pack file empty at the place its .json names.
    ! "$DEBUGFS" -c -R "stat /root/.ssh/authorized_keys" "$IMG" 2>&1 | grep -q '^Inode:'
    result "no authorized_keys" $?
    test "$(stat_of /var/lib/mowbite/pack)" = "0600 0:0"; result "pack 0600 root" $?
    # System parts of the pack (modules, firmware) before module loading and udev; nothing in it may wait for mounts.
    unit=$("$DEBUGFS" -c -R "cat /etc/systemd/system/mowbite-unpack.service" "$IMG" 2>/dev/null)
    (grep -qx 'DefaultDependencies=no' <<< "$unit" && grep -qx 'ExecStart=/usr/lib/mowbite/unpack system' <<< "$unit" \
        && ! grep -q '^After=.*\(local-fs\|udev\|modules-load\)' <<< "$unit" \
        && for u in sysinit.target systemd-modules-load.service systemd-udevd.service systemd-udev-trigger.service \
            mowbite-tron-hwinit.service; do grep -q "^Before=.*\b$u\b" <<< "$unit" || exit 1; done)
    result "unpack system before modules-load, udev and hwinit" $?
    "$DEBUGFS" -c -R "stat /etc/systemd/system/sysinit.target.wants/mowbite-unpack.service" "$IMG" 2>/dev/null \
        | grep -q '^Inode:' && "$DEBUGFS" -c -R \
        "stat /etc/systemd/system/multi-user.target.wants/mowbite-unpack-inbox.service" "$IMG" 2>/dev/null \
        | grep -q '^Inode:'
    result "unpack units enabled" $?
    python3 -I -B - "$IMG" "$IMG.json" "$HERE/files/usr/lib/mowbite/unpack" \
        "$(/usr/sbin/debugfs -R "cat /usr/lib/mowbite/base" "$IMG" 2>/dev/null)" <<'PY'
import hashlib, importlib.machinery, importlib.util, json, os, sys
img, meta, script, base = sys.argv[1:5]
loader = importlib.machinery.SourceFileLoader("unpack", script)
unpack = importlib.util.module_from_spec(importlib.util.spec_from_loader("unpack", loader))
loader.exec_module(unpack)
j = json.load(open(meta))
assert j["size"] == os.path.getsize(img) and j["file"] == os.path.basename(img)
assert type(j["base"]) is int and j["base"] == int(base)
h = hashlib.sha256()
with open(img, "rb") as f:
    while chunk := f.read(8 << 20):
        h.update(chunk)
    assert h.hexdigest() == j["sha256"]
    # The pieces in order hold the file as make-image wrote it: the empty mark over and over.
    got = b""
    for o, n in j["pack"]:
        f.seek(o)
        got += f.read(n)
    assert got == (unpack.EMPTY * (len(got) // len(unpack.EMPTY) + 1))[:len(got)] and len(got) == 268435456
PY
    result "base .json: size, sha256, base number and the empty pack in its ranges" $?
else
    test "$(stat_of /root/.ssh)" = "0700 0:0"; result "/root/.ssh 0700 root" $?
    test "$(stat_of /root/.ssh/authorized_keys)" = "0600 0:0"; result "authorized_keys 0600 root" $?
fi
for d in /etc/ssh /etc/NetworkManager/system-connections; do
    "$DEBUGFS" -c -R "ls -p $d" "$IMG" 2>/dev/null | awk -F/ -v d="$d" '
        $6 != "" && $6 != "." && $6 != ".." && ($6 ~ /_key$/ || d ~ /system-connections/) {
            n++; if ($3 != "100600" || $4 != 0 || $5 != 0) bad++ }
        END { printf "%s: %d secret files, %d not 0600 root\n", d, n, bad; exit bad > 0 }'
    result "secret files in $d 0600 root" $?
done

# Counts only, the profiles hold the WLAN key.
profiles=0
ignoring=0
while IFS= read -r n; do
    profiles=$((profiles + 1))
    "$DEBUGFS" -c -R "cat \"/etc/NetworkManager/system-connections/$n\"" "$IMG" 2>/dev/null \
        | awk '/^\[/ { sec = $0 } sec == "[ipv6]" && $0 == "ignore-auto-dns=true" { f = 1 } END { exit !f }' \
        && ignoring=$((ignoring + 1))
done < <("$DEBUGFS" -c -R "ls -p /etc/NetworkManager/system-connections" "$IMG" 2>/dev/null \
         | awk -F/ '$6 != "" && $6 != "." && $6 != ".." { print $6 }')
echo "WLAN profiles: $profiles, $ignoring without IPv6 name servers"
test "$profiles" = "$ignoring"; result "WLAN profiles ignore IPv6 name servers" $?

# Every program and library of the image against the image's loader and libraries.
"$DEBUGFS" -c -R "rdump / $WORK" "$IMG" >/dev/null 2>&1
TREE=$WORK

# Before the ELF walk below rewrites absolute links in the tree.
(cd "$HERE/files" && find . ! -type d) | while read -r f; do
    if [ -L "$HERE/files/$f" ]; then
        test "$(readlink "$TREE/$f")" = "$(readlink "$HERE/files/$f")"
    else
        cmp -s "$TREE/$f" "$HERE/files/$f"
    fi || echo "  differs: $f"
done > "$WORK/files.diff"
test ! -s "$WORK/files.diff"; result "files/ of the repo unchanged in the image" $?
cat "$WORK/files.diff"
# Reading /proc/bluetooth/sleep/btwrite oopses the vendor kernel: the lock runs before sysinit, and no file of ours
# walks or packs /proc or names the bluesleep folder (only per-process entries are read).
unit="$TREE/etc/systemd/system/mowbite-bluesleep-lock.service"
grep -qx 'DefaultDependencies=no' "$unit" && grep -q '^Before=.*\bsysinit.target\b' "$unit" \
    && grep -qx 'ExecStart=/usr/lib/mowbite/bluesleep-lock' "$unit" \
    && grep -q 'mount -t tmpfs -o ro,' "$TREE/usr/lib/mowbite/bluesleep-lock"
result "bluesleep lock before sysinit" $?
(cd "$HERE/files" && find . -type f ! -path ./usr/lib/mowbite/bluesleep-lock \
    ! -path ./etc/systemd/system/mowbite-bluesleep-lock.service) | while read -r f; do
    grep -aqE '/proc/bluetooth|(find|tar|du|rsync|zip|cp +-[A-Za-z]*[rRa]|grep +-[A-Za-z]*[rR]|walk\(|glob\()[^|;&]*["'"'"' (]/proc/?(["'"'"' )*]|$)' \
        "$TREE/$f" && echo "  touches /proc as a whole: $f"
done > "$WORK/proc.txt"
test ! -s "$WORK/proc.txt"; result "no file of ours walks /proc or names /proc/bluetooth" $?
cat "$WORK/proc.txt"
# Only the MowBite file sets the panic values; a later file would override it.
test "$(grep -rlE '^[[:space:]]*kernel\.panic' "$TREE/etc/sysctl.conf" "$TREE/etc/sysctl.d" "$TREE/usr/lib/sysctl.d" \
    2>/dev/null)" = "$TREE/etc/sysctl.d/60-mowbite-panic.conf"
result "kernel.panic only in 60-mowbite-panic.conf" $?
! grep -q sysctl "$TREE/usr/lib/mowbite/probezeit"; result "probezeit sets no sysctl" $?
# A second start job in the same boot must not count again.
for u in mowbite-crash mowbite-probezeit; do
    grep -qx 'RemainAfterExit=yes' "$TREE/etc/systemd/system/$u.service"; result "$u stays active after its run" $?
done
sh -n "$TREE/usr/lib/mowbite/crash"; result "crash script parses" $?
grep -qFe "-name 'console-*'" "$TREE/usr/lib/mowbite/crash"; result "crash script reads console records" $?
bash -n "$TREE/usr/lib/mowbite/run-os" && bash -n "$TREE/usr/lib/mowbite/os-setup"; result "run-os parses" $?
for f in mowbite-update mowbite-host; do
    python3 -I -c 'import ast, sys; ast.parse(open(sys.argv[1]).read())' "$TREE/usr/sbin/$f"; result "$f parses" $?
done
diff -r --exclude=__pycache__ "$HERE/../../../host/mowbite_host" "$TREE/usr/lib/mowbite-host/mowbite_host" > /dev/null
result "library of mowbite-host as in host/ of the repo" $?
cmp -s "$HERE/../../../host/boot/mowbite_boot.py" "$TREE/usr/lib/mowbite-boot/mowbite_boot.py"
result "mowbite_boot as in host/boot of the repo" $?
python3 -I -c 'import json, sys; m = json.load(open(sys.argv[1]))
sys.exit(not (m["component"] == "host" and isinstance(m["serial"], int) and m["serial"] > 0))' \
    "$TREE/usr/lib/mowbite-host/bundle.json"
result "seed of mowbite-host with its bundle.json" $?
(cd "$TREE" && find usr/lib/mowbite-host) | while read -r p; do
    if [ -d "$TREE/$p" ]; then want="0755 0:0"; else want="0644 0:0"; fi
    test "$(stat_of "/$p")" = "$want" || echo "  mode or owner: /$p"
done > "$WORK/seed.modes"
test ! -s "$WORK/seed.modes"; result "seed of mowbite-host 0755 and 0644, owned by root" $?
cat "$WORK/seed.modes"
grep -qx "d /opt/mowbite-host 0755 root root -" "$TREE/usr/lib/tmpfiles.d/mowbite-os.conf"
result "tmpfiles: /opt/mowbite-host" $?
cmp -s "$HERE/../../../web/mowbite_web.py" "$TREE/usr/lib/mowbite-web/mowbite_web.py"
result "mowbite-web as in web/ of the repo" $?
test "$(stat_of /usr/lib/mowbite-web) $(stat_of /usr/lib/mowbite-web/mowbite_web.py)" = "0755 0:0 0644 0:0"
result "mowbite-web readable for its user, owned by root" $?
test "$(readlink "$TREE/etc/systemd/system/sockets.target.wants/mowbite-web.socket")" = ../mowbite-web.socket
result "mowbite-web.socket enabled" $?
grep -qx 'User=mowbite-web' "$TREE/etc/systemd/system/mowbite-web.service" \
    && grep -qx 'CapabilityBoundingSet=' "$TREE/etc/systemd/system/mowbite-web.service" \
    && ! grep -qE '^(ReadWritePaths|StateDirectory|SupplementaryGroups|AmbientCapabilities)=' \
        "$TREE/etc/systemd/system/mowbite-web.service"
result "mowbite-web without root, capability, group or path to write" $?
grep -q '^u mowbite-web ' "$TREE/usr/lib/sysusers.d/mowbite-web.conf" \
    && ! grep -qE '^[mg] +mowbite-web' "$TREE"/usr/lib/sysusers.d/*.conf
result "user mowbite-web in no group" $?
# The folders of the components as mowbite-host knows them.
places=$(python3 -I -c 'import sys; sys.path[:0] = sys.argv[1:]; from mowbite_host.paths import PLACES
print(*(opt for opt, _ in PLACES.values()))' "$TREE/usr/lib/mowbite-host" "$TREE/usr/lib/mowbite-boot")
test -n "$places"; result "folders of the components read from mowbite-host" $?
for d in $places; do
    grep -qx "d $d 0755 root root -" "$TREE/usr/lib/tmpfiles.d/mowbite-os.conf"; result "tmpfiles: $d" $?
    grep -qx "ConditionPathExists=|$d/pending" "$TREE/etc/systemd/system/mowbite-update-recover.service"
    result "mowbite-update-recover for $d" $?
done
grep -q 'readlink -e /opt/mowbite-planner/active)' "$TREE/usr/lib/mowbite/os-setup"
result "os-setup takes the planner active names" $?
test "$(readlink "$TREE/etc/systemd/system/multi-user.target.wants/mowbite-host.service")" = ../mowbite-host.service
result "mowbite-host.service enabled" $?
grep -qx '[0-9]\+' "$TREE/usr/lib/mowbite/base"; result "rootfs version in /usr/lib/mowbite/base" $?
# A lost key must not lock every mower out: a main and a reserve key, both for bundles and the index.
namespaces="mowbite-os,mowbite-app,mowbite-planner,mowbite-host,mowbite-index"
test "$(grep -cE "^mowbite namespaces=\"$namespaces\" ssh-ed25519 " "$TREE/etc/mowbite/allowed_signers")" -ge 2
result "two keys in allowed_signers" $?
test "$(readlink "$TREE/etc/systemd/system/multi-user.target.wants/mowbite-os.service")" = ../mowbite-os.service
result "mowbite-os.service enabled" $?
test "$(readlink "$TREE/etc/systemd/system/multi-user.target.wants/mowbite-update-recover.service")" = \
    ../mowbite-update-recover.service
result "mowbite-update-recover.service enabled" $?
! grep -q '/run/mowbite/crash' "$TREE/etc/systemd/system/mowbite-os.service"
result "mowbite-os starts after a crash, its restart hold holds" $?
sh -n "$TREE/usr/sbin/mowbite-crash-ok"; result "mowbite-crash-ok parses" $?
bash -n "$TREE/usr/sbin/mowbite-measure"; result "mowbite-measure parses" $?
# mowbite-measure keeps the OS stopped with the lock of mowbite-host and asks its seed for a switch cut off.
lock=$(python3 -I -c 'import sys; sys.path[:0] = sys.argv[1:]; from mowbite_host.paths import Paths
print(Paths().lock)' "$TREE/usr/lib/mowbite-host" "$TREE/usr/lib/mowbite-boot")
grep -qx "HOST_LOCK=$lock" "$TREE/usr/sbin/mowbite-measure"; result "mowbite-measure takes the lock of mowbite-host" $?
test "$(grep -c '^mowbite-update unfinished ' "$TREE/usr/sbin/mowbite-measure")" = 2
result "mowbite-measure asks mowbite-update unfinished before it stops anything and under the lock" $?
MOWBITE_UPDATE_PREFIX=$TREE python3 -I -c 'import sys; sys.path[:0] = sys.argv[1:]; from mowbite_host import cli
sys.exit(cli.main(["mowbite-update", "unfinished"]))' "$TREE/usr/lib/mowbite-host" "$TREE/usr/lib/mowbite-boot"
result "mowbite-update unfinished of the seed answers 0 on the image" $?
test "$(stat_of /usr/sbin/mowbite-crash-ok)" = "0755 0:0"; result "mowbite-crash-ok 0755 root" $?
cmp -s "$TREE/etc/mosquitto/mosquitto.conf" "$HERE/../../../docker/mower/mosquitto.conf"
result "broker configuration of the Compose stack" $?
# keys <template> <prefix> <value>: every key the template of this systemd knows, set to <value>.
keys() { sed -n "s/^#\($2[A-Za-z]*\)=.*/\1=$3/p" "$TREE/$1" | sort; }
set_in() { grep "^$2" "$TREE/$1" | sort; }
test "$(keys etc/systemd/logind.conf Handle ignore)" = "$(set_in etc/systemd/logind.conf.d/mowbite.conf Handle)" \
    && test -n "$(keys etc/systemd/logind.conf Handle ignore)"
result "logind ignores every key and switch" $?
test "$(keys etc/systemd/sleep.conf Allow no)" = "$(set_in etc/systemd/sleep.conf.d/mowbite.conf Allow)" \
    && test -n "$(keys etc/systemd/sleep.conf Allow no)"
result "every sleep state off" $?
# ROS libraries come from the environment, systemd's private ones from the RUNPATH of its programs.
LIBPATH=/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu/systemd
# Loaded only for the ATM qdisc of tc, which needs libatm (a recommendation of iproute2, not installed).
UNUSED=/usr/lib/aarch64-linux-gnu/tc/q_atm.so
python3 -I - "$TREE" > "$WORK/elf.list" <<'EOF'
import os, struct, sys
root = sys.argv[1]
for top in ("usr", "opt", "etc", "vendor"):
    for d, _, files in os.walk(os.path.join(root, top)):
        for f in files:
            p = os.path.join(d, f)
            # Absolute links (alternatives) would resolve on the host under qemu -L; this copy points them into the tree.
            if os.path.islink(p) and os.readlink(p).startswith("/"):
                target = root + os.readlink(p)
                os.unlink(p)
                os.symlink(target, p)
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
# A failing loader counts as well: it may stop with "error while loading" before it prints "not found".
while read -r elf; do
    test "${elf#$TREE}" != "$UNUSED" || continue
    out=$(qemu-aarch64 -L "$TREE" "$TREE/usr/lib/ld-linux-aarch64.so.1" --library-path "$LIBPATH" --list "$elf" 2>&1)
    rc=$?
    if [ $rc != 0 ] || grep -qE 'not found|error while loading' <<< "$out"; then
        echo "${elf#$TREE}: rc $rc"
        grep -E 'not found|error while loading' <<< "$out" | sed 's/^[[:space:]]*/    /'
    fi >> "$WORK/missing.txt"
done < "$WORK/elf.list"
echo "$(wc -l < "$WORK/elf.list") ELF files checked"
test ! -s "$WORK/missing.txt"; result "all libraries found" $?
sed 's/^/  /' "$WORK/missing.txt" | head -40
exit $fail
