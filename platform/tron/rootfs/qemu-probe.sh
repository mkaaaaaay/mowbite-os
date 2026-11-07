#!/bin/bash
# Boot probe of a probe image (make-image.sh with "-" for the vendor rootfs) in QEMU: virt board, the generic Ubuntu
# kernel, no vendor parts, so it says nothing about the vendor kernel. Checks that systemd and sshd come up and that
# the trial period counts boots, falls back after three boots and without a login, and ends on request; the panic
# settings, that logind ignores every key and cannot suspend, and mowbite-crash on dummy dmesg records, by hand and
# in a real boot. Off the Rockchip board the fallback only logs. Each boot ends with a power-off; QEMU runs with
# -no-reboot.
#   qemu-probe.sh <probe .img> <private test key> <new log dir> <test password of the hash make-image.sh put in>
# Result: <log dir>/result.txt with one PASS or FAIL line per check, the consoles and command outputs beside it.
set -uo pipefail
IMG=${1:?probe .img}
KEY=${2:?private key whose .pub make-image.sh put in for root}
LOG=${3:?new log dir}
PASSWORD=${4:?file with the test password for root}
CLOUD=https://cloud-images.ubuntu.com/noble/current/unpacked
PORT=${PROBE_SSH_PORT:-2522}

test ! -e "$LOG" || { echo "$LOG exists" >&2; exit 1; }
mkdir -p "$LOG/kernel"
exec 3>"$LOG/result.txt"

fail=0
check() {
    local what=$1; shift
    if "$@" >>"$LOG/checks.txt" 2>&1; then echo "PASS $what" >&3; else echo "FAIL $what" >&3; fail=1; fi
}

info() {
    local what=$1; shift
    if "$@" >>"$LOG/checks.txt" 2>&1; then echo "INFO yes: $what" >&3; else echo "INFO no: $what" >&3; fi
}

for f in vmlinuz-generic initrd-generic; do
    curl -fsSL "$CLOUD/noble-server-cloudimg-arm64-$f" -o "$LOG/kernel/$f" || exit 1
done
curl -fsSL "$CLOUD/SHA256SUMS" -o "$LOG/kernel/SHA256SUMS" || exit 1
curl -fsSL "$CLOUD/SHA256SUMS.gpg" -o "$LOG/kernel/SHA256SUMS.gpg" || exit 1
gpgv --keyring /usr/share/keyrings/ubuntu-cloudimage-keyring.gpg "$LOG/kernel/SHA256SUMS.gpg" "$LOG/kernel/SHA256SUMS" \
    2>/dev/null || { echo "signature of SHA256SUMS" >&2; exit 1; }
(cd "$LOG/kernel" && sed -n 's/\(noble-server-cloudimg-arm64-\)\(.*generic\)$/\2/p' SHA256SUMS | sed 's/ \*/  /' \
    | sha256sum -c --quiet) || { echo "kernel checksum" >&2; exit 1; }

boot=0
vm_pid=
start_vm() {
    boot=$((boot + 1))
    rm -f "$LOG/qmp.sock"
    qemu-system-aarch64 -M virt -cpu max -smp 4 -m 2048 -no-reboot -display none -monitor none \
        -kernel "$LOG/kernel/vmlinuz-generic" -initrd "$LOG/kernel/initrd-generic" \
        -append "root=/dev/vda rw console=ttyAMA0" \
        -drive "file=$IMG,if=virtio,format=raw" \
        -netdev "user,id=n0,hostfwd=tcp:127.0.0.1:$PORT-:22" -device virtio-net-pci,netdev=n0 \
        -device qemu-xhci -device usb-kbd \
        -qmp "unix:$LOG/qmp.sock,server=on,wait=off" -serial "file:$LOG/console-$boot.log" &
    vm_pid=$!
}

wait_console() {
    local pattern=$1 limit=$2 t=0
    until grep -q "$pattern" "$LOG/console-$boot.log" 2>/dev/null; do
        kill -0 "$vm_pid" 2>/dev/null || return 1
        [ $t -lt "$limit" ] || return 1
        sleep 5; t=$((t + 5))
    done
}

# logind ignores the power button, so Ctrl-Alt-Del on a USB keyboard (the initrd of the probe kernel has no virtio
# input driver): a clean reboot, which -no-reboot turns into the end of QEMU.
stop_vm() {
    local t=0
    printf '%s\n' '{"execute":"qmp_capabilities"}' '{"execute":"send-key","arguments":{"keys":[
        {"type":"qcode","data":"ctrl"},{"type":"qcode","data":"alt"},{"type":"qcode","data":"delete"}]}}' \
        | socat - "UNIX-CONNECT:$LOG/qmp.sock" >/dev/null 2>&1
    while kill -0 "$vm_pid" 2>/dev/null && [ $t -lt 180 ]; do sleep 5; t=$((t + 5)); done
    if kill -0 "$vm_pid" 2>/dev/null; then
        echo "FAIL boot $boot: no clean power-off, killed" >&3; fail=1
        kill "$vm_pid"
    fi
    wait "$vm_pid" 2>/dev/null
}

vm() {
    ssh -i "$KEY" -p "$PORT" -o BatchMode=yes -o IdentitiesOnly=yes -o ConnectTimeout=10 \
        -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR root@127.0.0.1 "$@"
}

wait_ssh() {
    local t=0
    until vm true 2>/dev/null; do
        [ $t -lt 600 ] || return 1
        sleep 10; t=$((t + 10))
    done
}

# Only while QEMU is off: a file of the image, and whether its journal holds a message and at least n boots.
offline_cat() {
    /usr/sbin/debugfs -c -R "cat $1" "$IMG" 2>/dev/null
}
journal_has() {
    local dir="$LOG/journal-$boot"
    mkdir -p "$dir" && /usr/sbin/debugfs -c -R "rdump /var/log/journal $dir" "$IMG" >/dev/null 2>&1
    journalctl -D "$dir/journal" --no-pager -o cat | grep -qF "$1" \
        && [ "$(journalctl -D "$dir/journal" --no-pager --list-boots | tail -n +2 | wc -l)" -ge "$2" ]
}

trap 'kill "$vm_pid" 2>/dev/null' EXIT

# Boots 1 to 3 without a login: the counter climbs, the third boot falls back (logged only). The counter is read
# from the image between boots, a login would reset it.
for i in 1 2 3; do
    start_vm
    check "boot $boot reaches multi-user" wait_console 'Reached target.*Multi-User' 900
    stop_vm
    check "counter $boot after boot $boot" test "$(offline_cat /var/lib/mowbite/bootcount)" = "$boot"
done
# QEMU starts on the build time of systemd until the RTC module loads, so the early entries fall under the journal's
# 14 days and are gone; on the Tron the RTC is built in. Reported, not counted.
info "journal keeps boots 1 to 3" journal_has 'boot 3 without a confirmed login, reboot into the loader' 3

# Boot 4: no login for more than 10 minutes, then the first login.
start_vm
check "boot 4 reaches multi-user" wait_console 'Reached target.*Multi-User' 900
sleep 660
check "boot 4 ssh login" wait_ssh
vm 'journalctl -o cat -u mowbite-probezeit.service -u mowbite-probezeit-check.service --no-pager' > "$LOG/probezeit.txt" 2>&1
info "boot 4 counted in the journal" grep -q 'boot 4 of 3 without a confirmed login' "$LOG/probezeit.txt"
check "lttng-sessiond off" test "$(vm systemctl is-enabled lttng-sessiond.service)" = disabled
check "fallback without login" grep -q 'no SSH login since the boot, reboot into the loader' "$LOG/probezeit.txt"
check "fallback as a dry run off the board" grep -q 'no Rockchip board, dry run only' "$LOG/probezeit.txt"
check "login reset the counter" test "$(vm cat /var/lib/mowbite/bootcount)" = 0
check "kernel.panic 10, panic_on_oops 1, panic_on_rcu_stall 1" \
    test "$(vm sysctl -n kernel.panic kernel.panic_on_oops kernel.panic_on_rcu_stall | paste -sd' ')" = "10 1 1"
vm 'm="org.freedesktop.login1 /org/freedesktop/login1 org.freedesktop.login1.Manager"
    for p in $(busctl introspect --list $m | awk "\$1 ~ /^\.Handle/ { print substr(\$1, 2) }"); do
        echo "$p $(busctl get-property $m $p)"
    done
    for c in CanSuspend CanHibernate CanHybridSleep CanSuspendThenHibernate; do echo "$c $(busctl call $m $c)"; done' \
    > "$LOG/logind.txt" 2>&1
check "logind HandlePowerKey ignore" grep -qx 'HandlePowerKey s "ignore"' "$LOG/logind.txt"
check "logind ignores every key and switch" \
    awk '$1 ~ /^Handle/ { n++; if ($3 != "\"ignore\"") bad++ } END { exit !(n >= 11 && !bad) }' "$LOG/logind.txt"
# "no": disabled by sleep.conf ("na" would mean the hardware cannot).
check "CanSuspend no" grep -qx 'CanSuspend s "no"' "$LOG/logind.txt"
check "no sleep state" awk '$1 ~ /^Can/ { n++; if ($3 != "\"no\"") bad++ } END { exit !(n == 4 && !bad) }' "$LOG/logind.txt"
check "mowbite-crash ran in the boot" test "$(vm systemctl show -P ConditionResult mowbite-crash.service)" = yes
check "mowbite-crash exit 0" test "$(vm systemctl show -P ExecMainStatus mowbite-crash.service)" = 0
check "no crash flag after a clean boot" vm test ! -e /run/mowbite/crash
check "no failed units" test -z "$(vm systemctl --failed --no-legend --plain)"
check "hwinit skipped without vendor parts" \
    test "$(vm systemctl show -P ConditionResult mowbite-tron-hwinit.service)" = no
check "units verify" vm systemd-analyze verify /etc/systemd/system/mowbite-*.service /etc/systemd/system/mowbite-*.timer
check "ROS 2 Jazzy core" vm 'bash -c ". /opt/ros/jazzy/setup.bash && ros2 pkg prefix rclcpp"'
# Password login for root from private networks only; QEMU's user network connects from 10.0.2.2.
pw_of() {
    vm "sshd -T -C user=$1,host=probe,addr=$2" | awk '$1 == "passwordauthentication" { print $2 }'
}
check "sshd: root password from a private network" test "$(pw_of root 10.1.2.3)" = yes
check "sshd: no root password from outside" test "$(pw_of root 203.0.113.5)" = no
check "sshd: no password for other users" test "$(pw_of nobody 10.1.2.3)" = no
printf '#!/bin/sh\ncat "%s"\n' "$PASSWORD" > "$LOG/askpass"
printf '#!/bin/sh\necho wrong\n' > "$LOG/askpass-wrong"
chmod 700 "$LOG/askpass" "$LOG/askpass-wrong"
pw_login() {
    SSH_ASKPASS=$1 SSH_ASKPASS_REQUIRE=force DISPLAY=none setsid -w ssh -p "$PORT" -o PubkeyAuthentication=no \
        -o PreferredAuthentications=password -o NumberOfPasswordPrompts=1 -o ConnectTimeout=10 \
        -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR root@127.0.0.1 true </dev/null
}
check "root login with the password" pw_login "$LOG/askpass"
check "no root login with a wrong password" eval '! pw_login "$LOG/askpass-wrong"'
vm 'set -x; systemctl is-system-running; systemctl --failed --no-legend; systemctl is-enabled ssh.service ssh.socket
    ss -tlnp; nmcli -t device; resolvectl status | head -5; timedatectl; findmnt -no SOURCE,TARGET,OPTIONS /
    free -m; df -h /; ps -eo rss,comm --sort=-rss | head -15; systemd-analyze; systemd-analyze blame | head -15
    systemctl --version | head -1; systemctl reboot --help | grep -i -A1 "arg"; ls -l /etc/resolv.conf
    cat /etc/machine-id; dpkg -l | grep -c ^ii; du -sh /usr /var 2>/dev/null; date -u
    journalctl --list-boots --no-pager; ls -l /var/log/journal/*/ /run/log/journal 2>&1' > "$LOG/system.txt" 2>&1

# mowbite-crash by hand on dummy dmesg records; the trial mark is away while the latch is probed. Afterwards the
# state is cleared and one record stays for boot 5.
vm 'sh -s' > "$LOG/crash.txt" 2>&1 <<'CRASH'
set -u
P=/var/lib/systemd/pstore
C=/var/lib/mowbite/crashes
F=/run/mowbite/crash
L=/run/mowbite/crash-latch
now=$(date +%s)
record() {
    mkdir -p "$P"
    printf 'Panic#1 Part1\n<0>Kernel panic - not syncing: %s\n' "$1" > "$P/dmesg-ramoops-0"
    touch -d "@$((now - $2))" "$P/dmesg-ramoops-0"
}
run() { rm -f "$F" "$L"; /usr/lib/mowbite/crash; }
expect() { if eval "$2"; then echo "PASS crash: $1"; else echo "FAIL crash: $1"; fi; }
lines() { wc -l < "$C"; }
panic() { sysctl -n kernel.panic; }
mv /etc/mowbite/probezeit /root/probezeit.away
trap 'mv /root/probezeit.away /etc/mowbite/probezeit 2>/dev/null; sysctl -q kernel.panic=10' EXIT
rm -f "$P"/dmesg-* "$C" /var/lib/mowbite/pstore-seen
run
expect "no record, no flag" 'test ! -e $F && test ! -e $C'
record one 300; run
expect "one record, flag, no latch" 'grep -q "not syncing: one" $F && test ! -e $L && test $(lines) = 1 && test $(panic) = 10'
run
expect "same record again, no flag" 'test ! -e $F && test $(lines) = 1'
record two 200; run; record three 100; run
expect "three in 1 h, latch, kernel.panic 0" 'grep -q "not syncing: three" $F && test -e $L && test $(panic) = 0'
sysctl -q kernel.panic=10
printf '%s old\n' $((now - 7300)) $((now - 7200)) $((now - 7100)) > "$C"
record four 90; run
expect "three older than 1 h, no latch" 'test -e $F && test ! -e $L && test $(panic) = 10'
mv /root/probezeit.away /etc/mowbite/probezeit
printf '%s recent\n' $((now - 600)) $((now - 500)) > "$C"
record five 80; run
expect "three in 1 h with trial mark, no latch" 'test -e $F && test ! -e $L && test $(panic) = 10'
rm -f "$P"/dmesg-* "$C" /var/lib/mowbite/pstore-seen "$F" "$L"
record probe 10
sync
CRASH
grep -E '^(PASS|FAIL) ' "$LOG/crash.txt" >&3
test "$(grep -c '^PASS ' "$LOG/crash.txt")" = 6 || { echo "FAIL crash: not all 6 cases passed" >&3; fail=1; }
vm 'systemctl reboot' >/dev/null 2>&1
wait "$vm_pid" 2>/dev/null

# Boot 5 without a login: the counter starts again at 1, mowbite-crash records the dummy record of boot 4.
start_vm
check "boot 5 reaches multi-user" wait_console 'Reached target.*Multi-User' 900
stop_vm
check "counter restarted at 1" test "$(offline_cat /var/lib/mowbite/bootcount)" = 1
check "boot 5 recorded the crash" grep -q '^[0-9]* Kernel panic - not syncing: probe$' <(offline_cat /var/lib/mowbite/crashes)

# Boot 6: the owner ends the trial period.
start_vm
check "boot 6 ssh login" wait_ssh
check "boot 6: crash seen once, no flag" vm 'test ! -e /run/mowbite/crash && test $(wc -l < /var/lib/mowbite/crashes) = 1'
check "trial period ends" vm mowbite-probezeit-ende
vm 'systemctl reboot' >/dev/null 2>&1
wait "$vm_pid" 2>/dev/null

# Boot 7: no trial period any more.
start_vm
check "boot 7 ssh login" wait_ssh
check "no counting after the end" \
    test "$(vm systemctl show -P ConditionResult mowbite-probezeit.service)" = no
check "no counter file" vm test ! -e /var/lib/mowbite/bootcount
stop_vm

echo "boots: $boot, result: $([ $fail = 0 ] && echo PASS || echo FAIL)" >&3
exit $fail
