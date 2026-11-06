#!/bin/bash
# Boot probe of a probe image (make-image.sh with "-" for the vendor rootfs) in QEMU: virt board, the generic Ubuntu
# kernel, no vendor parts, so it says nothing about the vendor kernel. Checks that systemd and sshd come up and that
# the trial period counts boots, falls back after three boots and without a login, and ends on request. Off the
# Rockchip board the fallback only logs. Each boot ends with a power-off; QEMU runs with -no-reboot.
#   qemu-probe.sh <probe .img> <private test key> <new log dir>
# Result: <log dir>/result.txt with one PASS or FAIL line per check, the consoles and command outputs beside it.
set -uo pipefail
IMG=${1:?probe .img}
KEY=${2:?private key whose .pub make-image.sh put in for root}
LOG=${3:?new log dir}
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

stop_vm() {
    local t=0
    printf '{"execute":"qmp_capabilities"}\n{"execute":"system_powerdown"}\n' \
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
check "kernel.panic 10" test "$(vm sysctl -n kernel.panic)" = 10
check "no failed units" test -z "$(vm systemctl --failed --no-legend --plain)"
check "hwinit skipped without vendor parts" \
    test "$(vm systemctl show -P ConditionResult mowbite-tron-hwinit.service)" = no
check "units verify" vm systemd-analyze verify /etc/systemd/system/mowbite-*.service /etc/systemd/system/mowbite-*.timer
check "ROS 2 Jazzy core" vm 'bash -c ". /opt/ros/jazzy/setup.bash && ros2 pkg prefix rclcpp"'
vm 'set -x; systemctl is-system-running; systemctl --failed --no-legend; systemctl is-enabled ssh.service ssh.socket
    ss -tlnp; nmcli -t device; resolvectl status | head -5; timedatectl; findmnt -no SOURCE,TARGET,OPTIONS /
    free -m; df -h /; ps -eo rss,comm --sort=-rss | head -15; systemd-analyze; systemd-analyze blame | head -15
    systemctl --version | head -1; systemctl reboot --help | grep -i -A1 "arg"; ls -l /etc/resolv.conf
    cat /etc/machine-id; dpkg -l | grep -c ^ii; du -sh /usr /var 2>/dev/null; date -u
    journalctl --list-boots --no-pager; ls -l /var/log/journal/*/ /run/log/journal 2>&1' > "$LOG/system.txt" 2>&1
vm 'systemctl reboot' >/dev/null 2>&1
wait "$vm_pid" 2>/dev/null

# Boot 5 without a login: the counter starts again at 1.
start_vm
check "boot 5 reaches multi-user" wait_console 'Reached target.*Multi-User' 900
stop_vm
check "counter restarted at 1" test "$(offline_cat /var/lib/mowbite/bootcount)" = 1

# Boot 6: the owner ends the trial period.
start_vm
check "boot 6 ssh login" wait_ssh
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
