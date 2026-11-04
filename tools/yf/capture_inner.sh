#!/bin/bash
# Capture of the inner network of an OpenMower v2 mower (board and host on eth0 of the CM4) onto this PC, for
# xcore_capture.py. tcpdump runs on the mower under sudo; it only listens (-p, no promiscuous mode) and sends nothing.
#
#   capture_inner.sh <ssh target> <new .pcap> [minutes, 150] [MB, 300]
#   SSH_OPTS="-i <key> -o IdentitiesOnly=yes" capture_inner.sh openmower@openmower /path/nx60i-mb-$(date +%H%M%S).pcap
#
# Asks for the sudo password of the mower once; it reaches ssh only on fd 3, never in an argument or the environment.
# The capture runs in its own session: Ctrl+C or closing the window does not end it. It ends after the minutes, at
# the MB, or with: kill -- -$(cat <pcap without .pcap>.pid)
set -u
TARGET=${1:?ssh target, e.g. openmower@openmower}
OUT=${2:?new .pcap file}
MINUTES=${3:-150}
MB=${4:-300}
case $OUT in *.pcap) ;; *) echo "the file must end in .pcap"; exit 1;; esac
test ! -e "$OUT" || { echo "exists: $OUT"; exit 1; }
mkdir -p "$(dirname "$OUT")"
# Drop pending input (a pasted line break would otherwise become an empty password).
while read -r -t 0.2 -s -n 10000 _; do :; done
read -rsp "sudo password of $TARGET: " PW; echo
[ -n "$PW" ] || { echo "no password, nothing started"; exit 1; }
exec 3<<<"$PW"
unset PW
# Board and host datagrams (UDP), and the board's pass-through and GPS channels (TCP), to see a second host on them.
FILTER='udp or (tcp and (port 10000 or portrange 65102-65104))'
# shellcheck disable=SC2086
setsid bash -c 'ssh '"${SSH_OPTS:-}"' "$1" "sudo -S -p \"\" timeout $(( $3 * 60 )) tcpdump -i eth0 -p -n -U -s 0 -w - \"$4\"" \
  <&3 | head -c "$5"M > "$2"' _ "$TARGET" "$OUT" "$MINUTES" "$FILTER" "$MB" > /dev/null 2> "${OUT%.pcap}.err" < /dev/null &
echo $! > "${OUT%.pcap}.pid"
exec 3<&-
sleep 6
ls -l "$OUT"
if [ -s "${OUT%.pcap}.err" ]; then cat "${OUT%.pcap}.err"; fi
echo "The capture goes on in the background ($MINUTES min, $MB MB at most). Stop it: kill -- -$(cat "${OUT%.pcap}.pid")"
