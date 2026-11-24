#!/bin/bash
# Base rootfs of MowBite OS for the Airseekers Tron as a tar: Ubuntu 24.04 arm64, ROS 2 Jazzy core, the units and
# configuration under files/, no vendor parts, no keys. Without root: mmdebstrap in a user namespace, arm64 binaries
# through the qemu-user binfmt handler. make-image.sh turns the tar into the image of one mower and lays the current
# files/ over it, so only a change of packages.txt, of this script or a removal under files/ needs a new base. It
# rewrites base-packages.txt beside it with the packages the tar holds, the list make-bundle.sh checks bundles against.
#   build-base.sh <new .tar>
set -euo pipefail
OUT=${1:?new .tar}
HERE=$(cd "$(dirname "$0")" && pwd)
PORTS=http://ports.ubuntu.com/ubuntu-ports
ROS=http://packages.ros.org/ros2/ubuntu
UBUNTU_KEY=/usr/share/keyrings/ubuntu-archive-keyring.gpg
# Signing key of packages.ros.org (Open Robotics).
ROS_KEY_FPR=C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
UNITS="mowbite-probezeit.service mowbite-probezeit-check.timer mowbite-tron-hwinit.service mowbite-ssh-hostkeys.service"
# Off: tracing daemon, Ubuntu news and package lists fetched on their own (updates come through MowBite OS).
OFF="lttng-sessiond.service motd-news.timer apt-daily.timer apt-daily-upgrade.timer"

test ! -e "$OUT" || { echo "$OUT exists" >&2; exit 1; }
work=$(mktemp -d)
built=0
trap 'rm -rf "$work"; [ $built = 1 ] || rm -f "$OUT"' EXIT

curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$work/ros.key"
fpr=$(gpg --show-keys --with-colons "$work/ros.key" | awk -F: '$1 == "pub" { p = 1; next } p && $1 == "fpr" { print $10; p = 0 }')
test "$fpr" = "$ROS_KEY_FPR" || { echo "unexpected ROS key: $fpr" >&2; exit 1; }
gpg --dearmor < "$work/ros.key" > "$work/ros-archive-keyring.gpg"
# apt runs as root of the user namespace, which is not the owner of the work dir.
chmod 755 "$work"
chmod 644 "$work/ros-archive-keyring.gpg"
# files/ as root's, without write access for group and others, whoever checked out the repo.
tar -C "$HERE/files" -cf "$work/files.tar" --owner=0 --group=0 --numeric-owner --mode=go-w .
"$HERE/base-inputs.sh" > "$work/base-inputs"
chmod 644 "$work/base-inputs"

mmdebstrap --mode=unshare --architectures=arm64 --variant=apt --format=tar \
    --aptopt="$HERE/files/etc/apt/apt.conf.d/90mowbite" \
    --dpkgopt="$HERE/files/etc/dpkg/dpkg.cfg.d/90mowbite-nodoc" \
    --include="$(sed 's/#.*//; /^[[:space:]]*$/d' "$HERE/packages.txt" | paste -sd,)" \
    --customize-hook="tar-in '$work/files.tar' /" \
    --customize-hook="upload '$work/ros-archive-keyring.gpg' /usr/share/keyrings/ros-archive-keyring.gpg" \
    --customize-hook="upload '$work/base-inputs' /usr/lib/mowbite/base-inputs" \
    --customize-hook='rm -f "$1"/etc/apt/sources.list "$1"/etc/apt/sources.list.d/*' \
    --customize-hook="printf 'Types: deb\nURIs: $PORTS\nSuites: noble noble-updates noble-security\nComponents: main universe\nSigned-By: $UBUNTU_KEY\n' > \"\$1\"/etc/apt/sources.list.d/ubuntu.sources" \
    --customize-hook="printf 'Types: deb\nURIs: $ROS\nSuites: noble\nComponents: main\nSigned-By: /usr/share/keyrings/ros-archive-keyring.gpg\n' > \"\$1\"/etc/apt/sources.list.d/ros2.sources" \
    --customize-hook="chroot \"\$1\" systemctl enable $UNITS" \
    --customize-hook="chroot \"\$1\" systemctl disable $OFF" \
    --customize-hook='rm -f "$1"/etc/ssh/ssh_host_*' \
    --customize-hook='mkdir -m 755 "$1"/userdata "$1"/meta' \
    --customize-hook='ln -sf /usr/share/zoneinfo/Europe/Berlin "$1"/etc/localtime && echo Europe/Berlin > "$1"/etc/timezone' \
    --customize-hook="download /var/lib/dpkg/status '$work/status'" \
    noble "$OUT" \
    "deb [signed-by=$UBUNTU_KEY] $PORTS noble main universe" \
    "deb [signed-by=$UBUNTU_KEY] $PORTS noble-updates main universe" \
    "deb [signed-by=$UBUNTU_KEY] $PORTS noble-security main universe" \
    "deb [signed-by=$work/ros-archive-keyring.gpg] $ROS noble main"
built=1
python3 -I "$HERE/base_packages.py" write "$work/status" "$(cat "$HERE/files/usr/lib/mowbite/base")" "$HERE/packages.txt" \
    > "$HERE/base-packages.txt"
echo "base rootfs $OUT: $(stat -c %s "$OUT") B; commit base-packages.txt with it"
