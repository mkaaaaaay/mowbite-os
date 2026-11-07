"""Tests of mowbite-update under a prefix, with stand-ins for systemctl, dpkg-query, ldd and mowbite_health.

  python3 -I platform/tron/bundle/test_update.py
"""
import importlib.machinery
import importlib.util
import io
import json
import os
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest

HERE = os.path.dirname(os.path.realpath(__file__))
TOOL = os.path.join(HERE, "..", "rootfs", "files", "usr", "sbin", "mowbite-update")

# systemctl: the state of the service in $PREFIX/state/service; a start with a crash record does nothing (condition).
SYSTEMCTL = """#!/bin/sh
s="$MOWBITE_UPDATE_PREFIX/state/service"
echo "$*" >> "$MOWBITE_UPDATE_PREFIX/state/systemctl.log"
case "$1" in
  is-active) state=$(cat "$s" 2>/dev/null || echo inactive); [ "$2" = --quiet ] || echo "$state"; [ "$state" = active ];;
  stop) echo inactive > "$s";;
  start) [ -e "$MOWBITE_UPDATE_PREFIX/run/mowbite/crash" ] || echo active > "$s";;
  reset-failed) ;;
esac
"""
# dpkg-query: every package is installed except those listed in $PREFIX/state/missing.
DPKG_QUERY = """#!/bin/sh
for p; do :; done
if grep -qx "$p" "$MOWBITE_UPDATE_PREFIX/state/missing" 2>/dev/null; then exit 1; fi
printf 'install ok installed'
"""
# ldd: a file named broken.so misses a library.
LDD = """#!/bin/sh
case "$1" in
  */broken.so) echo "	libgone.so.1 => not found"; exit 0;;
  *) echo "	libc.so.6 => /lib/libc.so.6";;
esac
"""
# The environment of the OS at $1 without ROS.
OS_SETUP = """MOWBITE_OS_REVISION=$(cat "$1/revision")
export MOWBITE_OS_REVISION
"""
# mowbite_health of a bundle: healthy with the line in $PREFIX/state/health-<revision> while the service runs.
HEALTH = """#!/bin/sh
[ "$(cat "$MOWBITE_UPDATE_PREFIX/state/service" 2>/dev/null)" = active ] || { echo "MQTT_UNREACHABLE"; exit 1; }
line=$(cat "$MOWBITE_UPDATE_PREFIX/state/health-$MOWBITE_OS_REVISION" 2>/dev/null || echo "OK IDLE")
echo "$line"
case "$line" in OK*) exit 0;; *) exit 1;; esac
"""


# The tool has no .py name; its bytecode would land in files/ of the rootfs.
def load_tool():
    sys.dont_write_bytecode = True
    loader = importlib.machinery.SourceFileLoader("mowbite_update", TOOL)
    spec = importlib.util.spec_from_loader("mowbite_update", loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


class UpdateTest(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp(prefix="mowbite-update-")
        self.prefix = os.path.join(self.root, "fs")
        bin_dir = os.path.join(self.root, "bin")
        for folder in ("state", "etc/mowbite", "usr/lib/mowbite", "run/mowbite", "opt/mowbite"):
            os.makedirs(os.path.join(self.prefix, folder))
        os.makedirs(bin_dir)
        for name, text in (("systemctl", SYSTEMCTL), ("dpkg-query", DPKG_QUERY), ("ldd", LDD)):
            self.write(os.path.join(bin_dir, name), text, 0o755)
        self.write(os.path.join(self.prefix, "usr/lib/mowbite/os-setup"), OS_SETUP)
        self.write(os.path.join(self.prefix, "usr/lib/mowbite/os.defaults"), "MOWBITE_HARDWARE=false\n")
        self.key = self.make_key("good")
        self.other_key = self.make_key("other")
        with open(self.key + ".pub") as f:
            public = " ".join(f.read().split()[:2])
        self.write(os.path.join(self.prefix, "etc/mowbite/allowed_signers"),
                   f'mowbite namespaces="mowbite-bundle" {public}\n')
        self.env = dict(os.environ)
        os.environ["MOWBITE_UPDATE_PREFIX"] = self.prefix
        os.environ["MOWBITE_UPDATE_HEALTH_WAIT"] = "0"
        os.environ["PATH"] = bin_dir + os.pathsep + os.environ["PATH"]
        self.tool = load_tool()
        self.tool.HEALTH_EVERY = 0

    def tearDown(self):
        os.environ.clear()
        os.environ.update(self.env)
        shutil.rmtree(self.root)

    def write(self, path, text, mode=0o644):
        with open(path, "w") as f:
            f.write(text)
        os.chmod(path, mode)

    def make_key(self, name):
        path = os.path.join(self.root, name)
        subprocess.run(["ssh-keygen", "-q", "-t", "ed25519", "-N", "", "-C", "", "-f", path], check=True)
        return path

    def state(self, name):
        return os.path.join(self.prefix, "state", name)

    # A signed bundle of revision rev; extra entries (name -> bytes) beside or instead of the usual ones.
    def bundle(self, rev, key=None, packages=("ros-jazzy-rclcpp",), extra=None, meta=None):
        path = os.path.join(self.root, f"mowbite-os-{rev}.tar.gz")
        files = {
            f"{rev}/revision": f"{rev}\n".encode(),
            f"{rev}/bundle.json": json.dumps(meta or {"format": 1, "revision": rev, "commit": rev, "min_app": "1.4.0",
                                                      "runtime_packages": list(packages)}).encode(),
            f"{rev}/install/mowbite_bridge/lib/mowbite_bridge/mowbite_health": HEALTH.encode(),
            f"{rev}/install/lib/libok.so": b"\x7fELF fake",
            f"{rev}/planner/local_setup.bash": b"",
        }
        files.update(extra or {})
        with tarfile.open(path, "w:gz") as tar:
            for name, data in files.items():
                info = tarfile.TarInfo(name)
                info.size = len(data)
                info.mode = 0o755
                tar.addfile(info, io.BytesIO(data))
        if os.path.exists(path + ".sig"):
            os.remove(path + ".sig")
        subprocess.run(["ssh-keygen", "-q", "-Y", "sign", "-f", key or self.key, "-n", "mowbite-bundle", path],
                       check=True, capture_output=True, stdin=subprocess.DEVNULL, timeout=30)
        return path

    def install(self, path, sig=None):
        return self.tool.main(["mowbite-update", "install", path] + ([sig] if sig else []))

    def link(self, name):
        try:
            return os.readlink(os.path.join(self.prefix, "opt/mowbite", name))
        except FileNotFoundError:
            return None

    def installed(self):
        return sorted(os.listdir(os.path.join(self.prefix, "opt/mowbite")))

    def service(self):
        with open(self.state("service")) as f:
            return f.read().strip()

    def test_first_install(self):
        self.assertEqual(self.install(self.bundle("aaa")), 0)
        self.assertEqual(self.link("current"), "aaa")
        self.assertIsNone(self.link("previous"))
        self.assertEqual(self.service(), "active")

    def test_second_install_keeps_previous(self):
        self.install(self.bundle("aaa"))
        self.assertEqual(self.install(self.bundle("bbb")), 0)
        self.assertEqual((self.link("current"), self.link("previous")), ("bbb", "aaa"))

    def test_only_two_kept(self):
        for rev in ("aaa", "bbb", "ccc"):
            self.assertEqual(self.install(self.bundle(rev)), 0)
        self.assertEqual(self.installed(), ["bbb", "ccc", "current", "previous"])

    def test_bad_signature(self):
        self.install(self.bundle("aaa"))
        self.assertEqual(self.install(self.bundle("bbb", key=self.other_key)), 1)
        self.assertEqual(self.link("current"), "aaa")
        self.assertEqual(self.installed(), ["aaa", "current"])

    def test_signature_missing(self):
        path = self.bundle("aaa")
        os.remove(path + ".sig")
        self.assertEqual(self.install(path), 1)
        self.assertIsNone(self.link("current"))

    def test_changed_after_signing(self):
        path = self.bundle("aaa")
        with open(path, "ab") as f:
            f.write(b"\0")
        self.assertEqual(self.install(path), 1)
        self.assertIsNone(self.link("current"))

    def test_missing_package(self):
        self.write(self.state("missing"), "ros-jazzy-rclcpp\n")
        self.assertEqual(self.install(self.bundle("aaa")), 1)
        self.assertEqual(self.installed(), [])

    def test_missing_library(self):
        self.install(self.bundle("aaa"))
        self.assertEqual(self.install(self.bundle("bbb", extra={"bbb/install/lib/broken.so": b"\x7fELF"})), 1)
        self.assertEqual(self.link("current"), "aaa")
        self.assertEqual(self.installed(), ["aaa", "current"])

    def test_unsafe_entry(self):
        self.assertEqual(self.install(self.bundle("aaa", extra={"aaa/../../etc/evil": b"x"})), 1)
        self.assertEqual(self.installed(), [])
        self.assertFalse(os.path.exists(os.path.join(self.prefix, "etc/evil")))

    def test_two_directories(self):
        self.assertEqual(self.install(self.bundle("aaa", extra={"zzz/x": b"x"})), 1)
        self.assertEqual(self.installed(), [])

    def test_revision_not_directory(self):
        meta = {"format": 1, "revision": "other", "runtime_packages": []}
        self.assertEqual(self.install(self.bundle("aaa", meta=meta)), 1)

    def test_not_idle(self):
        self.install(self.bundle("aaa"))
        self.write(self.state("health-aaa"), "OK MOWING")
        self.assertEqual(self.install(self.bundle("bbb")), 1)
        self.assertEqual(self.link("current"), "aaa")
        self.assertEqual(self.service(), "active")

    def test_stopped_service_switches(self):
        self.install(self.bundle("aaa"))
        self.write(self.state("health-aaa"), "OK MOWING")
        self.write(self.state("service"), "inactive")
        self.assertEqual(self.install(self.bundle("bbb")), 0)
        self.assertEqual(self.link("current"), "bbb")

    def test_crash_record(self):
        self.install(self.bundle("aaa"))
        self.write(os.path.join(self.prefix, "run/mowbite/crash"), "1 Kernel panic")
        self.assertEqual(self.install(self.bundle("bbb")), 1)
        self.assertEqual(self.link("current"), "aaa")

    def test_unhealthy_goes_back(self):
        self.install(self.bundle("aaa"))
        self.install(self.bundle("bbb"))
        self.write(self.state("health-ccc"), "NO_ROBOT_STATE")
        self.assertEqual(self.install(self.bundle("ccc")), 1)
        self.assertEqual((self.link("current"), self.link("previous")), ("bbb", "aaa"))
        self.assertEqual(self.service(), "active")
        self.assertEqual(self.installed(), ["aaa", "bbb", "current", "previous"])

    def test_first_install_unhealthy_stays_off(self):
        self.write(self.state("health-aaa"), "NO_ROBOT_STATE")
        self.assertEqual(self.install(self.bundle("aaa")), 1)
        self.assertIsNone(self.link("current"))
        self.assertEqual(self.service(), "inactive")
        self.assertEqual(self.installed(), [])

    def test_rollback(self):
        self.install(self.bundle("aaa"))
        self.install(self.bundle("bbb"))
        self.assertEqual(self.tool.main(["mowbite-update", "rollback"]), 0)
        self.assertEqual((self.link("current"), self.link("previous")), ("aaa", "bbb"))

    def test_rollback_without_previous(self):
        self.install(self.bundle("aaa"))
        self.assertEqual(self.tool.main(["mowbite-update", "rollback"]), 1)
        self.assertEqual(self.link("current"), "aaa")

    def test_same_revision_again(self):
        self.install(self.bundle("aaa"))
        self.install(self.bundle("bbb"))
        self.assertEqual(self.install(self.bundle("bbb")), 0)
        self.assertEqual((self.link("current"), self.link("previous")), ("bbb", "aaa"))

    def test_leftovers_of_a_power_cut(self):
        self.install(self.bundle("aaa"))
        opt = os.path.join(self.prefix, "opt/mowbite")
        os.makedirs(os.path.join(opt, ".bbb.partial/bbb/install"))
        os.symlink("bbb", os.path.join(opt, "current.new"))
        self.assertEqual(self.install(self.bundle("bbb")), 0)
        self.assertEqual((self.link("current"), self.link("previous")), ("bbb", "aaa"))
        self.assertEqual(self.installed(), ["aaa", "bbb", "current", "previous"])

    def test_stop_before_switch(self):
        self.install(self.bundle("aaa"))
        self.install(self.bundle("bbb"))
        with open(self.state("systemctl.log")) as f:
            calls = [line.split()[0] for line in f]
        # The second install: is-active of aaa, stop, then start of bbb.
        self.assertEqual(calls[-3:], ["stop", "reset-failed", "start"])


if __name__ == "__main__":
    unittest.main()
