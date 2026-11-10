"""Tests of mowbite-update under a prefix, with stand-ins for systemctl, dpkg-query and ldd and an OS that only writes
its session file.

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
import time
import unittest

HERE = os.path.dirname(os.path.realpath(__file__))
TOOL = os.path.join(HERE, "..", "rootfs", "files", "usr", "sbin", "mowbite-update")
HOST_LIB = os.path.join(HERE, "..", "..", "..", "host")
SIGN = os.path.join(HERE, "sign-bundle.sh")

# systemctl: the state of the service in $PREFIX/state/service. A start writes os-session.json of this boot as the
# logic does, IDLE on the contacts (or $PREFIX/state/os-state), unless state/broken-<revision of current> is there; a
# stop writes it clean.
SYSTEMCTL = """#!/bin/sh
p="$MOWBITE_UPDATE_PREFIX"
s="$p/state/service"
echo "$*" >> "$p/state/systemctl.log"
session() {
  boot=$(cat "$p/proc/sys/kernel/random/boot_id")
  state=$(cat "$p/state/os-state" 2>/dev/null || echo IDLE)
  mkdir -p "$p/userdata/mowbite"
  printf '{"state": "%s", "job_id": "", "boot_id": "%s", "since": 0, "clean": %s, "emergency": false, \
"charging": true}' "$state" "$boot" "$1" > "$p/userdata/mowbite/os-session.json"
}
case "$1" in
  is-active) state=$(cat "$s" 2>/dev/null || echo inactive); [ "$2" = --quiet ] || echo "$state"; [ "$state" = active ];;
  stop) [ "$(cat "$s" 2>/dev/null)" = active ] && session true; echo inactive > "$s";;
  start) echo active > "$s"; rev=$(readlink "$p/opt/mowbite/current"); [ -e "$p/state/broken-$rev" ] || session false;;
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
        self.write(os.path.join(self.prefix, "usr/lib/mowbite/base"), "1\n")
        self.write(os.path.join(self.prefix, "proc/sys/kernel/random/boot_id"), "b1\n")
        self.key = self.make_key("good")
        self.other_key = self.make_key("other")
        with open(self.key + ".pub") as f:
            public = " ".join(f.read().split()[:2])
        self.write(os.path.join(self.prefix, "etc/mowbite/allowed_signers"),
                   f'mowbite namespaces="mowbite-os" {public}\n')
        self.env = dict(os.environ)
        os.environ["MOWBITE_UPDATE_PREFIX"] = self.prefix
        os.environ["MOWBITE_UPDATE_HEALTH_WAIT"] = "0"
        os.environ["MOWBITE_UPDATE_BROKER_WAIT"] = "0"
        os.environ["MOWBITE_HOST_LIB"] = HOST_LIB
        os.environ["PATH"] = bin_dir + os.pathsep + os.environ["PATH"]
        self.serial = 1000
        self.tool = self.load()

    # The tool with a health check that does not wait and an OS that grants the hold unless state/no-hold is there.
    def load(self):
        tool = load_tool()
        make = tool.make
        hold = os.path.join(self.prefix, "userdata/mowbite/hold.json")

        def quick(prefix, with_broker):
            updater = make(prefix, with_broker)
            updater.installer.health.stable = 0
            updater.installer.health.every = 0
            return updater

        def request_hold(_, seconds):
            if not os.path.exists(self.state("no-hold")):
                now = int(time.time())
                self.write(hold, json.dumps({"until": now + seconds, "set_at": now}))

        tool.make = quick
        tool.updater.HOLD_WAIT = 0.2
        tool.NoBroker.request_hold = request_hold
        tool.NoBroker.release_hold = lambda _: os.path.exists(hold) and os.remove(hold)
        return tool

    def tearDown(self):
        os.environ.clear()
        os.environ.update(self.env)
        shutil.rmtree(self.root)

    def write(self, path, text, mode=0o644):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write(text)
        os.chmod(path, mode)

    def make_key(self, name):
        path = os.path.join(self.root, name)
        subprocess.run(["ssh-keygen", "-q", "-t", "ed25519", "-N", "", "-C", "", "-f", path], check=True)
        return path

    def state(self, name):
        return os.path.join(self.prefix, "state", name)

    # os-session.json as the logic writes it, of boot b1 unless given.
    def session(self, **fields):
        record = {"state": "IDLE", "job_id": "", "boot_id": "b1", "since": 0, "clean": False, "emergency": False,
                  "charging": True, **fields}
        self.write(os.path.join(self.prefix, "userdata/mowbite/os-session.json"), json.dumps(record))

    # A signed bundle of revision rev, each one with a newer serial unless given; extra entries (name -> bytes)
    # beside or instead of the usual ones.
    def bundle(self, rev, key=None, packages=("ros-jazzy-rclcpp",), extra=None, meta=None, serial=None,
               namespace="mowbite-os"):
        self.serial += 1
        path = os.path.join(self.root, f"mowbite-os-{rev}.tar.gz")
        files = {
            f"{rev}/revision": f"{rev}\n".encode(),
            f"{rev}/bundle.json": json.dumps(meta or {
                "format": 2, "component": "os", "revision": rev, "commit": rev, "serial": serial or self.serial,
                "base": 1, "min_app": "1.4.0", "runtime_packages": list(packages)}).encode(),
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
        subprocess.run(["ssh-keygen", "-q", "-Y", "sign", "-f", key or self.key, "-n", namespace, path],
                       check=True, capture_output=True, stdin=subprocess.DEVNULL, timeout=30)
        return path

    def install(self, path, sig=None):
        return self.tool.main(["mowbite-update", "install", path] + ([sig] if sig else []))

    def last(self):
        with open(os.path.join(self.prefix, "var/lib/mowbite-host/update.json")) as f:
            return json.load(f)["last"]

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
        self.assertEqual(self.install(self.bundle(A)), 0)
        self.assertEqual(self.link("current"), A)
        self.assertIsNone(self.link("previous"))
        self.assertEqual(self.service(), "active")
        self.assertEqual(self.last()["result"], "installed")

    def test_second_install_keeps_previous(self):
        self.install(self.bundle(A))
        self.assertEqual(self.install(self.bundle(B)), 0)
        self.assertEqual((self.link("current"), self.link("previous")), (B, A))
        # The hold of the update is given back.
        self.assertFalse(os.path.exists(os.path.join(self.prefix, "userdata/mowbite/hold.json")))

    def test_only_two_kept(self):
        for rev in (A, B, C):
            self.assertEqual(self.install(self.bundle(rev)), 0)
        self.assertEqual(self.installed(), [B, C, "current", "previous"])

    def test_bad_signature(self):
        self.install(self.bundle(A))
        self.assertEqual(self.install(self.bundle(B, key=self.other_key)), 1)
        self.assertEqual(self.link("current"), A)
        self.assertEqual(self.installed(), [A, "current"])

    def test_signature_of_another_component(self):
        self.assertEqual(self.install(self.bundle(A, namespace="mowbite-planner")), 1)
        self.assertIsNone(self.link("current"))

    def test_signature_missing(self):
        path = self.bundle(A)
        os.remove(path + ".sig")
        self.assertEqual(self.install(path), 1)
        self.assertIsNone(self.link("current"))

    def test_changed_after_signing(self):
        path = self.bundle(A)
        with open(path, "ab") as f:
            f.write(b"\0")
        self.assertEqual(self.install(path), 1)
        self.assertIsNone(self.link("current"))

    def test_link_given_as_bundle(self):
        path = self.bundle(A)
        os.symlink(path, path + ".link.tar.gz")
        os.symlink(path + ".sig", path + ".link.tar.gz.sig")
        self.assertEqual(self.install(path + ".link.tar.gz"), 1)
        self.assertIsNone(self.link("current"))

    def test_old_serial(self):
        self.install(self.bundle(A, serial=2000))
        self.assertEqual(self.install(self.bundle(B, serial=1999)), 1)
        self.assertEqual(self.last()["reason"], "OLD_SERIAL")
        self.assertEqual(self.link("current"), A)

    def test_missing_package(self):
        self.write(self.state("missing"), "ros-jazzy-rclcpp\n")
        self.assertEqual(self.install(self.bundle(A)), 1)
        self.assertEqual(self.last()["reason"], "BASE_TOO_OLD")
        self.assertEqual(self.installed(), [])

    def test_base_too_old(self):
        meta = {"format": 2, "component": "os", "revision": A, "serial": 5, "base": 2, "runtime_packages": []}
        self.assertEqual(self.install(self.bundle(A, meta=meta)), 1)
        self.assertEqual(self.last()["reason"], "BASE_TOO_OLD")

    def test_missing_library(self):
        self.install(self.bundle(A))
        self.assertEqual(self.install(self.bundle(B, extra={f"{B}/install/lib/broken.so": b"\x7fELF"})), 1)
        self.assertEqual(self.last()["reason"], "LIBRARY_MISSING")
        self.assertEqual(self.link("current"), A)
        self.assertEqual(self.installed(), [A, "current"])

    def test_unsafe_entry(self):
        self.assertEqual(self.install(self.bundle(A, extra={f"{A}/../../etc/evil": b"x"})), 1)
        self.assertEqual(self.installed(), [])
        self.assertFalse(os.path.exists(os.path.join(self.prefix, "etc/evil")))

    def test_two_directories(self):
        self.assertEqual(self.install(self.bundle(A, extra={"zzz/x": b"x"})), 1)
        self.assertEqual(self.installed(), [])

    def test_revision_not_directory(self):
        meta = {"format": 2, "component": "os", "revision": "0123456789", "serial": 5, "runtime_packages": []}
        self.assertEqual(self.install(self.bundle(A, meta=meta)), 1)

    def test_format_1_refused(self):
        meta = {"format": 1, "revision": A, "runtime_packages": []}
        self.assertEqual(self.install(self.bundle(A, meta=meta)), 1)

    def test_not_idle(self):
        self.install(self.bundle(A))
        self.session(state="MOWING")
        self.assertEqual(self.install(self.bundle(B)), 1)
        self.assertEqual(self.last()["reason"], "BUSY")
        self.assertEqual(self.link("current"), A)
        self.assertEqual(self.service(), "active")

    def test_emergency(self):
        self.install(self.bundle(A))
        self.session(emergency=True)
        self.assertEqual(self.install(self.bundle(B)), 1)
        self.assertEqual(self.last()["reason"], "EMERGENCY")

    def test_not_charging(self):
        self.install(self.bundle(A))
        self.session(charging=False)
        self.assertEqual(self.install(self.bundle(B)), 1)
        self.assertEqual(self.last()["reason"], "NO_DOCK_CONTACT")

    def test_hold_not_granted(self):
        self.install(self.bundle(A))
        self.write(self.state("no-hold"), "")
        self.assertEqual(self.install(self.bundle(B)), 1)
        self.assertEqual(self.last()["reason"], "NO_HOLD")
        self.assertEqual(self.link("current"), A)

    def test_stopped_service_switches(self):
        self.install(self.bundle(A))
        self.session(state="MOWING")
        self.write(self.state("service"), "inactive")
        self.assertEqual(self.install(self.bundle(B)), 0)
        self.assertEqual(self.link("current"), B)

    def test_crash_record(self):
        self.install(self.bundle(A))
        self.write(os.path.join(self.prefix, "run/mowbite/crash"), "1 Kernel panic")
        self.write(os.path.join(self.prefix, "proc/sys/kernel/random/boot_id"), "b2\n")
        # Not judged by the OS of this boot yet: no session, or one of the boot before.
        os.remove(os.path.join(self.prefix, "userdata/mowbite/os-session.json"))
        self.assertEqual(self.install(self.bundle(B)), 1)
        self.session(boot_id="b1")
        self.assertEqual(self.install(self.bundle(B)), 1)
        self.assertEqual(self.last()["reason"], "KERNEL_CRASH")
        self.assertEqual(self.link("current"), A)
        # The OS started in this boot and judged it.
        self.session(boot_id="b2")
        self.assertEqual(self.install(self.bundle(B)), 0)
        self.assertEqual(self.link("current"), B)

    def test_restart_hold(self):
        self.install(self.bundle(A))
        hold = os.path.join(self.prefix, "userdata/mowbite/restart-hold.json")
        self.write(hold, "{}")
        self.assertEqual(self.install(self.bundle(B)), 1)
        self.assertEqual(self.last()["reason"], "EMERGENCY")
        self.assertEqual(self.link("current"), A)
        os.remove(hold)
        self.assertEqual(self.install(self.bundle(B)), 0)

    def test_unhealthy_goes_back(self):
        self.install(self.bundle(A))
        self.install(self.bundle(B))
        self.write(self.state(f"broken-{C}"), "")
        self.assertEqual(self.install(self.bundle(C)), 1)
        self.assertEqual(self.last()["result"], "rolled_back")
        self.assertEqual((self.link("current"), self.link("previous")), (B, A))
        self.assertEqual(self.service(), "active")
        self.assertEqual(self.installed(), [A, B, "current", "previous"])

    def test_halt_is_unhealthy(self):
        self.install(self.bundle(A))
        halt = os.path.join(self.prefix, "run/mowbite-os/halt")
        real_start = self.tool.carrier.Systemd.start

        def start_halted(service):
            ok = real_start(service)
            if os.readlink(os.path.join(self.prefix, "opt/mowbite/current")) == B:
                self.write(halt, "MOWBITE_OS_HALT DATA_NO_SPACE schema 3: no room")
            return ok
        self.tool.carrier.Systemd.start = start_halted
        try:
            self.assertEqual(self.install(self.bundle(B)), 1)
        finally:
            self.tool.carrier.Systemd.start = real_start
        self.assertIn("halted", self.last()["message"])
        self.assertEqual(self.link("current"), A)

    def test_first_install_unhealthy_stays_off(self):
        self.write(self.state(f"broken-{A}"), "")
        self.assertEqual(self.install(self.bundle(A)), 1)
        self.assertEqual(self.last()["result"], "failed")
        self.assertIsNone(self.link("current"))
        self.assertEqual(self.service(), "inactive")
        self.assertEqual(self.installed(), [])

    def test_rollback(self):
        self.install(self.bundle(A))
        self.install(self.bundle(B))
        self.assertEqual(self.tool.main(["mowbite-update", "rollback"]), 0)
        self.assertEqual((self.link("current"), self.link("previous")), (A, B))

    def test_rollback_without_previous(self):
        self.install(self.bundle(A))
        self.assertEqual(self.tool.main(["mowbite-update", "rollback"]), 1)
        self.assertEqual(self.link("current"), A)

    def test_same_revision_again(self):
        self.install(self.bundle(A))
        path = self.bundle(B)
        self.install(path)
        self.assertEqual(self.install(path), 1)
        self.assertEqual(self.last()["reason"], "OLD_SERIAL")
        self.assertEqual((self.link("current"), self.link("previous")), (B, A))

    def test_leftovers_of_a_power_cut(self):
        self.install(self.bundle(A))
        opt = os.path.join(self.prefix, "opt/mowbite")
        os.makedirs(os.path.join(opt, f".{B}.partial/{B}/install"))
        os.symlink(B, os.path.join(opt, "current.new"))
        self.assertEqual(self.install(self.bundle(B)), 0)
        self.assertEqual((self.link("current"), self.link("previous")), (B, A))
        self.assertEqual(self.installed(), [A, B, "current", "previous"])

    def test_stop_before_switch(self):
        self.install(self.bundle(A))
        self.install(self.bundle(B))
        with open(self.state("systemctl.log")) as f:
            calls = [line.split()[0] for line in f if line.split()[0] != "is-active"]
        self.assertEqual(calls[-3:], ["stop", "reset-failed", "start"])

    def test_no_pending_after_switches(self):
        self.install(self.bundle(A))
        self.write(self.state(f"broken-{B}"), "")
        self.install(self.bundle(B))
        self.assertNotIn("pending", self.installed())

    # Killed (or power cut) while it waits for the health of the new installation.
    def killed_during_health(self, rev, path=None):
        def killed(health, since):
            raise SystemExit("killed")
        real = self.tool.installer.Health.wait_healthy
        self.tool.installer.Health.wait_healthy = killed
        try:
            self.install(path or self.bundle(rev))
        finally:
            self.tool.installer.Health.wait_healthy = real
        self.tool = self.load()

    def test_killed_during_health_recovers(self):
        self.install(self.bundle(A))
        self.killed_during_health(B)
        self.assertEqual(self.link("current"), B)
        self.assertIn("pending", self.installed())
        self.assertEqual(self.install(self.bundle(C)), 1)
        self.assertEqual(self.last()["reason"], "UNFINISHED")
        self.assertEqual(self.tool.main(["mowbite-update", "recover"]), 0)
        self.assertEqual(self.link("current"), A)
        self.assertNotIn("pending", self.installed())
        self.assertEqual(self.service(), "active")
        self.assertEqual(self.last()["reason"], "INTERRUPTED")
        self.assertEqual(self.install(self.bundle(C)), 0)
        self.assertEqual(self.installed(), [A, C, "current", "previous"])

    def test_cut_off_twice_blocks(self):
        self.install(self.bundle(A))
        path = self.bundle(B)
        for _ in range(2):
            self.killed_during_health(B, path)
            self.tool.main(["mowbite-update", "recover"])
        with open(os.path.join(self.prefix, "var/lib/mowbite-host/update.json")) as f:
            blocked = json.load(f)["components"]["os"]["blocked"]
        self.assertEqual(blocked[B]["reason"], "INTERRUPTED")
        self.assertEqual(blocked[B]["result"], "failed")

    def test_recover_at_boot_keeps_service_off(self):
        self.install(self.bundle(A))
        self.killed_during_health(B)
        self.write(self.state("service"), "inactive")
        self.assertEqual(self.tool.main(["mowbite-update", "recover"]), 0)
        self.assertEqual(self.link("current"), A)
        self.assertEqual(self.service(), "inactive")

    def test_recover_first_install_turns_off(self):
        self.killed_during_health(A)
        self.assertEqual(self.tool.main(["mowbite-update", "recover"]), 0)
        self.assertIsNone(self.link("current"))
        self.assertEqual(self.service(), "inactive")

    def test_prune_keeps_pending(self):
        self.install(self.bundle(A))
        self.install(self.bundle(B))
        self.killed_during_health(C)
        self.tool.make(self.prefix, False).installer.prune()
        self.assertEqual(self.installed(), [A, B, C, "current", "pending", "previous"])

    def test_recover_without_pending(self):
        self.install(self.bundle(A))
        self.assertEqual(self.tool.main(["mowbite-update", "recover"]), 0)
        self.assertEqual(self.link("current"), A)

    def test_locked_by_the_host(self):
        import fcntl
        os.makedirs(os.path.join(self.prefix, "run"), exist_ok=True)
        with open(os.path.join(self.prefix, "run/mowbite-host.lock"), "w") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            self.assertEqual(self.install(self.bundle(A)), 1)
        self.assertIsNone(self.link("current"))

    # sign-bundle.sh with its state, the signers of this mower and a gate folder in the test folder.
    def sign(self, path, answer="y"):
        signers = os.path.join(self.prefix, "etc/mowbite/allowed_signers")
        env = {**self.env, "MOWBITE_SIGN_STATE": os.path.join(self.root, "sign"), "MOWBITE_SIGNERS": signers,
               "MOWBITE_GATE": os.path.join(self.root, "gate")}
        return subprocess.run([SIGN, path, self.key], input=answer + "\n", env=env, capture_output=True, text=True,
                              timeout=30)

    # It shows the bundle and the gate result, signs only on y and refuses a serial not newer than the last.
    def test_sign_newer_serials_only(self):
        os.makedirs(os.path.join(self.root, "gate"))
        self.write(os.path.join(self.root, "gate", "bbbbbbb.json"),
                   json.dumps({"ok": False, "updated": "now", "steps": {"chk:update": {"ok": False}}}))
        path = self.bundle(B, serial=2000)
        os.remove(path + ".sig")
        self.assertEqual(self.sign(path, "n").returncode, 1)
        self.assertFalse(os.path.exists(path + ".sig"))
        signed = self.sign(path)
        self.assertEqual(signed.returncode, 0, signed.stderr)
        self.assertIn("test gate: NOT OK", signed.stdout)
        self.assertIn("FAIL chk:update", signed.stdout)
        self.assertEqual(self.install(path), 0)
        # The same serial again, and an older one, are refused before any question.
        older = self.bundle(A, serial=1999)
        for again in (path, older):
            refused = self.sign(again)
            self.assertEqual(refused.returncode, 1)
            self.assertIn("is not newer than 2000", refused.stderr)
        self.assertIn("test gate: NO RESULT", self.sign(self.bundle(C, serial=2001), "n").stdout)


A, B, C = "aaaaaaa", "bbbbbbb", "ccccccc"

if __name__ == "__main__":
    unittest.main()
