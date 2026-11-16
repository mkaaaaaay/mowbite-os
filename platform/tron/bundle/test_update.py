"""Tests of mowbite-update under a prefix, with stand-ins for systemctl, dpkg-query and ldd and an OS that only writes
its session file.

  python3 -I platform/tron/bundle/test_update.py
"""
import contextlib
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
import zipfile

HERE = os.path.dirname(os.path.realpath(__file__))
TOOL = os.path.join(HERE, "..", "rootfs", "files", "usr", "sbin", "mowbite-update")
HOST_LIB = os.path.join(HERE, "..", "..", "..", "host")
SIGN = os.path.join(HERE, "sign-bundle.sh")
MAKE_APP = os.path.join(HERE, "make-app-bundle.sh")
PUBLISH = os.path.join(HERE, "publish.py")
sys.path.insert(0, os.path.join(HOST_LIB, "bundle"))
import app_export  # noqa: E402
import planner_meta  # noqa: E402

# systemctl: the state of the service in $PREFIX/state/service. A start writes os-session.json of this boot as the
# logic does, IDLE on the contacts (or $PREFIX/state/os-state), unless state/broken-<revision> of the OS or of the
# planner active names is there; a stop writes it clean.
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
  start) echo active > "$s"; date +%s > "$s.since"
    rev=$(readlink "$p/opt/mowbite/current"); planner=$(readlink "$p/opt/mowbite-planner/active" || echo none)
    [ -e "$p/state/broken-$rev" ] || [ -e "$p/state/broken-$planner" ] || session false;;
  show) echo "ActiveState=$(cat "$s" 2>/dev/null || echo inactive)"
    echo "ActiveEnterTimestamp=@$(cat "$s.since" 2>/dev/null || echo 0)";;
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


# A small web export of the app as a zip, as the release of the app has it.
def probe(zip_path, version="1.5.0", api_min=15, api_max=15, serial=1700000000, api_break=None):
    span = {"min": api_min, **({} if api_max is None else {"max": api_max}),
            **({} if api_break is None else {"break": api_break})}
    app = {"schema": 1, "owner": app_export.OWNER, "version": version, "api": span,
           "commit": "%040x" % serial, "serial": serial}
    page = "<!doctype html><title>MowBite probe</title>" + "<p>probe export</p>" * 40
    entries = {"index.html": page, "404.html": "<!doctype html><title>not found</title>",
               "guide/index.html": page, "version.json": json.dumps({"version": version, "level": 1}),
               "app.json": json.dumps(app), "_next/static/chunks/main.js": "console.log('probe');" * 60}
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as archive:
        for name, text in entries.items():
            archive.writestr(name, text)


# The tool has no .py name; its bytecode would land in files/ of the rootfs.
# The command line as the rootfs starts it, fresh for each test.
def load_tool():
    sys.dont_write_bytecode = True
    loader = importlib.machinery.SourceFileLoader("mowbite_update", TOOL)
    spec = importlib.util.spec_from_loader("mowbite_update", loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return importlib.reload(module.cli)


class Base(unittest.TestCase):
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
                   f'mowbite namespaces="mowbite-os,mowbite-planner,mowbite-app" {public}\n')
        self.env = dict(os.environ)
        os.environ["MOWBITE_UPDATE_PREFIX"] = self.prefix
        os.environ["MOWBITE_UPDATE_HEALTH_WAIT"] = "0"
        os.environ["MOWBITE_UPDATE_BROKER_WAIT"] = "0"
        os.environ["MOWBITE_HOST_LIB"] = HOST_LIB
        os.environ["MOWBITE_BOOT_LIB"] = os.path.join(HOST_LIB, "boot")
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
    # beside or instead of the usual ones. Of the planner or the app: only what those bring.
    def bundle(self, rev, key=None, packages=("ros-jazzy-rclcpp",), extra=None, meta=None, serial=None,
               namespace=None, component="os"):
        self.serial += 1
        path = os.path.join(self.root, f"mowbite-{component}-{rev}.tar.gz")
        common = {"format": 2, "component": component, "revision": rev, "commit": rev,
                  "serial": serial or self.serial}
        files = {f"{rev}/revision": f"{rev}\n".encode()}
        if component == "os":
            common.update(base=1, min_app="1.4.0", runtime_packages=list(packages), api=15, planner_interface=1,
                          planner_files_version=1)
            files.update({f"{rev}/install/lib/libok.so": b"\x7fELF fake", f"{rev}/planner/local_setup.bash": b""})
        elif component == "planner":
            common.update(version="2.0.0", interface=1, files_version=1)
            files[f"{rev}/planner/local_setup.bash"] = b""
        else:
            common.update(version="1.5.0", api={"min": 14, "max": 15})
            files.update({f"{rev}/app/index.html": b"<html></html>",
                          f"{rev}/app/version.json": b'{"version": "1.5.0", "level": 1}'})
        files[f"{rev}/bundle.json"] = json.dumps(meta or common).encode()
        files.update(extra or {})
        with tarfile.open(path, "w:gz") as tar:
            for name, data in files.items():
                info = tarfile.TarInfo(name)
                info.size = len(data)
                info.mode = 0o755 if component == "os" else 0o644
                tar.addfile(info, io.BytesIO(data))
        if os.path.exists(path + ".sig"):
            os.remove(path + ".sig")
        subprocess.run(["ssh-keygen", "-q", "-Y", "sign", "-f", key or self.key, "-n",
                        namespace or f"mowbite-{component}", path],
                       check=True, capture_output=True, stdin=subprocess.DEVNULL, timeout=30)
        return path

    def install(self, path, sig=None):
        return self.tool.main(["mowbite-update", "install", path] + ([sig] if sig else []))

    def last(self):
        with open(os.path.join(self.prefix, "var/lib/mowbite-host/update.json")) as f:
            return json.load(f)["last"]

    def link(self, name, opt="opt/mowbite"):
        try:
            return os.readlink(os.path.join(self.prefix, opt, name))
        except FileNotFoundError:
            return None

    def installed(self, opt="opt/mowbite"):
        return sorted(os.listdir(os.path.join(self.prefix, opt)))

    # What systemctl was asked for since mark (a count of calls), without the questions.
    def calls(self, mark=0):
        with open(self.state("systemctl.log")) as f:
            return [line.split()[0] for line in f if line.split()[0] not in ("is-active", "show")][mark:]

    def service(self):
        with open(self.state("service")) as f:
            return f.read().strip()


class UpdateTest(Base):
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

    # The component is the one the signature holds for; nothing of an unchecked bundle says which.
    def test_signed_as_app_is_never_installed_as_os(self):
        self.install(self.bundle(A))
        self.assertEqual(self.install(self.bundle(B, namespace="mowbite-app")), 1)
        self.assertEqual((self.link("current"), self.installed("opt/mowbite-app")), (A, []))

    def test_app_installs_beside_a_mowing_os(self):
        self.install(self.bundle(A))
        self.session(state="MOWING", emergency=True)
        mark = len(self.calls())
        self.assertEqual(self.install(self.bundle(D, component="app")), 0)
        self.assertEqual((self.link("current", APP), self.link("active", APP)), (D, D))
        self.assertEqual((self.calls(mark), self.service()), ([], "active"))
        self.assertEqual(self.last()["component"], "app")
        self.assertFalse(os.path.exists(os.path.join(self.prefix, "userdata/mowbite/hold.json")))
        # Back without an own one before it: the app of the OS.
        self.assertEqual(self.tool.main(["mowbite-update", "rollback", "app"]), 0)
        self.assertEqual((self.link("current", APP), self.link("active", APP), self.link("previous", APP)),
                         (None, None, D))
        self.assertEqual(self.calls(mark), [])

    # make-app-bundle.sh of the probe export, signed for the app: installed beside the OS, readable for mowbite-web.
    def test_app_bundle_of_the_probe_export(self):
        self.install(self.bundle(A))
        out = os.path.join(self.root, "out")
        os.makedirs(out)
        zip_path = os.path.join(self.root, "MowBite-web.zip")
        probe(zip_path, version="1.5.0", api_min=14, api_max=15)
        made = subprocess.run([MAKE_APP, zip_path, out], capture_output=True, text=True, timeout=60)
        self.assertEqual(made.returncode, 0, made.stderr)
        path = made.stdout.splitlines()[-1].split()[0]
        revision = os.path.basename(path)[len("mowbite-app-"):-len(".tar.gz")]
        with tarfile.open(path) as tar:
            meta = json.load(tar.extractfile(f"{revision}/bundle.json"))
            self.assertTrue(all(m.uid == 0 and m.mode & 0o004 for m in tar.getmembers()))
        self.assertEqual({k: meta[k] for k in ("component", "revision", "version", "api", "serial")},
                         {"component": "app", "revision": revision, "version": "1.5.0", "api": {"min": 14, "max": 15},
                          "serial": 1700000000})
        self.assertEqual(meta["commit"][:12], revision)
        subprocess.run(["ssh-keygen", "-q", "-Y", "sign", "-f", self.key, "-n", "mowbite-app", path], check=True,
                       capture_output=True, stdin=subprocess.DEVNULL, timeout=30)
        self.assertEqual(self.install(path), 0)
        self.assertEqual(self.link("active", APP), revision)
        app = os.path.join(self.prefix, APP, revision, "app")
        self.assertTrue(os.path.isfile(os.path.join(app, "_next/static/chunks/main.js.gz")))

    def test_app_needs_an_os_it_goes_with(self):
        self.assertEqual(self.install(self.bundle(D, component="app")), 1)
        self.assertEqual(self.last()["reason"], "OS_TOO_OLD")
        self.assertEqual(self.installed(APP), [])

    def test_planner_restarts_the_os(self):
        self.install(self.bundle(A))
        mark = len(self.calls())
        self.assertEqual(self.install(self.bundle(D, component="planner")), 0)
        self.assertEqual((self.link("current", PLANNER), self.link("active", PLANNER)), (D, D))
        self.assertEqual(self.calls(mark), ["stop", "reset-failed", "start"])
        self.assertEqual(self.link("current"), A)
        self.session(state="MOWING")
        self.assertEqual(self.tool.main(["mowbite-update", "rollback", "planner"]), 1)
        self.assertEqual(self.last()["reason"], "BUSY")

    def test_planner_waits_for_the_job_unless_told(self):
        self.install(self.bundle(A))
        self.write(os.path.join(self.prefix, "userdata/mowbite/checkpoint.json"),
                   json.dumps({"version": 1, "area": 0, "path": 1, "index": 12}))
        path = self.bundle(D, component="planner")
        self.assertEqual(self.install(path), 1)
        self.assertEqual(self.last()["reason"], "JOB_OPEN")
        self.assertEqual(self.tool.main(["mowbite-update", "install", "--reset-jobs", path]), 0)
        self.assertEqual(self.link("active", PLANNER), D)

    def test_unhealthy_planner_goes_back_to_the_one_of_the_os(self):
        self.install(self.bundle(A))
        self.write(self.state(f"broken-{D}"), "")
        self.assertEqual(self.install(self.bundle(D, component="planner")), 1)
        self.assertEqual(self.last()["result"], "rolled_back")
        self.assertEqual((self.link("current", PLANNER), self.link("active", PLANNER), self.service()),
                         (None, None, "active"))

    def test_recover_of_every_place(self):
        self.install(self.bundle(A))
        self.install(self.bundle(D, component="planner"))
        self.killed_during_health(E, self.bundle(E, component="planner"))
        self.assertEqual(self.link("current", PLANNER), E)
        self.assertIn("pending", self.installed(PLANNER))
        # Nothing switches meanwhile, not even the app.
        self.assertEqual(self.install(self.bundle(D, component="app")), 1)
        self.assertEqual(self.last()["reason"], "UNFINISHED")
        self.assertEqual(self.tool.main(["mowbite-update", "recover"]), 0)
        self.assertEqual((self.link("current", PLANNER), self.link("active", PLANNER)), (D, D))
        self.assertNotIn("pending", self.installed(PLANNER))
        self.assertEqual((self.last()["reason"], self.last()["component"], self.service()),
                         ("INTERRUPTED", "planner", "active"))

    def test_status_of_every_place(self):
        self.install(self.bundle(A))
        self.install(self.bundle(D, component="app"))
        out = subprocess.run([sys.executable, "-I", TOOL, "status"], capture_output=True, text=True, timeout=30)
        self.assertEqual(out.returncode, 0, out.stderr)
        for line in (f"os current: {A}", "service: active", "planner current: -", "planner in use: the one of the OS",
                     f"app current: {D}", "app in use: current"):
            self.assertIn(line, out.stdout.splitlines())

    def unfinished(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = self.tool.main(["mowbite-update", "unfinished"])
        return code, out.getvalue().splitlines()

    # The contract with the tools of the rootfs (mowbite-measure): 0 only while no switch of any component is open.
    def test_unfinished_names_every_open_switch(self):
        self.assertEqual(self.unfinished(), (0, []))
        self.install(self.bundle(A))
        self.assertEqual(self.unfinished(), (0, []))
        for component, opt in (("os", "opt/mowbite"), ("planner", PLANNER), ("app", APP), ("host", HOST)):
            pending = os.path.join(self.prefix, opt, "pending")
            for record in (json.dumps({"to": B, "from": A}), "cut off"):
                self.write(pending, record)
                code, lines = self.unfinished()
                self.assertEqual((code, len(lines)), (1, 1), (component, lines))
                self.assertTrue(lines[0].startswith(f"{component} unfinished update: "), lines)
            os.remove(pending)
        self.assertEqual(self.unfinished(), (0, []))

    def test_unfinished_after_a_cut_switch(self):
        self.install(self.bundle(A))
        self.killed_during_health(B)
        self.assertEqual(self.unfinished()[0], 1)
        self.assertEqual(self.tool.main(["mowbite-update", "recover"]), 0)
        self.assertEqual(self.unfinished(), (0, []))

    def test_usage(self):
        for args in ([], ["rollback", "camera"], ["status", "--reset-jobs"], ["install"], ["recover", "os"],
                     ["unfinished", "os"], ["unfinished", "--reset-jobs"]):
            self.assertEqual(self.tool.main(["mowbite-update", *args]), 2, args)

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

    # An OS bundle of serial 5 for platforms, without platforms when None, on a mower of platform (None: not set).
    def install_on(self, rev, platforms, platform):
        self.write(os.path.join(self.prefix, "usr/lib/mowbite/os.defaults"),
                   "MOWBITE_HARDWARE=false\n" + (f"MOWBITE_PLATFORM={platform}\n" if platform else ""))
        meta = {"format": 2, "component": "os", "revision": rev, "serial": 5, "base": 1, "runtime_packages": [],
                **({} if platforms is None else {"platforms": platforms})}
        return self.install(self.bundle(rev, meta=meta))

    def test_other_platform(self):
        self.assertEqual(self.install_on(A, ["xcore"], "tron"), 1)
        self.assertEqual(self.last()["reason"], self.tool.bundle.PLATFORM_MISMATCH)
        self.assertEqual(self.installed(), [])
        with open(os.path.join(self.prefix, "var/lib/mowbite-host/update.json")) as f:
            self.assertEqual(json.load(f)["components"]["os"]["blocked"][A]["reason"],
                             self.tool.bundle.PLATFORM_MISMATCH)

    def test_own_platform(self):
        self.assertEqual(self.install_on(A, ["xcore", "tron"], "tron"), 0)
        self.assertEqual(self.link("current"), A)

    def test_bundle_without_platforms(self):
        self.assertEqual(self.install_on(A, None, "tron"), 0)
        self.assertEqual(self.link("current"), A)

    def test_mower_without_platform(self):
        self.assertEqual(self.install_on(A, ["xcore"], None), 0)
        self.assertEqual(self.link("current"), A)

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
            calls = [line.split()[0] for line in f if line.split()[0] not in ("is-active", "show")]
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
    def sign(self, path, answer="y", yes=False):
        signers = os.path.join(self.prefix, "etc/mowbite/allowed_signers")
        env = {**self.env, "MOWBITE_SIGN_STATE": os.path.join(self.root, "sign"), "MOWBITE_SIGNERS": signers,
               "MOWBITE_GATE": os.path.join(self.root, "gate")}
        return subprocess.run([SIGN, *(["--yes"] if yes else []), path, self.key], input=answer + "\n", env=env,
                              capture_output=True, text=True, timeout=30)

    # --yes asks nothing and signs only with an ok gate result of the commit.
    def test_sign_without_asking_needs_the_gate(self):
        os.makedirs(os.path.join(self.root, "gate"))
        path = self.bundle(B, serial=2000)
        os.remove(path + ".sig")
        self.assertEqual(self.sign(path, "y", yes=True).returncode, 1)
        self.write(os.path.join(self.root, "gate", "bbbbbbb.json"),
                   json.dumps({"ok": False, "updated": "now", "steps": {"gtest": {"ok": False}}}))
        self.assertEqual(self.sign(path, "y", yes=True).returncode, 1)
        self.assertFalse(os.path.exists(path + ".sig"))
        self.write(os.path.join(self.root, "gate", "bbbbbbb.json"),
                   json.dumps({"ok": True, "updated": "now", "steps": {"gtest": {"ok": True}}}))
        signed = self.sign(path, "n", yes=True)
        self.assertEqual(signed.returncode, 0, signed.stderr)
        self.assertEqual(self.install(path), 0)

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

    # A bundle of the host has no version and no api: its base is shown.
    def test_sign_host_bundle(self):
        meta = {"format": 2, "component": "host", "revision": D, "commit": D, "serial": 3000, "base": 1}
        path = self.bundle(D, component="host", meta=meta, extra={f"{D}/mowbite_host/__init__.py": b""})
        os.remove(path + ".sig")
        signers = os.path.join(self.prefix, "etc/mowbite/allowed_signers")
        with open(signers) as f:
            line = f.read()
        self.write(signers, line.replace('mowbite-app"', 'mowbite-app,mowbite-host"'))
        signed = self.sign(path)
        self.assertEqual(signed.returncode, 0, signed.stderr)
        self.assertIn("component host", signed.stdout)
        self.assertIn("base 1", signed.stdout)
        self.assertTrue(os.path.exists(path + ".sig"))


class BundleToolsTest(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp(prefix="mowbite-bundle-")
        self.addCleanup(shutil.rmtree, self.root)

    # The probe export with entries replaced or added, and links.
    def zip(self, entries, links=()):
        path = os.path.join(self.root, "web.zip")
        probe(path)
        with zipfile.ZipFile(path) as archive:
            files = {name: archive.read(name) for name in archive.namelist()}
        with zipfile.ZipFile(path, "w") as archive:
            for name, text in {**files, **entries}.items():
                archive.writestr(name, text)
            for name in links:
                info = zipfile.ZipInfo(name)
                info.external_attr = (0o120777 << 16)
                archive.writestr(info, "index.html")
        return path

    # A zip that leaves its folder, brings links or .gz files, or an app.json not of the app is refused, nothing kept.
    def test_export_refuses(self):
        cases = {"../x": ({"../x": "x"}, ()), "/etc/x": ({"/etc/x": "x"}, ()), "link": ({}, ("guide/a.html",)),
                 ".gz": ({"index.html.gz": "x"}, ()),
                 "owner": ({"app.json": json.dumps({"schema": 1, "owner": "other"})}, ())}
        for case, (entries, links) in cases.items():
            with self.subTest(case), self.assertRaises(app_export.Refused):
                out = os.path.join(self.root, "out")
                try:
                    app_export.export(self.zip(entries, links), out)
                finally:
                    self.assertFalse(os.path.exists(out))
        good = os.path.join(self.root, "good.zip")
        probe(good, version="1.5.0", api_min=14, api_max=15)
        for checks in ({"sha": "0" * 64}, {"version": "1.6.0"}, {"min_app": "1.6.0"}, {"api": 16}, {"api": 13}):
            with self.subTest(checks), self.assertRaises(app_export.Refused):
                app_export.export(good, os.path.join(self.root, "out"), **checks)
        fields = app_export.export(good, os.path.join(self.root, "out"), sha=app_export.sha256(good),
                                   version="1.5.0", min_app="1.4.0-beta.20", api=15)
        self.assertEqual(fields["api"], {"min": 14, "max": 15})

    # With break in app.json the export goes with an OS of that break and api at least min, max is shown only;
    # without, api within min..max whatever the break of the OS.
    def test_app_export_break(self):
        def export(name, os_api, os_break, **app):
            path = os.path.join(self.root, f"{name}.zip")
            probe(path, **app)
            out = os.path.join(self.root, name)
            shutil.rmtree(out, ignore_errors=True)
            return app_export.export(path, out, api=os_api, api_break=os_break)
        with_break = {"api_min": 14, "api_max": 15, "api_break": 1}
        self.assertEqual(export("a", 16, 1, **with_break)["api"], {"min": 14, "max": 15, "break": 1})
        self.assertEqual(export("b", 14, 1, api_min=14, api_max=None, api_break=1)["api"], {"min": 14, "break": 1})
        self.assertEqual(export("c", 15, 2, api_min=14, api_max=15)["api"], {"min": 14, "max": 15})
        # An OS without x-api-break: the range with max.
        self.assertEqual(export("d", 15, None, **with_break)["api"], {"min": 14, "max": 15, "break": 1})
        for api, api_break, app, why in ((13, 1, with_break, "too old"), (16, 2, with_break, "another break"),
                                         (16, None, with_break, "app is too old"),
                                         (15, None, {**with_break, "api_max": None}, "OS that is installed is too old"),
                                         (16, 1, {"api_min": 14, "api_max": 15}, "too old"),
                                         (16, 1, {**with_break, "api_break": "1"}, "missing or wrong"),
                                         (16, 1, {**with_break, "api_max": None, "api_min": None},
                                          "missing or wrong")):
            with self.subTest(api=api, api_break=api_break, app=app):
                with self.assertRaisesRegex(app_export.Refused, why):
                    export("x", api, api_break, **app)
        # The command line takes api and break from x-api and x-api-break of the schema.
        schema = os.path.join(self.root, "api.schema.json")
        path = os.path.join(self.root, "cli.zip")
        probe(path, **with_break)
        for x_break, rc in ((1, 0), (2, 1)):
            with open(schema, "w") as f:
                json.dump({"x-api": 16, "x-api-break": x_break}, f)
            out = os.path.join(self.root, f"cli-{x_break}")
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(app_export.main(["export", path, out, "--api-schema", schema]), rc)

    def planner(self, contract=None, where="python"):
        base = os.path.join(self.root, "install")
        share = os.path.join(base, "mowbite_planner", "share", "mowbite_planner")
        os.makedirs(share)
        with open(os.path.join(share, "package.xml"), "w") as f:
            f.write("<package><name>mowbite_planner</name><version>2.1.0</version></package>")
        if contract is not None:
            folder = share if where == "share" else os.path.join(
                base, "mowbite_planner", "lib", "python3.12", "site-packages", "mowbite_planner")
            os.makedirs(folder, exist_ok=True)
            with open(os.path.join(folder, "contract.json"), "w") as f:
                f.write(json.dumps(contract))
        return base

    def test_planner_meta(self):
        terms = {"interface": 2, "files_version": 1, "requires": {"python": "3.8", "shapely": "2.0.3"}}
        self.assertEqual(planner_meta.read(self.planner(terms)), {"version": "2.1.0", **terms})
        shutil.rmtree(os.path.join(self.root, "install"))
        self.assertEqual(planner_meta.read(self.planner(terms, where="share"))["interface"], 2)
        shutil.rmtree(os.path.join(self.root, "install"))
        base = self.planner()
        self.assertEqual(planner_meta.read(base), {"version": "2.1.0"})
        with self.assertRaises(planner_meta.Refused):
            planner_meta.read(base, contract=True)
        shutil.rmtree(base)
        with self.assertRaises(planner_meta.Refused):
            planner_meta.read(self.planner({"interface": 1, "files_version": 1, "requires": ["numpy>=1.17"]}))


# gh for publish.py: releases as folders of $GH_STORE, .prerelease marks a pre-release.
GH = """#!/bin/sh
s="$GH_STORE"
echo "$*" >> "$s.log"
[ "$1" = release ] || exit 2
cmd=$2; tag=$3; shift 3
case $cmd in
  view) [ -d "$s/$tag" ];;
  create) [ -d "$s/$tag" ] && { echo "release $tag exists" >&2; exit 1; }
    mkdir -p "$s/$tag"; for a; do [ -f "$a" ] && cp "$a" "$s/$tag/"; done
    case " $* " in *" --prerelease "*) touch "$s/$tag/.prerelease";; esac; true;;
  upload) [ -d "$s/$tag" ] || exit 1; for a; do [ -f "$a" ] && cp "$a" "$s/$tag/"; done; true;;
  download) [ -d "$s/$tag" ] || exit 1; d=; p=
    while [ $# -gt 0 ]; do case $1 in -D) d=$2; shift;; -p) p="$p $2"; shift;; esac; shift; done
    for f in $p; do cp "$s/$tag/$f" "$d/" || exit 1; done;;
  edit) [ -d "$s/$tag" ] || exit 1; case " $* " in *" --prerelease=false "*) rm -f "$s/$tag/.prerelease";; esac;;
  *) exit 2;;
esac
"""


class PublishTest(Base):
    """publish.py against the gh above, then the net origin of mowbite-host reading what it published."""

    def setUp(self):
        super().setUp()
        self.store = os.path.join(self.root, "releases")
        os.makedirs(self.store)
        self.write(os.path.join(self.root, "bin", "gh"), GH, 0o755)
        signers = os.path.join(self.root, "allowed_signers")
        with open(self.key + ".pub") as f:
            public = " ".join(f.read().split()[:2])
        self.write(signers, f'mowbite namespaces="mowbite-os,mowbite-app,mowbite-index" {public}\n')
        os.environ.update(GH_STORE=self.store, MOWBITE_SIGNERS=signers,
                          MOWBITE_PUBLISH_LOCK=os.path.join(self.root, "publish.lock"))

    def publish(self, *args):
        return subprocess.run([sys.executable, "-I", PUBLISH, *args], capture_output=True, text=True)

    def index(self):
        with open(os.path.join(self.store, "index", "index.json")) as f:
            return json.load(f)

    # What the net origin of a mower takes from the published releases in channel.
    def taken(self, channel):
        sys.path.insert(0, HOST_LIB)
        from mowbite_host import net, paths
        where = paths.Paths(os.path.join(self.root, "mower"))
        os.makedirs(os.path.dirname(where.signers), exist_ok=True)
        shutil.copyfile(os.environ["MOWBITE_SIGNERS"], where.signers)

        def fetch(url, path, limit, seconds):
            shutil.copyfile(os.path.join(self.store, url[len(net.BASE):]), path)
        origin = net.Net(where, fetch=fetch)
        result = origin.refresh(channel, 0, lambda component: 0)
        return result, sorted((c["component"], c["revision"]) for c in origin.candidates(channel))

    def test_publish_and_take(self):
        os_bundle, app_bundle = self.bundle(A), self.bundle(B, component="app")
        done = self.publish(self.key, os_bundle, app_bundle)
        self.assertEqual(done.returncode, 0, done.stderr)
        index = self.index()
        self.assertEqual(sorted(e["url"] for e in index["bundles"]),
                         [f"app-{B}/mowbite-app-{B}.tar.gz", f"os-{A}/mowbite-os-{A}.tar.gz"])
        self.assertEqual({e["channel"] for e in index["bundles"]}, {"preview"})
        self.assertTrue(os.path.exists(os.path.join(self.store, f"os-{A}", ".prerelease")))
        result, taken = self.taken("preview")
        self.assertEqual((result["result"], taken), ("ok", [("app", B), ("os", A)]))
        self.assertEqual(self.taken("stable")[1], [])

    def test_serial_rises_and_twice_is_refused(self):
        a = self.bundle(A)
        self.assertEqual(self.publish(self.key, a).returncode, 0)
        first = self.index()["serial"]
        self.assertNotEqual(self.publish(self.key, a).returncode, 0)
        self.assertEqual(self.index()["serial"], first)
        self.assertEqual(self.publish(self.key, self.bundle(B)).returncode, 0)
        self.assertGreater(self.index()["serial"], first)

    def test_promote(self):
        self.assertEqual(self.publish(self.key, self.bundle(A)).returncode, 0)
        done = self.publish("--promote", self.key, f"os-{A}")
        self.assertEqual(done.returncode, 0, done.stderr)
        self.assertEqual([e["channel"] for e in self.index()["bundles"]], ["stable"])
        self.assertFalse(os.path.exists(os.path.join(self.store, f"os-{A}", ".prerelease")))
        self.assertEqual(self.taken("stable")[1], [("os", A)])

    def test_vendor_files_are_never_published(self):
        for name in ("install/share/models/x.rknn", "install/lib/librknnrt.so"):
            done = self.publish(self.key, self.bundle(A, extra={f"{A}/{name}": b"x"}))
            self.assertNotEqual(done.returncode, 0)
            self.assertIn("never published", done.stderr)
        self.assertEqual(os.listdir(self.store), [])

    def test_signed_by_another_key_publishes_nothing(self):
        done = self.publish(self.key, self.bundle(A, key=self.other_key))
        self.assertNotEqual(done.returncode, 0)
        self.assertEqual(os.listdir(self.store), [])


A, B, C, D, E = "aaaaaaa", "bbbbbbb", "ccccccc", "ddddddd", "eeeeeee"
PLANNER, APP, HOST = "opt/mowbite-planner", "opt/mowbite-app", "opt/mowbite-host"

if __name__ == "__main__":
    unittest.main()
