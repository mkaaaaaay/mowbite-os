"""Tests of the planner and the app as components of their own beside the OS: own or bundled, their gates, the way
back, a cut in every step of a switch.

  python3 -I host/test/test_components.py
"""
import contextlib
import hashlib
import json
import os
import sys
import threading
import time
import unittest

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from mowbite_host import Refused, bundle, files, gate, installer, origin, updater  # noqa: E402
from test_updater import A, B, Base  # noqa: E402

P1, P2, P3 = "1111111", "2222222", "3333333"
X, Y, Z = "d0d0d0d", "e0e0e0e", "f0f0f0f"
OS_META = {"api": 15, "planner_interface": 1, "planner_files_version": 1}


class Cut(BaseException):
    """The host is killed here: nothing after it runs."""


class ComponentBase(Base):
    def os_bundle(self, rev, **meta):
        return self.bundle(rev, **{**OS_META, **meta})

    def planner_bundle(self, rev, extra=None, **meta):
        return self.bundle(rev, component="planner", extra={"planner/local_setup.bash": b"", **(extra or {})},
                           **{"version": "2.0.0", "interface": 1, "files_version": 1, **meta})

    # An export of the app as the web server reads it; said: the version its version.json names.
    def app_bundle(self, rev, version="1.5.0", said=None, extra=None, modes=None, **meta):
        export = {"app/index.html": b"<html></html>", "app/_next/static/a.js": b"1",
                  "app/version.json": json.dumps({"version": said or version, "level": 1}).encode(), **(extra or {})}
        return self.bundle(rev, component="app", extra=export, modes=modes,
                           **{"version": version, "api": {"min": 14, "max": 15}, **meta})

    def install(self, component, rev, reset_jobs=False, **meta):
        getattr(self, f"{component}_bundle")(rev, **meta)
        self.updater.check()
        return self.run_update(component, rev, reset_jobs=reset_jobs)

    def installed(self, component, rev, **meta):
        last = self.install(component, rev, **meta)
        self.assertEqual((last["result"], last["component"]), ("installed", component), last)

    def link(self, component, name):
        return files.link_target(getattr(self.paths.places[component], name))

    def links(self, component):
        return tuple(self.link(component, name) for name in ("current", "previous", "active"))

    def entry(self, component):
        return next(c for c in self.updater.status()["components"] if c["id"] == component)

    def blocked(self, component="os"):
        return self.updater.state["components"][component]["blocked"]

    def checkpoint(self, **fields):
        self.write(self.paths.checkpoint, json.dumps({"version": 1, "job_id": "j1", "area": 0, "area_id": "a",
                                                      "path": 0, "index": 0, "fingerprint": "f", **fields}))

    def rollback(self, component, **kwargs):
        self.updater.rollback(component, **kwargs)
        return self.updater.join()


class AppTest(ComponentBase):
    def setUp(self):
        super().setUp()
        self.installed("os", A)

    # Decision S1: only files change, so neither mowing nor an emergency nor a restart hold wait for it.
    def test_installs_while_the_os_mows_without_touching_it(self):
        self.fake.session(state="MOWING", emergency=True, charging=False)
        self.write(self.paths.restart_hold, "{}")
        starts, holds = self.fake.starts, list(self.fake.holds)
        phases = []
        self.updater.on_change = lambda status: phases.append((status["phase"], self.updater.os_switching()))
        self.installed("app", X)
        self.assertEqual(self.links("app"), (X, None, X))
        # Not stopped, not started, and no word about a hold to it.
        self.assertEqual((self.fake.running, self.fake.starts, self.fake.stops, self.fake.holds),
                         (True, starts, 0, holds))
        self.assertFalse(os.path.exists(self.paths.hold))
        self.assertIn(("switching", False), phases)
        entry = self.entry("app")
        self.assertEqual((entry["source"], entry["own_skipped"], entry["running"]["revision"],
                          entry["running"]["version"]), ("own", None, X, "1.5.0"))
        self.assertIsInstance(entry["installed_at"], int)

    def test_installs_while_the_os_is_stopped(self):
        self.fake.stop()
        self.installed("app", X)
        self.assertFalse(self.fake.running)

    def test_bundled_until_an_own_one_is_installed(self):
        self.installed("os", B, bundled={"app": {"revision": "abcdef0", "version": "1.4.2", "serial": 5}})
        entry = self.entry("app")
        self.assertEqual((entry["source"], entry["running"], entry["previous"], entry["own_skipped"]),
                         ("bundled", {"revision": "abcdef0", "date": 5, "version": "1.4.2"}, None, None))
        self.assertNotIn("source", self.entry("os"))
        self.assertIsNone(self.entry("os")["running"]["version"])
        self.installed("app", X)
        # Without an own one before it, back is the one of the OS.
        self.assertEqual(self.entry("app")["previous"]["revision"], "abcdef0")

    def test_version_of_the_export_has_to_be_the_one_of_the_bundle(self):
        last = self.install("app", X, said="1.4.9")
        self.assertEqual((last["result"], last["reason"]), ("rolled_back", "UNHEALTHY"), last)
        self.assertEqual(self.links("app"), (None, None, None))
        self.assertEqual(self.blocked("app")[X]["result"], "rolled_back")
        self.assertEqual(os.listdir(self.paths.places["app"].opt), [])

    def test_unhealthy_goes_back_to_the_own_one_before(self):
        self.installed("app", X)
        last = self.install("app", Y, extra={"app/index.html": b""}, modes={"app/index.html": 0o600})
        self.assertEqual((last["result"], last["reason"]), ("rolled_back", "UNHEALTHY"), last)
        self.assertIn("not readable for the web server: app/index.html", last["message"])
        self.assertEqual(self.links("app"), (X, None, X))

    def test_export_without_its_page_is_no_bundle(self):
        self.bundle(X, component="app", version="1.5.0", api={"min": 14, "max": 15})
        self.app_bundle(Y, api={"min": 15, "max": 14})
        self.updater.check()
        self.assertIsNone(self.entry("app")["available"])

    def test_has_to_go_with_the_os(self):
        for meta, reason in (({"api": {"min": 13, "max": 14}}, "APP_TOO_OLD"),
                             ({"api": {"min": 16, "max": 16}}, "OS_TOO_OLD")):
            last = self.install("app", X, **meta)
            self.assertEqual((last["result"], last["reason"]), ("refused", reason), last)
            self.assertEqual(self.links("app"), (None, None, None))
            # Shown from the bundles, not kept: with another OS it may fit.
            self.assertEqual(self.entry("app")["available"]["blocked"], {"reason": reason, "t": None})
            self.assertEqual(self.blocked("app"), {})
        self.installed("os", B, api=16)
        self.assertIsNone(self.entry("app")["available"]["blocked"])
        self.assertEqual(self.run_update("app", X)["result"], "installed")

    def test_machine_code_refused(self):
        last = self.install("app", X, extra={"app/_next/static/tool": b"\x7fELF" + bytes(60)})
        self.assertEqual((last["result"], last["reason"]), ("refused", "BAD_BUNDLE"), last)
        self.assertIn("app/_next/static/tool", last["message"])
        self.assertEqual(self.blocked("app")[X]["reason"], "BAD_BUNDLE")
        self.assertEqual(os.listdir(self.paths.places["app"].opt), [])

    def test_serial_of_its_own(self):
        self.installed("os", B, serial=9000, bundled={"app": {"revision": "abcdef0", "version": "1.4.2",
                                                              "serial": 500}})
        # Older than the OS and newer than the app it brings along: kept and offered.
        kept = self.app_bundle(X, serial=600)
        gone = self.app_bundle(Y, serial=500)
        self.updater.check()
        self.assertTrue(os.path.exists(kept))
        self.assertFalse(os.path.exists(gone))
        self.assertEqual(self.entry("app")["available"]["revision"], X)
        self.assertEqual(self.run_update("app")["result"], "installed")
        # Given by hand, one not newer than the installed one is refused.
        old = self.app_bundle(Z, serial=600, folder=self.root)
        self.updater.offer({"component": "app", "revision": Z, "serial": 600, "path": old})
        last = self.run_update("app", Z)
        self.assertEqual((last["result"], last["reason"]), ("refused", "OLD_SERIAL"), last)

    def test_two_kept_in_its_own_place(self):
        self.installed("os", B)
        for rev in (X, Y, Z):
            self.installed("app", rev)
        self.assertEqual(sorted(os.listdir(self.paths.places["app"].opt)), sorted([Y, Z, "active", "current",
                                                                                 "previous"]))
        self.assertEqual(sorted(os.listdir(self.paths.os.opt)), sorted([A, B, "current", "previous"]))

    def test_back_and_forth_between_own_and_bundled(self):
        with self.assertRaises(Refused) as caught:
            self.updater.rollback("app")
        self.assertEqual(caught.exception.reason, "NO_PREVIOUS")
        self.installed("app", X)
        last = self.rollback("app")
        self.assertEqual((last["result"], last["revision"], last["component"]), ("installed", None, "app"), last)
        self.assertEqual(self.links("app"), (None, X, None))
        self.assertEqual(self.blocked("app")[X]["reason"], "ROLLED_BACK_BY_HAND")
        self.assertEqual(self.entry("app")["previous"]["revision"], X)
        self.assertEqual(self.rollback("app")["revision"], X)
        self.assertEqual(self.links("app"), (X, None, X))
        self.installed("app", Y)
        self.assertEqual(self.rollback("app")["revision"], X)
        self.assertEqual(self.links("app"), (X, Y, X))

    def test_never_with_an_unfinished_switch(self):
        self.write(self.paths.places["planner"].pending, "{}")
        last = self.install("app", X)
        self.assertEqual((last["result"], last["reason"]), ("refused", "UNFINISHED"), last)


class PlannerTest(ComponentBase):
    def setUp(self):
        super().setUp()
        self.installed("os", A)

    # Decision S2: the OS restarts with it, through the gate of the OS and held like an update of the OS.
    def test_restarts_the_os_with_it(self):
        start, left = self.fake.start, []

        def start_and_look():
            left.append((files.read_json(self.paths.hold) or {}).get("until", 0) - time.time())
            return start()
        self.fake.start = start_and_look
        starts, asked = self.fake.starts, len(self.fake.holds)
        self.installed("planner", P1)
        self.assertEqual(self.links("planner"), (P1, None, P1))
        self.assertEqual((self.fake.starts, self.fake.stops, self.fake.planner), (starts + 1, 1, P1))
        self.assertGreater(left[0], installer.HOLD_SECONDS - 60)
        self.assertEqual(self.fake.holds[asked:], [900, 0])
        self.assertFalse(os.path.exists(self.paths.hold))
        self.assertEqual(self.link("os", "current"), A)
        entry = self.entry("planner")
        self.assertEqual((entry["source"], entry["running"]["revision"], entry["running"]["version"]),
                         ("own", P1, "2.0.0"))

    def test_gate_of_the_os(self):
        self.planner_bundle(P1)
        self.updater.check()
        for session, reason in (({"state": "MOWING"}, "BUSY"), ({"emergency": True}, "EMERGENCY"),
                                ({"charging": False}, "NO_DOCK_CONTACT")):
            self.fake.session(**session)
            self.assertEqual(self.run_update("planner", P1)["reason"], reason)
        self.assertEqual((self.fake.stops, self.links("planner")), (0, (None, None, None)))

    def test_unhealthy_goes_back_to_the_bundled_one_and_the_os_runs(self):
        self.fake.broken.add(P1)
        last = self.install("planner", P1)
        self.assertEqual((last["result"], last["reason"]), ("rolled_back", "UNHEALTHY"), last)
        self.assertEqual(self.links("planner"), (None, None, None))
        self.assertEqual((self.fake.running, self.fake.planner), (True, None))
        self.assertEqual(self.blocked("planner")[P1]["result"], "rolled_back")
        self.assertFalse(os.path.lexists(self.paths.places["planner"].pending))

    def test_unhealthy_goes_back_to_the_own_one_before(self):
        self.installed("planner", P1)
        self.fake.broken.add(P2)
        self.assertEqual(self.install("planner", P2)["result"], "rolled_back")
        self.assertEqual(self.links("planner"), (P1, None, P1))
        self.assertEqual((self.fake.running, self.fake.planner), (True, P1))

    def test_planner_that_does_not_answer_is_unhealthy(self):
        self.updater.installer.health.probe.planner = lambda: (False, "planner.settings does not answer")
        last = self.install("planner", P1)
        self.assertEqual((last["result"], last["message"]), ("rolled_back", "planner.settings does not answer"))
        self.assertEqual(self.links("planner"), (None, None, None))
        # An update of the OS does not ask the planner.
        self.assertEqual(self.install("os", B)["result"], "installed")

    def test_has_to_go_with_the_os(self):
        for meta, reason in (({"interface": 2}, "PLANNER_INTERFACE"), ({"files_version": 2}, "PLANNER_DATA")):
            last = self.install("planner", P1, **meta)
            self.assertEqual((last["result"], last["reason"]), ("refused", reason), last)
            self.assertEqual(self.entry("planner")["available"]["blocked"], {"reason": reason, "t": None})
        self.assertEqual((self.fake.stops, self.blocked("planner")), (0, {}))

    def test_requires_of_the_rootfs(self):
        last = self.install("planner", P1, requires={"python": "99.0", "no-such-distribution": "1.0"})
        self.assertEqual((last["result"], last["reason"]), ("refused", "BASE_TOO_OLD"), last)
        self.assertIn("python 99.0", last["message"])
        self.assertIn("no-such-distribution 1.0 (found none)", last["message"])
        self.assertEqual(self.blocked("planner")[P1]["reason"], "BASE_TOO_OLD")
        self.assertEqual(self.install("planner", P2, requires={"python": "3.8"})["result"], "installed")

    def test_machine_code_refused(self):
        last = self.install("planner", P1, extra={"planner/lib/fast.so": b"\x7fELF" + bytes(60)})
        self.assertEqual((last["result"], last["reason"]), ("refused", "BAD_BUNDLE"), last)
        self.assertEqual((self.fake.stops, os.listdir(self.paths.places["planner"].opt)), (0, []))

    def test_smaller_limit_than_the_os(self):
        limits = dict(bundle.MAX_BUNDLE)
        bundle.MAX_BUNDLE.update(planner=600, os=10 ** 6)
        try:
            padding = {"planner/pad": os.urandom(2000)}
            self.planner_bundle(P1, extra=padding)
            self.os_bundle(B, extra=padding)
            self.updater.check()
        finally:
            bundle.MAX_BUNDLE.update(limits)
        self.assertIsNone(self.entry("planner")["available"])
        self.assertEqual(self.entry("os")["available"]["revision"], B)

    def test_back_to_the_bundled_one_restarts_the_os(self):
        self.installed("planner", P1)
        starts = self.fake.starts
        self.assertEqual(self.rollback("planner")["result"], "installed")
        self.assertEqual(self.links("planner"), (None, P1, None))
        self.assertEqual((self.fake.starts, self.fake.planner), (starts + 1, None))


class JobTest(ComponentBase):
    def setUp(self):
        super().setUp()
        self.installed("os", A)
        self.planner_bundle(P1)
        self.updater.check()

    def test_what_counts_as_an_open_job(self):
        self.assertFalse(gate.job_open(self.paths))
        for fields, is_open in (({}, False), ({"index": 1}, True), ({"path": 2}, True), ({"area": 1}, True),
                                ({"version": 2}, True), ({"index": None}, True), ({"index": True}, True)):
            self.checkpoint(**fields)
            self.assertEqual(gate.job_open(self.paths), is_open, fields)
        self.write(self.paths.checkpoint, "{ half")
        self.assertTrue(gate.job_open(self.paths))
        os.remove(self.paths.checkpoint)
        os.symlink(self.paths.session, self.paths.checkpoint)
        self.assertTrue(gate.job_open(self.paths))

    def test_open_job_closes_the_gate_of_the_planner_only(self):
        self.checkpoint(index=40)
        holds = list(self.fake.holds)
        last = self.run_update("planner", P1)
        self.assertEqual((last["result"], last["reason"]), ("refused", "JOB_OPEN"), last)
        self.assertEqual((self.fake.stops, self.fake.holds), (0, holds))
        # The host drops nothing.
        self.assertEqual(files.read_json(self.paths.checkpoint)["index"], 40)
        self.assertEqual(self.install("app", X)["result"], "installed")
        self.assertEqual(self.install("os", B)["result"], "installed")

    def test_waits_for_the_end_of_the_job(self):
        self.checkpoint(path=3)
        self.updater.gate_wait = 30
        seen = threading.Event()
        self.updater.on_change = lambda status: (status["phase"], status["reason"]) == (
            "waiting_gate", "JOB_OPEN") and seen.set()
        self.updater.start("planner", P1)
        self.assertTrue(seen.wait(10))
        self.checkpoint()
        self.assertEqual(self.updater.join()["result"], "installed")

    def test_reset_jobs_goes_on(self):
        self.checkpoint(index=40)
        self.assertEqual(self.run_update("planner", P1, reset_jobs=True)["result"], "installed")
        self.assertEqual(files.read_json(self.paths.checkpoint)["index"], 40)
        self.checkpoint(index=41)
        self.assertEqual(self.rollback("planner")["reason"], "JOB_OPEN")
        self.assertEqual(self.rollback("planner", reset_jobs=True)["result"], "installed")

    # A job that begins while the hold is asked for: no switch, and the hold is given back.
    def test_job_begun_before_the_hold_was_granted(self):
        request_hold = self.fake.request_hold

        def begin_and_grant(seconds):
            self.checkpoint(index=1)
            request_hold(seconds)
        self.fake.request_hold = begin_and_grant
        last = self.run_update("planner", P1)
        self.assertEqual((last["result"], last["reason"]), ("refused", "JOB_OPEN"), last)
        self.assertEqual((self.fake.stops, self.fake.holds[-1]), (0, 0))


class SettleTest(ComponentBase):
    def setUp(self):
        super().setUp()
        self.installed("os", A)
        self.installed("planner", P1)
        self.installed("app", X)

    def test_own_ones_left_aside_by_an_os_they_do_not_go_with(self):
        self.installed("os", B, api=16, planner_interface=2)
        self.assertEqual((self.links("planner"), self.links("app")), ((P1, None, None), (X, None, None)))
        # The OS started without the own planner.
        self.assertIsNone(self.fake.planner)
        planner, app = self.entry("planner"), self.entry("app")
        self.assertEqual((planner["source"], planner["own_skipped"]),
                         ("bundled", {"revision": P1, "reason": "PLANNER_INTERFACE"}))
        self.assertEqual((app["source"], app["own_skipped"]), ("bundled", {"revision": X, "reason": "APP_TOO_OLD"}))
        # Back at the OS they go with they are used again.
        self.assertEqual(self.rollback("os")["result"], "installed")
        self.assertEqual((self.links("planner"), self.links("app"), self.fake.planner),
                         ((P1, None, P1), (X, None, X), P1))
        self.assertEqual((self.entry("planner")["source"], self.entry("planner")["own_skipped"]), ("own", None))

    def test_os_that_brings_a_newer_one_along(self):
        newer = {"planner": {"revision": "abcdef1", "version": "2.1.0", "serial": 10 ** 6},
                 "app": {"revision": "abcdef2", "version": "1.6.0", "serial": 10 ** 6}}
        self.installed("os", B, bundled=newer)
        for component in ("planner", "app"):
            entry = self.entry(component)
            self.assertEqual((entry["source"], entry["own_skipped"]["reason"], entry["running"]["revision"]),
                             ("bundled", "OLD_SERIAL", newer[component]["revision"]))
        # What comes now has to be newer than the one the OS brings along.
        self.planner_bundle(P2)
        self.updater.check()
        self.assertIsNone(self.entry("planner")["available"])

    def test_unhealthy_os_goes_back_and_the_own_ones_with_it(self):
        self.fake.broken.add(B)
        self.assertEqual(self.install("os", B, planner_interface=2)["result"], "rolled_back")
        self.assertEqual((self.link("os", "current"), self.links("planner"), self.fake.planner),
                         (A, (P1, None, P1), P1))

    def test_same_files_same_links(self):
        os.remove(self.paths.places["app"].active)
        os.symlink("0000000", self.paths.places["planner"].active + ".new")
        installer.settle(self.paths)
        installer.settle(self.paths)
        self.assertEqual((self.links("planner"), self.links("app")), ((P1, None, P1), (X, None, X)))
        # Without an OS nothing own is used.
        os.remove(self.paths.os.current)
        installer.settle(self.paths)
        self.assertEqual((self.links("planner"), self.links("app")), ((P1, None, None), (X, None, None)))


class CutTest(ComponentBase):
    """A kill of the host in each step of a switch, then its next start."""

    def setUp(self):
        super().setUp()
        self.installed("os", A)

    @contextlib.contextmanager
    def cut_at(self, step, switching):
        def cut(*args):
            raise Cut()
        set_link, settle = files.set_link, installer.settle

        def cut_link(link, target):
            if link == switching.place.current:
                raise Cut()
            set_link(link, target)
        holder, name, stand_in = {"stop": (self.fake, "stop", cut), "link": (files, "set_link", cut_link),
                                  "settle": (installer, "settle", cut), "start": (self.fake, "start", cut),
                                  "health": (switching.health, "wait_healthy", cut)}[step]
        setattr(holder, name, stand_in)
        try:
            yield
        finally:
            if holder in (files, installer):
                files.set_link, installer.settle = set_link, settle
            else:
                delattr(holder, name)

    # A switch of component to rev cut at step, then a host that starts: the last result it leaves.
    def cut_and_recover(self, component, rev, step):
        with self.cut_at(step, self.updater.installers[component]):
            with self.assertRaises(Cut):
                self.updater.installers[component].switch(rev)
        self.assertTrue(os.path.exists(self.paths.places[component].pending), step)
        again = updater.Updater(self.paths, self.fake, self.updater.installer.health, origin.Inbox(self.paths),
                                self.fake)
        again.recover(start_stopped=True)
        installer.settle(self.paths)
        self.assertFalse(gate.unfinished(self.paths), step)
        last = again.state["last"]
        self.assertEqual((last["result"], last["reason"], last["component"], last["revision"]),
                         ("failed", "INTERRUPTED", component, rev), step)

    def test_planner(self):
        # P2 lies unpacked beside the own P1, then beside none.
        self.installed("planner", P1)
        self.installed("planner", P2)
        self.rollback("planner")
        for before in (P1, None):
            if before is None:
                self.rollback("planner", reset_jobs=True)
                self.rollback("planner")
                os.remove(self.paths.places["planner"].previous)
                self.updater.installers["planner"]._set_current(None)
                self.fake.restart()
            for step in ("stop", "link", "settle", "start", "health"):
                with self.subTest(before=before, step=step):
                    self.cut_and_recover("planner", P2, step)
                    self.assertEqual((self.link("planner", "current"), self.link("planner", "active")),
                                     (before, before))
                    self.assertEqual((self.fake.running, self.fake.planner, self.link("os", "current")),
                                     (True, before, A))

    def test_app(self):
        self.installed("app", X)
        self.installed("app", Y)
        self.rollback("app")
        starts = self.fake.starts
        for before in (X, None):
            if before is None:
                os.remove(self.paths.places["app"].previous)
                self.updater.installers["app"]._set_current(None)
            for step in ("link", "settle", "health"):
                with self.subTest(before=before, step=step):
                    self.cut_and_recover("app", Y, step)
                    self.assertEqual((self.link("app", "current"), self.link("app", "active")), (before, before))
        self.assertEqual((self.fake.running, self.fake.starts, self.fake.stops), (True, starts, 0))

    # The new OS does not go with the own planner; cut before active followed, the old OS finds its planner again.
    def test_os_with_an_own_planner(self):
        self.installed("planner", P1)
        self.installed("os", B, planner_interface=2)
        self.rollback("os")
        for step in ("stop", "link", "settle", "start", "health"):
            with self.subTest(step=step):
                self.cut_and_recover("os", B, step)
                self.assertEqual((self.link("os", "current"), self.links("planner")[::2], self.fake.planner),
                                 (A, (P1, P1), P1))
                self.assertTrue(self.fake.running)

    def test_before_without_its_folder_leaves_the_os_off(self):
        files.write_json(self.paths.os.pending, {"to": B, "from": "0000000", "ran": True})
        self.fake.stop()
        starts = self.fake.starts
        self.updater.recover(start_stopped=True)
        self.assertEqual((self.link("os", "current"), self.fake.starts), (None, starts))


class InboxTest(ComponentBase):
    def test_unknown_component_stays_and_is_no_candidate(self):
        self.installed("os", A)
        path = self.bundle(B, component="host")
        self.updater.check()
        self.assertTrue(os.path.exists(path))
        self.assertEqual([c["available"] for c in self.updater.status()["components"]], [None, None, None])

    def test_index_names_what_goes_with_the_os(self):
        self.installed("os", A)
        path = self.app_bundle(X, api={"min": 16, "max": 16})
        with open(path, "rb") as f:
            data = f.read()
        entry = {"component": "app", "revision": X, "serial": self.serial, "size": len(data),
                 "sha256": hashlib.sha256(data).hexdigest(), "file": os.path.basename(path),
                 "version": "1.5.0", "api": {"min": 16, "max": 16}}
        index = os.path.join(self.paths.inbox, origin.INDEX)
        with open(index, "w") as f:
            json.dump({"bundles": [entry, {**entry, "component": "host"}]}, f)
        self.sign(index, "mowbite-index")
        self.updater.check()
        self.assertEqual(self.entry("app")["available"]["blocked"], {"reason": "OS_TOO_OLD", "t": None})
        self.assertTrue(os.path.exists(path))


class AutoTest(ComponentBase):
    def setUp(self):
        super().setUp()
        self.write(self.paths.prefix + "/run/systemd/timesync/synchronized", "")
        self.installed("os", A)
        self.updater.set_auto(True, "00:00", "23:59")
        self.updater.remember_app("1.5.0")

    def tick(self, now):
        self.updater.state["last"] = None
        self.updater.tick(now)
        return self.updater.join() or {}

    def test_app_then_planner_then_os(self):
        self.os_bundle(B)
        self.planner_bundle(P1)
        self.app_bundle(X)
        now = time.time()
        self.assertEqual([self.tick(now + n).get("component") for n in range(4)], ["app", "planner", "os", None])
        self.assertEqual((self.link("os", "current"), self.links("planner"), self.links("app")),
                         (B, (P1, None, P1), (X, None, X)))

    # A revision tried this night does not hold up the components after it.
    def test_tried_one_lets_the_next_have_its_turn(self):
        self.checkpoint(index=7)
        self.planner_bundle(P1)
        self.os_bundle(B)
        now = time.time()
        last = self.tick(now)
        self.assertEqual((last["component"], last["reason"]), ("planner", "JOB_OPEN"))
        last = self.tick(now + 300)
        self.assertEqual((last["component"], last["result"]), ("os", "installed"))
        self.assertEqual(self.tick(now + 600), {})

    def test_what_does_not_go_with_the_os_is_not_tried(self):
        self.app_bundle(X, api={"min": 16, "max": 16})
        self.assertEqual(self.tick(time.time()), {})
        self.assertIsNone(self.updater.state["components"]["app"]["auto_tried"])

    def test_os_stopped_by_hand_keeps_its_planner_the_app_updates(self):
        self.fake.stop()
        self.planner_bundle(P1)
        self.app_bundle(X)
        now = time.time()
        self.assertEqual(self.tick(now).get("component"), "app")
        self.assertEqual(self.tick(now + 300), {})
        self.assertEqual((self.fake.running, self.links("planner")), (False, (None, None, None)))


if __name__ == "__main__":
    unittest.main()
