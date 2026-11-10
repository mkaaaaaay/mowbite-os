"""Tests of the update of mowbite-host itself: mowbite_boot choosing the library, the switch, the confirmation and the
way back.

  python3 -I host/test/test_host_update.py
"""
import json
import os
import shutil
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "boot"))
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import mowbite_boot  # noqa: E402
from mowbite_host import files, updater  # noqa: E402
from test_updater import A, B, Base  # noqa: E402

LIBRARY = {"mowbite_host/__init__.py": b"VERSION = 1\n"}


class BootTest(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp(prefix="mowbite-boot-")
        self.addCleanup(shutil.rmtree, self.root)
        self.prefix, mowbite_boot.PREFIX = mowbite_boot.PREFIX, self.root
        self.addCleanup(setattr, mowbite_boot, "PREFIX", self.prefix)
        self.alarms = []
        self.alarm, mowbite_boot.signal.alarm = mowbite_boot.signal.alarm, self.alarms.append
        self.addCleanup(setattr, mowbite_boot.signal, "alarm", self.alarm)
        self.opt = self.root + mowbite_boot.OPT
        self.seed = self.root + mowbite_boot.SEED
        os.makedirs(self.opt)
        for rev in (A, B):
            os.makedirs(os.path.join(self.opt, rev, "mowbite_host"))
            open(os.path.join(self.opt, rev, "mowbite_host", "__init__.py"), "w").close()

    def link(self, name, rev):
        os.symlink(rev, os.path.join(self.opt, name))

    def pending(self, **record):
        with open(os.path.join(self.opt, "pending"), "w") as f:
            json.dump(record, f)

    def read_pending(self):
        return files.read_json(os.path.join(self.opt, "pending"))

    def test_seed_without_current(self):
        self.assertEqual(mowbite_boot.choose(True), self.seed)
        self.link("current", "c000003")
        self.assertEqual(mowbite_boot.choose(True), self.seed, "an installation that is gone")

    def test_current(self):
        self.link("current", A)
        self.assertEqual(mowbite_boot.choose(True), os.path.join(self.opt, A))
        self.assertEqual(self.alarms, [])

    def test_a_new_one_is_tried_once(self):
        self.link("current", B)
        self.pending(to=B, **{"from": A}, broker=True, tried=False)
        self.assertEqual(mowbite_boot.choose(True), os.path.join(self.opt, B))
        self.assertEqual(self.alarms, [mowbite_boot.TRIAL])
        self.assertEqual(self.read_pending()["tried"], True)
        self.assertEqual(self.read_pending()["broker"], True)
        # It did not confirm itself: the next start goes back.
        self.assertEqual(mowbite_boot.choose(True), os.path.join(self.opt, A))
        self.assertEqual(os.readlink(os.path.join(self.opt, "current")), A)
        self.assertFalse(os.path.lexists(os.path.join(self.opt, "pending")))
        note = files.read_json(self.root + mowbite_boot.BACK)
        self.assertEqual((note["to"], note["from"]), (B, A))

    def test_back_to_the_seed_without_one_before(self):
        self.link("current", B)
        self.pending(to=B, **{"from": ""}, tried=True)
        self.assertEqual(mowbite_boot.choose(True), self.seed)
        self.assertFalse(os.path.lexists(os.path.join(self.opt, "current")))
        self.assertIsNone(files.read_json(self.root + mowbite_boot.BACK)["from"])

    def test_a_pending_one_that_is_not_there_goes_back_at_once(self):
        self.link("current", A)
        self.pending(to="c000003", **{"from": A}, tried=False)
        self.assertEqual(mowbite_boot.choose(True), os.path.join(self.opt, A))
        self.assertEqual(self.alarms, [])

    def test_unreadable_pending_goes_back(self):
        self.link("current", B)
        with open(os.path.join(self.opt, "pending"), "w") as f:
            f.write("{")
        self.assertEqual(mowbite_boot.choose(True), self.seed)
        self.assertFalse(os.path.lexists(os.path.join(self.opt, "current")))

    def test_seed_after_starts_that_did_not_settle(self):
        self.link("current", A)
        for _ in range(mowbite_boot.MAX_UNSETTLED):
            self.assertEqual(mowbite_boot.choose(True), os.path.join(self.opt, A))
        self.assertEqual(mowbite_boot.choose(True), self.seed)
        mowbite_boot.settled()
        self.assertEqual(mowbite_boot.choose(True), os.path.join(self.opt, A))

    def test_the_command_line_takes_what_is_confirmed(self):
        self.link("current", B)
        self.pending(to=B, **{"from": A}, tried=False)
        self.assertEqual(mowbite_boot.choose(False), os.path.join(self.opt, A))
        self.assertEqual(self.read_pending()["tried"], False)
        self.assertFalse(os.path.exists(self.root + mowbite_boot.UNSETTLED))
        self.pending(to=B, **{"from": ""}, tried=True)
        self.assertEqual(mowbite_boot.choose(False), self.seed)


class HostUpdateTest(Base):
    def setUp(self):
        super().setUp()
        self.restarts = []
        self.updater.restart_self = lambda: self.restarts.append(True)
        self.seed(100)
        self.running, updater.RUNNING_HOST = updater.RUNNING_HOST, self.paths.host_seed
        os.makedirs(self.paths.host.opt)

    def tearDown(self):
        updater.RUNNING_HOST = self.running
        super().tearDown()

    def seed(self, serial):
        self.write(os.path.join(self.paths.host_seed, "bundle.json"),
                   json.dumps({"format": 2, "component": "host", "revision": "5eed000", "serial": serial}))

    def host_bundle(self, rev, extra=None, **meta):
        return self.bundle(rev, component="host", extra={**LIBRARY, **(extra or {})}, **meta)

    def switch_to(self, rev):
        self.host_bundle(rev)
        self.updater.check()
        self.updater.start("host", rev)
        return self.updater.join()

    # What the start of the new host does: mowbite_boot tried it, it runs its library and confirms itself.
    def new_host(self, rev):
        pending = files.read_json(self.paths.host.pending)
        if pending is not None:
            files.write_json(self.paths.host.pending, {**pending, "tried": True})
        updater.RUNNING_HOST = self.paths.host.of(rev) if rev else self.paths.host_seed
        self.updater = updater.Updater(self.paths, self.fake, self.updater.installer.health, self.updater.inbox,
                                       self.fake)
        self.updater.host_went_back()
        return self.updater.confirm_host()

    def host(self):
        return next(c for c in self.updater.status()["components"] if c["id"] == "host")

    def test_switch_ends_the_process_and_the_new_one_confirms(self):
        last = self.switch_to(A)
        self.assertIsNone(last)
        self.assertEqual(self.restarts, [True])
        self.assertEqual(self.updater.phase(), "checking")
        self.assertEqual(os.readlink(self.paths.host.current), A)
        self.assertEqual(files.read_json(self.paths.host.pending),
                         {"to": A, "from": "", "broker": False, "by_hand_from": "", "tried": False})
        self.assertFalse(self.fake.stops, "the OS is not touched")
        self.assertTrue(self.new_host(A))
        self.assertEqual(self.updater.state["last"]["result"], "installed")
        self.assertFalse(os.path.lexists(self.paths.host.pending))
        self.assertEqual(self.host()["running"]["revision"], A)
        self.assertEqual(self.host()["previous"]["revision"], "5eed000")
        # The next one keeps this one as previous.
        self.switch_to(B)
        self.assertTrue(self.new_host(B))
        self.assertEqual(os.readlink(self.paths.host.previous), A)
        self.assertEqual(sorted(os.listdir(self.paths.host.opt)), [A, B, "current", "previous"])

    def test_no_confirmation_by_another_library(self):
        self.switch_to(A)
        self.assertFalse(self.new_host(None))
        self.assertTrue(os.path.lexists(self.paths.host.pending))

    def test_back_by_the_start_is_the_result(self):
        self.switch_to(A)
        files.write_json(self.paths.host_back, {"to": A, "from": None, "t": 1, "message": "no confirmation"})
        files.remove(self.paths.host.pending)
        files.remove(self.paths.host.current)
        self.assertFalse(self.new_host(None))
        last = self.updater.state["last"]
        self.assertEqual((last["result"], last["reason"], last["revision"]), ("rolled_back", "UNHEALTHY", A))
        self.updater.check()
        self.assertEqual(self.host()["available"]["blocked"]["result"], "rolled_back")
        self.assertFalse(os.path.exists(self.paths.host_back))

    def test_rollback_by_hand_to_the_seed(self):
        self.switch_to(A)
        self.new_host(A)
        self.updater.restart_self = lambda: None
        self.updater.rollback("host")
        self.updater.join()
        self.assertEqual(files.read_json(self.paths.host.pending)["by_hand_from"], A)
        self.assertFalse(os.path.lexists(self.paths.host.current))
        self.assertTrue(self.new_host(None))
        self.assertEqual(self.updater.state["components"]["host"]["blocked"][A]["reason"], "ROLLED_BACK_BY_HAND")
        self.assertEqual(self.host()["running"]["revision"], "5eed000")

    def test_nothing_installs_while_a_new_host_is_on_trial(self):
        self.switch_to(A)
        self.bundle(B)
        self.updater.check()
        with self.assertRaises(updater.Refused) as refused:
            self.updater.start("os", B)
        self.assertEqual(refused.exception.reason, "BUSY")

    def test_python_that_does_not_parse_is_refused(self):
        self.host_bundle(A, extra={"mowbite_host/broken.py": b"def (:\n"})
        self.updater.check()
        last = self.run_update("host", A)
        self.assertEqual((last["result"], last["reason"]), ("refused", "BAD_BUNDLE"))
        self.assertIn("broken.py", last["message"])
        self.assertFalse(os.path.lexists(self.paths.host.current))

    def test_not_older_than_the_seed(self):
        self.seed(10 ** 12)
        self.host_bundle(A)
        self.updater.check()
        self.assertIsNone(self.host()["available"])

    def test_host_comes_first_at_night(self):
        self.assertEqual(updater.AUTO_ORDER[0], "host")


if __name__ == "__main__":
    unittest.main()
