"""The table of what goes with what: an app or a planner beside an OS, and an own installation beside a bundled one.

  python3 -I host/test/test_compat.py
"""
import json
import os
import sys
import unittest

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "boot"))
from mowbite_host import bundle, compat, paths, updater  # noqa: E402

OS = {"api": 15, "api_break": 1, "min_app": "1.5.0", "planner_interface": 2, "planner_files_version": 3,
      "bundled": {"app": {"revision": "aaaaaaa", "version": "1.5.0", "serial": 100},
                  "planner": {"revision": "bbbbbbb", "version": "2.0.0", "serial": 200}}}
APP = {"version": "1.5.0", "api": {"min": 14, "max": 15}, "serial": 101}
PLANNER = {"version": "2.1.0", "interface": 2, "files_version": 3, "serial": 201}

# component, changes to its bundle.json, changes to the one of the OS, the reason.
FITS = (
    ("app", {}, {}, None),
    ("app", {"api": {"min": 15, "max": 15}}, {}, None),
    ("app", {"api": {"min": 14, "max": 14}}, {}, "APP_TOO_OLD"),
    ("app", {"api": {"min": 16, "max": 17}}, {}, "OS_TOO_OLD"),
    ("app", {"version": "1.4.9"}, {}, "APP_TOO_OLD"),
    ("app", {"version": "1.5.0-beta.3"}, {}, "APP_TOO_OLD"),
    ("app", {"version": "1.10.0"}, {}, None),
    ("app", {"version": None}, {}, "APP_TOO_OLD"),
    ("app", {"version": None}, {"min_app": None}, None),
    ("app", {"api": None}, {}, "APP_TOO_OLD"),
    ("app", {"api": {"min": 14}}, {}, "APP_TOO_OLD"),
    ("app", {"api": {"min": "14", "max": 15}}, {}, "APP_TOO_OLD"),
    ("app", {"api": {"min": True, "max": 15}}, {}, "APP_TOO_OLD"),
    ("app", {}, {"api": None}, "OS_TOO_OLD"),
    ("app", {}, {"api": "15"}, "OS_TOO_OLD"),
    # Too old in both ways: the app says so.
    ("app", {"version": "1.4.0", "api": {"min": 16, "max": 17}}, {}, "APP_TOO_OLD"),
    # Without break the break of the OS does not count.
    ("app", {}, {"api_break": 2}, None),
    ("app", {}, {"api_break": None}, None),
    # With break: the same break and api at least min; max is shown only.
    ("app", {"api": {"min": 14, "max": 15, "break": 1}}, {}, None),
    ("app", {"api": {"min": 15, "break": 1}}, {}, None),
    ("app", {"api": {"min": 14, "max": 14, "break": 1}}, {}, None),
    ("app", {"api": {"min": 16, "max": 17, "break": 1}}, {}, "OS_TOO_OLD"),
    ("app", {"api": {"min": 16, "break": 1}}, {}, "OS_TOO_OLD"),
    ("app", {"api": {"min": 14, "max": 15, "break": 2}}, {}, "APP_API_BREAK"),
    ("app", {"api": {"min": 14, "max": 15, "break": 0}}, {}, "APP_API_BREAK"),
    ("app", {"api": {"min": 14, "break": 1}}, {"api_break": 2}, "APP_API_BREAK"),
    ("app", {"api": {"min": 16, "break": 2}}, {}, "APP_API_BREAK"),
    ("app", {"api": {"min": 14, "break": 1}}, {"api_break": None}, "OS_TOO_OLD"),
    ("app", {"api": {"min": 14, "break": 1}}, {"api_break": "1"}, "OS_TOO_OLD"),
    ("app", {"api": {"min": 14, "break": 1}}, {"api": None}, "OS_TOO_OLD"),
    ("app", {"api": {"min": 14, "break": "1"}}, {}, "APP_TOO_OLD"),
    ("app", {"api": {"min": 14, "break": None}}, {}, "APP_TOO_OLD"),
    ("app", {"api": {"min": 14, "break": True}}, {}, "APP_TOO_OLD"),
    ("app", {"api": {"break": 1}}, {}, "APP_TOO_OLD"),
    ("app", {"api": {"min": 15, "max": 14, "break": 1}}, {}, "APP_TOO_OLD"),
    ("app", {"api": {"min": 14, "max": None, "break": 1}}, {}, "APP_TOO_OLD"),
    ("app", {"version": "1.4.0", "api": {"min": 14, "break": 2}}, {}, "APP_TOO_OLD"),
    ("planner", {}, {}, None),
    ("planner", {"interface": 1}, {}, "PLANNER_INTERFACE"),
    ("planner", {"interface": 3}, {}, "PLANNER_INTERFACE"),
    ("planner", {"interface": None}, {}, "PLANNER_INTERFACE"),
    ("planner", {}, {"planner_interface": None}, "PLANNER_INTERFACE"),
    ("planner", {"interface": None}, {"planner_interface": None}, "PLANNER_INTERFACE"),
    ("planner", {"interface": True}, {"planner_interface": 1}, "PLANNER_INTERFACE"),
    ("planner", {"files_version": 2}, {}, "PLANNER_DATA"),
    ("planner", {"files_version": 4}, {}, "PLANNER_DATA"),
    ("planner", {"files_version": None}, {"planner_files_version": None}, "PLANNER_DATA"),
    ("planner", {"interface": 1, "files_version": 2}, {}, "PLANNER_INTERFACE"),
    ("os", {}, {}, None),
)

# component, changes to its own bundle.json, changes to the one of the OS, why the own one is left aside.
OWN = (
    ("app", {}, {}, None),
    ("app", {"serial": 100}, {}, "OLD_SERIAL"),
    ("app", {"serial": 99}, {}, "OLD_SERIAL"),
    ("app", {"serial": None}, {}, "OLD_SERIAL"),
    ("app", {}, {"bundled": None}, None),
    ("app", {}, {"bundled": {"app": {"serial": "100"}}}, None),
    ("app", {"api": {"min": 14, "max": 14}, "serial": 99}, {}, "APP_TOO_OLD"),
    ("app", {"api": {"min": 14, "break": 2}}, {}, "APP_API_BREAK"),
    ("planner", {}, {}, None),
    ("planner", {"serial": 200}, {}, "OLD_SERIAL"),
    ("planner", {"interface": 1}, {}, "PLANNER_INTERFACE"),
)


def changed(base, changes):
    return {**base, **changes}


class CompatTest(unittest.TestCase):
    def rows(self, table, judge):
        for component, changes, os_changes, expected in table:
            meta = changed({"app": APP, "planner": PLANNER, "os": OS}[component], changes)
            with self.subTest(component=component, changes=changes, os=os_changes):
                self.assertEqual(judge(component, meta, changed(OS, os_changes)), expected)

    def test_what_goes_with_the_os(self):
        self.rows(FITS, compat.reason)

    def test_own_or_bundled(self):
        self.rows(OWN, compat.skip_own)

    def test_without_an_os_nothing_fits(self):
        self.assertEqual(compat.reason("app", APP, {}), "OS_TOO_OLD")
        self.assertEqual(compat.reason("planner", PLANNER, {}), "PLANNER_INTERFACE")

    def test_every_reason_has_its_text(self):
        reasons = {row[3] for row in FITS if row[3]}
        self.assertEqual(reasons, set(compat.EXPLAIN))

    def test_tables_name_the_same_components(self):
        # The host is no place of the installer: mowbite_boot takes it back.
        self.assertEqual(set(updater.COMPONENTS), {*paths.PLACES, "host"})
        self.assertEqual(set(updater.COMPONENTS), set(bundle.MAX_BUNDLE))
        self.assertEqual(sorted(updater.AUTO_ORDER), sorted(updater.COMPONENTS))
        self.assertEqual({c for c, (_, overlay) in paths.PLACES.items() if overlay}, set(compat.CHECKS))

    # The app learns the components and the reasons of the host from the catalog of the OS.
    def test_the_catalog_names_them(self):
        with open(os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", "src", "mowbite_api",
                               "api.schema.json")) as f:
            schema = json.load(f)
        enum = schema["$defs"]["update.start.params"]["properties"]["component"]["enum"]
        self.assertEqual(enum, list(updater.COMPONENTS))
        self.assertLessEqual(set(compat.EXPLAIN), set(schema["x-reasons"]["UPDATE"]))


if __name__ == "__main__":
    unittest.main()
