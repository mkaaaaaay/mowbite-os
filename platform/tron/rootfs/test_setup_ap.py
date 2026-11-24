#!/usr/bin/env python3
"""files/usr/lib/mowbite/setup-ap against a stand-in nmcli: no access point next to a connected wlan0, up when wlan0
stays without network, down again once wlan0 connects.
  python3 -I platform/tron/rootfs/test_setup_ap.py
"""
import os
import subprocess
import tempfile
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, "files", "usr", "lib", "mowbite", "setup-ap")
# Answers GENERAL.STATE of wlan0 from the file state and notes every other call.
NMCLI = """#!/bin/bash
dir=$(dirname "$0")
if [ "$*" = "-t -g GENERAL.STATE device show wlan0" ]; then cat "$dir/state"; exit 0; fi
echo "$*" >> "$dir/calls"
"""


class TestSetupAp(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        d = self.dir.name
        with open(os.path.join(d, "nmcli"), "w") as f:
            f.write(NMCLI)
        os.chmod(os.path.join(d, "nmcli"), 0o755)
        self.env = dict(os.environ, PATH=f"{d}:/usr/bin:/bin", SETUP_AP_WAIT_S="1", SETUP_AP_POLL_S="1")

    def tearDown(self):
        self.dir.cleanup()

    def state(self, value):
        with open(os.path.join(self.dir.name, "state"), "w") as f:
            f.write(value + "\n")

    def calls(self):
        try:
            with open(os.path.join(self.dir.name, "calls")) as f:
                return f.read().splitlines()
        except FileNotFoundError:
            return []

    def test_a_connected_wlan0_gets_no_access_point(self):
        self.state("100 (connected)")
        r = subprocess.run([TOOL], env=self.env, capture_output=True, text=True, timeout=20)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertEqual(self.calls(), [])

    def test_the_access_point_comes_and_goes_with_wlan0(self):
        self.state("30 (disconnected)")
        p = subprocess.Popen([TOOL], env=self.env, stdout=subprocess.PIPE, text=True)
        deadline = time.time() + 10
        while "connection up mowbite-setup ifname wlan1" not in self.calls() and time.time() < deadline:
            time.sleep(0.1)
        self.assertEqual(self.calls(), ["device set wlan1 managed yes", "connection up mowbite-setup ifname wlan1"])
        self.state("100 (connected)")
        self.assertEqual(p.wait(timeout=10), 0)
        self.assertEqual(self.calls()[2:], ["connection down mowbite-setup", "device set wlan1 managed no"])

    def test_a_stop_takes_the_access_point_down(self):
        self.state("30 (disconnected)")
        p = subprocess.Popen([TOOL], env=self.env, stdout=subprocess.PIPE, text=True)
        deadline = time.time() + 10
        while len(self.calls()) < 2 and time.time() < deadline:
            time.sleep(0.1)
        p.terminate()
        self.assertEqual(p.wait(timeout=10), 0)
        self.assertEqual(self.calls()[-2:], ["connection down mowbite-setup", "device set wlan1 managed no"])


if __name__ == "__main__":
    unittest.main()
