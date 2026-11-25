#!/usr/bin/env python3
"""files/usr/lib/mowbite/crash on a stand-in tree (CRASH_ROOT): the crash record names the site of the oops that led
to the panic, not of a warning before it, and stays without a site when the record shows none.
  python3 -I platform/tron/rootfs/test_crash.py
"""
import os
import subprocess
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, "files", "usr", "lib", "mowbite", "crash")

WARNING = "[   12.10] WARNING: CPU: 2 PID: 1 at drivers/x.c:10 rkcif_probe+0x10/0x20\n" \
          "[   12.11] pc : rkcif_probe+0x10/0x20\n[   12.11] lr : platform_probe+0x6c/0xd4\n"
OOPS = "[46322.26] Unable to handle kernel access to user memory outside uaccess routines\n" \
       "[46322.27] pc : bluesleep_read_proc_btwrite+0x18/0x30\n" \
       "[46322.27] lr : proc_reg_read+0xa8/0xfc\n"
PANIC = "[46322.30] Kernel panic - not syncing: Oops: Fatal exception\n"


class TestCrash(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.root = self.dir.name
        self.pstore = os.path.join(self.root, "var/lib/systemd/pstore")
        os.makedirs(self.pstore)

    def tearDown(self):
        self.dir.cleanup()

    def record(self, name, text):
        with open(os.path.join(self.pstore, name), "w") as f:
            f.write(text)

    def run_crash(self):
        r = subprocess.run(["sh", TOOL], env=dict(os.environ, CRASH_ROOT=self.root),
                           capture_output=True, text=True, timeout=20)
        self.assertEqual(r.returncode, 0, r.stderr)
        with open(os.path.join(self.root, "run/mowbite/crash")) as f:
            return f.read().split(" ", 1)[1].rstrip("\n")

    def test_site_of_the_oops_before_the_panic(self):
        self.record("console-ramoops-0", WARNING + OOPS + PANIC + "[46322.31] pc : later_line+0x1/0x2\n")
        self.assertEqual(self.run_crash(),
                         "Kernel panic - not syncing: Oops: Fatal exception at bluesleep_read_proc_btwrite")

    def test_no_site_without_a_pc_line(self):
        self.record("console-ramoops-0", "[  350.49] Kernel panic - not syncing: RCU Stall\n")
        self.assertEqual(self.run_crash(), "Kernel panic - not syncing: RCU Stall")

    def test_dmesg_record_without_panic_line_takes_its_last_site(self):
        self.record("dmesg-ramoops-0", "Oops#1 Part1\n" + WARNING + OOPS)
        self.assertEqual(self.run_crash(), "Oops#1 Part1 at bluesleep_read_proc_btwrite")


if __name__ == "__main__":
    unittest.main()
