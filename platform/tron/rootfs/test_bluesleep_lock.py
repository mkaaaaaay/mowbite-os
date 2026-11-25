#!/usr/bin/env python3
"""files/usr/lib/mowbite/bluesleep-lock on a stand-in folder in a user and mount namespace: the entries lose every
mode bit, an empty read-only tmpfs hides them, a second run and a missing folder change nothing.
  python3 -I platform/tron/rootfs/test_bluesleep_lock.py
"""
import os
import shutil
import subprocess
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, "files", "usr", "lib", "mowbite", "bluesleep-lock")


def in_namespace(script, *args):
    return subprocess.run(["unshare", "-rm", "sh", "-c", script, "sh", *args],
                          capture_output=True, text=True, timeout=20)


class TestBluesleepLock(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil.which("unshare") or in_namespace("true").returncode != 0:
            raise unittest.SkipTest("no user namespaces here")

    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.sleep = os.path.join(self.dir.name, "sleep")
        os.mkdir(self.sleep)
        for name in ("btwrite", "lpm", "powerupkey"):
            with open(os.path.join(self.sleep, name), "w") as f:
                f.write("1\n")

    def tearDown(self):
        os.chmod(self.sleep, 0o755)
        for name in os.listdir(self.sleep):
            os.chmod(os.path.join(self.sleep, name), 0o644)
        self.dir.cleanup()

    def test_hides_the_entries_and_locks_them(self):
        r = in_namespace('"$1" "$2" && "$1" "$2" && ls -A "$2" | wc -l && '
                         'if touch "$2/x" 2>/dev/null; then echo writable; fi && '
                         'grep -c " $2 tmpfs ro," /proc/self/mounts',
                         TOOL, self.sleep)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(r.stdout.split(), ["0", "1"])
        # Outside the namespace no tmpfs: the modes show what any other proc mount keeps.
        self.assertEqual(os.stat(self.sleep).st_mode & 0o7777, 0)
        os.chmod(self.sleep, 0o700)
        for name in ("btwrite", "lpm", "powerupkey"):
            self.assertEqual(os.stat(os.path.join(self.sleep, name)).st_mode & 0o7777, 0, name)

    def test_missing_folder_changes_nothing(self):
        r = in_namespace('"$1" "$2" && grep -c tmpfs /proc/self/mounts || true',
                         TOOL, os.path.join(self.dir.name, "none"))
        self.assertEqual(r.returncode, 0, r.stderr)
        before = in_namespace("grep -c tmpfs /proc/self/mounts || true").stdout
        self.assertEqual(r.stdout, before)


if __name__ == "__main__":
    unittest.main()
