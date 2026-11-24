#!/usr/bin/env python3
"""base_packages.py and base-packages.txt of the repo.
  python3 -I platform/tron/rootfs/test_base_packages.py
"""
import hashlib
import os
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import base_packages as bp  # noqa: E402

STATUS = """Package: libc6
Status: install ok installed
Version: 2.39

Package: e2fsprogs
Status: deinstall ok config-files

Package: ros-jazzy-rclcpp
Status: install ok installed
"""


class TestBasePackages(unittest.TestCase):
    def test_only_installed_packages_count(self):
        self.assertEqual(bp.installed(STATUS), {"libc6", "ros-jazzy-rclcpp"})

    def test_written_file_reads_back(self):
        text = bp.write(STATUS, 1, b"libc6\n")
        self.assertEqual(text.splitlines()[1:], ["libc6", "ros-jazzy-rclcpp"])
        base, sha, names = bp.read(text)
        self.assertEqual((base, sha, names), (1, hashlib.sha256(b"libc6\n").hexdigest(), {"libc6", "ros-jazzy-rclcpp"}))

    def test_a_bundle_names_what_its_base_lacks(self):
        text = bp.write(STATUS, 1, b"")
        self.assertEqual(bp.missing(["libc6", "ros-jazzy-tf2-ros", "ros-jazzy-rclcpp"], text, 1), ["ros-jazzy-tf2-ros"])
        self.assertEqual(bp.missing(["libc6"], text, 1), [])

    def test_a_list_of_another_base_is_refused(self):
        with self.assertRaises(ValueError):
            bp.missing(["libc6"], bp.write(STATUS, 1, b""), 2)
        with self.assertRaises(ValueError):
            bp.read("libc6\n")

    def test_same_compares_the_tar_with_the_list(self):
        with tempfile.TemporaryDirectory() as tmp:
            status, listed = os.path.join(tmp, "status"), os.path.join(tmp, "list")
            with open(status, "w") as f:
                f.write(STATUS)
            with open(listed, "w") as f:
                f.write(bp.write(STATUS, 1, b""))
            run = [sys.executable, "-I", os.path.join(HERE, "base_packages.py"), "same", status, listed]
            self.assertEqual(subprocess.run(run, capture_output=True).returncode, 0)
            with open(listed, "a") as f:
                f.write("dnsmasq-base\n")
            result = subprocess.run(run, capture_output=True, text=True)
            self.assertEqual(result.returncode, 1)
            self.assertIn("dnsmasq-base", result.stderr)


class TestRepoList(unittest.TestCase):
    """The list of the repo belongs to its packages.txt and the version in files/: build a base after changing either."""

    def test_the_list_matches_packages_txt_and_the_base_version(self):
        with open(os.path.join(HERE, "base-packages.txt")) as f:
            base, sha, names = bp.read(f.read())
        with open(os.path.join(HERE, "packages.txt"), "rb") as f:
            self.assertEqual(sha, hashlib.sha256(f.read()).hexdigest(), "packages.txt changed: build a new base")
        with open(os.path.join(HERE, "files", "usr", "lib", "mowbite", "base")) as f:
            self.assertEqual(base, int(f.read().strip()))
        with open(os.path.join(HERE, "packages.txt")) as f:
            wanted = {line.split("#")[0].strip() for line in f} - {""}
        self.assertEqual(wanted - names, set(), "a package of packages.txt is not in the base")


if __name__ == "__main__":
    unittest.main()
