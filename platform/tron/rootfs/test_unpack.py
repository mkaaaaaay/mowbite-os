#!/usr/bin/env python3
"""files/usr/lib/mowbite/unpack, the first-boot unit for the pack of the installer.
  python3 -I platform/tron/rootfs/test_unpack.py
"""
import hashlib
import importlib.machinery
import importlib.util
import io
import os
import tarfile
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
_loader = importlib.machinery.SourceFileLoader("unpack", os.path.join(HERE, "files/usr/lib/mowbite/unpack"))
_spec = importlib.util.spec_from_loader("unpack", _loader)
up = importlib.util.module_from_spec(_spec)
_loader.exec_module(up)

SIZE = 1 << 20


def archive(entries):
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w", format=tarfile.USTAR_FORMAT) as t:
        for name, data, mode in entries:
            info = tarfile.TarInfo(name)
            info.size = len(data)
            info.mode = mode
            t.addfile(info, io.BytesIO(data))
    return buf.getvalue()


def pack(data, sha=None):
    head = up.MAGIC + f"len {len(data)}\nsha256 {sha or hashlib.sha256(data).hexdigest()}\n".encode()
    out = head.ljust(up.BLOCK, b"\0") + data
    return out.ljust(SIZE, b"\0")


class TestUnpack(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.root = os.path.join(self.dir.name, "root")
        os.mkdir(self.root)
        self.file = os.path.join(self.dir.name, "pack")

    def tearDown(self):
        self.dir.cleanup()

    def run_on(self, content):
        with open(self.file, "wb") as f:
            f.write(content)
        self.assertEqual(up.main(self.file, self.root), 0)
        with open(self.file, "rb") as f:
            return f.read()

    def test_a_good_pack_is_unpacked_and_cleared(self):
        tar = archive([
            ("etc/NetworkManager/system-connections/mowbite-wifi-1.nmconnection", b"psk=x\n", 0o600),
            ("usr/lib/modules/5.10.209/a.ko", b"ko", 0o644),
            ("userdata/mowbite-update/inbox/os.bundle", b"b", 0o644),
        ])
        after = self.run_on(pack(tar))
        wifi = os.path.join(self.root, "etc/NetworkManager/system-connections/mowbite-wifi-1.nmconnection")
        with open(wifi, "rb") as f:
            self.assertEqual(f.read(), b"psk=x\n")
        self.assertEqual(os.stat(wifi).st_mode & 0o777, 0o600)
        self.assertTrue(os.path.exists(os.path.join(self.root, "userdata/mowbite-update/inbox/os.bundle")))
        self.assertEqual(len(after), SIZE)
        self.assertTrue(after.startswith(up.EMPTY))
        self.assertEqual(after[len(up.EMPTY):], bytes(SIZE - len(up.EMPTY)))

    def test_the_empty_mark_does_nothing(self):
        content = up.EMPTY.ljust(SIZE, b"\0")
        self.assertEqual(self.run_on(content), content)
        self.assertEqual(os.listdir(self.root), [])

    def test_bad_packs_stay_and_unpack_nothing(self):
        tar = archive([("etc/hostname", b"mowbite-ab12\n", 0o644)])
        outside = archive([("root/.ssh/authorized_keys", b"k", 0o600)])
        for content in [
            pack(tar, sha="0" * 64),
            pack(outside),
            pack(tar)[:SIZE // 2].replace(b"len ", b"len 9", 1).ljust(SIZE, b"\0"),
            b"something else".ljust(SIZE, b"\0"),
            up.MAGIC.ljust(SIZE, b"\0"),
        ]:
            self.assertEqual(self.run_on(content), content)
            self.assertEqual(os.listdir(self.root), [])


if __name__ == "__main__":
    unittest.main()
