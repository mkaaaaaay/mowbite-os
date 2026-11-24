#!/usr/bin/env python3
"""files/usr/lib/mowbite/unpack, the first-boot unit for the pack of the installer.
  python3 -I platform/tron/rootfs/test_unpack.py
"""
import hashlib
import importlib.machinery
import importlib.util
import io
import os
import sys
import tarfile
import tempfile
import unittest

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
_loader = importlib.machinery.SourceFileLoader("unpack", os.path.join(HERE, "files/usr/lib/mowbite/unpack"))
_spec = importlib.util.spec_from_loader("unpack", _loader)
up = importlib.util.module_from_spec(_spec)
_loader.exec_module(up)

SIZE = 1 << 20


def archive(entries):
    """entries: (name, data, mode), (name, None, link target) for a symlink, (name, None, mode) for a folder,
    (name, "hard", target) for a hard link."""
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w", format=tarfile.USTAR_FORMAT) as t:
        for name, data, extra in entries:
            info = tarfile.TarInfo(name)
            if data is None and isinstance(extra, int):
                info.type, info.mode = tarfile.DIRTYPE, extra
                t.addfile(info)
                continue
            if data is None or data == "hard":
                info.type, info.linkname, info.mode = tarfile.SYMTYPE if data is None else tarfile.LNKTYPE, extra, 0o777
                t.addfile(info)
                continue
            info.size = len(data)
            info.mode = extra
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

    def run_on(self, content, stage="all"):
        with open(self.file, "wb") as f:
            f.write(content)
        self.assertEqual(up.main(stage, self.file, self.root), 0)
        if not os.path.exists(self.file):
            return None
        with open(self.file, "rb") as f:
            return f.read()

    def test_a_good_pack_is_unpacked_and_removed(self):
        tar = archive([
            ("etc/hostname", b"mowbite-ab12\n", 0o644),
            ("etc/mowbite/install-id", b"0123456789abcdef0123456789abcdef\n", 0o600),
            ("usr/lib/modules/5.10.209", None, 0o755),
            ("etc/NetworkManager/system-connections/mowbite-wifi-1.nmconnection", b"psk=x\n", 0o600),
            ("usr/lib/modules/5.10.209/a.ko", b"ko", 0o644),
            ("userdata/mowbite-update/inbox/os.bundle", b"b", 0o644),
            ("usr/lib/aarch64-linux-gnu/librga.so.2", None, "librga.so.2.1.0"),
        ])
        after = self.run_on(pack(tar))
        self.assertEqual(os.readlink(os.path.join(self.root, "usr/lib/aarch64-linux-gnu/librga.so.2")),
                         "librga.so.2.1.0")
        wifi = os.path.join(self.root, "etc/NetworkManager/system-connections/mowbite-wifi-1.nmconnection")
        with open(wifi, "rb") as f:
            self.assertEqual(f.read(), b"psk=x\n")
        self.assertEqual(os.stat(wifi).st_mode & 0o777, 0o600)
        self.assertTrue(os.path.exists(os.path.join(self.root, "userdata/mowbite-update/inbox/os.bundle")))
        self.assertEqual(os.stat(os.path.join(self.root, "etc/mowbite/install-id")).st_mode & 0o777, 0o600)
        self.assertIsNone(after, "the pack file stays")

    def test_the_system_stage_leaves_userdata_and_the_pack(self):
        tar = archive([
            ("usr/lib/modules/5.10.209/a.ko", b"ko", 0o644),
            ("userdata/mowbite-update/inbox/os.bundle", b"b", 0o644),
        ])
        content = pack(tar)
        self.assertEqual(self.run_on(content, "system"), content)
        self.assertTrue(os.path.exists(os.path.join(self.root, "usr/lib/modules/5.10.209/a.ko")))
        self.assertFalse(os.path.exists(os.path.join(self.root, "userdata")))
        self.assertIsNone(self.run_on(content))
        self.assertTrue(os.path.exists(os.path.join(self.root, "userdata/mowbite-update/inbox/os.bundle")))

    def test_an_unknown_stage_does_nothing(self):
        content = pack(archive([("etc/hostname", b"mowbite-ab12\n", 0o644)]))
        with open(self.file, "wb") as f:
            f.write(content)
        self.assertEqual(up.main("rest", self.file, self.root), 2)
        self.assertEqual(os.listdir(self.root), [])

    def test_the_empty_mark_does_nothing(self):
        content = up.EMPTY.ljust(SIZE, b"\0")
        self.assertEqual(self.run_on(content), content)
        self.assertEqual(os.listdir(self.root), [])

    def test_bad_packs_stay_and_unpack_nothing(self):
        tar = archive([("etc/hostname", b"mowbite-ab12\n", 0o644)])
        outside = archive([("root/.ssh/authorized_keys", b"k", 0o600)])
        # The good entry comes first: a refused link later in the pack must not leave it half unpacked.
        absolute = archive([("etc/hostname", b"mowbite-ab12\n", 0o644),
                            ("usr/lib/aarch64-linux-gnu/librga.so", None, "/usr/lib/aarch64-linux-gnu/librga.so.2")])
        escaping = archive([("etc/hostname", b"mowbite-ab12\n", 0o644), ("etc/x", None, "../../../root")])
        # A link inside a vendor part that leads out of the parts, and a file written through it.
        through = archive([("usr/lib/kernel/driver/x", None, "../../../../root"),
                           ("usr/lib/kernel/driver/x/.ssh/authorized_keys", b"k", 0o600)])
        settings = archive([("etc/mowbite/os.conf", b"MOWBITE_HARDWARE=true\n", 0o644)])
        shadow = archive([("etc/shadow", b"root::0:0\n", 0o600)])
        folder = archive([("etc", None, 0o755), ("etc/hostname", b"mowbite-ab12\n", 0o644)])
        deeper = archive([("etc/NetworkManager/system-connections/a/b.nmconnection", b"x", 0o600)])
        hard = archive([("usr/lib/modules/5.10.209/a.ko", b"ko", 0o644),
                        ("usr/lib/modules/5.10.209/b.ko", "hard", "usr/lib/modules/5.10.209/a.ko")])
        for content in [
            pack(through),
            pack(settings),
            pack(shadow),
            pack(folder),
            pack(deeper),
            pack(hard),
            pack(tar, sha="0" * 64),
            pack(outside),
            pack(absolute),
            pack(escaping),
            pack(tar)[:SIZE // 2].replace(b"len ", b"len 9", 1).ljust(SIZE, b"\0"),
            b"something else".ljust(SIZE, b"\0"),
            up.MAGIC.ljust(SIZE, b"\0"),
        ]:
            for stage in ("system", "all"):
                self.assertEqual(self.run_on(content, stage), content)
                self.assertEqual(os.listdir(self.root), [])


if __name__ == "__main__":
    unittest.main()
