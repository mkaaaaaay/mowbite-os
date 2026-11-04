"""repack_gzip.py against a small archive in the layout of docker save: gzip layers whose contents match the diff_ids,
the same config and tags, and refusal of a layer that does not match its name.

  python3 -m unittest discover -s tools/yf/test
"""
import gzip
import hashlib
import io
import json
import os
import subprocess
import sys
import tarfile
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
REPACK = os.path.join(HERE, "..", "repack_gzip.py")


def layer(files):
    out = io.BytesIO()
    with tarfile.open(fileobj=out, mode="w", format=tarfile.PAX_FORMAT) as tar:
        for name, data in files.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
    return out.getvalue()


def digest(data):
    return hashlib.sha256(data).hexdigest()


def saved_archive(path, layers, break_layer=False):
    """An archive as docker save writes it (OCI layout, plain layers), gzipped as a whole."""
    blobs = {digest(raw): raw for raw in layers}
    config = json.dumps({"architecture": "arm64", "os": "linux",
                         "rootfs": {"type": "layers", "diff_ids": [f"sha256:{digest(raw)}" for raw in layers]}}).encode()
    blobs[digest(config)] = config
    manifest = json.dumps({"schemaVersion": 2, "mediaType": "application/vnd.oci.image.manifest.v1+json",
                           "config": {"mediaType": "application/vnd.oci.image.config.v1+json",
                                      "digest": f"sha256:{digest(config)}", "size": len(config)},
                           "layers": [{"mediaType": "application/vnd.oci.image.layer.v1.tar",
                                       "digest": f"sha256:{digest(raw)}", "size": len(raw)} for raw in layers]}).encode()
    blobs[digest(manifest)] = manifest
    index = json.dumps({"schemaVersion": 2, "manifests": [
        {"mediaType": "application/vnd.oci.image.manifest.v1+json", "digest": f"sha256:{digest(manifest)}",
         "size": len(manifest), "annotations": {"io.containerd.image.name": "docker.io/library/m:1"}}]}).encode()
    legacy = json.dumps([{"Config": f"blobs/sha256/{digest(config)}", "RepoTags": ["m:1"],
                          "Layers": [f"blobs/sha256/{digest(raw)}" for raw in layers]}]).encode()
    files = {"oci-layout": b'{"imageLayoutVersion":"1.0.0"}', "index.json": index, "manifest.json": legacy}
    for d, raw in blobs.items():
        files[f"blobs/sha256/{d}"] = raw + (b"x" if break_layer and raw is layers[0] else b"")
    with gzip.open(path, "wb") as gz, tarfile.open(fileobj=gz, mode="w", format=tarfile.PAX_FORMAT) as tar:
        for name, data in files.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
    return config


class RepackTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="repack_")
        self.layers = [layer({"etc/a": b"a" * 5000}), layer({"usr/b": os.urandom(3000), "usr/c": b"c" * 70000})]

    def run_repack(self, src, out):
        return subprocess.run([sys.executable, "-I", REPACK, src, out], capture_output=True, text=True)

    def test_gzip_layers_with_the_same_contents(self):
        src = os.path.join(self.dir, "saved.tar.gz")
        config = saved_archive(src, self.layers)
        out = os.path.join(self.dir, "out")
        done = self.run_repack(src, out)
        self.assertEqual(done.returncode, 0, done.stderr)
        self.assertIn("2 layers verified", done.stdout)
        with tarfile.open(os.path.join(out, "image.tar")) as tar:
            index = json.loads(tar.extractfile("index.json").read())
            self.assertEqual(index["manifests"][0]["annotations"]["io.containerd.image.name"],
                             "docker.io/library/m:1")
            manifest = json.loads(tar.extractfile(f"blobs/sha256/{index['manifests'][0]['digest'][7:]}").read())
            self.assertEqual(manifest["config"]["digest"], f"sha256:{digest(config)}")
            self.assertEqual(tar.extractfile(f"blobs/sha256/{digest(config)}").read(), config)
            for entry, raw in zip(manifest["layers"], self.layers):
                self.assertEqual(entry["mediaType"], "application/vnd.oci.image.layer.v1.tar+gzip")
                blob = tar.extractfile(f"blobs/sha256/{entry['digest'][7:]}").read()
                self.assertEqual(len(blob), entry["size"])
                self.assertEqual(gzip.decompress(blob), raw)
            self.assertEqual(json.loads(tar.extractfile("manifest.json").read())[0]["RepoTags"], ["m:1"])

    def test_refuses_a_layer_that_does_not_match_its_name(self):
        src = os.path.join(self.dir, "broken.tar.gz")
        saved_archive(src, self.layers, break_layer=True)
        done = self.run_repack(src, os.path.join(self.dir, "out"))
        self.assertNotEqual(done.returncode, 0)
        self.assertIn("content has sha256", done.stderr)


if __name__ == "__main__":
    unittest.main()
