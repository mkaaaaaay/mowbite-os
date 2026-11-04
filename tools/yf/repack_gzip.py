"""Repacks a docker-save archive (OCI layout, uncompressed layers, gzipped as a whole) into an OCI archive with gzip
layers, so that the containerd image store of the mower keeps the compressed blobs instead of the uncompressed ones
(with the archive of docker save an image took twice its size there). Config and layer contents stay the same (the
diff_ids of the config are checked against the new layers at the end). Only the standard library.

  python3 -I repack_gzip.py <in.tar.gz> <out-dir>    ->  <out-dir>/image.tar for docker load
"""
import gzip
import hashlib
import json
import os
import sys
import tarfile

SMALL = 1 << 20
CHUNK = 1 << 20


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def read_small(path):
    small = {}
    with tarfile.open(path, "r|gz") as tar:
        for m in tar:
            if m.isfile() and m.size < SMALL:
                small[m.name] = tar.extractfile(m).read()
    return small


def compress_layers(path, layers, blob_dir):
    """layers: digest hex -> None; fills in (gz hex, gz size) for each, checking the input against its name."""
    wanted = {f"blobs/sha256/{d}": d for d in layers}
    with tarfile.open(path, "r|gz") as tar:
        for m in tar:
            digest = wanted.get(m.name)
            if digest is None or not m.isfile():
                continue
            src = tar.extractfile(m)
            tmp = os.path.join(blob_dir, f".tmp-{digest}")
            h_in, h_out = hashlib.sha256(), hashlib.sha256()
            with open(tmp, "wb") as raw:
                class Tee:
                    def write(self, b):
                        h_out.update(b)
                        return raw.write(b)

                    def flush(self):
                        raw.flush()
                with gzip.GzipFile(filename="", mode="wb", compresslevel=6, fileobj=Tee(), mtime=0) as gz:
                    while True:
                        chunk = src.read(CHUNK)
                        if not chunk:
                            break
                        h_in.update(chunk)
                        gz.write(chunk)
            if h_in.hexdigest() != digest:
                sys.exit(f"layer {digest}: content has sha256 {h_in.hexdigest()}")
            out = h_out.hexdigest()
            os.replace(tmp, os.path.join(blob_dir, out))
            layers[digest] = (out, os.path.getsize(os.path.join(blob_dir, out)))
    missing = [d for d, v in layers.items() if v is None]
    if missing:
        sys.exit(f"layers not found: {missing}")


def write_blob(blob_dir, data):
    digest = sha256_bytes(data)
    with open(os.path.join(blob_dir, digest), "wb") as f:
        f.write(data)
    return digest


def verify(tar_path):
    """Every blob matches its name; the gunzipped layers match the diff_ids of the config, in order."""
    with tarfile.open(tar_path, "r:") as tar:
        names = tar.getnames()
        index = json.loads(tar.extractfile("index.json").read())
        man_d = index["manifests"][0]["digest"].split(":")[1]
        manifest = json.loads(tar.extractfile(f"blobs/sha256/{man_d}").read())
        config_d = manifest["config"]["digest"].split(":")[1]
        config = json.loads(tar.extractfile(f"blobs/sha256/{config_d}").read())
        for name in names:
            if not name.startswith("blobs/sha256/"):
                continue
            h = hashlib.sha256()
            f = tar.extractfile(name)
            for chunk in iter(lambda: f.read(CHUNK), b""):
                h.update(chunk)
            if h.hexdigest() != name.rsplit("/", 1)[1]:
                sys.exit(f"verify: {name} has sha256 {h.hexdigest()}")
        diff_ids = config["rootfs"]["diff_ids"]
        if len(diff_ids) != len(manifest["layers"]):
            sys.exit("verify: number of layers differs from the diff_ids")
        for layer, diff_id in zip(manifest["layers"], diff_ids):
            h = hashlib.sha256()
            with gzip.GzipFile(fileobj=tar.extractfile(f"blobs/sha256/{layer['digest'].split(':')[1]}")) as gz:
                for chunk in iter(lambda: gz.read(CHUNK), b""):
                    h.update(chunk)
            if f"sha256:{h.hexdigest()}" != diff_id:
                sys.exit(f"verify: layer {layer['digest']} unpacks to {h.hexdigest()}, config says {diff_id}")
    return len(diff_ids)


def main():
    src, out_dir = sys.argv[1], sys.argv[2]
    os.makedirs(out_dir, exist_ok=False)
    blob_dir = os.path.join(out_dir, "blobs", "sha256")
    os.makedirs(blob_dir)
    small = read_small(src)
    index = json.loads(small["index.json"])
    if len(index["manifests"]) != 1:
        sys.exit("expected one manifest in index.json")
    entry = index["manifests"][0]
    man_d = entry["digest"].split(":")[1]
    manifest = json.loads(small[f"blobs/sha256/{man_d}"])
    config_d = manifest["config"]["digest"].split(":")[1]
    config_raw = small[f"blobs/sha256/{config_d}"]
    if sha256_bytes(config_raw) != config_d:
        sys.exit("config does not match its digest")
    layers = {l["digest"].split(":")[1]: None for l in manifest["layers"]}
    compress_layers(src, layers, blob_dir)
    write_blob(blob_dir, config_raw)
    new_manifest = dict(manifest)
    new_manifest["layers"] = [{"mediaType": "application/vnd.oci.image.layer.v1.tar+gzip",
                               "digest": f"sha256:{layers[l['digest'].split(':')[1]][0]}",
                               "size": layers[l["digest"].split(":")[1]][1]} for l in manifest["layers"]]
    man_raw = json.dumps(new_manifest, separators=(",", ":")).encode()
    new_man_d = write_blob(blob_dir, man_raw)
    new_entry = dict(entry, digest=f"sha256:{new_man_d}", size=len(man_raw))
    new_index = dict(index, manifests=[new_entry])
    tags = json.loads(small["manifest.json"])[0]["RepoTags"]
    legacy = [{"Config": f"blobs/sha256/{config_d}", "RepoTags": tags,
               "Layers": [f"blobs/sha256/{l['digest'].split(':')[1]}" for l in new_manifest["layers"]]}]
    files = {"index.json": json.dumps(new_index, separators=(",", ":")).encode(),
             "manifest.json": json.dumps(legacy, separators=(",", ":")).encode(),
             "oci-layout": small["oci-layout"]}
    tar_path = os.path.join(out_dir, "image.tar")
    with tarfile.open(tar_path, "x:", format=tarfile.PAX_FORMAT) as tar:
        tar.add(os.path.join(out_dir, "blobs"), arcname="blobs", recursive=False)
        tar.add(blob_dir, arcname="blobs/sha256", recursive=False)
        for digest in sorted(os.listdir(blob_dir)):
            tar.add(os.path.join(blob_dir, digest), arcname=f"blobs/sha256/{digest}")
        for name, data in files.items():
            path = os.path.join(out_dir, name)
            with open(path, "xb") as f:
                f.write(data)
            tar.add(path, arcname=name)
    count = verify(tar_path)
    print(f"tags {tags}, manifest sha256:{new_man_d}, config sha256:{config_d}, {count} layers verified")
    for l in new_manifest["layers"]:
        print(f"  {l['digest']} {l['size']}")


if __name__ == "__main__":
    main()
