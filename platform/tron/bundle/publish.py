"""Publishes signed bundles for the updater of mowbite-host (host/mowbite_host/net.py): each bundle as release
<component>-<revision> of the release repo with its .sig, then index.json with a new serial, signed for mowbite-index.

  python3 -I publish.py [--stable] <private key> <bundle.tar.gz>...
  python3 -I publish.py --promote <private key> <component>-<revision>

preview by default, --stable and --promote (preview to stable) only on the owner's word. Each bundle of any component
has to hold for allowed_signers of the rootfs (or MOWBITE_SIGNERS); one with a model or the NPU runtime of the vendor
in it, or a revision the index names already, is refused. Signing (sign-bundle.sh, with the test gate of the
component) comes before. One publish at a time on this PC (LOCK). The index keeps the newest KEEP entries per
component; releases stay. Needs gh logged in with write access to the repo; GH runs another gh (tests).
"""
import argparse
import fcntl
import hashlib
import json
import os
import re
import subprocess
import sys
import tarfile
import tempfile
import time
import urllib.parse

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "host"))
from mowbite_host import Refused, bundle, net, origin  # noqa: E402

REPO = "/".join(urllib.parse.urlsplit(net.BASE).path.strip("/").split("/")[:2])
SIGNERS = os.environ.get("MOWBITE_SIGNERS") or os.path.join(HERE, "..", "rootfs", "files", "etc", "mowbite",
                                                            "allowed_signers")
INDEX_TAG = net.INDEX_URL[len(net.BASE):].split("/")[0]
KEEP = 10
LOCK = os.path.expanduser(os.environ.get("MOWBITE_PUBLISH_LOCK", "~/.local/state/mowbite-sign/publish.lock"))
# Never published: models and the NPU runtime of the vendor.
VENDOR = re.compile(r"\.rknn$|librknnrt", re.IGNORECASE)


class Stop(Exception):
    pass


def gh(*args, check=True):
    result = subprocess.run([os.environ.get("GH", "gh"), *args, "-R", REPO], capture_output=True, text=True)
    if check and result.returncode != 0:
        raise Stop(f"gh {' '.join(args)}: {result.stderr.strip()}")
    return result


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(1 << 20):
            digest.update(chunk)
    return digest.hexdigest()


# The index of the release repo, an empty one before the first.
def current_index(work):
    if gh("release", "view", INDEX_TAG, check=False).returncode != 0:
        return {"schema": net.SCHEMA, "serial": 0, "bundles": []}
    gh("release", "download", INDEX_TAG, "-p", origin.INDEX, "-p", origin.INDEX + ".sig", "-D", work, "--clobber")
    path = os.path.join(work, origin.INDEX)
    try:
        bundle.verify(path, path + ".sig", "index", SIGNERS)
    except Refused as why:
        raise Stop(f"the index of {REPO} does not hold: {why}")
    with open(path) as f:
        return json.load(f)


# The index entry of a signed bundle.
def entry_of(path, channel):
    try:
        meta, _ = bundle.read_meta(path)
        bundle.verify(path, path + ".sig", meta["component"], SIGNERS)
    except (Refused, KeyError) as why:
        raise Stop(f"{path}: {why}")
    component, size = meta["component"], os.path.getsize(path)
    if component not in bundle.MAX_BUNDLE or size > bundle.MAX_BUNDLE[component]:
        raise Stop(f"{path}: component {component}, {size} bytes: no bundle the updater takes")
    with tarfile.open(path) as tar:
        vendor = [member for member in tar.getnames() if VENDOR.search(member)]
    if vendor:
        raise Stop(f"{path} holds files of the vendor that are never published: {', '.join(vendor[:5])}")
    name = os.path.basename(path)
    if name != f"mowbite-{component}-{meta['revision']}.tar.gz":
        raise Stop(f"{path}: the name is not mowbite-{component}-{meta['revision']}.tar.gz")
    return {"component": component, "revision": meta["revision"], "serial": meta["serial"],
            "date": meta.get("date", meta["serial"]), "size": size, "sha256": sha256(path), "file": name,
            "url": f"{component}-{meta['revision']}/{name}", "channel": channel,
            **{key: meta[key] for key in origin.FIELDS if key in meta}}


def keep_newest(entries):
    kept = []
    for component in sorted({e["component"] for e in entries}):
        mine = sorted((e for e in entries if e["component"] == component), key=lambda e: e["serial"], reverse=True)
        kept += mine[:KEEP]
    return kept


def write_index(work, index, key):
    index = {**index, "schema": net.SCHEMA, "serial": max(index["serial"] + 1, int(time.time())),
             "date": int(time.time()), "bundles": keep_newest(index["bundles"])}
    path = os.path.join(work, "new", origin.INDEX)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump(index, f, indent=1)
        f.write("\n")
    if os.path.getsize(path) > origin.MAX_INDEX:
        raise Stop(f"the index has {os.path.getsize(path)} bytes, more than the updater takes")
    subprocess.run(["ssh-keygen", "-q", "-Y", "sign", "-f", key, "-n", bundle.namespace("index"), path],
                   check=True, stdin=subprocess.DEVNULL)
    try:
        bundle.verify(path, path + ".sig", "index", SIGNERS)
    except Refused as why:
        raise Stop(f"the new index does not hold for allowed_signers: {why}")
    return path, index["serial"]


def upload_index(path, fresh):
    files = (path + ".sig", path)
    if fresh:
        gh("release", "create", INDEX_TAG, *files, "--title", INDEX_TAG, "--notes", "Index of the releases",
           "--latest=false")
    else:
        gh("release", "upload", INDEX_TAG, *files, "--clobber")


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--stable", action="store_true")
    parser.add_argument("--promote", action="store_true")
    parser.add_argument("key")
    parser.add_argument("items", nargs="+")
    args = parser.parse_args(argv)
    os.makedirs(os.path.dirname(LOCK), exist_ok=True)
    try:
        with open(LOCK, "w") as lock, tempfile.TemporaryDirectory(prefix="mowbite-publish-") as work:
            fcntl.flock(lock, fcntl.LOCK_EX)
            index = current_index(work)
            fresh = index["serial"] == 0 and not index["bundles"]
            named = {f"{e['component']}-{e['revision']}": e for e in index["bundles"]}
            if args.promote:
                for tag in args.items:
                    if tag not in named:
                        raise Stop(f"{tag} is not in the index")
                    named[tag]["channel"] = "stable"
                    gh("release", "edit", tag, "--prerelease=false", "--latest=false")
            else:
                channel = "stable" if args.stable else "preview"
                entries = [entry_of(path, channel) for path in args.items]
                for path, entry in zip(args.items, entries):
                    tag = entry["url"].split("/")[0]
                    if tag in named:
                        raise Stop(f"{tag} is in the index already")
                    flags = ["--prerelease"] if channel == "preview" else []
                    gh("release", "create", tag, path + ".sig", path, "--title", tag, "--notes",
                       f"{entry['component']} {entry['revision']}, serial {entry['serial']}, {channel}",
                       "--latest=false", *flags)
                    index["bundles"].append(entry)
            path, serial = write_index(work, index, args.key)
            upload_index(path, fresh)
            print(f"index serial {serial}: {', '.join(args.items)}")
    except Stop as why:
        print(f"publish: {why}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
