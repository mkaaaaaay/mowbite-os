"""A signed bundle: a root-owned copy of it, its signature, its bundle.json and what it needs before a switch.

A bundle is a tar.gz of one directory <revision>/ with bundle.json (format 2: component, revision, commit, serial,
base, min_app, runtime_packages) beside what the component runs. Only the copy in the stage folder is checked and
unpacked: the file in the inbox may change after its signature was checked. What comes out of a bundle is never run
as root.
"""
import json
import os
import re
import shutil
import stat
import subprocess
import tarfile

from mowbite_host import Refused

FORMAT = 2
MAX_BUNDLE = 300 * 1024 * 1024
MAX_SIGNATURE = 64 * 1024
# What the unpacked bundle may take of the free space beside it.
SPACE_MARGIN = 100 * 1024 * 1024
PRINCIPAL = "mowbite"
REVISION = re.compile(r"[0-9a-f]{7,40}")
# The user that runs the OS; the libraries of a bundle are checked as this user.
USER = "mowbite"


def namespace(component):
    return f"mowbite-{component}"


def run(args, **kwargs):
    return subprocess.run(args, capture_output=True, **{"text": True, **kwargs})


# args as USER when this process is root, else as itself (tests).
def as_user(args):
    if os.geteuid() != 0:
        return args
    return ["setpriv", f"--reuid={USER}", f"--regid={USER}", "--init-groups", "--no-new-privs", "--", *args]


# Copies src (a regular file, not a link, at most limit bytes) to dst in a folder only root writes.
def copy_in(src, dst, limit):
    try:
        fd = os.open(src, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    except OSError as error:
        raise Refused("BAD_BUNDLE", f"{src} cannot be opened: {error.strerror}")
    with os.fdopen(fd, "rb") as source:
        info = os.fstat(source.fileno())
        if not stat.S_ISREG(info.st_mode):
            raise Refused("BAD_BUNDLE", f"{src} is no regular file")
        if info.st_size > limit:
            raise Refused("BAD_BUNDLE", f"{src} is larger than {limit // 1048576} MB")
        os.makedirs(os.path.dirname(dst), mode=0o700, exist_ok=True)
        with open(dst, "wb") as target:
            copied = 0
            while chunk := source.read(1 << 20):
                copied += len(chunk)
                if copied > limit:
                    raise Refused("BAD_BUNDLE", f"{src} grew past {limit // 1048576} MB while it was copied")
                target.write(chunk)
            target.flush()
            os.fsync(target.fileno())


def verify(path, signature, component, signers):
    with open(path, "rb") as data:
        result = run(["ssh-keygen", "-Y", "verify", "-f", signers, "-I", PRINCIPAL, "-n", namespace(component),
                      "-s", signature], stdin=data, text=False)
    if result.returncode != 0:
        raise Refused("BAD_SIGNATURE", f"bad signature: {result.stderr.decode(errors='replace').strip()}")


class Bundle:
    """A copy in the stage folder whose signature holds for its component."""

    def __init__(self, path, meta, size):
        self.path = path
        self.meta = meta
        # Bytes once unpacked.
        self.size = size

    @property
    def component(self):
        return self.meta["component"]

    @property
    def revision(self):
        return self.meta["revision"]

    @property
    def serial(self):
        return self.meta["serial"]


# Copies src and its signature into stage, checks the signature for component and reads bundle.json.
def stage(src, signature, component, stage_dir, signers):
    shutil.rmtree(stage_dir, ignore_errors=True)
    os.makedirs(stage_dir, mode=0o700)
    path = os.path.join(stage_dir, "bundle.tar.gz")
    sig = path + ".sig"
    copy_in(signature, sig, MAX_SIGNATURE)
    copy_in(src, path, MAX_BUNDLE)
    verify(path, sig, component, signers)
    meta, size = read_meta(path)
    if meta["component"] != component:
        raise Refused("BAD_BUNDLE", f"the bundle is of {meta['component']}, not of {component}")
    return Bundle(path, meta, size)


def _integer(meta, key, default=None):
    value = meta.get(key, default)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise Refused("BAD_BUNDLE", f"{key} in bundle.json is {value!r}, not a whole number")
    return value


# bundle.json of a signed bundle and the bytes it takes unpacked.
def read_meta(path):
    try:
        with tarfile.open(path) as tar:
            members = tar.getmembers()
            tops = {member.name.split("/", 1)[0] for member in members}
            if len(tops) != 1:
                raise Refused("BAD_BUNDLE", f"the bundle has to hold one directory, it holds {sorted(tops)}")
            top = tops.pop()
            if any(not member.name.isprintable() for member in members):
                raise Refused("BAD_BUNDLE", "a name in the bundle has a control character")
            member = tar.extractfile(f"{top}/bundle.json")
            meta = json.load(member)
    except (tarfile.TarError, KeyError, OSError, ValueError, AttributeError) as error:
        raise Refused("BAD_BUNDLE", f"the bundle cannot be read: {error}")
    if not isinstance(meta, dict) or meta.get("format") != FORMAT:
        raise Refused("BAD_BUNDLE", f"bundle format {meta.get('format') if isinstance(meta, dict) else '?'}, "
                                    f"this host reads {FORMAT}")
    if meta.get("revision") != top or not REVISION.fullmatch(top):
        raise Refused("BAD_BUNDLE", f"revision {meta.get('revision')} does not match the directory {top}")
    if not isinstance(meta.get("component"), str) or not isinstance(meta.get("runtime_packages", []), list):
        raise Refused("BAD_BUNDLE", "component or runtime_packages in bundle.json are missing or wrong")
    if _integer(meta, "serial") == 0:
        raise Refused("BAD_BUNDLE", "serial in bundle.json is 0")
    _integer(meta, "base", 0)
    return meta, sum(member.size for member in members)


# The version of the rootfs or base image at path, 0 without one.
def base_version(path):
    try:
        with open(path) as f:
            return int(f.read().strip())
    except (OSError, ValueError):
        return 0


def missing_packages(packages):
    missing = []
    for package in packages:
        result = run(["dpkg-query", "-W", "-f", "${Status}", str(package)])
        if result.returncode != 0 or result.stdout.strip() != "install ok installed":
            missing.append(str(package))
    return missing


# Refused BASE_TOO_OLD when the rootfs is older than the bundle needs or misses a package it runs on.
def check_base(bundle, base_file):
    have = base_version(base_file)
    need = bundle.meta.get("base", 0)
    if need > have:
        raise Refused("BASE_TOO_OLD", f"the bundle needs base {need}, this mower has {have}")
    missing = missing_packages(bundle.meta.get("runtime_packages", []))
    if missing:
        raise Refused("BASE_TOO_OLD", f"runtime packages missing: {' '.join(missing)}")


def check_space(bundle, folder):
    free = shutil.disk_usage(folder).free
    if bundle.size + SPACE_MARGIN > free:
        raise Refused("NO_SPACE", f"the bundle needs {(bundle.size + SPACE_MARGIN) // 1048576} MB in {folder}, "
                                  f"{free // 1048576} MB are free")


# Unpacks into <opt>/.<revision>.partial, then renames it to <opt>/<revision>; owned by root, writable only by it.
def unpack(bundle, opt):
    revision = bundle.revision
    partial = os.path.join(opt, f".{revision}.partial")
    shutil.rmtree(partial, ignore_errors=True)
    os.makedirs(partial)
    try:
        with tarfile.open(bundle.path) as tar:
            tar.extractall(partial, filter="data")
    except (tarfile.TarError, OSError) as error:
        shutil.rmtree(partial, ignore_errors=True)
        raise Refused("BAD_BUNDLE", f"the bundle cannot be unpacked: {error}")
    os.sync()
    return partial


def _is_elf(path):
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    except OSError:
        return False
    with os.fdopen(fd, "rb") as f:
        return stat.S_ISREG(os.fstat(f.fileno()).st_mode) and f.read(4) == b"\x7fELF"


# Sources os-setup for the tree and runs ldd on each file named on stdin, as USER.
LDD_SCRIPT = 'set -eu; source "$1" "$2"; while IFS= read -r f; do echo "== $f"; ldd "$f" 2>&1 || true; done'


# Files of the tree at root whose libraries are not found in the environment of the OS: [text].
def unresolved_libraries(root, os_setup, env):
    files = []
    for folder, _, names in os.walk(root):
        files += [os.path.join(folder, name) for name in names if _is_elf(os.path.join(folder, name))]
    if not files:
        return []
    result = run(as_user(["bash", "-c", LDD_SCRIPT, "ldd-check", os_setup, root]), input="\n".join(files) + "\n",
                 env=env, timeout=600)
    if result.returncode != 0:
        return [f"the environment of {root} cannot be set up: {result.stderr.strip()}"]
    problems, path, bad = [], None, []

    def close():
        if path and bad:
            problems.append(f"{os.path.relpath(path, root)}: {'; '.join(bad)}")
    for line in result.stdout.splitlines():
        if line.startswith("== "):
            close()
            path, bad = line[3:], []
        elif "not found" in line or "error while loading" in line:
            bad.append(line.strip())
    close()
    return problems
