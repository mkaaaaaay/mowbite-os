"""A signed bundle: a root-owned copy of it, its signature, its bundle.json and what it needs before a switch.

A bundle is a tar.gz of one directory <revision>/ with bundle.json (format 2: component, revision, commit, serial,
date) beside what the component brings: os with base, platforms (the mowers it is built for, any without it), api,
api_break, min_app, runtime_packages, planner_interface, planner_files_version and bundled beside install/, planner/,
app/ and tools/; planner with version, interface, files_version and requires beside planner/; app with version and
api {min, max, break} beside app/; host with base beside mowbite_host/. Only the copy in the stage folder is checked
and unpacked: the file in the inbox may change after its signature was checked. What comes out of a bundle is never
run as root, but the host itself, and a bundle of the planner, the app or the host holds no machine code.
"""
import json
import os
import re
import shutil
import stat
import subprocess
import sys
import tarfile

from mowbite_host import Refused, compat

FORMAT = 2
# The largest bundle of each component, checked before its signature.
MAX_BUNDLE = {"os": 300 * 1024 * 1024, "planner": 20 * 1024 * 1024, "app": 50 * 1024 * 1024,
              "host": 5 * 1024 * 1024}
# What a bundle of the component has to hold beside bundle.json.
NEEDS = {"planner": "planner/local_setup.bash", "app": "app/index.html", "host": "mowbite_host/__init__.py"}
MAX_SIGNATURE = 64 * 1024
# What the unpacked bundle may take of the free space beside it.
SPACE_MARGIN = 100 * 1024 * 1024
PRINCIPAL = "mowbite"
REVISION = re.compile(r"[0-9a-f]{7,40}")
# The reason of a bundle built for other platforms than the one of this mower.
PLATFORM_MISMATCH = "PLATFORM_MISMATCH"
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
    def revision(self):
        return self.meta["revision"]

    @property
    def serial(self):
        return self.meta["serial"]


def _copies(src, signature, stage_dir, limit):
    shutil.rmtree(stage_dir, ignore_errors=True)
    os.makedirs(stage_dir, mode=0o700)
    path = os.path.join(stage_dir, "bundle.tar.gz")
    copy_in(signature, path + ".sig", MAX_SIGNATURE)
    copy_in(src, path, limit)
    return path


def _checked(path, component):
    meta, size = read_meta(path)
    if meta["component"] != component:
        raise Refused("BAD_BUNDLE", f"the bundle is of {meta['component']}, not of {component}")
    return Bundle(path, meta, size)


# Copies src and its signature into stage, checks the signature for component and reads bundle.json.
def stage(src, signature, component, stage_dir, signers):
    if component not in MAX_BUNDLE:
        raise Refused("BAD_BUNDLE", f"no component {component}")
    path = _copies(src, signature, stage_dir, MAX_BUNDLE[component])
    verify(path, path + ".sig", component, signers)
    return _checked(path, component)


# As stage for a bundle given by hand: its component is the one whose namespace the signature holds for, nothing of
# the bundle is read before that.
def stage_any(src, signature, stage_dir, signers):
    path = _copies(src, signature, stage_dir, max(MAX_BUNDLE.values()))
    for component, limit in MAX_BUNDLE.items():
        try:
            verify(path, path + ".sig", component, signers)
        except Refused:
            continue
        if os.path.getsize(path) > limit:
            raise Refused("BAD_BUNDLE", f"{src} is larger than {limit // 1048576} MB")
        return _checked(path, component)
    raise Refused("BAD_SIGNATURE", f"the signature of {src} holds for no component")


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
    platforms = meta.get("platforms")
    if "platforms" in meta and (not isinstance(platforms, list) or not platforms or
                                not all(isinstance(name, str) and name for name in platforms)):
        raise Refused("BAD_BUNDLE", f"platforms in bundle.json is {platforms!r}, not a list of names")
    if _integer(meta, "serial") == 0:
        raise Refused("BAD_BUNDLE", "serial in bundle.json is 0")
    _integer(meta, "base", 0)
    _check_fields(meta, {member.name for member in members if member.isfile()})
    return meta, sum(member.size for member in members)


# What bundle.json of the planner and the app has to name, and the file the component is used through.
def _check_fields(meta, names):
    component = meta["component"]
    if component == "app":
        if compat.version_tuple(meta.get("version")) is None or not compat.valid_span(meta.get("api")):
            raise Refused("BAD_BUNDLE", "version or api {min, max, break} in bundle.json are missing or wrong")
    elif component == "planner":
        requires = meta.get("requires", {})
        if (not isinstance(meta.get("version"), str) or not compat.whole(meta.get("interface")) or
                not compat.whole(meta.get("files_version")) or not isinstance(requires, dict) or
                not all(isinstance(need, str) for need in requires.values())):
            raise Refused("BAD_BUNDLE", "version, interface, files_version or requires in bundle.json are missing "
                                        "or wrong")
    need = NEEDS.get(component)
    if need and f"{meta['revision']}/{need}" not in names:
        raise Refused("BAD_BUNDLE", f"the bundle has no {need}")


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
        result = run(["dpkg-query", "-W", "-f", "${Status}", "--", str(package)])
        if result.returncode != 0 or result.stdout.strip() != "install ok installed":
            missing.append(str(package))
    return missing


# Refused PLATFORM_MISMATCH when the bundle names its platforms and platform, the one of this mower, is not one of them.
def check_platform(bundle, platform):
    platforms = bundle.meta.get("platforms")
    if platforms and platform and platform not in platforms:
        raise Refused(PLATFORM_MISMATCH, f"the bundle is built for {', '.join(platforms)}, this mower is {platform}")


# Refused BASE_TOO_OLD when the rootfs is older than the bundle needs or misses a package it runs on.
def check_base(bundle, base_file):
    have = base_version(base_file)
    need = bundle.meta.get("base", 0)
    if need > have:
        raise Refused("BASE_TOO_OLD", f"the bundle needs base {need}, this mower has {have}")
    missing = missing_packages(bundle.meta.get("runtime_packages", []))
    if missing:
        raise Refused("BASE_TOO_OLD", f"runtime packages missing: {' '.join(missing)}")


# Prints each entry of requires {name: oldest version} on stdin that this python does not meet: python itself, any
# other name as an installed distribution.
REQUIRES_SCRIPT = """
import json, sys
from importlib import metadata

def numbers(text):
    found = []
    for part in text.split("."):
        digits = part[:len(part) - len(part.lstrip("0123456789"))]
        if not digits:
            break
        found.append(int(digits))
    return found

for name, need in sorted(json.load(sys.stdin).items()):
    if name == "python":
        have = ".".join(str(n) for n in sys.version_info[:3])
    else:
        try:
            have = metadata.version(name)
        except metadata.PackageNotFoundError:
            have = None
    if have is None or numbers(have) < numbers(need):
        print(f"{name} {need} (found {have or 'none'})")
"""


# Refused BASE_TOO_OLD when the python of the rootfs, asked as USER, misses what the bundle requires.
def check_requires(bundle, env):
    requires = bundle.meta.get("requires")
    if not requires:
        return
    result = run(as_user([sys.executable, "-I", "-c", REQUIRES_SCRIPT]), input=json.dumps(requires), env=env,
                 timeout=60)
    if result.returncode != 0:
        raise Refused("BASE_TOO_OLD", f"what the bundle requires cannot be checked: {result.stderr.strip()[-300:]}")
    if result.stdout.strip():
        raise Refused("BASE_TOO_OLD", "the bundle requires " + ", ".join(result.stdout.splitlines()))


def check_space(bundle, folder):
    free = shutil.disk_usage(folder).free
    if bundle.size + SPACE_MARGIN > free:
        raise Refused("UPDATE_NO_SPACE", f"the bundle needs {(bundle.size + SPACE_MARGIN) // 1048576} MB in {folder}, "
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


# The files of machine code in the tree at root.
def elf_files(root):
    files = []
    for folder, _, names in os.walk(root):
        files += [os.path.join(folder, name) for name in names if _is_elf(os.path.join(folder, name))]
    return files


# Files of the tree at root whose libraries are not found in the environment of the OS: [text].
def unresolved_libraries(root, os_setup, env):
    files = elf_files(root)
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


PARSE_SCRIPT = """
import ast, sys
for path in sys.stdin.read().splitlines():
    try:
        with open(path, "rb") as f:
            ast.parse(f.read(), path)
    except (OSError, SyntaxError, ValueError) as error:
        print(f"{path}: {error}")
"""


# Python files of the tree at root that do not parse, checked as the user mowbite: [text].
def unparsable_python(root, env):
    paths = [os.path.join(folder, name) for folder, _, names in os.walk(root) for name in names if name.endswith(".py")]
    result = run(as_user([sys.executable, "-I", "-c", PARSE_SCRIPT]), input="\n".join(paths) + "\n", env=env,
                 timeout=120)
    if result.returncode != 0:
        return [f"the files of {root} cannot be read: {result.stderr.strip()[-300:]}"]
    return [line.replace(root + os.sep, "", 1) for line in result.stdout.splitlines()]
