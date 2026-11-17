"""Which library of mowbite-host runs: part of the rootfs, never of a bundle, so a broken update cannot take it along.

The library is an installation /opt/mowbite-host/<revision> (current, previous, pending as for the OS) or the seed
in the rootfs, /usr/lib/mowbite-host; each holds bundle.json and mowbite_host/. The old host switches by writing
pending {to, from, ...} and current -> to, then it ends and systemd starts the service again.

At a start of the service: pending that was tried already goes back to from (the seed without one) and leaves a note
for the result; pending not yet tried is marked tried and gets TRIAL s to confirm, else SIGALRM ends it. After
MAX_UNSETTLED starts in this boot that never ran SETTLE s the seed runs. The command line never tries a pending one.
"""
import json
import os
import signal
import sys
import time

PREFIX = os.environ.get("MOWBITE_UPDATE_PREFIX", "")
OPT = "/opt/mowbite-host"
SEED = "/usr/lib/mowbite-host"
VAR = "/var/lib/mowbite-host"
RUN = "/run/mowbite-host"
# The note of a revision the start took back, read by the host for the result of the update.
BACK = VAR + "/host-back.json"
# Starts in this boot that did not run SETTLE s.
UNSETTLED = RUN + "/unsettled"
# How long a new host has to confirm itself, and how long a host runs before its start counts as good.
TRIAL = 180
SETTLE = 60
MAX_UNSETTLED = 3


def log(text):
    print(f"mowbite-boot: {text}", file=sys.stderr, flush=True)


def _path(name):
    return PREFIX + name


def _read_json(path):
    try:
        with open(path) as f:
            value = json.load(f)
    except (OSError, ValueError):
        return None
    return value if isinstance(value, dict) else None


def _write(path, text):
    os.makedirs(os.path.dirname(path), mode=0o700, exist_ok=True)
    with open(path + ".tmp", "w") as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    os.replace(path + ".tmp", path)
    _sync_dir(os.path.dirname(path))


def _sync_dir(folder):
    fd = os.open(folder, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _target(link):
    try:
        return os.readlink(link)
    except OSError:
        return None


def _library(revision):
    """The folder of an installation when it holds the library, else None."""
    if not revision or "/" in revision or revision.startswith("."):
        return None
    folder = os.path.join(_path(OPT), revision)
    return folder if os.path.isfile(os.path.join(folder, "mowbite_host", "__init__.py")) else None


def _set_current(revision):
    current = os.path.join(_path(OPT), "current")
    if revision:
        if os.path.lexists(current + ".new"):
            os.remove(current + ".new")
        os.symlink(revision, current + ".new")
        os.replace(current + ".new", current)
    elif os.path.lexists(current):
        os.remove(current)
    _sync_dir(_path(OPT))


def _back(pending):
    to, before = pending.get("to") or None, pending.get("from") or None
    if not _library(before):
        before = None
    _set_current(before)
    _write(_path(BACK), json.dumps({"to": to, "from": before, "t": int(time.time()),
                                    "message": "the new host did not confirm itself within its trial"}) + "\n")
    os.remove(os.path.join(_path(OPT), "pending"))
    _sync_dir(_path(OPT))
    log(f"{to} did not confirm itself: back to {before or 'the seed'}")
    return _library(before) or _path(SEED)


def _unsettled():
    try:
        with open(_path(UNSETTLED)) as f:
            return int(f.read().strip())
    except (OSError, ValueError):
        return 0


def choose(service):
    """The folder of the library to run. service: the start of mowbite-host, which tries and counts; else the
    command line, which takes what is confirmed."""
    pending_file = os.path.join(_path(OPT), "pending")
    pending = _read_json(pending_file) if os.path.lexists(pending_file) else None
    if os.path.lexists(pending_file) and pending is None:
        pending = {}
    if pending is not None and not service:
        return _library(pending.get("from")) or _path(SEED)
    if pending is not None:
        if pending.get("tried") is True or not _library(pending.get("to")):
            return _back(pending)
        _write(pending_file, json.dumps({**pending, "tried": True}) + "\n")
        signal.alarm(TRIAL)
        log(f"trying {pending['to']} for {TRIAL} s")
        return _library(pending["to"])
    if service:
        unsettled = _unsettled()
        _write(_path(UNSETTLED), f"{unsettled + 1}\n")
        if unsettled >= MAX_UNSETTLED:
            log(f"{unsettled} starts did not run {SETTLE} s: the seed runs")
            return _path(SEED)
    current = _target(os.path.join(_path(OPT), "current"))
    return _library(current) or _path(SEED)


def settled():
    """The host ran SETTLE s: this start counts as good."""
    _write(_path(UNSETTLED), "0\n")


def confirmed():
    """The trial of a new host ended well."""
    signal.alarm(0)


def load(service):
    """Puts the chosen library first on the path, MOWBITE_HOST_LIB instead when given (tests); the folder it took."""
    folder = os.environ.get("MOWBITE_HOST_LIB") or choose(service)
    sys.path.insert(0, folder)
    return folder
