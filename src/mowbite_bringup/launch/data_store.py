"""data_dir before any node of MowBite OS starts (tron.launch.py, updater contract 3.5).

One number, the data schema, stands for the formats of everything in data_dir, planner/ and calibration/ included.
<data_dir>/data_schema.json holds the number of the data, data_schema.json of mowbite_bridge the one of the image.
Data from before the file counts as schema 1. On start:
  1. flock on <data_dir>/.lock for the life of the launch: a second OS on the same data_dir waits, then halts.
  2. Leftovers of a crash go: *.tmp; backup-N.old comes back when backup-N is missing, else it goes.
  3. An operation the crash cut off (pending in data_schema.json) starts over from the backup it made first.
  4. Data older than the image: backup-D, then the steps D -> D+1 -> ... -> S. Newer data with backup-S of an earlier
     update: backup-D of it, then backup-S back, and a note for the bridge. Newer data without backup-S: halt.
  5. At most two backups, the lowest number goes first, but never the one just made (needed to go back).
Every change is a copy made to a .tmp first and renamed when complete, so a crash at any point leaves data the next
start can go on from. data_schema.json is written last.

Run as a script to compare the FILES_VERSION of the installed planner with the table of data_schema.json:
  python3 data_store.py --check-planner
"""
import ast
import errno
import fcntl
import importlib.util
import json
import os
import shutil
import stat
import sys
import time

SCHEMA_FILE = "data_schema.json"
LOCK_FILE = ".lock"
# Written by the logic for system.hold; it outlives an update and a way back.
HOLD_FILE = "hold.json"
# For the bridge: a way back took the data since the update away and left it in a backup.
ROLLBACK_NOTE = "data_rollback.json"
# Written by the logic (mowbite_hold/restart.h): the restart hold and the session it is judged from outlive an update
# and a way back.
RESTART_HOLD_FILE = "restart-hold.json"
SESSION_FILE = "os-session.json"
# Written by tron_mcu before STOP is cleared on the MCU during an emergency, removed once it is released; a start with
# it latches. It outlives an update and a way back.
STOP_CLEAR_MARK_FILE = "stop-clear-mark"
# The measured values of this Tron (safety measurement, firmware), read after the platform file; kept like the data.
TRON_PARAMS_FILE = "tron.yaml"
BACKUP = "backup-"
TEMP = ".tmp"
OLD = ".old"
KEEP_BACKUPS = 2
# Free space a backup leaves at least.
ROOM_MARGIN = 50 * 1024 * 1024
LOCK_WAIT = 30.0
# Where the halt line waits for mowbite_health, in the container and not in data_dir; MOWBITE_HALT_FILE for both.
HALT_FILE = "/tmp/mowbite_os_halt"

# Steps from schema n to n + 1, each a function of data_dir that can run again on a copy cut off half way.
STEPS = {}


class Halt(Exception):
    """No node may start: reason (DATA_DIR_LOCKED, DATA_NO_SPACE, SCHEMA_NEWER, DATA_ERROR, DDS_PROFILE,
    PLATFORM_CONFIG) and one sentence."""

    def __init__(self, reason, text, data_schema=None):
        super().__init__(text)
        self.reason = reason
        self.text = text
        self.data_schema = data_schema

    def line(self, image_schema):
        data = "-" if self.data_schema is None else self.data_schema
        return (f"MOWBITE_OS_HALT reason={self.reason} data_schema={data} image_schema={image_schema} "
                f"{self.text}")


def image_schema():
    """The schema of this image and the planner table, from the share folder of mowbite_bridge."""
    from ament_index_python.packages import get_package_share_directory
    with open(os.path.join(get_package_share_directory("mowbite_bridge"), SCHEMA_FILE)) as f:
        table = json.load(f)
    return table["schema"], {int(k): v for k, v in table["planner_files_version"].items()}


def kept_out(name):
    """Not data: never backed up, never replaced by a way back."""
    return (name in (LOCK_FILE, HOLD_FILE, SCHEMA_FILE, ROLLBACK_NOTE, RESTART_HOLD_FILE, SESSION_FILE,
                     STOP_CLEAR_MARK_FILE)
            or name.startswith(BACKUP) or name.endswith(TEMP))


def data_names(folder):
    return sorted(name for name in os.listdir(folder) if not kept_out(name))


def backup_path(data_dir, number):
    return os.path.join(data_dir, f"{BACKUP}{number}")


def backups(data_dir):
    numbers = []
    for name in os.listdir(data_dir):
        rest = name[len(BACKUP):]
        if name.startswith(BACKUP) and rest.isdigit() and os.path.isdir(os.path.join(data_dir, name)):
            numbers.append(int(rest))
    return sorted(numbers)


def sync_folder(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def remove(path):
    if os.path.isdir(path) and not os.path.islink(path):
        shutil.rmtree(path)
    elif os.path.lexists(path):
        os.remove(path)


def copy_entry(source, target):
    """Copies a file, folder or link with its mode bits and syncs every file and folder it writes."""
    info = os.lstat(source)
    if stat.S_ISLNK(info.st_mode):
        os.symlink(os.readlink(source), target)
    elif stat.S_ISDIR(info.st_mode):
        os.mkdir(target, 0o700)
        for name in sorted(os.listdir(source)):
            copy_entry(os.path.join(source, name), os.path.join(target, name))
        os.chmod(target, stat.S_IMODE(info.st_mode))
        sync_folder(target)
    elif stat.S_ISREG(info.st_mode):
        # Created for the owner only and opened up afterwards, so ntrip.yaml is never readable by others on the way.
        fd = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        try:
            with open(source, "rb") as src, os.fdopen(os.dup(fd), "wb") as dst:
                shutil.copyfileobj(src, dst, 1024 * 1024)
                dst.flush()
            os.fchmod(fd, stat.S_IMODE(info.st_mode))
            os.fsync(fd)
        finally:
            os.close(fd)
        os.utime(target, ns=(info.st_atime_ns, info.st_mtime_ns), follow_symlinks=False)


def disk_size(path):
    """Bytes a copy of path takes at most, in 4 KiB blocks."""
    def blocks(size):
        return (size + 4095) // 4096 * 4096
    info = os.lstat(path)
    if not stat.S_ISDIR(info.st_mode):
        return blocks(info.st_size)
    return 4096 + sum(disk_size(os.path.join(path, name)) for name in os.listdir(path))


def data_size(folder):
    return sum(disk_size(os.path.join(folder, name)) for name in data_names(folder))


def free_space(path):
    info = os.statvfs(path)
    return info.f_bavail * info.f_frsize


def write_json(data_dir, name, value):
    path = os.path.join(data_dir, name)
    temp = path + TEMP
    with open(temp, "w") as f:
        json.dump(value, f)
        f.flush()
        os.fsync(f.fileno())
    os.replace(temp, path)
    sync_folder(data_dir)


def read_schema(data_dir):
    """The content of data_schema.json, None without it."""
    path = os.path.join(data_dir, SCHEMA_FILE)
    try:
        with open(path) as f:
            state = json.load(f)
    except FileNotFoundError:
        return None
    except (OSError, ValueError) as error:
        raise Halt("DATA_ERROR", f"{path} cannot be read: {error}.")
    if not isinstance(state, dict) or type(state.get("schema")) is not int or state["schema"] < 1:
        raise Halt("DATA_ERROR", f"{path} has no schema number.")
    return state


def lock(data_dir, wait):
    """The lock on data_dir, held until the file descriptor closes."""
    fd = os.open(os.path.join(data_dir, LOCK_FILE), os.O_RDWR | os.O_CREAT | os.O_CLOEXEC, 0o600)
    deadline = time.monotonic() + wait
    while True:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return fd
        except OSError as error:
            if error.errno not in (errno.EWOULDBLOCK, errno.EAGAIN) or time.monotonic() >= deadline:
                os.close(fd)
                raise Halt("DATA_DIR_LOCKED", f"Another MowBite OS holds {data_dir} for more than {wait:g} s.")
        time.sleep(0.2)


def clean(data_dir):
    names = os.listdir(data_dir)
    for name in names:
        if name.endswith(TEMP):
            remove(os.path.join(data_dir, name))
    for name in names:
        if name.startswith(BACKUP) and name.endswith(OLD):
            old = os.path.join(data_dir, name)
            current = old[:-len(OLD)]
            if os.path.lexists(current):
                remove(old)
            else:
                os.rename(old, current)
    sync_folder(data_dir)


def make_backup(data_dir, number):
    """backup-<number> from the data now, in place of an older one of that number only once it is complete."""
    final = backup_path(data_dir, number)
    temp = final + TEMP
    remove(temp)
    os.mkdir(temp, 0o700)
    os.chmod(temp, 0o700)
    for name in data_names(data_dir):
        copy_entry(os.path.join(data_dir, name), os.path.join(temp, name))
    sync_folder(temp)
    if os.path.lexists(final):
        remove(final + OLD)
        os.rename(final, final + OLD)
    os.rename(temp, final)
    sync_folder(data_dir)
    remove(final + OLD)
    sync_folder(data_dir)


def restore(data_dir, number):
    """The data of backup-<number> in place of the data now; the backup stays."""
    source = backup_path(data_dir, number)
    for name in data_names(data_dir):
        remove(os.path.join(data_dir, name))
    sync_folder(data_dir)
    for name in data_names(source):
        copy_entry(os.path.join(source, name), os.path.join(data_dir, name))
    sync_folder(data_dir)


def prune(data_dir, keep):
    """At most KEEP_BACKUPS backups, the lowest number first, but never backup-<keep> just made."""
    numbers = [n for n in backups(data_dir) if n != keep]
    for number in numbers[:max(0, len(numbers) + 1 - KEEP_BACKUPS)]:
        remove(backup_path(data_dir, number))
    sync_folder(data_dir)


def need_room(data_dir, needed, schema):
    free = free_space(data_dir)
    if free < needed + ROOM_MARGIN:
        raise Halt("DATA_NO_SPACE", f"A backup needs {(needed + ROOM_MARGIN) // 1048576} MB free in {data_dir}, "
                   f"there are {free // 1048576} MB; nothing was changed.", schema)


def prepare(data_dir, schema, version="unknown", steps=None, wait=LOCK_WAIT):
    """Brings data_dir to schema and returns the file descriptor of its lock, to keep open while the OS runs, and
    the note of a way back, if there was one. Raises Halt when no node may start."""
    steps = STEPS if steps is None else steps
    os.makedirs(data_dir, 0o700, exist_ok=True)
    fd = lock(data_dir, wait)
    try:
        return fd, _prepare(data_dir, schema, version, steps)
    except OSError as error:
        os.close(fd)
        raise Halt("DATA_ERROR", f"{data_dir} cannot be prepared: {error}.")
    except BaseException:
        os.close(fd)
        raise


def _prepare(data_dir, schema, version, steps):
    clean(data_dir)
    remove(os.path.join(data_dir, ROLLBACK_NOTE))
    state = read_schema(data_dir)
    if state and "pending" in state:
        # An update or a way back was cut off after its backup was complete: back to that backup, then on as usual.
        start = state["schema"]
        if start not in backups(data_dir):
            raise Halt("DATA_ERROR", f"An operation on {data_dir} was cut off and {BACKUP}{start} is missing.", start)
        restore(data_dir, start)
        state = {"schema": start, "by": state.get("by", "unknown")}
        write_json(data_dir, SCHEMA_FILE, state)
    if state is None:
        current = 1 if data_names(data_dir) else schema
    else:
        current = state["schema"]
    note = None
    if current < schema:
        missing = [n for n in range(current, schema) if n not in steps]
        if missing:
            raise Halt("DATA_ERROR", f"This image has no step from schema {missing[0]}.", current)
        need_room(data_dir, data_size(data_dir), current)
        make_backup(data_dir, current)
        write_json(data_dir, SCHEMA_FILE, {"schema": current, "by": state.get("by", "unknown") if state else "unknown",
                                           "pending": {"to": schema}})
        for n in range(current, schema):
            steps[n](data_dir)
    elif current > schema:
        if schema not in backups(data_dir):
            raise Halt("SCHEMA_NEWER", f"The data is newer than this image and there is no {BACKUP}{schema} "
                       "to go back to.", current)
        need_room(data_dir, max(data_size(data_dir), data_size(backup_path(data_dir, schema))), current)
        make_backup(data_dir, current)
        write_json(data_dir, SCHEMA_FILE, {"schema": current, "by": state.get("by", "unknown"),
                                           "pending": {"to": schema}})
        restore(data_dir, schema)
        # Before the schema, so that a cut in between does the way back once more, note included.
        note = {"from": current, "to": schema, "backup": f"{BACKUP}{current}", "t": time.time()}
        write_json(data_dir, ROLLBACK_NOTE, note)
    if state is None or current != schema:
        write_json(data_dir, SCHEMA_FILE, {"schema": schema, "by": version})
    if current != schema:
        prune(data_dir, current)
    return note


def write_halt(path, line):
    """The halt line for mowbite_health, written whole or not at all."""
    temp = path + TEMP
    with open(temp, "w") as f:
        f.write(line + "\n")
    os.replace(temp, path)


def planner_files_version():
    """FILES_VERSION of the installed planner, read without importing it (shapely); 1 for a planner without it."""
    spec = importlib.util.find_spec("mowbite_planner")
    if spec is None or not spec.origin:
        raise RuntimeError("mowbite_planner is not installed")
    with open(spec.origin) as f:
        tree = ast.parse(f.read(), spec.origin)
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "FILES_VERSION"
                                                for t in node.targets):
            return ast.literal_eval(node.value)
    return 1


def check_planner():
    schema, table = image_schema()
    found = planner_files_version()
    if table.get(schema) != found:
        print(f"Planner FILES_VERSION {found}, but data schema {schema} of the image expects {table.get(schema)}: "
              "a new FILES_VERSION needs a new data schema and a step for it", file=sys.stderr)
        return 1
    print(f"data schema {schema}, planner FILES_VERSION {found}")
    return 0


if __name__ == "__main__":
    if sys.argv[1:] != ["--check-planner"]:
        print(__doc__, file=sys.stderr)
        sys.exit(2)
    sys.exit(check_planner())
