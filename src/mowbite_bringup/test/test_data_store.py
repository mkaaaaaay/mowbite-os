"""data_dir before the nodes start (launch/data_store.py): schema, lock, backups, way back, halt.

Run: python3 -m pytest /ws/src/mowbite_bringup/test/test_data_store.py
"""
import fcntl
import json
import os
import stat
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH_DIR = os.path.normpath(os.path.join(HERE, "..", "launch"))
sys.path.insert(0, LAUNCH_DIR)
import data_store  # noqa: E402
from data_store import Halt, prepare  # noqa: E402

SOURCE_SCHEMA = os.path.join(HERE, "..", "..", "mowbite_bridge", "data_schema.json")


class Cut(BaseException):
    """Stands for the power going away."""


def write(path, text, mode=0o644):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)
    os.chmod(path, mode)


def read(path):
    with open(path) as f:
        return f.read()


def tree(folder, skip_kept_out=True):
    """Relative path -> (content or None for folders and links, mode) of everything below folder."""
    found = {}
    for root, dirs, files in os.walk(folder):
        rel_root = os.path.relpath(root, folder)
        if rel_root == ".":
            if skip_kept_out:
                dirs[:] = [d for d in dirs if not data_store.kept_out(d)]
                files = [f for f in files if not data_store.kept_out(f)]
        for name in dirs + files:
            path = os.path.join(root, name)
            rel = os.path.normpath(os.path.join(rel_root, name))
            info = os.lstat(path)
            if stat.S_ISLNK(info.st_mode):
                found[rel] = ("-> " + os.readlink(path), None)
            elif stat.S_ISDIR(info.st_mode):
                found[rel] = (None, stat.S_IMODE(info.st_mode))
            else:
                found[rel] = (read(path), stat.S_IMODE(info.st_mode))
    return found


def schema_of(data_dir):
    return json.loads(read(os.path.join(data_dir, "data_schema.json")))


def garden(data_dir, label="a"):
    """data_dir as the OS leaves it, with a secret, folders of their own and a link."""
    write(os.path.join(data_dir, "map.json"), f'{{"map": "{label}"}}')
    os.link(os.path.join(data_dir, "map.json"), os.path.join(data_dir, "map.json.bak"))
    write(os.path.join(data_dir, "ntrip.yaml"), f"secret: {label}\n", 0o600)
    write(os.path.join(data_dir, "params", "mower_logic.json"), f'{{"p": "{label}"}}')
    write(os.path.join(data_dir, "planner", "plans", "p1.json"), f"plan {label}")
    os.chmod(os.path.join(data_dir, "planner"), 0o750)
    write(os.path.join(data_dir, "event_history", "2026-11-04.jsonl"), f"event {label}\n")
    os.symlink("map.json", os.path.join(data_dir, "current"))


def mark(n):
    """A step that leaves its trace in map.json and a file of its own."""
    def step(data_dir):
        path = os.path.join(data_dir, "map.json")
        value = json.loads(read(path))
        value.setdefault("steps", []).append(n)
        write(path, json.dumps(value))
        write(os.path.join(data_dir, f"step{n}"), "done")
    return step


STEPS = {1: mark(1), 2: mark(2), 3: mark(3)}


def start(data_dir, schema, steps=STEPS, **kwargs):
    fd, note = prepare(data_dir, schema, version=f"v{schema}", steps=steps, **kwargs)
    os.close(fd)
    return note


@pytest.fixture
def data_dir(tmp_path):
    return str(tmp_path / "mowbite")


def test_a_new_data_dir_gets_the_schema_of_the_image(data_dir):
    assert start(data_dir, 1) is None
    assert stat.S_IMODE(os.stat(data_dir).st_mode) == 0o700
    assert schema_of(data_dir) == {"schema": 1, "by": "v1"}
    assert data_store.backups(data_dir) == []
    # Empty but for what is no data: still new.
    other = data_dir + "2"
    os.makedirs(other)
    write(os.path.join(other, "hold.json"), "{}")
    start(other, 3)
    assert schema_of(other)["schema"] == 3
    assert data_store.backups(other) == []


def test_data_from_before_the_schema_counts_as_schema_1(data_dir):
    garden(data_dir)
    before = tree(data_dir)
    start(data_dir, 1)
    assert schema_of(data_dir) == {"schema": 1, "by": "v1"}
    assert tree(data_dir) == before
    assert data_store.backups(data_dir) == []
    # The same, for an image with schema 2: it takes them through step 1.
    other = data_dir + "2"
    garden(other)
    start(other, 2)
    assert schema_of(other) == {"schema": 2, "by": "v2"}
    assert json.loads(read(os.path.join(other, "map.json")))["steps"] == [1]
    assert data_store.backups(other) == [1]


def test_the_same_schema_changes_nothing(data_dir):
    garden(data_dir)
    start(data_dir, 1)
    before = tree(data_dir, skip_kept_out=False)
    stamp = os.stat(os.path.join(data_dir, "data_schema.json")).st_mtime_ns
    start(data_dir, 1)
    assert tree(data_dir, skip_kept_out=False) == before
    assert os.stat(os.path.join(data_dir, "data_schema.json")).st_mtime_ns == stamp


def test_an_update_backs_up_everything_then_takes_every_step(data_dir):
    garden(data_dir)
    write(os.path.join(data_dir, "hold.json"), '{"until": 1}')
    write(os.path.join(data_dir, "container_exit.json"), "{}")
    start(data_dir, 1)
    before = tree(data_dir)
    start(data_dir, 4)
    assert schema_of(data_dir) == {"schema": 4, "by": "v4"}
    assert json.loads(read(os.path.join(data_dir, "map.json")))["steps"] == [1, 2, 3]
    assert data_store.backups(data_dir) == [1]
    backup = os.path.join(data_dir, "backup-1")
    # All of the data, nothing that is no data, with the rights it had.
    assert tree(backup) == before
    assert sorted(os.listdir(backup)) == sorted(data_store.data_names(backup))
    assert "hold.json" not in os.listdir(backup) and "data_schema.json" not in os.listdir(backup)
    assert stat.S_IMODE(os.stat(backup).st_mode) == 0o700
    assert stat.S_IMODE(os.stat(os.path.join(backup, "ntrip.yaml")).st_mode) == 0o600
    assert stat.S_IMODE(os.stat(os.path.join(backup, "planner")).st_mode) == 0o750
    assert os.readlink(os.path.join(backup, "current")) == "map.json"
    assert read(os.path.join(data_dir, "hold.json")) == '{"until": 1}'


def test_an_image_without_a_step_halts_before_any_change(data_dir):
    garden(data_dir)
    start(data_dir, 1)
    before = tree(data_dir, skip_kept_out=False)
    with pytest.raises(Halt) as halt:
        start(data_dir, 3, steps={1: mark(1)})
    assert halt.value.reason == "DATA_ERROR"
    assert halt.value.data_schema == 1
    assert tree(data_dir, skip_kept_out=False) == before


def test_newer_data_goes_back_to_the_backup_of_the_update(data_dir):
    garden(data_dir, "old")
    start(data_dir, 2)
    old = tree(data_dir)
    start(data_dir, 3)
    # Changed under image 3, then image 2 comes back.
    write(os.path.join(data_dir, "map.json"), '{"map": "new"}')
    write(os.path.join(data_dir, "only-new"), "x")
    write(os.path.join(data_dir, "hold.json"), '{"until": 5}')
    newer = tree(data_dir)
    note = start(data_dir, 2)
    assert schema_of(data_dir) == {"schema": 2, "by": "v2"}
    assert tree(data_dir) == old
    assert tree(os.path.join(data_dir, "backup-3")) == newer
    assert note["from"] == 3 and note["to"] == 2 and note["backup"] == "backup-3"
    assert json.loads(read(os.path.join(data_dir, "data_rollback.json")))["backup"] == "backup-3"
    assert read(os.path.join(data_dir, "hold.json")) == '{"until": 5}'
    assert stat.S_IMODE(os.stat(os.path.join(data_dir, "ntrip.yaml")).st_mode) == 0o600
    assert data_store.backups(data_dir) == [2, 3]
    # The note lasts until the next start.
    assert start(data_dir, 2) is None
    assert not os.path.exists(os.path.join(data_dir, "data_rollback.json"))


def test_newer_data_without_a_backup_halts(data_dir):
    garden(data_dir)
    start(data_dir, 3)
    before = tree(data_dir, skip_kept_out=False)
    with pytest.raises(Halt) as halt:
        start(data_dir, 2)
    assert halt.value.reason == "SCHEMA_NEWER"
    assert halt.value.line(2).startswith("MOWBITE_OS_HALT reason=SCHEMA_NEWER data_schema=3 image_schema=2 ")
    assert tree(data_dir, skip_kept_out=False) == before


def test_no_room_for_the_backup_halts_without_a_change(data_dir, monkeypatch):
    garden(data_dir)
    start(data_dir, 1)
    before = tree(data_dir, skip_kept_out=False)
    monkeypatch.setattr(data_store, "free_space", lambda path: data_store.ROOM_MARGIN)
    with pytest.raises(Halt) as halt:
        start(data_dir, 2)
    assert halt.value.reason == "NO_ROOM"
    assert tree(data_dir, skip_kept_out=False) == before
    # A way back needs the room as well.
    monkeypatch.undo()
    start(data_dir, 2)
    start(data_dir, 3)
    before = tree(data_dir, skip_kept_out=False)
    monkeypatch.setattr(data_store, "free_space", lambda path: data_store.ROOM_MARGIN)
    with pytest.raises(Halt) as halt:
        start(data_dir, 2)
    assert halt.value.reason == "NO_ROOM"
    assert tree(data_dir, skip_kept_out=False) == before


def test_a_second_os_on_the_same_data_dir_waits_then_halts(data_dir):
    fd, _ = prepare(data_dir, 1)
    try:
        with pytest.raises(Halt) as halt:
            prepare(data_dir, 1, wait=0.5)
        assert halt.value.reason == "DATA_DIR_LOCKED"
        assert halt.value.line(1).startswith("MOWBITE_OS_HALT reason=DATA_DIR_LOCKED data_schema=- image_schema=1 ")
    finally:
        os.close(fd)
    # Free again once the first one ends; the lock is the owner's alone and not handed to child processes.
    fd, _ = prepare(data_dir, 1, wait=0.5)
    try:
        assert stat.S_IMODE(os.stat(os.path.join(data_dir, ".lock")).st_mode) == 0o600
        assert fcntl.fcntl(fd, fcntl.F_GETFD) & fcntl.FD_CLOEXEC
    finally:
        os.close(fd)


def test_leftovers_of_a_crash_are_cleaned_up(data_dir):
    garden(data_dir)
    start(data_dir, 2)
    start(data_dir, 3)
    # A backup cut off while copying, a file of a write cut off, an old backup whose new one is complete and one
    # that was moved away for a new one that never came.
    os.makedirs(os.path.join(data_dir, "backup-3.tmp", "planner"))
    write(os.path.join(data_dir, "data_schema.json.tmp"), "{")
    os.rename(os.path.join(data_dir, "backup-1"), os.path.join(data_dir, "backup-1.old"))
    os.makedirs(os.path.join(data_dir, "backup-2.old"))
    start(data_dir, 3)
    names = sorted(os.listdir(data_dir))
    assert not [n for n in names if n.endswith(".tmp") or n.endswith(".old")]
    assert data_store.backups(data_dir) == [1, 2]
    assert os.path.exists(os.path.join(data_dir, "backup-1", "map.json"))
    assert os.path.exists(os.path.join(data_dir, "backup-2", "map.json"))


def test_a_backup_cut_off_starts_over_on_the_next_start(data_dir, monkeypatch):
    garden(data_dir)
    start(data_dir, 1)
    before = tree(data_dir)
    copied = []
    real_copy = data_store.copy_entry

    def failing_copy(source, target):
        if len(copied) == 3:
            raise Cut()
        copied.append(source)
        real_copy(source, target)

    monkeypatch.setattr(data_store, "copy_entry", failing_copy)
    with pytest.raises(Cut):
        start(data_dir, 2)
    assert os.path.isdir(os.path.join(data_dir, "backup-1.tmp"))
    assert schema_of(data_dir)["schema"] == 1
    monkeypatch.undo()
    start(data_dir, 2)
    assert tree(os.path.join(data_dir, "backup-1")) == before
    assert schema_of(data_dir)["schema"] == 2
    assert json.loads(read(os.path.join(data_dir, "map.json")))["steps"] == [1]


def test_a_backup_cut_off_while_replacing_an_older_one(data_dir, monkeypatch):
    garden(data_dir, "first")
    start(data_dir, 2)
    # backup-1 exists; a new backup-1 is made from other data, the power goes after the old one was moved away.
    write(os.path.join(data_dir, "map.json"), '{"map": "second"}')
    os.remove(os.path.join(data_dir, "data_schema.json"))
    write(os.path.join(data_dir, "data_schema.json"), json.dumps({"schema": 1, "by": "x"}))
    real_rename = os.rename

    def failing_rename(source, target):
        real_rename(source, target)
        if target.endswith("backup-1.old"):
            raise Cut()

    monkeypatch.setattr(data_store.os, "rename", failing_rename)
    with pytest.raises(Cut):
        start(data_dir, 2)
    monkeypatch.undo()
    assert not os.path.exists(os.path.join(data_dir, "backup-1"))
    assert os.path.isdir(os.path.join(data_dir, "backup-1.old"))
    start(data_dir, 2)
    assert json.loads(read(os.path.join(data_dir, "backup-1", "map.json"))) == {"map": "second"}
    assert data_store.backups(data_dir) == [1]


def test_a_step_cut_off_starts_over_from_the_backup(data_dir):
    garden(data_dir)
    start(data_dir, 1)
    before = tree(data_dir)
    runs = []

    def cut_step(data_dir_):
        runs.append(1)
        mark(1)(data_dir_)
        write(os.path.join(data_dir_, "params", "mower_logic.json"), "half")
        raise Cut()

    with pytest.raises(Cut):
        start(data_dir, 3, steps={1: cut_step, 2: mark(2)})
    assert schema_of(data_dir)["pending"] == {"to": 3}
    # The next start of the same image: back to the backup, every step once more.
    start(data_dir, 3)
    assert schema_of(data_dir) == {"schema": 3, "by": "v3"}
    assert json.loads(read(os.path.join(data_dir, "map.json")))["steps"] == [1, 2]
    assert read(os.path.join(data_dir, "params", "mower_logic.json")) == read(
        os.path.join(data_dir, "backup-1", "params", "mower_logic.json"))
    assert tree(os.path.join(data_dir, "backup-1")) == before


def test_an_update_cut_off_and_the_old_image_back(data_dir):
    garden(data_dir)
    start(data_dir, 1)
    before = tree(data_dir)

    def cut_step(data_dir_):
        write(os.path.join(data_dir_, "map.json"), "half")
        raise Cut()

    with pytest.raises(Cut):
        start(data_dir, 2, steps={1: cut_step})
    # The updater went back to the image with schema 1: the data of the backup, as before the update.
    assert start(data_dir, 1) is None
    assert schema_of(data_dir) == {"schema": 1, "by": "v1"}
    assert tree(data_dir) == before


def test_a_way_back_cut_off_ends_like_one_in_one_go(data_dir, monkeypatch):
    garden(data_dir, "old")
    start(data_dir, 2)
    old = tree(data_dir)
    start(data_dir, 3)
    write(os.path.join(data_dir, "map.json"), '{"map": "new"}')
    newer = tree(data_dir)
    copied = []
    real_copy = data_store.copy_entry

    def failing_copy(source, target):
        # Cut once the backup of the newer data is complete and the old data comes back.
        if "/backup-2/" in source + "/" and len(copied) >= 2:
            raise Cut()
        if "/backup-2/" in source + "/":
            copied.append(source)
        real_copy(source, target)

    monkeypatch.setattr(data_store, "copy_entry", failing_copy)
    with pytest.raises(Cut):
        start(data_dir, 2)
    monkeypatch.undo()
    assert schema_of(data_dir)["pending"] == {"to": 2}
    note = start(data_dir, 2)
    assert note["backup"] == "backup-3"
    assert tree(data_dir) == old
    assert tree(os.path.join(data_dir, "backup-3")) == newer
    # The same with image 3 back instead: it finds its data as before.
    other = data_dir + "2"
    garden(other, "old")
    start(other, 2)
    start(other, 3)
    write(os.path.join(other, "map.json"), '{"map": "new"}')
    newer = tree(other)
    copied.clear()
    monkeypatch.setattr(data_store, "copy_entry", failing_copy)
    with pytest.raises(Cut):
        start(other, 2)
    monkeypatch.undo()
    start(other, 3)
    assert schema_of(other)["schema"] == 3
    assert tree(other) == newer


def test_a_schema_file_with_only_the_number_will_do(data_dir):
    garden(data_dir)
    start(data_dir, 2)
    write(os.path.join(data_dir, "data_schema.json"), '{"schema": 2}')
    start(data_dir, 3)
    write(os.path.join(data_dir, "data_schema.json"), '{"schema": 3}')
    assert start(data_dir, 2)["backup"] == "backup-3"
    assert schema_of(data_dir) == {"schema": 2, "by": "v2"}


def test_a_cut_off_operation_without_its_backup_halts(data_dir):
    garden(data_dir)
    write(os.path.join(data_dir, "data_schema.json"), json.dumps({"schema": 1, "by": "x", "pending": {"to": 2}}))
    with pytest.raises(Halt) as halt:
        start(data_dir, 2)
    assert halt.value.reason == "DATA_ERROR"


@pytest.mark.parametrize("content", ["{", "[]", '{"schema": "2"}', '{"schema": 0}', '{"schema": true}', "{}"])
def test_a_schema_file_without_a_number_halts(data_dir, content):
    garden(data_dir)
    write(os.path.join(data_dir, "data_schema.json"), content)
    with pytest.raises(Halt) as halt:
        start(data_dir, 1)
    assert halt.value.reason == "DATA_ERROR"


def test_at_most_two_backups_but_never_the_one_just_made(data_dir):
    garden(data_dir)
    for schema in (2, 3, 4):
        start(data_dir, schema)
    assert data_store.backups(data_dir) == [2, 3]
    # Back to 3 (backup-3), then 3 to 4 again: backup-4 of the way back and backup-3 of the new update stay.
    start(data_dir, 3)
    assert data_store.backups(data_dir) == [3, 4]
    start(data_dir, 4)
    assert data_store.backups(data_dir) == [3, 4]
    # Down to 2 is no longer possible.
    with pytest.raises(Halt):
        start(data_dir, 2)


def test_the_backup_of_a_lower_schema_stays_after_an_update(data_dir):
    garden(data_dir)
    start(data_dir, 2)
    start(data_dir, 3)
    os.rename(os.path.join(data_dir, "backup-1"), os.path.join(data_dir, "backup-5"))
    os.rename(os.path.join(data_dir, "backup-2"), os.path.join(data_dir, "backup-6"))
    # Backups 5 and 6 there, data at 3, image 4: the new backup-3 has the lowest number but is needed to go back.
    start(data_dir, 4)
    assert data_store.backups(data_dir) == [3, 6]
    start(data_dir, 3)
    assert schema_of(data_dir)["schema"] == 3


def test_the_halt_line_waits_for_mowbite_health(tmp_path):
    path = str(tmp_path / "halt")
    line = Halt("NO_ROOM", "No room.", 1).line(2)
    assert line == "MOWBITE_OS_HALT reason=NO_ROOM data_schema=1 image_schema=2 No room."
    data_store.write_halt(path, line)
    assert read(path) == line + "\n"
    assert not os.path.exists(path + ".tmp")


def test_every_schema_of_the_image_has_its_steps_and_its_planner_version():
    with open(SOURCE_SCHEMA) as f:
        table = json.load(f)
    schema = table["schema"]
    assert isinstance(schema, int) and schema >= 1
    assert all(n in data_store.STEPS for n in range(1, schema))
    assert str(schema) in table["planner_files_version"]


def test_the_planner_version_is_read_without_importing_the_planner(tmp_path, monkeypatch):
    package = tmp_path / "mowbite_planner"
    package.mkdir()
    (package / "__init__.py").write_text("import not_there\n__version__ = '0.1.0'\n")
    monkeypatch.syspath_prepend(str(tmp_path))
    monkeypatch.delitem(sys.modules, "mowbite_planner", raising=False)
    assert data_store.planner_files_version() == 1
    (package / "__init__.py").write_text("import not_there\nFILES_VERSION = 2\n__version__ = '0.1.0'\n")
    assert data_store.planner_files_version() == 2


def test_the_pinned_planner_matches_the_table():
    try:
        data_store.image_schema()
    except Exception:
        pytest.skip("mowbite_bridge is not installed")
    if data_store.importlib.util.find_spec("mowbite_planner") is None:
        pytest.skip("mowbite_planner is not installed")
    assert data_store.check_planner() == 0
