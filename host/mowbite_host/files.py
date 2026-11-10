"""Small file helpers: whole-or-nothing writes, links and JSON that may be missing or broken."""
import json
import os
import stat

# What a file read for its text may hold: they are small, and some lie in folders of another user.
MAX_TEXT = 64 * 1024


def sync_dir(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


# Written next to it as <path>.tmp, synced, renamed over it, the folder synced.
def write_json(path, value):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(value, f, indent=1, sort_keys=True)
        f.write("\n")
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    sync_dir(os.path.dirname(path) or ".")


# The text of the regular file at path, at most limit bytes and never through a link; None for anything else.
def read_text(path, limit=MAX_TEXT):
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        with os.fdopen(fd, "rb") as f:
            if not stat.S_ISREG(os.fstat(f.fileno()).st_mode):
                return None
            data = f.read(limit + 1)
    except OSError:
        return None
    return data.decode(errors="replace") if len(data) <= limit else None


# The JSON object at path; None when it is missing, unreadable or no object.
def read_json(path):
    text = read_text(path)
    try:
        value = json.loads(text) if text is not None else None
    except ValueError:
        return None
    return value if isinstance(value, dict) else None


# text as the file at path in a folder of another user: whole or not at all, and nothing is written through a link
# that waits there.
def write_new(path, text):
    tmp = path + ".tmp"
    remove(tmp)
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o644)
    with os.fdopen(fd, "w") as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    sync_dir(os.path.dirname(path))


# Whether path is a regular file itself, not a link to one.
def regular(path):
    try:
        return stat.S_ISREG(os.lstat(path).st_mode)
    except OSError:
        return False


def remove(path):
    try:
        os.remove(path)
    except FileNotFoundError:
        pass


def link_target(link):
    try:
        return os.readlink(link)
    except OSError:
        return None


# link -> target, atomic: a new link renamed over the old one.
def set_link(link, target):
    tmp = link + ".new"
    remove(tmp)
    os.symlink(target, tmp)
    os.replace(tmp, link)
    sync_dir(os.path.dirname(link))
