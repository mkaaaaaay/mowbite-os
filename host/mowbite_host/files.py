"""Small file helpers: whole-or-nothing writes, links and JSON that may be missing or broken."""
import json
import os


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


# The JSON object at path; None when it is missing, unreadable or no object.
def read_json(path):
    try:
        with open(path) as f:
            value = json.load(f)
    except (OSError, ValueError):
        return None
    return value if isinstance(value, dict) else None


def remove(path):
    try:
        os.remove(path)
    except FileNotFoundError:
        pass


def link_target(link):
    try:
        return os.readlink(link)
    except (FileNotFoundError, OSError):
        return None


# link -> target, atomic: a new link renamed over the old one.
def set_link(link, target):
    tmp = link + ".new"
    remove(tmp)
    os.symlink(target, tmp)
    os.replace(tmp, link)
    sync_dir(os.path.dirname(link))
