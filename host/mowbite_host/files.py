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


# KEY=value of a file read as systemd reads EnvironmentFile: comments with # or ;, blanks around key and value, one
# pair of matching quotes around the value removed; None without the file.
def read_env(path):
    try:
        with open(path) as f:
            lines = f.readlines()
    except FileNotFoundError:
        return None
    env = {}
    for line in lines:
        line = line.strip()
        if line and line[0] not in "#;" and "=" in line:
            key, value = (part.strip() for part in line.split("=", 1))
            quote = value[:1]
            if quote in ("'", '"') and len(value) > 1 and value.endswith(quote):
                value = value[1:-1]
            env[key] = value
    return env


# The file of KEY=value lines at path with key set to value: its first line of key takes it, further ones of key go,
# comments and other keys stay; without such a line it is added. Whole or not at all.
def set_env(path, key, value):
    try:
        with open(path) as f:
            lines = f.read().splitlines()
    except FileNotFoundError:
        lines = []
    kept, done = [], False
    for line in lines:
        text = line.strip()
        if text and text[0] not in "#;" and "=" in text and text.split("=", 1)[0].strip() == key:
            if not done:
                kept.append(f"{key}={value}")
                done = True
            continue
        kept.append(line)
    if not done:
        kept.append(f"{key}={value}")
    write_new(path, "\n".join(kept) + "\n")


# The JSON object at path; None when it is missing, unreadable or no object.
def read_json(path):
    text = read_text(path)
    try:
        value = json.loads(text) if text is not None else None
    except ValueError:
        return None
    return value if isinstance(value, dict) else None


# text (str or bytes) as the file at path in a folder of another user: whole or not at all, and nothing is written
# through a link that waits there.
def write_new(path, text, mode=0o644):
    tmp = path + ".tmp"
    remove(tmp)
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, mode)
    with os.fdopen(fd, "wb" if isinstance(text, bytes) else "w") as f:
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
