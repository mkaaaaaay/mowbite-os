"""Bundles from the net: a signed index.json at a fixed HTTPS address and, per component, its newest fitting bundle
downloaded into a folder of their own, which is then read like the inbox.

Only HTTPS with a checked certificate, redirects only to github.com and *.githubusercontent.com. The index is at most
origin.MAX_INDEX and signed for mowbite-index, has schema 1 and a serial that never goes back; each entry names its
release file as url (<tag>/<file>, relative to BASE) and its channel, stable or preview (preview takes both). A bundle
is downloaded when it is newer than what is installed, no larger than its entry and bundle.MAX_BUNDLE of its component,
with SPARE left free beside it, one at a time; it is kept only when size and sha256 are those of its entry, and its
signature is checked again when it is installed. Downloading never installs.
"""
import hashlib
import http.client
import json
import os
import re
import shutil
import ssl
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

from mowbite_host import Refused, bundle, origin

BASE = "https://github.com/mkaaaaaay/mowbite-releases/releases/download/"
INDEX_URL = BASE + "index/" + origin.INDEX
SCHEMA = 1
CHANNELS = {"stable": ("stable",), "preview": ("stable", "preview")}
HOSTS = re.compile(r"github\.com|[a-z0-9-]+\.githubusercontent\.com")
URL = re.compile(r"[A-Za-z0-9._-]+/[A-Za-z0-9._-]+")
CONNECT_S = 15.0
INDEX_S = 60.0
BUNDLE_S = 30 * 60.0
SPARE = 500 * 1024 * 1024
NEW = ".new"


class Failed(Exception):
    """result: unreachable, bad_index, too_large, no_space or bad_bundle."""

    def __init__(self, result, message):
        super().__init__(message)
        self.result = result


def _allowed(url):
    parts = urllib.parse.urlsplit(url)
    if parts.scheme != "https" or not HOSTS.fullmatch(parts.hostname or ""):
        raise Failed("unreachable", f"{url} is no allowed address")


class _Redirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        _allowed(newurl)
        return super().redirect_request(req, fp, code, msg, headers, newurl)


# Writes url into path: at most limit bytes within seconds.
def download(url, path, limit, seconds):
    _allowed(url)
    opener = urllib.request.build_opener(urllib.request.HTTPSHandler(context=ssl.create_default_context()),
                                         _Redirect())
    deadline = time.monotonic() + seconds
    try:
        with opener.open(url, timeout=CONNECT_S) as answer, open(path, "wb") as f:
            length = answer.headers.get("Content-Length")
            if length is not None and length.isdigit() and int(length) > limit:
                raise Failed("too_large", f"{url} has {length} bytes, more than {limit}")
            got = 0
            while chunk := answer.read(1 << 16):
                got += len(chunk)
                if got > limit:
                    raise Failed("too_large", f"{url} has more than {limit} bytes")
                if time.monotonic() > deadline:
                    raise Failed("unreachable", f"{url} took longer than {seconds:.0f} s")
                f.write(chunk)
    except (urllib.error.URLError, http.client.HTTPException, OSError, ValueError) as error:
        raise Failed("unreachable", f"{url}: {error}")


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(1 << 20):
            digest.update(chunk)
    return digest.hexdigest()


def _entry_ok(entry):
    if not isinstance(entry, dict) or entry.get("component") not in bundle.MAX_BUNDLE:
        return False
    url = entry.get("url")
    if not isinstance(url, str) or not URL.fullmatch(url) or entry.get("file") != url.split("/")[1]:
        return False
    if entry["file"].startswith(".") or entry.get("channel") not in CHANNELS["preview"]:
        return False
    return (all(isinstance(entry.get(key), str) for key in ("revision", "sha256")) and
            all(isinstance(entry.get(key), int) and entry[key] > 0 for key in ("serial", "size")))


# The net origin, None when the environment files of the OS say MOWBITE_UPDATE_NET=off.
def of(paths):
    return None if paths.env().get("MOWBITE_UPDATE_NET", "on") == "off" else Net(paths)


class Net:
    """The folder paths.net, filled by a thread of its own. fetch(url, path, limit, seconds) is download, a stand-in
    in tests. The other methods are those of origin.Inbox for the updater, each under the lock the thread takes to
    change the folder."""

    def __init__(self, paths, fetch=download):
        self.paths, self.fetch = paths, fetch
        self.folder = paths.net
        self.inbox = origin.Inbox(paths, self.folder)
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.result = None
        self._thread = None

    def busy(self):
        return bool(self._thread and self._thread.is_alive())

    # Starts a refresh unless one runs; done is set when it ended, with its result in result.
    def start(self, channel, index_serial, serial_of, platform=None):
        if self.busy():
            return
        # A result not taken yet is replaced by the one of this refresh.
        self.done.clear()
        self._thread = threading.Thread(target=self._run, args=(channel, index_serial, serial_of, platform),
                                        daemon=True)
        self._thread.start()

    def _run(self, *args):
        try:
            self.result = self.refresh(*args)
        except Exception as error:  # noqa: BLE001 - the status has to say it, the host goes on
            self.result = {"result": "failed", "message": f"{type(error).__name__}: {error}"}
        self.result["checked_at"] = int(time.time())
        self.done.set()

    # Index, then the newest bundle per component newer than serial_of(component) in channel; {result, message,
    # index_serial, index_date}.
    def refresh(self, channel, index_serial, serial_of, platform=None):
        os.makedirs(self.folder, mode=0o700, exist_ok=True)
        new = os.path.join(self.folder, NEW)
        shutil.rmtree(new, ignore_errors=True)
        os.mkdir(new, mode=0o700)
        try:
            index = self._index(new, index_serial)
            failure = None
            for entry in self._wanted(index["bundles"], channel, serial_of, platform):
                try:
                    self._bundle(new, entry)
                except Failed as why:
                    failure = failure or why
            self._place(new, index["bundles"])
            result = {"result": "ok", "message": None}
            if failure:
                result = {"result": failure.result, "message": str(failure)}
            return {**result, "index_serial": index["serial"], "index_date": index.get("date")}
        except Failed as why:
            return {"result": why.result, "message": str(why)}
        finally:
            shutil.rmtree(new, ignore_errors=True)

    def _index(self, new, index_serial):
        path = os.path.join(new, origin.INDEX)
        self.fetch(INDEX_URL, path, origin.MAX_INDEX, INDEX_S)
        self.fetch(INDEX_URL + ".sig", path + ".sig", bundle.MAX_SIGNATURE, INDEX_S)
        try:
            bundle.verify(path, path + ".sig", "index", self.paths.signers)
            with open(path) as f:
                index = json.load(f)
        except (Refused, OSError, ValueError) as error:
            raise Failed("bad_index", f"the index cannot be used: {error}")
        if (not isinstance(index, dict) or index.get("schema") != SCHEMA or not isinstance(index.get("serial"), int)
                or not isinstance(index.get("bundles"), list)):
            raise Failed("bad_index", "the index lacks schema 1, serial or bundles")
        if index["serial"] < index_serial:
            raise Failed("bad_index", f"index serial {index['serial']} is older than the {index_serial} seen")
        index["bundles"] = [entry for entry in index["bundles"] if _entry_ok(entry)]
        return index

    @staticmethod
    def _wanted(entries, channel, serial_of, platform=None):
        best = {}
        for entry in entries:
            component = entry["component"]
            if (entry["channel"] in CHANNELS[channel] and entry["serial"] > serial_of(component)
                    and bundle.fits_platform(entry, platform)):
                if component not in best or entry["serial"] > best[component]["serial"]:
                    best[component] = entry
        return list(best.values())

    def _bundle(self, new, entry):
        kept = os.path.join(self.folder, entry["file"])
        if os.path.isfile(kept) and os.path.isfile(kept + ".sig") and os.path.getsize(kept) == entry["size"]:
            return
        if entry["size"] > bundle.MAX_BUNDLE[entry["component"]]:
            raise Failed("too_large", f"{entry['file']} has {entry['size']} bytes, more than a bundle may")
        if shutil.disk_usage(self.folder).free < entry["size"] + SPARE:
            raise Failed("no_space", f"{entry['file']} needs {entry['size'] + SPARE} bytes free")
        path = os.path.join(new, entry["file"])
        self.fetch(BASE + entry["url"], path, entry["size"], BUNDLE_S)
        if os.path.getsize(path) != entry["size"] or _sha256(path) != entry["sha256"]:
            os.remove(path)
            raise Failed("bad_bundle", f"{entry['file']} is not the file its index names")
        self.fetch(BASE + entry["url"] + ".sig", path + ".sig", bundle.MAX_SIGNATURE, INDEX_S)
        with self.lock:
            os.replace(path + ".sig", kept + ".sig")
            os.replace(path, kept)

    # The new index in place; files it does not name leave.
    def _place(self, new, entries):
        names = {origin.INDEX, origin.INDEX + ".sig", NEW}
        for entry in entries:
            names |= {entry["file"], entry["file"] + ".sig"}
        with self.lock:
            # Signature first: a reader between the two sees a mismatch and no candidates, never an unsigned index.
            os.replace(os.path.join(new, origin.INDEX + ".sig"), os.path.join(self.folder, origin.INDEX + ".sig"))
            os.replace(os.path.join(new, origin.INDEX), os.path.join(self.folder, origin.INDEX))
            for name in os.listdir(self.folder):
                path = os.path.join(self.folder, name)
                if name not in names:
                    shutil.rmtree(path) if os.path.isdir(path) and not os.path.islink(path) else os.remove(path)

    # The candidates of channel: [{... like origin.Inbox, from "net"}].
    def candidates(self, channel):
        with self.lock:
            if not os.path.isfile(os.path.join(self.folder, origin.INDEX)):
                return []
            found = self.inbox.candidates()
        return [{**c, "from": "net"} for c in found if c.get("channel") in CHANNELS[channel]]

    def prune(self, candidates, serial_of):
        with self.lock:
            return self.inbox.prune(candidates, serial_of)

    # Under the lock: the folder does not change while the bundle is copied out.
    def stage(self, candidate, progress=None):
        with self.lock:
            return self.inbox.stage(candidate, progress)

    def install_stage(self):
        return self.inbox.install_stage()
