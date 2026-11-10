"""Where bundles come from: for now only the inbox folder on the mower (scp, then update.check).

With a signed index.json (namespace mowbite-index; entries component, revision, serial, date, size, sha256, file,
min_app) only its entries count and each file has to match its entry; without one every mowbite-<component>-*.tar.gz
with its .sig is read itself. The inbox keeps at most MAX_PER_COMPONENT bundles newer than what runs, MAX_BYTES in all.
"""
import hashlib
import json
import os
import shutil
import stat

from mowbite_host import Refused, bundle

INDEX = "index.json"
MAX_INDEX = 1024 * 1024
MAX_PER_COMPONENT = 2
MAX_BYTES = 1024 * 1024 * 1024


def _regular(path):
    try:
        return stat.S_ISREG(os.lstat(path).st_mode)
    except OSError:
        return False


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(1 << 20):
            digest.update(chunk)
    return digest.hexdigest()


class Inbox:
    def __init__(self, paths):
        self.paths = paths
        self.folder = paths.inbox
        # Bundles read without an index: (name, size, mtime_ns) -> candidate or None when it is no good bundle.
        self._read = {}

    def _stage(self, name):
        return os.path.join(self.paths.stage, "check-" + name.replace("/", "_"))

    def _index(self):
        path = os.path.join(self.folder, INDEX)
        if not _regular(path):
            return None
        stage = self._stage("index")
        shutil.rmtree(stage, ignore_errors=True)
        try:
            copy = os.path.join(stage, INDEX)
            bundle.copy_in(path + ".sig", copy + ".sig", bundle.MAX_SIGNATURE)
            bundle.copy_in(path, copy, MAX_INDEX)
            bundle.verify(copy, copy + ".sig", "index", self.paths.signers)
            with open(copy) as f:
                entries = json.load(f).get("bundles", [])
        except (OSError, ValueError, AttributeError) as error:
            raise Refused("BAD_ORIGIN", f"{path} cannot be read: {error}")
        finally:
            shutil.rmtree(stage, ignore_errors=True)
        found = []
        for entry in entries if isinstance(entries, list) else []:
            name = entry.get("file") if isinstance(entry, dict) else None
            if not isinstance(name, str) or "/" in name or name.startswith("."):
                continue
            if not all(isinstance(entry.get(key), str) for key in ("component", "revision", "sha256")):
                continue
            if not all(isinstance(entry.get(key), int) for key in ("serial", "size")):
                continue
            found.append({**entry, "path": os.path.join(self.folder, name)})
        return found

    # A bundle in the inbox read itself: its candidate, None when it is no good bundle (logged once).
    def _read_bundle(self, name):
        path = os.path.join(self.folder, name)
        info = os.lstat(path)
        key = (name, info.st_size, info.st_mtime_ns)
        if key in self._read:
            return self._read[key]
        component = name[len("mowbite-"):].split("-", 1)[0]
        stage = self._stage(name)
        try:
            staged = bundle.stage(path, path + ".sig", component, stage, self.paths.signers)
            meta = staged.meta
            candidate = {"component": component, "revision": meta["revision"], "serial": meta["serial"],
                         "date": meta.get("date", meta["serial"]), "min_app": meta.get("min_app"),
                         "size": info.st_size, "path": path}
        except Refused as why:
            print(f"inbox: {name} left out: {why}", flush=True)
            candidate = None
        finally:
            shutil.rmtree(stage, ignore_errors=True)
        self._read[key] = candidate
        return candidate

    # The candidates in the inbox: [{component, revision, serial, date, min_app, size, path, sha256?}].
    def candidates(self):
        if not os.path.isdir(self.folder):
            return []
        indexed = self._index()
        if indexed is not None:
            return [entry for entry in indexed if _regular(entry["path"])]
        found = []
        for name in sorted(os.listdir(self.folder)):
            if name.startswith("mowbite-") and name.endswith(".tar.gz") and _regular(os.path.join(self.folder, name)):
                candidate = self._read_bundle(name)
                if candidate:
                    found.append(candidate)
        return found

    def install_stage(self):
        return os.path.join(self.paths.stage, "install")

    # Stages the bundle of candidate and checks that it is what the candidate says.
    def stage(self, candidate):
        signature = candidate.get("sig", candidate["path"] + ".sig")
        staged = bundle.stage(candidate["path"], signature, candidate["component"], self.install_stage(),
                              self.paths.signers)
        if "sha256" in candidate:
            if (os.path.getsize(staged.path) != candidate["size"] or
                    _sha256(staged.path) != candidate["sha256"]):
                raise Refused("BAD_ORIGIN", f"{candidate['path']} is not the file its index names")
        if (staged.revision, staged.serial) != (candidate["revision"], candidate["serial"]):
            raise Refused("BAD_ORIGIN", f"{candidate['path']} holds {staged.revision} serial {staged.serial}, "
                                        f"not what the inbox named")
        return staged

    # Removes bundles not newer than serial_of(component) and the oldest beyond the limits.
    def prune(self, candidates, serial_of):
        keep, drop = [], []
        by_component = {}
        for candidate in candidates:
            if candidate["serial"] <= serial_of(candidate["component"]):
                drop.append(candidate)
            else:
                by_component.setdefault(candidate["component"], []).append(candidate)
        for group in by_component.values():
            group.sort(key=lambda c: c["serial"], reverse=True)
            keep += group[:MAX_PER_COMPONENT]
            drop += group[MAX_PER_COMPONENT:]
        keep.sort(key=lambda c: c["serial"], reverse=True)
        total = 0
        for candidate in list(keep):
            total += candidate["size"]
            if total > MAX_BYTES:
                keep.remove(candidate)
                drop.append(candidate)
        for candidate in drop:
            for path in (candidate["path"], candidate["path"] + ".sig"):
                if _regular(path):
                    os.remove(path)
            print(f"inbox: removed {os.path.basename(candidate['path'])}", flush=True)
        return keep
