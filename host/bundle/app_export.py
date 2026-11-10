"""The web export of the MowBite app (MowBite-web.zip of its release) as the OS image and an app bundle carry it.

  app_export.py export <zip> <out dir> [--sha256 <hex>] [--version <v>] [--min-app <version>] [--api <n>]
      unpacks the zip into the new folder <out dir>, readable by everyone, with a .gz copy of each text file worth
      it; refused: a zip of another sha256, names outside the folder, links, .gz files, more than MAX_FILES files or
      MAX_BYTES, an export without the files of NEEDS, app.json not as the app writes it, version.json or --version
      of another version, a version older than min_app or an api range without api
  app_export.py meta <dir>
      prints what bundle.json takes from app.json of an export: version, api {min, max}, commit and serial
  app_export.py probe <zip> [--version <v>] [--api-min <n>] [--api-max <n>] [--serial <n>]
      writes a small export as a zip, for tests and image builds before an app with app.json
"""
import argparse
import gzip
import hashlib
import json
import os
import re
import shutil
import stat
import sys
import zipfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from mowbite_host.compat import version_tuple, whole  # noqa: E402

MAX_FILES = 5000
MAX_BYTES = 100 * 1024 * 1024
NEEDS = ("index.html", "404.html", "version.json", "app.json")
OWNER = "mowbite-app"
# Kinds of files mowbite-web may send compressed; a copy only when it saves a tenth or more.
GZIP_SUFFIXES = (".html", ".js", ".css", ".json", ".txt", ".svg", ".map", ".webmanifest", ".xml")
GZIP_MIN = 512
COMMIT = re.compile(r"[0-9a-f]{40}")


class Refused(Exception):
    pass


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


# The relative path of a zip entry, refused when it leaves the folder or is no plain file or folder.
def _entry_path(info):
    name = info.filename
    kind = stat.S_IFMT(info.external_attr >> 16)
    if kind not in (0, stat.S_IFREG, stat.S_IFDIR):
        raise Refused(f"{name!r}: not a plain file or folder")
    parts = name.rstrip("/").split("/")
    if (not name.isprintable() or "\\" in name or name.startswith("/") or
            any(part in ("", ".", "..") for part in parts)):
        raise Refused(f"{name!r}: not a name inside the export")
    if not info.is_dir() and name.endswith(".gz"):
        raise Refused(f"{name!r}: .gz copies are made here, not taken from the zip")
    return os.path.join(*parts)


def _unpack(zip_path, out):
    with zipfile.ZipFile(zip_path) as archive:
        infos = archive.infolist()
        files = [info for info in infos if not info.is_dir()]
        if len(files) > MAX_FILES:
            raise Refused(f"{len(files)} files, at most {MAX_FILES}")
        if sum(info.file_size for info in files) > MAX_BYTES:
            raise Refused(f"more than {MAX_BYTES // 1048576} MB unpacked")
        seen = set()
        left = MAX_BYTES
        for info in infos:
            path = _entry_path(info)
            if path in seen:
                raise Refused(f"{info.filename!r} twice")
            seen.add(path)
            target = os.path.join(out, path)
            if info.is_dir():
                os.makedirs(target, exist_ok=True)
                continue
            os.makedirs(os.path.dirname(target), exist_ok=True)
            # The sizes in the zip may lie: the bytes are counted as they come.
            with archive.open(info) as src, open(target, "xb") as dst:
                while block := src.read(1 << 16):
                    left -= len(block)
                    if left < 0:
                        raise Refused(f"more than {MAX_BYTES // 1048576} MB unpacked")
                    dst.write(block)


def _read_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError) as error:
        raise Refused(f"{os.path.basename(path)}: {error}")


# What bundle.json takes from app.json, refused when app.json is not as the app writes it.
def meta(folder):
    app = _read_json(os.path.join(folder, "app.json"))
    if not isinstance(app, dict) or app.get("schema") != 1 or app.get("owner") != OWNER:
        raise Refused(f"app.json is not schema 1 of {OWNER}")
    span = app.get("api")
    if (version_tuple(app.get("version")) is None or not isinstance(span, dict) or not whole(span.get("min")) or
            not whole(span.get("max")) or span["min"] > span["max"]):
        raise Refused("app.json: version or api {min, max} missing or wrong")
    if not isinstance(app.get("commit"), str) or not COMMIT.fullmatch(app["commit"]) or not whole(app.get("serial")) \
            or app["serial"] == 0:
        raise Refused("app.json: commit and serial missing (an app built without git)")
    return {"version": app["version"], "api": {"min": span["min"], "max": span["max"]}, "commit": app["commit"],
            "serial": app["serial"]}


def _check(out, version, min_app, api):
    for name in NEEDS:
        if not os.path.isfile(os.path.join(out, name)):
            raise Refused(f"the export has no {name}")
    fields = meta(out)
    shown = _read_json(os.path.join(out, "version.json"))
    shown = shown.get("version") if isinstance(shown, dict) else None
    if shown != fields["version"] or version not in (None, shown):
        raise Refused(f"version.json says {shown}, app.json {fields['version']}, wanted {version or 'any'}")
    if min_app is not None:
        need = version_tuple(min_app)
        if need is None or version_tuple(fields["version"]) < need:
            raise Refused(f"app {fields['version']} is older than min_app {min_app}")
    if api is not None and not fields["api"]["min"] <= api <= fields["api"]["max"]:
        raise Refused(f"api {api} of the OS is outside {fields['api']['min']}..{fields['api']['max']} of the app")
    return fields


# .gz copies (fixed time, so the same export gives the same bytes) and modes 0755/0644 for everyone to read.
def _finish(out):
    for folder, dirs, names in os.walk(out):
        os.chmod(folder, 0o755)
        for name in names:
            path = os.path.join(folder, name)
            os.chmod(path, 0o644)
            if not name.endswith(GZIP_SUFFIXES) or os.path.getsize(path) < GZIP_MIN:
                continue
            with open(path, "rb") as f:
                data = f.read()
            packed = gzip.compress(data, compresslevel=9, mtime=0)
            if len(packed) <= len(data) * 0.9:
                with open(path + ".gz", "xb") as f:
                    f.write(packed)
                os.chmod(path + ".gz", 0o644)


def export(zip_path, out, sha=None, version=None, min_app=None, api=None):
    if sha is not None and sha256(zip_path) != sha:
        raise Refused(f"{zip_path} has sha256 {sha256(zip_path)}, not {sha}")
    os.mkdir(out)
    try:
        _unpack(zip_path, out)
        fields = _check(out, version, min_app, api)
        _finish(out)
    except BaseException:
        shutil.rmtree(out, ignore_errors=True)
        raise
    return fields


def probe(zip_path, version="1.5.0", api_min=15, api_max=15, serial=1700000000):
    app = {"schema": 1, "owner": OWNER, "version": version, "api": {"min": api_min, "max": api_max},
           "commit": hashlib.sha1(version.encode()).hexdigest(), "serial": serial}
    page = "<!doctype html><title>MowBite probe</title>" + "<p>probe export</p>" * 40
    entries = {"index.html": page, "404.html": "<!doctype html><title>not found</title>",
               "guide/index.html": page, "version.json": json.dumps({"version": version, "level": 1}),
               "app.json": json.dumps(app), "_next/static/chunks/main.js": "console.log('probe');" * 60}
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as archive:
        for name, text in entries.items():
            archive.writestr(name, text)


def main(argv):
    parser = argparse.ArgumentParser(prog="app_export.py", usage=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    one = commands.add_parser("export")
    one.add_argument("zip")
    one.add_argument("out")
    one.add_argument("--sha256")
    one.add_argument("--version")
    one.add_argument("--min-app")
    one.add_argument("--api", type=int)
    commands.add_parser("meta").add_argument("dir")
    test = commands.add_parser("probe")
    test.add_argument("zip")
    test.add_argument("--version", default="1.5.0")
    test.add_argument("--api-min", type=int, default=15)
    test.add_argument("--api-max", type=int, default=15)
    test.add_argument("--serial", type=int, default=1700000000)
    args = parser.parse_args(argv)
    try:
        if args.command == "export":
            fields = export(args.zip, args.out, args.sha256, args.version, args.min_app, args.api)
            print(f"app {fields['version']}, api {fields['api']['min']}..{fields['api']['max']}, "
                  f"commit {fields['commit'][:12]} in {args.out}")
        elif args.command == "meta":
            print(json.dumps(meta(args.dir)))
        else:
            probe(args.zip, args.version, args.api_min, args.api_max, args.serial)
    except (Refused, zipfile.BadZipFile, OSError) as why:
        print(f"refused: {why}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
