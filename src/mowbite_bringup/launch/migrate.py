"""A mower from OpenMower to MowBite OS and back (updater contract, K3).

Import, on the start of a platform that sets OPENMOWER_IMPORT, before any node (stack.py): when data_dir has no data
yet, or an import there was cut off, it takes from <import_dir> (OpenMower's folders, read only)
  ros/map.json                 -> map.json as it is, if mowbite_map_check passes it, and map_datum.json from the datum
  params/mower_params.yaml     -> the antenna, track and ticks into geometry.json of the logic (not confirmed: the
                                  setup program asks for them), the NTRIP access into ntrip.yaml (mowbite_ntrip_write)
Everything else of mower_params.yaml stays behind, by name and reason in <data_dir>/migrate.json, never with its value.
No date of the safety measurement and no allow_blade: the lock stays shut.

Export, the way back while the OS runs and rests:
  mowbite-migrate export <folder> [--data-dir <data_dir>]     (default: MOWBITE_DATA_DIR)
writes <folder>/map.json in the fields OpenMower has and <folder>/report.json with what it cannot take in lost.
Exit 0 done, 2 refused because the mower is not at rest (IDLE without sub state), 1 error.
"""
import hashlib
import json
import math
import os
import subprocess
import sys
import time

# data_store.py lies next to this file, in the source tree and in the install space alike.
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import data_store  # noqa: E402

NOTE = "migrate.json"
MAP = "map.json"
MAP_BACKUP = "map.json.bak"
DATUM = "map_datum.json"
GEOMETRY = "geometry.json"
NTRIP = "ntrip.yaml"
REPORT = "report.json"
MAP_SOURCE = "ros/map.json"
PARAMS_SOURCE = "params/mower_params.yaml"
PARAMS_MAX_BYTES = 256 * 1024
# The version map_datum.json of mowbite_map reads.
DATUM_SCHEMA = 1
STATE_WAIT = 5.0

GEOMETRY_KEYS = {
    "xbot_positioning/antenna_offset_x": "antenna_x",
    "xbot_positioning/antenna_offset_y": "antenna_y",
    "ll/services/diff_drive/wheel_distance_m": "track",
    "ll/services/diff_drive/ticks_per_m": "ticks_per_m",
}
DATUM_KEYS = {
    "ll/services/gps/datum_lat": "datum_lat",
    "ll/services/gps/datum_long": "datum_long",
    "ll/services/gps/datum_height": "datum_height",
}
# OpenMower's name under ntrip_client -> the one in ntrip.yaml.
NTRIP_KEYS = {"ntrip_client/" + key: name for key, name in (
    ("host", "host"), ("port", "port"), ("mountpoint", "mountpoint"), ("username", "user"), ("password", "password"))}
# Why the rest stays behind, by the first prefix that fits: never (limits of charging and the access to another
# broker), platform (values of the board the platform keeps), os (the OS decides itself; angle and outlines of an area
# come with the map).
LEFT = [
    ("never", ("ll/services/power/charge_voltage", "ll/services/power/battery_critical_high_voltage",
               "ll/services/power/charge_critical_high_voltage", "ll/services/power/charge_critical_high_current",
               "ll/services/power/termination_current", "xbot_monitoring")),
    ("platform", ("ll/bind_ip", "ll/services/power", "ll/services/gps", "ll/services/input",
                  "ll/services/diff_drive")),
    ("os", ("mower_logic", "ll/services/sound", "ntrip_client", "xbot_positioning")),
]
# Shown as a whole: its keys hold an access.
COLLAPSED = ("xbot_monitoring",)

# What OpenMower's map.json has; the rest of an area is lost on the way back.
AREA_PROPERTIES = ("name", "type", "angle", "outline_count")
OPENMOWER_TYPES = ("mow", "nav", "obstacle")
# Properties OpenMower lacks that lose nothing at this value.
DEFAULT_PROPERTIES = {"active": True, "mowable": True}


def under(key, prefix):
    return key == prefix or key.startswith(prefix + "/")


def flatten(value, prefix=""):
    if not isinstance(value, dict):
        return {prefix: value}
    out = {}
    for key, item in value.items():
        out.update(flatten(item, f"{prefix}/{key}" if prefix else str(key).lstrip("/")))
    return out


def number(value):
    """A finite number, None for anything else (a boolean too)."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return None
    return float(value)


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def read_source(path, limit):
    """The bytes of path, None without it; OSError for one larger than limit or that cannot be read."""
    try:
        with open(path, "rb") as f:
            data = f.read(limit + 1)
    except FileNotFoundError:
        return None
    if len(data) > limit:
        raise OSError(f"{path} is larger than {limit} bytes")
    return data


def write_file(folder, name, data):
    """name in folder, from a .tmp renamed when complete."""
    path = os.path.join(folder, name)
    with open(path + data_store.TEMP, "wb") as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    os.replace(path + data_store.TEMP, path)
    data_store.sync_folder(folder)


# The revision of the running OS, empty where the start does not name it.
def revision():
    return os.environ.get("MOWBITE_OS_REVISION", "")


def image():
    return revision() or "unknown"


def read_json(path):
    with open(path) as f:
        return json.load(f)


def read_note(data_dir):
    """migrate.json, None without it; {} for one that cannot be read."""
    try:
        note = read_json(os.path.join(data_dir, NOTE))
    except FileNotFoundError:
        return None
    except (OSError, ValueError):
        return {}
    return note if isinstance(note, dict) else {}


def wanted(data_dir, besides=()):
    """Whether this start imports: data_dir has no data and no note yet, or the note of an import cut off; besides as
    in data_store.has_data."""
    note = read_note(data_dir)
    if note is None:
        return not data_store.has_data(data_dir, besides)
    return note.get("state") == "pending"


def geometry_ranges(schema_path=None):
    """{key: (min, max)} of the numbers of x-geometry in the interface schema of the image."""
    if schema_path is None:
        from ament_index_python.packages import get_package_share_directory
        schema_path = os.path.join(get_package_share_directory("mowbite_api"), "api.schema.json")
    keys = read_json(schema_path)["x-geometry"]
    return {name: (entry["min"], entry["max"]) for name, entry in keys.items()
            if entry.get("type") == "number" and entry.get("settable") and not entry.get("relative")}


def run_tool(package, name, args, data):
    """None when the tool of package (lib/<package>/<name>) ends with 0, else the first line of why; it gets data on
    stdin and never prints a value it was given."""
    from ament_index_python.packages import get_package_prefix
    result = subprocess.run([os.path.join(get_package_prefix(package), "lib", package, name), *args], input=data,
                            capture_output=True, timeout=20)
    if result.returncode == 0:
        return None
    lines = (result.stdout or result.stderr).decode(errors="replace").strip().splitlines()
    return lines[0] if lines else "not valid"


def ntrip_access(params):
    """What mower_params gives for ntrip.yaml, texts as texts; None without host or mountpoint."""
    access = {}
    for key, name in NTRIP_KEYS.items():
        value = params.get(key)
        if value is not None:
            access[name] = value if name == "port" else str(value)
    return access if access.get("host") and access.get("mountpoint") else None


def left_behind(params, used):
    """[{key, why}] for every key of mower_params not taken, its value never."""
    out, shown = [], set()
    for key in sorted(params):
        if key in used:
            continue
        why = next((reason for reason, prefixes in LEFT if any(under(key, p) for p in prefixes)), "unknown")
        name = next((p for p in COLLAPSED if under(key, p)), key)
        if name not in shown:
            shown.add(name)
            out.append({"key": name, "why": why})
    return out


def take(data_dir, import_dir, map_max_bytes, logger, ranges=None, run=run_tool):
    """The import into data_dir; its note says what came over. Errors are logged, the start goes on."""
    note = {"state": "pending", "t": time.time(), "image": image(),
            "from": "openmower", "sources": {}, "taken": [], "not_taken": [], "imported": {}}
    try:
        data_store.write_json(data_dir, NOTE, note)
        # What an import cut off may have left.
        for name in (MAP, MAP_BACKUP, DATUM, GEOMETRY, NTRIP, NTRIP + ".new"):
            data_store.remove(os.path.join(data_dir, name))
        _take(data_dir, import_dir, map_max_bytes, note, ranges, run)
        note["state"] = "done"
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        note["state"] = "failed"
        note["error"] = str(error)
        logger.error(f"The import from OpenMower failed, what came over stays: {error}")
    try:
        data_store.write_json(data_dir, NOTE, note)
    except OSError as error:
        logger.error(f"{NOTE} cannot be written: {error}")
        # Never pending once the nodes start with what came over: the next start would remove it and import again.
        try:
            data_store.remove(os.path.join(data_dir, NOTE))
        except OSError as gone:
            logger.error(f"{NOTE} cannot be removed: {gone}")
    if note["state"] == "done":
        logger.info(f"Taken over from OpenMower: {', '.join(note['taken']) or 'nothing'}; "
                    f"{len(note['not_taken'])} left behind, see {os.path.join(data_dir, NOTE)}")
    return note


def _take(data_dir, import_dir, map_max_bytes, note, ranges, run):
    import yaml
    map_data = read_source(os.path.join(import_dir, MAP_SOURCE), map_max_bytes)
    params_data = read_source(os.path.join(import_dir, PARAMS_SOURCE), PARAMS_MAX_BYTES)
    for name, data in ((MAP_SOURCE, map_data), (PARAMS_SOURCE, params_data)):
        note["sources"][name] = {"missing": True} if data is None else {"bytes": len(data), "sha256": sha256(data)}
    params = {}
    if params_data is not None:
        try:
            loaded = yaml.safe_load(params_data.decode("utf-8", errors="replace"))
        except yaml.YAMLError as error:
            # The kind and the line only: the text of PyYAML quotes the line, which may hold the NTRIP password.
            mark = getattr(error, "problem_mark", None)
            where = f" in line {mark.line + 1}" if mark is not None else ""
            raise ValueError(f"{PARAMS_SOURCE} cannot be read: {type(error).__name__}{where}")
        if not isinstance(loaded, dict):
            raise ValueError(f"{PARAMS_SOURCE} holds no values")
        params = flatten(loaded)
    used = set()

    datum = {name: number(params.get(key)) for key, name in DATUM_KEYS.items()}
    used.update(DATUM_KEYS)
    if None in datum.values() or abs(datum["datum_lat"]) > 90 or abs(datum["datum_long"]) > 180:
        datum = None
        if params:
            note["not_taken"].append({"key": "ll/services/gps/datum", "why": "invalid"})

    if map_data is not None:
        why = run("mowbite_map", "mowbite_map_check", [], map_data)
        if why:
            note["not_taken"].append({"key": MAP_SOURCE, "why": f"invalid: {why}"})
        else:
            write_file(data_dir, MAP, map_data)
            note["taken"].append("map")
            if datum:
                data_store.write_json(data_dir, DATUM, {"schema": DATUM_SCHEMA, **datum})
                note["taken"].append("datum")

    entries = {}
    if ranges is None:
        ranges = geometry_ranges() if any(key in params for key in GEOMETRY_KEYS) else {}
    for key, name in GEOMETRY_KEYS.items():
        if key not in params:
            continue
        used.add(key)
        value = number(params[key])
        low, high = ranges.get(name, (None, None))
        if value is None or low is None or not low <= value <= high:
            note["not_taken"].append({"key": key, "why": "invalid"})
            continue
        entries[name] = value
    if entries:
        data_store.write_json(data_dir, GEOMETRY, {"version": 1, "entries": entries})
        note["taken"].extend(f"geometry.{name}" for name in entries)
        note["imported"]["geometry"] = entries

    if any(key in params for key in NTRIP_KEYS):
        used.update(NTRIP_KEYS)
        access = ntrip_access(params)
        path = os.path.join(data_dir, NTRIP)
        why = "host or mountpoint missing" if access is None else run(
            "mowbite_ntrip", "mowbite_ntrip_write", [path], json.dumps(access).encode())
        if why:
            note["not_taken"].append({"key": "ntrip_client", "why": f"invalid: {why}"})
        else:
            note["taken"].append("ntrip")
            with open(path, "rb") as f:
                note["imported"]["ntrip_sha256"] = sha256(f.read())

    note["not_taken"].extend(left_behind(params, used))


def openmower_map(stored):
    """The map in the fields of OpenMower's map.json and what it loses on the way: (map, lost)."""
    areas, docks, lost = [], [], []
    for area in stored.get("areas", []):
        properties = area.get("properties", {})
        name = properties.get("name", "")
        where = {"id": area.get("id", ""), "name": name}
        if properties.get("type") not in OPENMOWER_TYPES:
            lost.append({"kind": "area", **where, "type": properties.get("type")})
            continue
        if properties.get("active") is False:
            lost.append({"kind": "inactive", **where})
        for key, value in properties.items():
            if key not in AREA_PROPERTIES and key != "active" and DEFAULT_PROPERTIES.get(key) is not value:
                lost.append({"kind": "property", **where, "key": key})
        areas.append({"id": where["id"],
                      "properties": {k: properties[k] for k in AREA_PROPERTIES if k in properties},
                      "outline": [{"x": p["x"], "y": p["y"]} for p in area.get("outline", [])]})
    for dock in stored.get("docking_stations", []):
        properties = dock.get("properties", {})
        where = {"id": dock.get("id", ""), "name": properties.get("name", "")}
        out = {"id": where["id"]}
        if "name" in properties:
            out["properties"] = {"name": properties["name"]}
        lost.extend({"kind": "property", **where, "key": key} for key in properties if key != "name")
        out["position"] = {"x": dock["position"]["x"], "y": dock["position"]["y"]}
        out["heading"] = dock["heading"]
        docks.append(out)
    return {"areas": areas, "docking_stations": docks}, lost


def changed_data(data_dir):
    """[{kind: data, key}] for values only the OS keeps that differ from what the import brought."""
    note = read_note(data_dir) or {}
    imported = note.get("imported", {}) if note.get("state") == "done" else {}
    lost = []
    try:
        entries = read_json(os.path.join(data_dir, GEOMETRY)).get("entries", {})
    except (OSError, ValueError, AttributeError):
        entries = {}
    if entries and entries != imported.get("geometry"):
        lost.append({"kind": "data", "key": "geometry"})
    try:
        with open(os.path.join(data_dir, NTRIP), "rb") as f:
            ntrip = sha256(f.read())
    except OSError:
        ntrip = None
    if ntrip and ntrip != imported.get("ntrip_sha256"):
        lost.append({"kind": "data", "key": "ntrip"})
    return lost


def robot_state(wait=STATE_WAIT):
    """(current_state, current_sub_state) of the logic, None without one within wait s."""
    import rclpy
    from mowbite_msgs.msg import RobotState
    rclpy.init(args=None)
    try:
        node = rclpy.create_node("mowbite_migrate")
        got = []
        node.create_subscription(RobotState, "/bridge/robot_state",
                                 lambda m: got.append((m.current_state, m.current_sub_state)), 10)
        deadline = time.monotonic() + wait
        while not got and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()
        return got[0] if got else None
    finally:
        rclpy.shutdown()


def export(folder, data_dir, state=robot_state):
    """Exit code and one line: 0 written, 2 not at rest, 1 error."""
    now = state()
    if now is None:
        return 1, f"No state of the logic within {STATE_WAIT:g} s: is MowBite OS running?"
    if now != ("IDLE", ""):
        return 2, f"Not at rest ({' '.join(part for part in now if part)}): export refused, nothing written."
    try:
        try:
            stored = read_json(os.path.join(data_dir, MAP))
        except FileNotFoundError:
            stored = {}
        if not isinstance(stored, dict):
            raise ValueError(f"{MAP} holds no map")
        try:
            datum = read_json(os.path.join(data_dir, DATUM))
            datum = {key: datum[key] for key in DATUM_KEYS.values()}
        except (OSError, ValueError, KeyError, TypeError):
            datum = None
        result, lost = openmower_map(stored)
        lost.extend(changed_data(data_dir))
        os.makedirs(folder, exist_ok=True)
        data_store.write_json(folder, MAP, result)
        data_store.write_json(folder, REPORT, {"t": time.time(), "image": image(), "datum": datum,
                                               "areas": len(result["areas"]),
                                               "docking_stations": len(result["docking_stations"]), "lost": lost})
    except (OSError, ValueError, KeyError, TypeError) as error:
        return 1, f"Export failed: {error}"
    return 0, (f"Exported {len(result['areas'])} areas and {len(result['docking_stations'])} docking stations to "
               f"{folder}, {len(lost)} lost (report.json)")


def main(argv):
    args = list(argv)
    data_dir = os.environ.get("MOWBITE_DATA_DIR", "")
    if "--data-dir" in args:
        i = args.index("--data-dir")
        data_dir = args[i + 1] if i + 1 < len(args) else ""
        del args[i:i + 2]
    if len(args) != 2 or args[0] != "export" or not data_dir:
        print(__doc__, file=sys.stderr)
        return 1
    code, line = export(args[1], data_dir)
    print(line, file=sys.stdout if code == 0 else sys.stderr)
    return code


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
