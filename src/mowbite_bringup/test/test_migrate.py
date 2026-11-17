"""A mower from OpenMower and back (launch/migrate.py): what the import takes and leaves, the export.

Run: python3 -m pytest /ws/src/mowbite_bringup/test/test_migrate.py
"""
import json
import logging
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "launch")))
import migrate  # noqa: E402

SCHEMA = os.path.normpath(os.path.join(HERE, "..", "..", "mowbite_api", "api.schema.json"))
LOG = logging.getLogger("test_migrate")
SECRET = "pass'wo#rd"

MAP = {
    "areas": [
        {"id": "a1", "properties": {"name": "Front", "type": "mow", "angle": 0.5, "outline_count": 2},
         "outline": [{"x": 0.0, "y": 0.0}, {"x": 4.0, "y": 0.0}, {"x": 4.0, "y": 3.0}]},
        {"id": "a2", "properties": {"type": "obstacle"},
         "outline": [{"x": 1.0, "y": 1.0}, {"x": 1.5, "y": 1.0}, {"x": 1.5, "y": 1.5}]},
    ],
    "docking_stations": [{"id": "d1", "properties": {"name": "Dock"}, "position": {"x": 0.5, "y": -0.5},
                          "heading": 1.2}],
}

PARAMS = f"""
ll:
  bind_ip: 10.0.0.2
  services:
    gps:
      port_index: 6
      datum_lat: 52.1
      datum_long: 9.9
      datum_height: 80.5
    diff_drive:
      wheel_distance_m: 0.36
      ticks_per_m: 2000.0
    power:
      charge_voltage: 29.0
      battery_full_voltage: 28.0
      battery_critical_high_voltage: 29.5
ntrip_client:
  host: caster.example
  port: 2101
  username: someone
  password: "{SECRET}"
  mountpoint: MOUNT
xbot_positioning:
  antenna_offset_x: -0.12
  antenna_offset_y: 0.01
mower_logic:
  undock_distance: 1.5
xbot_monitoring:
  external_mqtt_hostname: broker.example
  external_mqtt_password: other
"""


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)


read_json = migrate.read_json


def source(tmp_path, map_value=MAP, params=PARAMS):
    folder = tmp_path / "import"
    if map_value is not None:
        write(str(folder / "ros" / "map.json"), json.dumps(map_value))
    if params is not None:
        write(str(folder / "params" / "mower_params.yaml"), params)
    return str(folder)


def tools(map_why=None, ntrip_why=None):
    """Stands in for mowbite_map_check and mowbite_ntrip_write, which keeps the access it gets as JSON."""
    def run(package, name, args, data):
        if name == "mowbite_map_check":
            return map_why
        assert name == "mowbite_ntrip_write"
        if ntrip_why:
            return ntrip_why
        fd = os.open(args[0], os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "wb") as f:
            f.write(data)
        return None
    return run


def take(tmp_path, folder, run=None):
    data_dir = tmp_path / "data"
    data_dir.mkdir(exist_ok=True)
    note = migrate.take(str(data_dir), folder, 1500000, LOG, migrate.geometry_ranges(SCHEMA), run or tools())
    return str(data_dir), note


def all_text(folder):
    out = ""
    for root, _, names in os.walk(folder):
        for name in names:
            with open(os.path.join(root, name), errors="replace") as f:
                out += f.read()
    return out


def test_the_import_takes_map_datum_geometry_and_ntrip(tmp_path):
    folder = source(tmp_path)
    data_dir, note = take(tmp_path, folder)
    assert note["state"] == "done"
    assert note["taken"] == ["map", "datum", "geometry.antenna_x", "geometry.antenna_y", "geometry.track",
                             "geometry.ticks_per_m", "ntrip"]
    with open(os.path.join(folder, "ros", "map.json"), "rb") as f, \
            open(os.path.join(data_dir, "map.json"), "rb") as g:
        assert f.read() == g.read()
    assert read_json(os.path.join(data_dir, "map_datum.json")) == {
        "schema": 1, "datum_lat": 52.1, "datum_long": 9.9, "datum_height": 80.5}
    assert read_json(os.path.join(data_dir, "geometry.json")) == {
        "version": 1, "entries": {"antenna_x": -0.12, "antenna_y": 0.01, "track": 0.36, "ticks_per_m": 2000.0}}
    # The access as mowbite_ntrip_write gets it, texts as texts.
    assert read_json(os.path.join(data_dir, "ntrip.yaml")) == {
        "host": "caster.example", "port": 2101, "mountpoint": "MOUNT", "user": "someone", "password": SECRET}
    assert read_json(os.path.join(data_dir, "migrate.json")) == note


def test_what_stays_behind_is_named_with_its_reason_and_never_with_a_value(tmp_path):
    _, note = take(tmp_path, source(tmp_path))
    assert note["not_taken"] == [
        {"key": "ll/bind_ip", "why": "platform"},
        {"key": "ll/services/gps/port_index", "why": "platform"},
        {"key": "ll/services/power/battery_critical_high_voltage", "why": "never"},
        {"key": "ll/services/power/battery_full_voltage", "why": "platform"},
        {"key": "ll/services/power/charge_voltage", "why": "never"},
        {"key": "mower_logic/undock_distance", "why": "os"},
        {"key": "xbot_monitoring", "why": "never"},
    ]
    note_text = json.dumps(note)
    for value in ("caster.example", "someone", "MOUNT", "wo#rd", "broker.example", "other", "10.0.0.2"):
        assert value not in note_text


def test_the_lock_stays_shut_and_nothing_of_the_blade_comes_over(tmp_path):
    data_dir, _ = take(tmp_path, source(tmp_path))
    assert sorted(os.listdir(data_dir)) == ["geometry.json", "map.json", "map_datum.json", "migrate.json",
                                            "ntrip.yaml"]
    os.remove(os.path.join(data_dir, "migrate.json"))
    text = all_text(data_dir)
    for word in ("allow_blade", "safety_measurement_date", "charge_voltage", "29.0", "external_mqtt"):
        assert word not in text


def test_a_map_the_check_refuses_stays_behind_with_why(tmp_path):
    data_dir, note = take(tmp_path, source(tmp_path), tools(map_why="areas must be an array"))
    assert "map" not in note["taken"] and "datum" not in note["taken"]
    assert {"key": "ros/map.json", "why": "invalid: areas must be an array"} in note["not_taken"]
    assert not os.path.exists(os.path.join(data_dir, "map.json"))


def test_values_out_of_range_stay_behind(tmp_path):
    params = PARAMS.replace("antenna_offset_x: -0.12", "antenna_offset_x: -12").replace(
        "datum_lat: 52.1", "datum_lat: 152.1").replace("  host: caster.example\n", "")
    data_dir, note = take(tmp_path, source(tmp_path, params=params))
    assert note["taken"] == ["map", "geometry.antenna_y", "geometry.track", "geometry.ticks_per_m"]
    for item in ({"key": "xbot_positioning/antenna_offset_x", "why": "invalid"},
                 {"key": "ll/services/gps/datum", "why": "invalid"},
                 {"key": "ntrip_client", "why": "invalid: host or mountpoint missing"}):
        assert item in note["not_taken"]
    assert not os.path.exists(os.path.join(data_dir, "map_datum.json"))
    assert not os.path.exists(os.path.join(data_dir, "ntrip.yaml"))


def test_an_access_the_ntrip_package_refuses_stays_behind_with_why(tmp_path):
    data_dir, note = take(tmp_path, source(tmp_path), tools(ntrip_why="line 1: port: not a port number"))
    assert "ntrip" not in note["taken"]
    assert {"key": "ntrip_client", "why": "invalid: line 1: port: not a port number"} in note["not_taken"]
    assert "ntrip_sha256" not in note["imported"]


def test_missing_sources_are_noted_and_nothing_is_written(tmp_path):
    data_dir, note = take(tmp_path, source(tmp_path, map_value=None, params=None))
    assert note["state"] == "done" and note["taken"] == [] and note["not_taken"] == []
    assert note["sources"] == {"ros/map.json": {"missing": True}, "params/mower_params.yaml": {"missing": True}}
    assert os.listdir(data_dir) == ["migrate.json"]


def test_a_broken_parameter_file_fails_the_import_but_not_the_start(tmp_path):
    data_dir, note = take(tmp_path, source(tmp_path, params="ll: [1, 2"))
    assert note["state"] == "failed" and note["error"]
    assert read_json(os.path.join(data_dir, "migrate.json"))["state"] == "failed"
    assert not migrate.wanted(data_dir)


def test_only_a_first_start_or_a_cut_off_import_imports(tmp_path):
    data_dir = tmp_path / "data"
    data_dir.mkdir()
    (data_dir / "data_schema.json").write_text('{"schema": 3}')
    assert migrate.wanted(str(data_dir))
    (data_dir / "migrate.json").write_text('{"state": "pending"}')
    assert migrate.wanted(str(data_dir))
    (data_dir / "migrate.json").write_text('{"state": "done"}')
    assert not migrate.wanted(str(data_dir))
    (data_dir / "migrate.json").write_text("{")
    assert not migrate.wanted(str(data_dir))
    (data_dir / "migrate.json").unlink()
    (data_dir / "map.json").write_text("{}")
    assert not migrate.wanted(str(data_dir))


def test_a_cut_off_import_starts_over_without_what_it_left(tmp_path):
    data_dir = tmp_path / "data"
    data_dir.mkdir()
    (data_dir / "migrate.json").write_text('{"state": "pending"}')
    (data_dir / "map.json.bak").write_text("left")
    (data_dir / "geometry.json").write_text("left")
    _, note = take(tmp_path, source(tmp_path, params=None))
    assert note["taken"] == ["map"]
    assert sorted(os.listdir(data_dir)) == ["map.json", "migrate.json"]


def export(tmp_path, data_dir, state=("IDLE", "")):
    out = tmp_path / "export"
    code, line = migrate.export(str(out), data_dir, state=lambda: state)
    return code, line, str(out)


def test_the_export_gives_back_the_map_in_openmowers_fields(tmp_path):
    data_dir, _ = take(tmp_path, source(tmp_path))
    code, line, out = export(tmp_path, data_dir)
    assert code == 0, line
    assert read_json(os.path.join(out, "map.json")) == MAP
    report = read_json(os.path.join(out, "report.json"))
    assert report["lost"] == [] and report["areas"] == 2 and report["docking_stations"] == 1
    assert report["datum"] == {"datum_lat": 52.1, "datum_long": 9.9, "datum_height": 80.5}
    assert SECRET not in all_text(out) and "caster.example" not in all_text(out)


def test_the_export_names_what_openmower_cannot_hold(tmp_path):
    data_dir = str(tmp_path / "data")
    stored = json.loads(json.dumps(MAP))
    stored["areas"][0]["properties"].update({"active": False, "outline_offset": 0.1, "mowable": True})
    stored["areas"].append({"id": "a3", "properties": {"name": "Bed", "type": "decor"}, "outline": []})
    os.makedirs(data_dir)
    write(os.path.join(data_dir, "map.json"), json.dumps(stored))
    write(os.path.join(data_dir, "geometry.json"), json.dumps({"version": 2, "entries": {"track": 0.4}}))
    write(os.path.join(data_dir, "ntrip.yaml"), "host: 'x'\n")
    code, _, out = export(tmp_path, data_dir)
    assert code == 0
    assert read_json(os.path.join(out, "map.json"))["areas"] == [
        MAP["areas"][0], MAP["areas"][1]]
    assert read_json(os.path.join(out, "report.json"))["lost"] == [
        {"kind": "inactive", "id": "a1", "name": "Front"},
        {"kind": "property", "id": "a1", "name": "Front", "key": "outline_offset"},
        {"kind": "area", "id": "a3", "name": "Bed", "type": "decor"},
        {"kind": "data", "key": "geometry"},
        {"kind": "data", "key": "ntrip"},
    ]


def test_values_changed_since_the_import_are_lost_and_unchanged_ones_not(tmp_path):
    data_dir, _ = take(tmp_path, source(tmp_path))
    code, _, out = export(tmp_path, data_dir)
    assert code == 0 and read_json(os.path.join(out, "report.json"))["lost"] == []
    write(os.path.join(data_dir, "geometry.json"), json.dumps({"version": 2, "entries": {"track": 0.4}}))
    code, _, out = export(tmp_path, data_dir)
    assert read_json(os.path.join(out, "report.json"))["lost"] == [{"kind": "data", "key": "geometry"}]


def test_the_export_refuses_unless_the_mower_rests(tmp_path):
    data_dir, _ = take(tmp_path, source(tmp_path))
    for state in (("MOWING", ""), ("IDLE", "SETUP"), ("AREA_RECORDING", "")):
        code, line, out = export(tmp_path, data_dir, state)
        assert code == 2, line
        assert not os.path.exists(out)
    code, line, out = export(tmp_path, data_dir, None)
    assert code == 1 and "running" in line


def test_the_export_without_a_map_gives_an_empty_one(tmp_path):
    data_dir = tmp_path / "data"
    data_dir.mkdir()
    code, _, out = export(tmp_path, str(data_dir))
    assert code == 0
    assert read_json(os.path.join(out, "map.json")) == {"areas": [], "docking_stations": []}


def test_the_command_needs_export_a_folder_and_data_dir(tmp_path, monkeypatch):
    monkeypatch.delenv("MOWBITE_DATA_DIR", raising=False)
    assert migrate.main(["export", str(tmp_path)]) == 1
    assert migrate.main(["import", str(tmp_path), "--data-dir", str(tmp_path)]) == 1
