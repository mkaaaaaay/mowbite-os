"""The dimensions of the platform stand in one file, its geometry.yaml. One place repeats one of them, the
station of fake_tron in sim.yaml: it has to agree. No node may be given an antenna offset or contact position of its
own any more, and no planner section the body or the blade, which the planner takes from /mowbite_geometry.
The fence of mowbite_safety lies beyond the one of the follower, so that it acts only when the follower does not.
The NX60i the same way: its platform.yaml on top of stack.yaml has the battery voltages, which its platform.py hands
to the driver, so its own file has none; the driver
charges at 29.4 V at most (G29) and has the speed model of A2.

Run in a sourced workspace: python3 /ws/src/mowbite_sim/test/test_config_consistency.py
"""
import glob
import json
import math
import os
import subprocess
import sys
import unittest

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from mowbite_planner_os.node import GEOMETRY_KEYS

BRINGUP = get_package_share_directory("mowbite_bringup")
SIM = get_package_share_directory("mowbite_sim")
XCORE = get_package_share_directory("xcore_driver")

sys.path.insert(0, os.path.join(XCORE, "launch"))
sys.path.insert(0, os.path.join(BRINGUP, "launch"))
import layers  # noqa: E402
import logic_params  # noqa: E402


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def from_geometry(config):
    """The keys of a planner section that the planner takes from /mowbite_geometry instead."""
    section = config.get("mowbite_planner", {}).get("ros__parameters", {})
    return set(section) & set(GEOMETRY_KEYS)


class ConfigConsistency(unittest.TestCase):
    def setUp(self):
        platform = load(layers.geometry("tron"))
        self.geometry = platform["mowbite_logic"]["ros__parameters"]["geometry"]
        self.files = layers.files("tron")
        self.configs = [load(path) for path in self.files]
        self.sim = load(os.path.join(SIM, "config", "sim.yaml"))

    def test_no_planner_section_has_the_body_or_the_blade(self):
        self.assertIn("mower_width", GEOMETRY_KEYS)
        for config in (*self.configs, self.sim):
            self.assertFalse(from_geometry(config))

    def test_the_fence_of_the_safety_filter_lies_beyond_the_one_of_the_follower(self):
        safety = layers.parameters(self.files, "mowbite_safety")
        follower = layers.parameters(self.files, "mowbite_follower")
        self.assertGreater(safety["fence_margin"], follower["geofence_margin"])
        self.assertLess(safety["tilt_release"], safety["tilt_limit"])

    def test_the_battery_voltages_go_critical_empty_full(self):
        logic = layers.parameters(self.files, "mowbite_logic")
        self.assertLess(0, logic["battery_critical_voltage"])
        self.assertLess(logic["battery_critical_voltage"], logic["battery_empty_voltage"])
        self.assertLess(logic["battery_empty_voltage"], logic["battery_full_voltage"])

    def test_the_station_of_the_simulation_has_the_contacts_of_the_platform(self):
        self.assertAlmostEqual(self.sim["fake_tron"]["ros__parameters"]["dock_contact_x"],
                               self.geometry["dock_contact_x"])

    def test_no_node_has_an_antenna_offset_or_contacts_of_its_own(self):
        for config in (*self.configs, self.sim):
            for node, section in config.items():
                for key in ("antenna_offset_x", "antenna_offset_y", "dock_contact_x"):
                    if node == "fake_tron":
                        continue
                    self.assertNotIn(key, section.get("ros__parameters", {}), f"{node}.{key}")

    def test_the_simulation_changes_only_known_keys_of_the_geometry(self):
        changes = self.sim["mowbite_logic"]["ros__parameters"]["geometry"]
        self.assertTrue(set(changes) <= set(self.geometry), set(changes) - set(self.geometry))


class Nx60iConsistency(unittest.TestCase):
    def setUp(self):
        platform = load(layers.geometry("nx60i"))
        self.geometry = platform["mowbite_logic"]["ros__parameters"]["geometry"]
        self.nx60i = load(layers.platform_file("nx60i"))
        self.driver = load(os.path.join(XCORE, "config", "nx60i.yaml"))["xcore_driver"]["ros__parameters"]

    def test_the_planner_section_has_no_body_or_blade(self):
        self.assertFalse(from_geometry(self.nx60i))

    def test_the_driver_gets_the_battery_of_the_platform_file_only(self):
        battery = logic_params.battery_voltages(layers.parameters(layers.files("nx60i"), "mowbite_logic"))
        logic = self.nx60i["mowbite_logic"]["ros__parameters"]
        self.assertEqual(battery, {key: logic[key] for key in logic_params.BATTERY_VOLTAGES})
        self.assertLess(battery["battery_critical_voltage"], battery["battery_empty_voltage"])
        self.assertLess(battery["battery_empty_voltage"], battery["battery_full_voltage"])
        self.assertLess(battery["battery_full_voltage"], self.driver["battery_critical_high_voltage"])
        self.assertFalse([key for key in self.driver if key in logic_params.BATTERY_VOLTAGES or "critical_low" in key])

    def test_the_driver_charges_within_g29_and_has_the_speed_model(self):
        self.assertLessEqual(self.driver["charge_voltage"], 29.4)
        self.assertNotIn("full_duty_speed", self.driver)
        self.assertGreater(self.driver["duty_slope"], 0)
        self.assertLess(self.driver["duty_deadband"], self.driver["max_duty"])
        self.assertFalse(self.driver["allow_blade"])
        self.assertEqual(self.driver["safety_measurement_date"], "")


class DriverContract(unittest.TestCase):
    def test_every_platform_has_its_case_of_the_driver_contract(self):
        here = os.path.dirname(os.path.abspath(__file__))
        cases = {os.path.basename(path)[len("contract_"):-len(".py")]
                 for path in glob.glob(os.path.join(here, "contract_*.py"))}
        self.assertEqual(cases, set(layers.platforms()))


def inside(polygon, x, y, slack):
    """Whether the point lies in the polygon or within slack of its edge."""
    hit = False
    for (ax, ay), (bx, by) in zip(polygon, polygon[1:] + polygon[:1]):
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / ((dx * dx + dy * dy) or 1.0)))
        if math.hypot(ax + t * dx - x, ay + t * dy - y) <= slack:
            return True
        if (ay > y) != (by > y) and x < ax + (y - ay) * dx / dy:
            hit = not hit
    return hit


class ParameterFiles(unittest.TestCase):
    """What a node cannot read ends its start, the whole file with it."""

    def mixed(self, value, path=""):
        """The lists under value whose values are not of one type, by their path."""
        if isinstance(value, dict):
            return [found for name, inner in value.items() for found in self.mixed(inner, f"{path}.{name}")]
        return [path] if isinstance(value, list) and len({type(item) for item in value}) > 1 else []

    def test_every_list_has_values_of_one_type(self):
        # rcl reads no list of whole and other numbers side by side (0 beside 0.5).
        for platform in layers.platforms():
            for path in (layers.geometry(platform), layers.platform_file(platform)):
                self.assertEqual(self.mixed(load(path)), [], path)


class SetupProgram(unittest.TestCase):
    """What every platform gives the setup program and the app: its step list and how it looks from above."""
    # Parts the app derives from the dimensions; a step may mark them like a part of robot_shape.
    DERIVED = {"wheel_left", "wheel_right", "blade", "antenna", "contacts", "outline"}

    def setUp(self):
        with open(os.path.join(get_package_share_directory("mowbite_api"), "api.schema.json")) as f:
            self.catalog = json.load(f)["x-setup"]

    def shape(self, platform):
        geometry = load(layers.geometry(platform))["mowbite_logic"]["ros__parameters"]["geometry"]
        shape = geometry.get("robot_shape", {})
        parts = {name: shape[name] for name in shape.get("parts", [])}
        return geometry, parts

    def test_the_step_list_holds_by_the_rules_of_the_node(self):
        tool = os.path.join(get_package_prefix("mowbite_setup"), "lib", "mowbite_setup", "mowbite_setup_check")
        with_list = []
        for platform in layers.platforms():
            setup = layers.setup(platform)
            if not setup["steps"]:
                continue
            with_list.append(platform)
            result = subprocess.run([tool, setup["image_dir"]], input=setup["steps"], capture_output=True, text=True)
            self.assertEqual((result.returncode, result.stdout), (0, ""), platform)
            _, parts = self.shape(platform)
            for step in json.loads(setup["steps"])["steps"]:
                self.assertFalse(set(step.get("mark", [])) - set(parts) - self.DERIVED, (platform, step["id"]))
        self.assertEqual(with_list, ["nx60i", "tron"])

    def test_the_outline_covers_the_shape_from_above(self):
        limits = self.catalog["limits"]
        shaped = []
        for platform in layers.platforms():
            geometry, parts = self.shape(platform)
            if not parts:
                continue
            shaped.append(platform)
            flat = geometry.get("robot_outline")
            if flat:
                body = list(zip(flat[::2], flat[1::2]))
            else:
                front, rear, half = geometry["robot_front"], -geometry["robot_rear"], geometry["robot_width"] / 2
                body = [(front, half), (rear, half), (rear, -half), (front, -half)]
            self.assertLessEqual(len(parts), limits["shape_parts"], platform)
            total = 0
            for name, part in parts.items():
                self.assertIn(part["kind"], self.catalog["shape_kinds"], (platform, name))
                points = list(zip(part["points"][::2], part["points"][1::2]))
                self.assertEqual(len(points) * 2, len(part["points"]), (platform, name))
                self.assertTrue(0 < len(points) <= limits["shape_part_points"], (platform, name))
                total += len(points)
                # A millimetre is what a ruler does not tell apart, as robot.geometry.set counts it.
                outside = [point for point in points if not inside(body, *point, 0.001)]
                self.assertFalse(outside, (platform, name))
            self.assertLessEqual(total, limits["shape_points"], platform)
        self.assertEqual(shaped, ["tron"])


if __name__ == "__main__":
    unittest.main()
