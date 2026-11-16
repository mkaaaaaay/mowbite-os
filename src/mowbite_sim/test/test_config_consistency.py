"""The dimensions of the platform stand in one file, config/geometry/tron.yaml. One place repeats one of them, the
station of fake_tron in sim.yaml: it has to agree. No node may be given an antenna offset or contact position of its
own any more, and no planner section the body or the blade, which the planner takes from /mowbite_geometry.
The fence of mowbite_safety lies beyond the one of the follower, so that it acts only when the follower does not.
The NX60i the same way: config/nx60i.yaml on top of stack.yaml has the battery voltages, which nx60i.launch.py hands
to the driver, so its own file has none; the driver
charges at 29.4 V at most (G29) and has the speed model of A2.

Run in a sourced workspace: python3 /ws/src/mowbite_sim/test/test_config_consistency.py
"""
import os
import sys
import unittest

import yaml
from ament_index_python.packages import get_package_share_directory
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
        platform = load(os.path.join(BRINGUP, "config", "geometry", "tron.yaml"))
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
        platform = load(os.path.join(BRINGUP, "config", "geometry", "nx60i.yaml"))
        self.geometry = platform["mowbite_logic"]["ros__parameters"]["geometry"]
        self.nx60i = load(os.path.join(BRINGUP, "config", "nx60i.yaml"))
        self.driver = load(os.path.join(XCORE, "config", "nx60i.yaml"))["xcore_driver"]["ros__parameters"]

    def test_the_planner_section_has_no_body_or_blade(self):
        self.assertFalse(from_geometry(self.nx60i))

    def test_the_driver_gets_the_battery_of_the_platform_file_only(self):
        battery = logic_params.battery_voltages(layers.files("nx60i"))
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


if __name__ == "__main__":
    unittest.main()
