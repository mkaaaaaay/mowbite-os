"""The dimensions of the platform stand in one file, config/geometry/tron.yaml. Two places repeat them for now: the
planner section of tron.yaml, until the planner takes the body from /mowbite_geometry, and the station of fake_tron in
sim.yaml. They have to agree, and no node may be given an antenna offset or contact position of its own any more.
The fence of mowbite_safety lies beyond the one of the follower, so that it acts only when the follower does not.
The NX60i the same way: config/nx60i.yaml on top of tron.yaml has the body of config/geometry/nx60i.yaml for the
planner and the battery of the driver's xcore_driver/config/nx60i.yaml for the logic; the driver charges at 29.4 V at
most (G29) and has the speed model of A2.

Run in a sourced workspace: python3 /ws/src/mowbite_sim/test/test_config_consistency.py
"""
import os
import unittest

import yaml
from ament_index_python.packages import get_package_share_directory

BRINGUP = get_package_share_directory("mowbite_bringup")
SIM = get_package_share_directory("mowbite_sim")
XCORE = get_package_share_directory("xcore_driver")
BODY = ("robot_width", "robot_front", "robot_rear", "mower_width", "blade_ahead")


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


class ConfigConsistency(unittest.TestCase):
    def setUp(self):
        platform = load(os.path.join(BRINGUP, "config", "geometry", "tron.yaml"))
        self.geometry = platform["mowbite_logic"]["ros__parameters"]["geometry"]
        self.tron = load(os.path.join(BRINGUP, "config", "tron.yaml"))
        self.sim = load(os.path.join(SIM, "config", "sim.yaml"))

    def test_the_planner_section_has_the_body_of_the_platform(self):
        planner = self.tron["mowbite_planner"]["ros__parameters"]
        for key in BODY:
            self.assertAlmostEqual(planner[key], self.geometry[key], msg=key)

    def test_the_fence_of_the_safety_filter_lies_beyond_the_one_of_the_follower(self):
        safety = self.tron["mowbite_safety"]["ros__parameters"]
        self.assertGreater(safety["fence_margin"], self.tron["mowbite_follower"]["ros__parameters"]["geofence_margin"])
        self.assertLess(safety["tilt_release"], safety["tilt_limit"])

    def test_the_station_of_the_simulation_has_the_contacts_of_the_platform(self):
        self.assertAlmostEqual(self.sim["fake_tron"]["ros__parameters"]["dock_contact_x"],
                               self.geometry["dock_contact_x"])

    def test_no_node_has_an_antenna_offset_or_contacts_of_its_own(self):
        for config in (self.tron, self.sim):
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
        self.tron = load(os.path.join(BRINGUP, "config", "tron.yaml"))
        self.nx60i = load(os.path.join(BRINGUP, "config", "nx60i.yaml"))
        self.driver = load(os.path.join(XCORE, "config", "nx60i.yaml"))["xcore_driver"]["ros__parameters"]

    def test_the_planner_has_the_body_of_the_platform(self):
        planner = self.nx60i["mowbite_planner"]["ros__parameters"]
        for key in BODY:
            self.assertAlmostEqual(planner[key], self.geometry[key], msg=key)

    def test_the_logic_has_the_battery_of_the_driver(self):
        logic = self.nx60i["mowbite_logic"]["ros__parameters"]
        self.assertEqual(logic["battery_empty_voltage"], self.driver["battery_empty_voltage"])
        self.assertEqual(logic["battery_full_voltage"], self.driver["battery_full_voltage"])
        self.assertEqual(logic["battery_critical_voltage"], self.driver["battery_critical_low_voltage"])

    def test_only_known_nodes_and_keys_change(self):
        for node, section in self.nx60i.items():
            self.assertIn(node, self.tron, node)
            known = self.tron[node]["ros__parameters"]
            self.assertTrue(set(section["ros__parameters"]) <= set(known), set(section["ros__parameters"]) - set(known))

    def test_the_driver_charges_within_g29_and_has_the_speed_model(self):
        self.assertLessEqual(self.driver["charge_voltage"], 29.4)
        self.assertNotIn("full_duty_speed", self.driver)
        self.assertGreater(self.driver["duty_slope"], 0)
        self.assertLess(self.driver["duty_deadband"], self.driver["max_duty"])
        self.assertFalse(self.driver["allow_blade"])
        self.assertEqual(self.driver["safety_measurement_date"], "")


if __name__ == "__main__":
    unittest.main()
