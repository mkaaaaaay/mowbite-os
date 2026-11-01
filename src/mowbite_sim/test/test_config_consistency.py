"""The dimensions of the platform stand in one file, config/geometry/tron.yaml. Two places repeat them for now: the
planner section of tron.yaml, until the planner takes the body from /mowbite_geometry, and the station of fake_tron in
sim.yaml. They have to agree, and no node may be given an antenna offset or contact position of its own any more.

Run in a sourced workspace: python3 /ws/src/mowbite_sim/test/test_config_consistency.py
"""
import os
import unittest

import yaml
from ament_index_python.packages import get_package_share_directory

BRINGUP = get_package_share_directory("mowbite_bringup")
SIM = get_package_share_directory("mowbite_sim")


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
        for key in ("robot_width", "robot_front", "robot_rear", "mower_width", "blade_ahead"):
            self.assertAlmostEqual(planner[key], self.geometry[key], msg=key)

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


if __name__ == "__main__":
    unittest.main()
