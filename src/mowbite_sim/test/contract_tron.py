"""The Tron in the driver contract (test_driver_contract.py): tron_mcu against fake_tron on a pty."""
from launch_ros.actions import Node
from sim_common import TRON_CUTTER as CUTTER, TRON_SPEED as SPEED
from sim_common import fake_tron, last_time, layers, sim_files, sim_links, sub_packets, write_json


class Case:
    # The pty has to exist before the driver looks for it.
    driver_delay = 1.0

    def __init__(self, work, hold_file):
        self.links = sim_links(work)
        _, self.flags, self.log, _ = sim_files(work)
        self.hold_file = hold_file
        self.files = layers.files("tron")
        self.cmd_vel_timeout = layers.parameters(self.files, "tron_mcu")["cmd_vel_timeout"]

    def board(self):
        return fake_tron(self.links, "--flags", self.flags, "--log", self.log, "--log-max-mb", "5")

    def driver(self):
        # Name and namespace as in the start, so the values below go over those of the files.
        return Node(package="tron_mcu", executable="tron_mcu_node", name="tron_mcu", namespace="/", output="screen",
                    parameters=[*self.files, {"port": self.links["mcu"], "send_imu": False, "allow_blade": True,
                                              "hold_file": self.hold_file}])

    def inputs(self, stop=False, lift=False):
        write_json(self.flags, {"stop": int(stop), "lift": int(lift)})

    def time(self):
        return last_time(self.log)

    def drive(self, since):
        return [(e["t"], e["speed"][0], e["speed"][1]) for e in sub_packets(self.log, since, SPEED)]

    def blade(self, since):
        return [(e["t"], e["hex"][:2] == "01") for e in sub_packets(self.log, since, CUTTER)]
