"""The NX60i in the driver contract (test_driver_contract.py): xcore_driver against fake_xcore on UDP over the
loopback."""
import math
import struct

from sim_common import (fake_xcore, layers, log_entries, loopback_group, read_json, sim_files, write_json,
                        xcore_config, xcore_driver)

DIFF_DRIVE, MOWER = 2, 3
DATA = 1


class Case:
    driver_delay = 0.5

    def __init__(self, work, hold_file):
        self.state, self.flags, self.log, _ = sim_files(work)
        self.hold_file = hold_file
        self.group_port, self.bind_ip = loopback_group()
        values = layers.parameters([xcore_config()], "xcore_driver")
        self.slope, self.deadband, self.wheel_distance = (
            values["duty_slope"], values["duty_deadband"], values["wheel_distance"])
        self.cmd_vel_timeout = values["cmd_vel_timeout"]

    def board(self):
        return fake_xcore(self.group_port, "--flags", self.flags, "--log", self.log, "--log-max-mb", "20",
                          "--state", self.state)

    def driver(self):
        return xcore_driver(self.bind_ip, self.group_port, allow_blade=True, hold_file=self.hold_file)

    def inputs(self, stop=False, lift=False):
        write_json(self.flags, {"stop": int(stop), "lift_left": int(lift)})

    def time(self):
        return read_json(self.state)["t"]

    def _received(self, since, sid):
        return [(e["t"], bytes.fromhex(e["hex"])) for e in log_entries(self.log, since)
                if e.get("sid") == sid and e["type"] == DATA and e["arg2"] == 0]

    def _wheel(self, duty):
        """The speed (m/s) a wheel turns at with that duty."""
        return math.copysign(self.slope * (abs(duty) - self.deadband), duty) if abs(duty) > self.deadband else 0.0

    def drive(self, since):
        commands = []
        for t, data in self._received(since, DIFF_DRIVE):
            twist = struct.unpack("<6d", data)
            # The board splits the twist into the duties of the wheels with the distance it was configured with.
            left = self._wheel(twist[0] - twist[5] * self.wheel_distance / 2)
            right = self._wheel(twist[0] + twist[5] * self.wheel_distance / 2)
            commands.append((t, (left + right) / 2, (right - left) / self.wheel_distance))
        return commands

    def blade(self, since):
        return [(t, struct.unpack("<f", data)[0] != 0) for t, data in self._received(since, MOWER)]
