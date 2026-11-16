"""Tests of tron-stereo-ae against stand-in registers of the bridge: pages, the bypass to both sensors, take-over flags.

  python3 -I platform/tron/rootfs/test_stereo_ae.py
"""
import contextlib
import importlib.machinery
import importlib.util
import io
import os
import sys
import unittest

sys.dont_write_bytecode = True
TOOL = os.path.join(os.path.dirname(os.path.realpath(__file__)), "files", "usr", "lib", "mowbite", "tron-stereo-ae")
XC9080, SC132 = 0x1B, 0x30
LIMITS_PAGE, BYPASS_PAGE, KERNEL_PAGE = (0x80, 0x14), (0x80, 0x50), (0x80, 0x26)
EXPOSURE, GAIN, GAIN_MIN, TAKE_OVER = (0x00E2, 0x0A96), (0x00FA, 0x0AAE), (0x00FC, 0x0AB0), (0x0050, 0x0A04)


class Bridge:
    """XC9080 registers per page, the SC132 behind 004d = 1 or 2; a take-over flag reads 0 again."""

    def __init__(self, exposure=(0x20, 0x00), gain=(0x00, 0x40)):
        self.page = KERNEL_PAGE
        self.regs = {}
        for regs, pair in ((EXPOSURE, exposure), (GAIN, gain), (GAIN_MIN, (0x00, 0x20))):
            for reg in regs:
                self.set_pair(reg, pair)
        self.sensor = {1: {0x3E03: 0x0B}, 2: {0x3E03: 0x0B}}
        self.writes = []
        self.stuck = set()
        self.read_error = set()

    def set_pair(self, reg, pair):
        self.regs[(LIMITS_PAGE, reg)], self.regs[(LIMITS_PAGE, reg + 1)] = pair

    def pair(self, reg):
        return self.regs[(LIMITS_PAGE, reg)], self.regs[(LIMITS_PAGE, reg + 1)]

    def bypass(self):
        return self.regs.get((BYPASS_PAGE, 0x004D), 0)

    def read(self, _fd, addr, reg):
        if addr == SC132:
            return self.sensor.get(self.bypass(), {}).get(reg, 0)
        if reg == 0xFFFD:
            return self.page[0]
        if reg == 0xFFFE:
            return self.page[1]
        if (self.page, reg) in self.read_error:
            raise OSError(121, "Remote I/O error")
        return self.regs.get((self.page, reg), 0)

    def put(self, _fd, addr, reg, value):
        if addr == SC132:
            if self.bypass() not in self.sensor:
                raise OSError(6, "No such device or address")
            self.writes.append((f"sensor{self.bypass()}", reg, value))
            self.sensor[self.bypass()][reg] = value
        elif reg == 0xFFFD:
            self.page = (value, self.page[1])
        elif reg == 0xFFFE:
            self.page = (self.page[0], value)
        else:
            self.writes.append((self.page, reg, value))
            if (self.page, reg) in self.stuck or (self.page == LIMITS_PAGE and reg in TAKE_OVER):
                return
            self.regs[(self.page, reg)] = value

    def limit_writes(self):
        return [(reg, value) for page, reg, value in self.writes if page == LIMITS_PAGE]


def load():
    loader = importlib.machinery.SourceFileLoader("tron_stereo_ae", TOOL)
    module = importlib.util.module_from_spec(importlib.util.spec_from_loader("tron_stereo_ae", loader))
    loader.exec_module(module)
    return module


class StereoAe(unittest.TestCase):
    def run_tool(self, bridge):
        tool = load()
        tool.read, tool.put = bridge.read, bridge.put
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            try:
                code = tool.control_on(None)
            except SystemExit as e:
                code = e.code
        return code, out.getvalue()

    def assert_left_clean(self, bridge):
        self.assertEqual(bridge.bypass(), 0)
        self.assertEqual(bridge.page, KERNEL_PAGE)

    def assert_control_on(self, bridge):
        self.assertEqual([bridge.sensor[n][0x3E03] for n in (1, 2)], [0x2B, 0x2B])
        self.assert_left_clean(bridge)

    def test_bridge_table_gets_both_limits(self):
        bridge = Bridge()
        code, out = self.run_tool(bridge)
        self.assertEqual(code, 0)
        self.assertEqual([bridge.pair(reg) for reg in EXPOSURE], [(0x53, 0x00)] * 2)
        self.assertEqual([bridge.pair(reg) for reg in GAIN], [(0x00, 0x7F)] * 2)
        self.assertEqual([bridge.pair(reg) for reg in GAIN_MIN], [(0x00, 0x20)] * 2)
        self.assertIn("exposure limit raised", out)
        self.assertIn("gain limit raised", out)
        self.assert_control_on(bridge)

    def test_limits_are_written_with_the_control_off_and_taken_over_once(self):
        bridge = Bridge()
        self.run_tool(bridge)
        first_limit = next(i for i, (page, _, _) in enumerate(bridge.writes) if page == LIMITS_PAGE)
        self.assertIn((BYPASS_PAGE, 0x004D, 0x03), bridge.writes[:first_limit])
        self.assertEqual(bridge.limit_writes()[-2:], [(reg, 0x01) for reg in TAKE_OVER])
        self.assertEqual(sum(1 for reg, _ in bridge.limit_writes() if reg in TAKE_OVER), 2)

    def test_raised_limits_are_not_written_again(self):
        bridge = Bridge(exposure=(0x53, 0x00), gain=(0x00, 0x7F))
        code, out = self.run_tool(bridge)
        self.assertEqual(code, 0)
        self.assertEqual(bridge.limit_writes(), [])
        self.assertIn("gain limit already raised", out)
        self.assert_control_on(bridge)

    def test_raised_exposure_still_gets_the_gain(self):
        bridge = Bridge(exposure=(0x53, 0x00))
        code, _ = self.run_tool(bridge)
        self.assertEqual(code, 0)
        self.assertEqual([bridge.pair(reg) for reg in GAIN], [(0x00, 0x7F)] * 2)
        self.assertEqual({reg for reg, _ in bridge.limit_writes()}, {0x00FA, 0x00FB, 0x0AAE, 0x0AAF, *TAKE_OVER})

    def test_a_fixed_coarse_step_stays(self):
        bridge = Bridge(gain=(0x27, 0x3F))
        code, out = self.run_tool(bridge)
        self.assertEqual(code, 0)
        self.assertEqual([bridge.pair(reg) for reg in GAIN], [(0x27, 0x3F)] * 2)
        self.assertEqual([bridge.pair(reg) for reg in EXPOSURE], [(0x53, 0x00)] * 2)
        self.assertIn("gain limit left at 27 3f, 27 3f", out)
        self.assert_control_on(bridge)

    def test_channels_that_differ_stay(self):
        bridge = Bridge()
        bridge.set_pair(GAIN[1], (0x00, 0x7F))
        code, out = self.run_tool(bridge)
        self.assertEqual(code, 0)
        self.assertEqual([bridge.pair(reg) for reg in GAIN], [(0x00, 0x40), (0x00, 0x7F)])
        self.assertIn("gain limit left at 00 40, 00 7f", out)

    def test_a_limit_that_does_not_read_back_skips_the_take_over(self):
        bridge = Bridge()
        bridge.stuck.add((LIMITS_PAGE, 0x0AAF))
        code, out = self.run_tool(bridge)
        self.assertEqual(code, 0)
        self.assertFalse([reg for reg, _ in bridge.limit_writes() if reg in TAKE_OVER])
        self.assertIn("limits not raised, no take-over", out)
        self.assert_control_on(bridge)

    def test_a_read_error_at_the_limits_leaves_the_control_on(self):
        bridge = Bridge()
        bridge.read_error.add((LIMITS_PAGE, 0x00FA))
        code, out = self.run_tool(bridge)
        self.assertEqual(code, 0)
        self.assertFalse([reg for reg, _ in bridge.limit_writes() if reg in TAKE_OVER])
        self.assertIn("limits: [Errno 121]", out)
        self.assert_control_on(bridge)

    def test_a_sensor_that_does_not_answer_ends_with_1_and_a_clean_bridge(self):
        bridge = Bridge()
        del bridge.sensor[2]
        code, out = self.run_tool(bridge)
        self.assertEqual(code, 1)
        self.assertIn("exposure stays fixed", out)
        self.assert_left_clean(bridge)

    def test_a_bypass_that_cannot_be_set_back_ends_with_2(self):
        stuck = Bridge(exposure=(0x53, 0x00), gain=(0x00, 0x7F))
        stuck.regs[(BYPASS_PAGE, 0x004D)] = 0x02
        stuck.stuck.add((BYPASS_PAGE, 0x004D))
        code, out = self.run_tool(stuck)
        self.assertEqual(code, 2)
        self.assertIn("004d or page not set back", out)


if __name__ == "__main__":
    unittest.main()
