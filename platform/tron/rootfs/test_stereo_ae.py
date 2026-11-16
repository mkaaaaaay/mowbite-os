"""Tests of tron-stereo-ae against stand-in registers of the bridge: pages, the bypass to both sensors, take-over flags.

  python3 -I platform/tron/rootfs/test_stereo_ae.py
"""
import contextlib
import fcntl
import importlib.machinery
import importlib.util
import io
import os
import shutil
import sys
import tempfile
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


def quiet(call, *args):
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        return call(*args), out.getvalue()


class GainLevel(unittest.TestCase):
    def setUp(self):
        self.tool = load()
        self.bridge = Bridge(exposure=(0x53, 0x00), gain=(0x00, 0x7F))
        self.tool.read, self.tool.put = self.bridge.read, self.bridge.put
        for sensor in self.bridge.sensor.values():
            sensor[0x3E03] = 0x2B

    def set_level(self, level):
        return quiet(self.tool.set_level, None, level)[0]

    def limits(self):
        return [self.bridge.pair(reg) for reg in GAIN_MIN + GAIN]

    def assert_left_clean(self):
        self.assertEqual(self.bridge.bypass(), 0)
        self.assertEqual(self.bridge.page, KERNEL_PAGE)

    def test_a_coarse_step_gets_both_limits_of_both_channels(self):
        self.assertEqual(self.set_level("dusk"), "set")
        self.assertEqual(self.limits(), [(0x27, 0x3F)] * 4)
        self.assertEqual(self.set_level("night"), "set")
        self.assertEqual(self.limits(), [(0x2F, 0x3F)] * 4)
        self.assert_left_clean()

    def test_day_is_the_fine_range(self):
        self.set_level("night")
        self.assertEqual(self.set_level("day"), "set")
        self.assertEqual(self.limits(), [(0x00, 0x20)] * 2 + [(0x00, 0x7F)] * 2)
        self.assertEqual([self.bridge.pair(reg) for reg in EXPOSURE], [(0x53, 0x00)] * 2)

    def test_the_limits_change_with_the_control_off_and_one_take_over(self):
        self.set_level("dusk")
        writes = self.bridge.writes
        first_limit = next(i for i, (page, _, _) in enumerate(writes) if page == LIMITS_PAGE)
        self.assertIn((BYPASS_PAGE, 0x004D, 0x03), writes[:first_limit])
        self.assertEqual(self.bridge.limit_writes()[-2:], [(reg, 0x01) for reg in TAKE_OVER])
        self.assertEqual([w for w in writes if w[0] in ("sensor1", "sensor2")],
                         [("sensor1", 0x3E03, 0x2B), ("sensor2", 0x3E03, 0x2B)])
        self.assertEqual(writes[-1], (BYPASS_PAGE, 0x004D, 0x00))

    def test_a_level_that_is_there_writes_no_register(self):
        self.assertEqual(self.set_level("day"), "kept")
        self.assertEqual(self.bridge.writes, [])
        self.assert_left_clean()

    def test_the_bridge_table_takes_a_level(self):
        for reg in GAIN:
            self.bridge.set_pair(reg, (0x00, 0x40))
        self.assertEqual(self.set_level("dusk"), "set")
        self.assertEqual(self.limits(), [(0x27, 0x3F)] * 4)

    def test_limits_of_someone_else_stay(self):
        self.bridge.set_pair(GAIN[1], (0x00, 0xFF))
        self.assertEqual(self.set_level("dusk"), "foreign")
        self.assertEqual(self.bridge.writes, [])
        self.assert_left_clean()

    def test_limits_left_half_written_are_written_again(self):
        self.bridge.set_pair(GAIN_MIN[0], (0x27, 0x3F))
        self.bridge.set_pair(GAIN[1], (0x27, 0x7F))
        self.assertEqual(self.set_level("day"), "set")
        self.assertEqual(self.limits(), [(0x00, 0x20)] * 2 + [(0x00, 0x7F)] * 2)

    def test_a_value_that_does_not_read_back_leaves_the_control_on(self):
        self.bridge.stuck.add((LIMITS_PAGE, 0x0AB0))
        self.assertEqual(self.set_level("dusk"), "failed")
        self.assertFalse([reg for reg, _ in self.bridge.limit_writes() if reg in TAKE_OVER])
        self.assertEqual([w for w in self.bridge.writes if w[0] in ("sensor1", "sensor2")],
                         [("sensor1", 0x3E03, 0x2B), ("sensor2", 0x3E03, 0x2B)])
        self.assert_left_clean()

    def test_a_bus_error_counts_as_failed(self):
        self.bridge.read_error.add((LIMITS_PAGE, 0x00FA))
        self.assertEqual(self.set_level("dusk"), "failed")
        self.assertEqual(self.bridge.writes, [])
        self.assert_left_clean()

    def test_a_bypass_that_stays_is_unknown(self):
        self.bridge.regs[(BYPASS_PAGE, 0x004D)] = 0x02
        self.bridge.stuck.add((BYPASS_PAGE, 0x004D))
        self.assertEqual(self.set_level("dusk"), "unknown")


class Wish(unittest.TestCase):
    def setUp(self):
        self.tool = load()
        self.dir = tempfile.mkdtemp(prefix="stereo-ae-")
        self.addCleanup(shutil.rmtree, self.dir)
        self.path = os.path.join(self.dir, "wish")

    def wished(self, text, now=1000.0):
        if text is not None:
            with open(self.path, "w") as f:
                f.write(text)
        return self.tool.wished_level(now, self.path)

    def test_a_fresh_wish_counts(self):
        self.assertEqual(self.wished("night 990.5 1794800000.0\n"), "night")
        self.assertEqual(self.wished("dusk 940.0 1794800000.0\n"), "dusk")

    def test_anything_else_is_the_base_level(self):
        for text in (None, "", "night\n", "night 939.9 1.0\n", "night 1000.1 1.0\n", "night abc 1.0\n",
                     "night nan 1.0\n", "noon 990.0 1.0\n", "night 990.0 1.0 extra\n", "night  990.0 1.0\n",
                     "\xff\xfe 990.0 1.0\n", "night 990.0 " + "1" * 60 + "\n"):
            with self.subTest(text=text):
                self.assertEqual(self.wished(text), "day")

    def test_a_link_and_a_directory_are_not_read(self):
        target = os.path.join(self.dir, "target")
        with open(target, "w") as f:
            f.write("night 990.0 1.0\n")
        os.symlink(target, self.path)
        self.assertEqual(self.wished(None), "day")
        os.unlink(self.path)
        os.mkdir(self.path)
        self.assertEqual(self.wished(None), "day")


class Watching(unittest.TestCase):
    def setUp(self):
        self.tool = load()
        self.bridge = Bridge(exposure=(0x53, 0x00), gain=(0x00, 0x7F))
        self.tool.read, self.tool.put = self.bridge.read, self.bridge.put
        self.dir = tempfile.mkdtemp(prefix="stereo-ae-")
        self.addCleanup(shutil.rmtree, self.dir)
        self.paths = {name: os.path.join(self.dir, name) for name in ("wish", "level_set", "lock", "setup_result")}
        self.now = 1000.0
        self.put("setup_result", "configured\n")
        self.watch = self.tool.Watch(None, clock=lambda: self.now, sleep=lambda s: None, **self.paths)

    def put(self, name, text):
        with open(self.paths[name], "w") as f:
            f.write(text)

    def wish(self, level):
        self.put("wish", f"{level} {self.now} 1794800000.0\n")

    def step(self, after=0.0):
        self.now += after
        return quiet(self.watch.step)[0]

    def level_set(self):
        with open(self.paths["level_set"]) as f:
            return f.read().split(" ")[0]

    def gain_max(self):
        return [self.bridge.pair(reg) for reg in GAIN]

    def test_follows_the_wish_and_names_the_level(self):
        self.assertIsNone(self.step())
        self.assertEqual(self.level_set(), "day")
        self.assertEqual(self.bridge.writes, [])
        self.wish("dusk")
        self.assertIsNone(self.step(31.0))
        self.assertEqual(self.gain_max(), [(0x27, 0x3F)] * 2)
        self.assertEqual(self.level_set(), "dusk")

    def test_waits_between_two_writes(self):
        self.wish("dusk")
        self.step()
        self.wish("night")
        self.step(29.0)
        self.assertEqual(self.gain_max(), [(0x27, 0x3F)] * 2)
        self.wish("night")
        self.step(1.5)
        self.assertEqual(self.gain_max(), [(0x2F, 0x3F)] * 2)

    def test_a_wish_that_goes_stale_ends_at_day(self):
        self.wish("night")
        self.step()
        self.assertEqual(self.level_set(), "night")
        self.step(61.0)
        self.assertEqual(self.gain_max(), [(0x00, 0x7F)] * 2)
        self.assertEqual(self.level_set(), "day")

    def test_nothing_without_a_camera_that_is_set_up(self):
        self.put("setup_result", "failed exposure\n")
        self.wish("night")
        self.assertIsNone(self.step())
        self.assertEqual(self.bridge.writes, [])
        self.assertFalse(os.path.exists(self.paths["level_set"]))

    def test_a_held_lock_puts_the_write_off(self):
        self.wish("dusk")
        holder = os.open(self.paths["lock"], os.O_RDONLY | os.O_CREAT, 0o644)
        fcntl.flock(holder, fcntl.LOCK_EX)
        self.assertIsNone(self.step())
        self.assertEqual(self.bridge.writes, [])
        os.close(holder)
        self.assertIsNone(self.step(5.0))
        self.assertEqual(self.gain_max(), [(0x27, 0x3F)] * 2)

    def test_ends_at_limits_of_someone_else(self):
        self.bridge.set_pair(GAIN[0], (0x00, 0xFF))
        self.assertEqual(self.step(), 0)

    def test_ends_after_three_failed_writes(self):
        self.bridge.stuck.add((LIMITS_PAGE, 0x00FB))
        self.wish("dusk")
        self.assertIsNone(self.step())
        self.wish("dusk")
        self.assertIsNone(self.step(31.0))
        self.wish("dusk")
        self.assertEqual(self.step(31.0), 1)

    def test_an_unknown_bridge_ends_it_and_marks_the_setup(self):
        self.bridge.regs[(BYPASS_PAGE, 0x004D)] = 0x02
        self.bridge.stuck.add((BYPASS_PAGE, 0x004D))
        self.wish("dusk")
        self.assertEqual(self.step(), 2)
        with open(self.paths["setup_result"]) as f:
            self.assertEqual(f.read(), "failed exposure\n")


if __name__ == "__main__":
    unittest.main()
