"""Tests of fake_tron: kinematics, stop rules, NMEA and WIT output, and a run on real ptys.

Run: python3 -m unittest /ws/src/mowbite_sim/test/test_fake_tron.py -v   (no ROS needed)
"""
import datetime
import importlib.machinery
import importlib.util
import json
import math
import os
import struct
import subprocess
import sys
import tempfile
import time
import unittest

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts", "fake_tron")
loader = importlib.machinery.SourceFileLoader("fake_tron", SCRIPT)
ft = importlib.util.module_from_spec(importlib.util.spec_from_loader("fake_tron", loader))
loader.exec_module(ft)

DATUM = (52.0, 9.0, 50.0)
UTC = datetime.datetime(2026, 10, 28, 12, 0, 1, 250000, tzinfo=datetime.timezone.utc)


def nmea_ok(line):
    body, _, checksum = line.strip().lstrip("$").partition("*")
    value = 0
    for c in body.encode():
        value ^= c
    return f"{value:02X}" == checksum


class PoseTest(unittest.TestCase):
    def test_straight_line(self):
        pose = ft.Pose(1.0, 2.0, math.pi / 2)
        for _ in range(100):
            pose.step(0.01, 0.3, 0.0)
        self.assertAlmostEqual(pose.x, 1.0, places=9)
        self.assertAlmostEqual(pose.y, 2.3, places=9)

    def test_circle_closes(self):
        pose = ft.Pose()
        radius = 1.0
        speed = 0.3
        steps = int(2 * math.pi * radius / speed / 0.001)
        for _ in range(steps):
            pose.step(0.001, speed, speed / radius)
        self.assertLess(math.hypot(pose.x, pose.y), 2e-3)
        self.assertAlmostEqual(pose.yaw, 0.0, places=2)

    def test_turn_in_place_keeps_position(self):
        pose = ft.Pose(3.0, 4.0, 0.0)
        for _ in range(200):
            pose.step(0.01, 0.0, 0.3)
        self.assertEqual((pose.x, pose.y), (3.0, 4.0))
        self.assertAlmostEqual(pose.yaw, 0.6, places=9)

    def test_yaw_stays_in_range(self):
        pose = ft.Pose()
        for _ in range(2000):
            pose.step(0.01, 0.0, 0.3)
        self.assertTrue(-math.pi < pose.yaw <= math.pi)

    def test_antenna_offset_follows_heading(self):
        pose = ft.Pose(0.0, 0.0, math.pi / 2)
        x, y = pose.antenna(0.098, 0.0)
        self.assertAlmostEqual(x, 0.0, places=9)
        self.assertAlmostEqual(y, 0.098, places=9)

    def test_antenna_velocity_matches_finite_difference(self):
        pose = ft.Pose(0.0, 0.0, 0.7)
        pose.step(0.0, 0.25, 0.3)
        vx, vy = pose.antenna_velocity(0.098, 0.02)
        x0, y0 = pose.antenna(0.098, 0.02)
        pose.step(1e-6, 0.25, 0.3)
        x1, y1 = pose.antenna(0.098, 0.02)
        self.assertAlmostEqual((x1 - x0) / 1e-6, vx, places=4)
        self.assertAlmostEqual((y1 - y0) / 1e-6, vy, places=4)


class HostLinkTest(unittest.TestCase):
    def test_no_traffic_no_motion(self):
        host = ft.HostLink(2.0)
        host.on_speed(0.3, 0.0)
        self.assertEqual(host.setpoint(0.0, False), (0.0, 0.0))

    def test_holds_while_host_talks(self):
        host = ft.HostLink(2.0)
        host.on_frame(0.0)
        host.on_speed(0.3, 0.1)
        for t in (0.5, 1.5, 2.0):
            host.on_frame(t)
            self.assertEqual(host.setpoint(t, False), (0.3, 0.1))

    def test_stops_after_two_seconds_of_silence(self):
        host = ft.HostLink(2.0)
        host.on_frame(0.0)
        host.on_speed(0.3, 0.0)
        self.assertEqual(host.setpoint(2.0, False), (0.3, 0.0))
        self.assertEqual(host.setpoint(2.01, False), (0.0, 0.0))

    def test_new_traffic_alone_does_not_restart(self):
        host = ft.HostLink(2.0)
        host.on_frame(0.0)
        host.on_speed(0.3, 0.0)
        host.setpoint(2.5, False)
        host.on_frame(3.0)
        self.assertEqual(host.setpoint(3.0, False), (0.0, 0.0))
        host.on_speed(0.2, 0.0)
        self.assertEqual(host.setpoint(3.1, False), (0.2, 0.0))

    def test_block_clears_until_next_speed(self):
        host = ft.HostLink(2.0)
        host.on_frame(0.0)
        host.on_speed(0.3, 0.0)
        self.assertEqual(host.setpoint(0.1, True), (0.0, 0.0))
        host.on_frame(0.2)
        self.assertEqual(host.setpoint(0.2, False), (0.0, 0.0))


class GeodesyTest(unittest.TestCase):
    def test_origin(self):
        lat, lon, h = ft.enu_to_geodetic(DATUM, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(lat, 52.0, places=10)
        self.assertAlmostEqual(lon, 9.0, places=10)
        self.assertAlmostEqual(h, 50.0, places=6)

    def test_north_is_one_degree_per_111km(self):
        lat, lon, _ = ft.enu_to_geodetic(DATUM, 0.0, 111.0, 0.0)
        self.assertAlmostEqual(lat - 52.0, 111.0 / 111250.0, delta=2e-6)
        self.assertAlmostEqual(lon, 9.0, delta=1e-7)

    def test_east_shrinks_with_latitude(self):
        _, lon, _ = ft.enu_to_geodetic(DATUM, 100.0, 0.0, 0.0)
        meters_per_degree = 100.0 / (lon - 9.0)
        self.assertAlmostEqual(meters_per_degree, 111320.0 * math.cos(math.radians(52.0)), delta=150.0)


class NmeaTest(unittest.TestCase):
    def sentences(self, **kw):
        args = dict(utc=UTC, lat=52.0003, lon=9.0004, height=50.2, quality=4, speed=0.3, course=90.0)
        args.update(kw)
        return ft.gnss_sentences(**args).decode().splitlines()

    def test_minutes_never_round_to_sixty(self):
        # Just south of a whole degree: 59.99999999 minutes are the next degree, not "60.0000000".
        self.assertEqual(ft.nmea_coordinate(52.0 - 1e-12, "N", "S", 2), ("5200.0000000", "N"))
        self.assertEqual(ft.nmea_coordinate(9.0 - 1e-12, "E", "W", 3), ("00900.0000000", "E"))
        self.assertEqual(ft.nmea_coordinate(52.0 - 1e-6, "N", "S", 2), ("5159.9999400", "N"))

    def test_checksums_and_kinds(self):
        lines = self.sentences()
        self.assertEqual([l[3:6] for l in lines], ["GGA", "RMC", "VTG"])
        self.assertTrue(all(nmea_ok(l) for l in lines))

    def test_gga_fields(self):
        f = self.sentences()[0].split(",")
        self.assertEqual(f[1], "120001.25")
        self.assertEqual((f[3], f[5]), ("N", "E"))
        self.assertAlmostEqual(int(f[2][:2]) + float(f[2][2:]) / 60.0, 52.0003, places=9)
        self.assertAlmostEqual(int(f[4][:3]) + float(f[4][3:]) / 60.0, 9.0004, places=9)
        self.assertEqual(f[6], "4")

    def test_southwest_hemispheres(self):
        f = self.sentences(lat=-33.5, lon=-151.25)[0].split(",")
        self.assertEqual((f[3], f[5]), ("S", "W"))
        self.assertEqual(f[2][:2], "33")
        self.assertEqual(f[4][:3], "151")

    def test_rmc_vtg_motion(self):
        rmc = self.sentences(speed=0.3, course=123.4)[1].split(",")
        vtg = self.sentences(speed=0.3, course=123.4)[2].split(",")
        self.assertEqual(rmc[2], "A")
        self.assertAlmostEqual(float(rmc[7]) * ft.KNOTS, 0.3, places=3)
        self.assertEqual(rmc[8], "123.40")
        self.assertEqual(vtg[1], "123.40")
        self.assertAlmostEqual(float(vtg[7]) / 3.6, 0.3, places=3)

    def test_no_fix_has_no_position(self):
        lines = self.sentences(quality=0)
        self.assertTrue(all(nmea_ok(l) for l in lines))
        self.assertEqual(lines[0].split(",")[6], "0")
        self.assertEqual(lines[0].split(",")[2], "")
        self.assertEqual(lines[1].split(",")[2], "V")

    def test_unknown_course_is_empty(self):
        self.assertEqual(self.sentences(course=None)[1].split(",")[8], "")


def parse_wit(data):
    frames = []
    assert len(data) % 11 == 0
    for i in range(0, len(data), 11):
        f = data[i:i + 11]
        assert f[0] == 0x55 and sum(f[:10]) % 256 == f[10]
        frames.append((f[1], struct.unpack("<hhhh", f[2:10])))
    return frames


class WitTest(unittest.TestCase):
    def test_upside_down_at_rest(self):
        frames = parse_wit(ft.wit_frames(ft.Pose(), (180.0, 0.0, 0.0)))
        self.assertEqual([t for t, _ in frames], [0x51, 0x52, 0x53])
        acc = [v / 32768.0 * 16.0 for v in frames[0][1][:3]]
        self.assertAlmostEqual(acc[0], 0.0, places=3)
        self.assertAlmostEqual(acc[1], 0.0, places=3)
        self.assertAlmostEqual(acc[2], -1.0, places=3)
        self.assertEqual(frames[1][1][:3], (0, 0, 0))
        roll = frames[2][1][0] / 32768.0 * 180.0
        self.assertAlmostEqual(abs(roll), 180.0, delta=0.01)

    def test_yaw_rate_flips_sign_when_upside_down(self):
        pose = ft.Pose()
        pose.step(0.0, 0.0, 0.3)
        frames = parse_wit(ft.wit_frames(pose, (180.0, 0.0, 0.0)))
        self.assertAlmostEqual(frames[1][1][2] / 32768.0 * 2000.0, -math.degrees(0.3), delta=0.1)
        upright = parse_wit(ft.wit_frames(pose, (0.0, 0.0, 0.0)))
        self.assertAlmostEqual(upright[1][1][2] / 32768.0 * 2000.0, math.degrees(0.3), delta=0.1)
        self.assertAlmostEqual(upright[0][1][2] / 32768.0 * 16.0, 1.0, places=3)

    def test_gyro_bias_in_sensor_frame(self):
        frames = parse_wit(ft.wit_frames(ft.Pose(), (180.0, 0.0, 0.0), gyro_bias_dps=0.5))
        self.assertAlmostEqual(frames[1][1][2] / 32768.0 * 2000.0, 0.5, delta=0.1)

    def test_angle_follows_yaw(self):
        # Upright, the yaw angle of the sensor is the base yaw.
        pose = ft.Pose(yaw=math.radians(40.0))
        frames = parse_wit(ft.wit_frames(pose, (0.0, 0.0, 0.0)))
        self.assertAlmostEqual(frames[2][1][2] / 32768.0 * 180.0, 40.0, delta=0.01)

    def test_full_scale_does_not_wrap(self):
        self.assertEqual(ft.wit_scaled(180.0, 180.0), 32767)
        self.assertEqual(ft.wit_scaled(-180.0, 180.0), -32768)


def read_available(fd, seconds):
    end = time.monotonic() + seconds
    data = b""
    while time.monotonic() < end:
        try:
            chunk = os.read(fd, 65536)
            if chunk:
                data += chunk
                continue
        except (BlockingIOError, OSError):
            pass
        time.sleep(0.01)
    return data


class ProcessTest(unittest.TestCase):
    """fake_tron as a process on ptys, with a host that writes frames like the driver."""

    @classmethod
    def setUpClass(cls):
        cls.dir = tempfile.mkdtemp(prefix="fake_tron_")
        d = cls.dir
        cls.paths = {k: os.path.join(d, k) for k in ("mcu", "gnss", "imu", "flags", "state", "log")}
        cls.proc = subprocess.Popen(
            [sys.executable, "-I", SCRIPT, "--link", cls.paths["mcu"], "--gnss-link", cls.paths["gnss"],
             "--imu-link", cls.paths["imu"], "--flags", cls.paths["flags"], "--state", cls.paths["state"],
             "--log", cls.paths["log"], "--host-timeout", "0.6", "--datum", *map(str, DATUM)],
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        for _ in range(100):
            if all(os.path.lexists(cls.paths[k]) for k in ("mcu", "gnss", "imu")):
                break
            time.sleep(0.05)
        cls.fds = {k: os.open(cls.paths[k], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK) for k in ("mcu", "gnss", "imu")}
        import tty
        for fd in cls.fds.values():
            tty.setraw(fd)

    @classmethod
    def tearDownClass(cls):
        for fd in cls.fds.values():
            os.close(fd)
        cls.proc.terminate()
        cls.proc.wait(timeout=5)
        cls.err = cls.proc.stderr.read()
        cls.proc.stderr.close()

    def set_flags(self, **flags):
        with open(self.paths["flags"], "w") as f:
            json.dump(flags, f)

    def speed_frames(self, data):
        """Speed values (mod 4) of the MCU frames in data."""
        out = []
        for packets in ft.Decoder().feed(data):
            for t, m, payload in packets or []:
                if t == ft.FROM_MCU and m == 4:
                    out.append(struct.unpack("<ff", payload))
        return out

    def drive(self, linear, angular, seconds):
        """Host side: a speed command and a heartbeat every 100 ms; returns the data read."""
        data = b""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            os.write(self.fds["mcu"], ft.encode([(ft.TO_MCU, 4, struct.pack("<ff", linear, angular))]))
            data += read_available(self.fds["mcu"], 0.1)
        return data

    def state(self):
        with open(self.paths["state"]) as f:
            return json.load(f)

    def test_1_drive_stop_and_rules(self):
        self.set_flags()
        read_available(self.fds["mcu"], 0.3)
        start = self.state()
        data = self.drive(0.2, 0.0, 1.0)
        speeds = self.speed_frames(data)
        self.assertAlmostEqual(speeds[-1][0], 0.2, places=5)
        moved = self.state()
        self.assertGreater(math.hypot(moved["x"] - start["x"], moved["y"] - start["y"]), 0.12)

        # No host traffic: the speed reported goes to 0 within the timeout (0.6 s here, 2 s default).
        data = read_available(self.fds["mcu"], 1.2)
        self.assertEqual(self.speed_frames(data)[-1], (0.0, 0.0))
        halted = self.state()
        time.sleep(0.5)
        later = self.state()
        self.assertEqual((halted["x"], halted["y"]), (later["x"], later["y"]))

        # Lift: SpeedData has no effect and the speed stays 0 until the lift is released and SpeedData comes again.
        self.set_flags(lift=1)
        time.sleep(0.1)
        data = self.drive(0.2, 0.0, 0.5)
        self.assertTrue(all(s == (0.0, 0.0) for s in self.speed_frames(data)))
        self.set_flags()
        time.sleep(0.1)
        data = read_available(self.fds["mcu"], 0.3)
        self.assertEqual(self.speed_frames(data)[-1], (0.0, 0.0))
        data = self.drive(0.2, 0.0, 0.4)
        self.assertAlmostEqual(self.speed_frames(data)[-1][0], 0.2, places=5)

        # Stop flag while driving.
        self.set_flags(stop=1)
        data = self.drive(0.2, 0.0, 0.4)
        self.assertEqual(self.speed_frames(data)[-1], (0.0, 0.0))
        self.set_flags()
        read_available(self.fds["mcu"], 0.9)

    def test_2_gnss_follows_pose(self):
        self.set_flags()
        read_available(self.fds["gnss"], 0.1)
        self.drive(0.0, 0.0, 0.3)
        before = self.state()
        lines = read_available(self.fds["gnss"], 0.6).decode().splitlines()
        self.assertTrue(lines and all(nmea_ok(l) for l in lines))
        self.assertEqual({l[3:6] for l in lines}, {"GGA", "RMC", "VTG"})
        gga = [l for l in lines if l[3:6] == "GGA"][-1].split(",")
        lat = int(gga[2][:2]) + float(gga[2][2:]) / 60.0
        lon = int(gga[4][:3]) + float(gga[4][3:]) / 60.0
        # The antenna sits 0.098 m ahead of the base, the base has not moved: it is at (0.098, 0) east/north of start.
        e, n = (lon - DATUM[1]) * 111320.0 * math.cos(math.radians(52.0)), (lat - DATUM[0]) * 111250.0
        self.assertAlmostEqual(e, before["x"] + 0.098, delta=0.05)
        self.assertAlmostEqual(n, before["y"], delta=0.05)
        # A bad fix quality is passed on.
        self.set_flags(gnss_quality=5)
        time.sleep(0.1)
        read_available(self.fds["gnss"], 0.1)
        lines = read_available(self.fds["gnss"], 0.5).decode().splitlines()
        self.assertEqual([l for l in lines if l[3:6] == "GGA"][-1].split(",")[6], "5")
        self.set_flags()

    def test_3_imu_rate_and_content(self):
        read_available(self.fds["imu"], 0.2)
        data = read_available(self.fds["imu"], 1.0)
        frames = parse_wit(data[:len(data) // 11 * 11])
        types = [t for t, _ in frames]
        self.assertGreater(types.count(0x53), 35)
        self.assertLess(types.count(0x53), 65)
        angle = [v for t, v in frames if t == 0x53][-1]
        self.assertAlmostEqual(abs(angle[0] / 32768.0 * 180.0), 180.0, delta=0.1)

    def test_4_symlinks_removed_at_exit(self):
        # Checked in a separate process so the class fixtures stay up.
        d = tempfile.mkdtemp(prefix="fake_tron_exit_")
        links = [os.path.join(d, n) for n in ("a", "b", "c")]
        p = subprocess.Popen([sys.executable, "-I", SCRIPT, "--link", links[0], "--gnss-link", links[1],
                              "--imu-link", links[2]], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(100):
            if all(os.path.lexists(l) for l in links):
                break
            time.sleep(0.05)
        self.assertTrue(all(os.path.lexists(l) for l in links))
        p.terminate()
        p.wait(timeout=5)
        self.assertFalse(any(os.path.lexists(l) for l in links))


if __name__ == "__main__":
    unittest.main()
