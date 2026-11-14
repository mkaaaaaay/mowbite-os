"""Tests of fake_tron: kinematics, stop rules, NMEA and WIT output, and a run on real ptys.

Run: python3 -m unittest /ws/src/mowbite_sim/test/test_fake_tron.py -v   (no ROS needed)
"""
import datetime
import importlib.machinery
import importlib.util
import json
import math
import os
import random
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


class SlipAndNoiseTest(unittest.TestCase):
    def test_drift_turns_the_base_while_driving_straight(self):
        pose = ft.Pose()
        for _ in range(1000):
            pose.step(0.01, 0.2, 0.0, 0.05)
        # 2 m driven at 0.05 rad/m
        self.assertAlmostEqual(pose.yaw, 0.1, places=6)
        self.assertEqual(pose.angular, 0.0)

    def test_state_reports_the_turn_with_the_slip(self):
        pose = ft.Pose()
        pose.step(0.01, 0.2, 0.1, 0.05)
        self.assertAlmostEqual(json.loads(ft.state_text(0.0, pose))["angular"], 0.11)

    def test_no_drift_without_driving(self):
        pose = ft.Pose()
        pose.step(1.0, 0.0, 0.0, 0.05)
        self.assertEqual(pose.yaw, 0.0)

    def test_gyro_sees_the_slip(self):
        pose = ft.Pose()
        pose.step(0.01, 0.2, 0.0, 0.05)
        rate_z = parse_wit(ft.wit_frames(pose, (0.0, 0.0, 0.0)))[1][1][2]
        self.assertAlmostEqual(rate_z * 2000.0 / 32768.0, math.degrees(0.01), delta=0.07)

    def test_noise_is_seeded_and_scaled(self):
        a, b = ft.random.Random(1), ft.random.Random(1)
        self.assertEqual(ft.noisy(a, 0.02, 1.0, 2.0), ft.noisy(b, 0.02, 1.0, 2.0))
        self.assertEqual(ft.noisy(a, 0.0, 1.0, 2.0), (1.0, 2.0))
        rng = ft.random.Random(2)
        errors = [ft.noisy(rng, 0.02, 0.0, 0.0)[0] for _ in range(4000)]
        mean = sum(errors) / len(errors)
        sigma = math.sqrt(sum((e - mean) ** 2 for e in errors) / len(errors))
        self.assertAlmostEqual(sigma, 0.02, delta=0.002)
        self.assertAlmostEqual(mean, 0.0, delta=0.002)


class DriveTest(unittest.TestCase):
    def test_ideal_wheels_pass_the_setpoint(self):
        drive = ft.Drive(0.40, 0.40)
        self.assertEqual(tuple(round(v, 12) for v in drive.move(0.2, 0.3)), (0.2, 0.3))
        self.assertEqual(tuple(round(v, 12) for v in drive.measured()), (0.2, 0.3))
        self.assertAlmostEqual(drive.left, 0.14)
        self.assertAlmostEqual(drive.right, 0.26)

    def test_real_track_turns_the_base_faster(self):
        drive = ft.Drive(0.40, 0.395)
        linear, angular = drive.move(0.0, 0.3)
        self.assertEqual(linear, 0.0)
        self.assertAlmostEqual(angular, 0.3 * 0.40 / 0.395)
        # The MCU counts on its own track and reports what it commanded.
        self.assertAlmostEqual(drive.measured()[1], 0.3)

    def test_inner_wheel_below_the_deadband_stands(self):
        drive = ft.Drive(0.40, 0.40)
        # r 0.27 at 0.3 rad/s: the inner wheel would run 0.021 m/s, the base pivots on it.
        linear, angular = drive.move(0.081, 0.3, 0.06)
        self.assertEqual(drive.left, 0.0)
        self.assertAlmostEqual(drive.right, 0.141)
        self.assertAlmostEqual(linear / angular, 0.20)
        self.assertEqual(drive.measured(), (linear, angular))

    def test_slow_pivot_below_the_deadband_does_not_turn(self):
        drive = ft.Drive(0.40, 0.40)
        self.assertEqual(drive.move(0.0, 0.25, 0.06), (0.0, 0.0))
        self.assertEqual(drive.measured(), (0.0, 0.0))
        # 0.3 rad/s gives 0.06 m/s on each wheel: the edge, it turns.
        self.assertAlmostEqual(drive.move(0.0, 0.3, 0.06)[1], 0.3)

    def test_odometry_scale_changes_only_the_report(self):
        drive = ft.Drive(0.40, 0.40)
        self.assertAlmostEqual(drive.move(0.2, 0.1)[0], 0.2)
        linear, angular = drive.measured(0.94)
        self.assertAlmostEqual(linear, 0.188)
        self.assertAlmostEqual(angular, 0.094)


class StallTest(unittest.TestCase):
    def test_a_stalled_base_turns_for_no_sensor(self):
        pose, drive = ft.Pose(), ft.Drive(0.40, 0.40)
        pose.step(0.02, *drive.move(-0.06, 0.05), 0.01)
        ft.stall(pose, drive)
        rate_z = parse_wit(ft.wit_frames(pose, (0.0, 0.0, 0.0)))[1][1][2]
        self.assertEqual(rate_z, 0)
        self.assertEqual(pose.antenna_velocity(0.098, 0.0), (0.0, 0.0))
        self.assertEqual(drive.measured(), (0.0, 0.0))


class HistoryTest(unittest.TestCase):
    def test_late_values_and_the_limit(self):
        history = ft.History(limit=1.0)
        # Steps of 1/64 s are exact in binary.
        for i in range(97):
            history.push(i / 64.0, i)
        # now 1.5 s: 0.125 s late is the value of 1.375 s, everything older than 1 s is gone.
        self.assertEqual(history.get(1.5, 0.0), 96)
        self.assertEqual(history.get(1.5, 0.125), 88)
        self.assertEqual(history.get(1.5, 5.0), 32)
        self.assertEqual(history.get(1.5, -1.0), 96)
        self.assertEqual(len(history.items), 65)

    def test_too_young_history_gives_the_oldest(self):
        history = ft.History()
        history.push(0.0, "a")
        history.push(0.01, "b")
        self.assertEqual(history.get(0.01, 0.5), "a")


class HostLinkTest(unittest.TestCase):
    def test_no_traffic_no_motion(self):
        host = ft.HostLink(0.2)
        self.assertEqual(host.setpoint(0.0, False), (0.0, 0.0))

    def test_holds_for_the_timeout_after_the_last_speed(self):
        host = ft.HostLink(0.2)
        host.on_speed(0.3, 0.1, 0.0)
        for t in (0.1, 0.2):
            self.assertEqual(host.setpoint(t, False), (0.3, 0.1))
        self.assertEqual(host.setpoint(0.21, False), (0.0, 0.0))

    def test_heartbeats_do_not_hold_the_setpoint(self):
        host = ft.HostLink(0.2)
        host.on_frame(0.0)
        host.on_speed(0.3, 0.0, 0.0)
        for t in (0.1, 0.2, 0.3):
            host.on_frame(t)
        self.assertEqual(host.setpoint(0.3, False), (0.0, 0.0))
        self.assertFalse(host.silent(0.3))

    def test_repeated_speed_keeps_it(self):
        host = ft.HostLink(0.2)
        for t in (0.0, 0.05, 0.1, 0.15, 0.2, 0.25):
            host.on_speed(0.3, 0.0, t)
            self.assertEqual(host.setpoint(t + 0.05, False), (0.3, 0.0))

    def test_fast_speed_holds_the_setpoint_long(self):
        host = ft.HostLink(0.2, 3.6)
        for t in (0.0, 0.05, 0.1, 0.15):
            host.on_speed(0.3, 0.0, t)
        self.assertEqual(host.setpoint(3.7, False), (0.3, 0.0))
        self.assertEqual(host.setpoint(3.76, False), (0.0, 0.0))

    def test_a_single_short_gap_does_not_hold(self):
        host = ft.HostLink(0.2, 3.6)
        for t in (0.0, 0.1, 0.14, 0.24):
            host.on_speed(0.3, 0.0, t)
        self.assertEqual(host.setpoint(0.44, False), (0.3, 0.0))
        self.assertEqual(host.setpoint(0.45, False), (0.0, 0.0))

    def test_speed_every_100ms_drops_after_the_timeout(self):
        host = ft.HostLink(0.2, 3.6)
        for t in (0.0, 0.1, 0.2):
            host.on_speed(0.3, 0.0, t)
        self.assertEqual(host.setpoint(0.4, False), (0.3, 0.0))
        self.assertEqual(host.setpoint(0.41, False), (0.0, 0.0))

    def test_block_clears_until_next_speed(self):
        host = ft.HostLink(0.2)
        host.on_speed(0.3, 0.0, 0.0)
        self.assertEqual(host.setpoint(0.05, True), (0.0, 0.0))
        self.assertEqual(host.setpoint(0.1, False), (0.0, 0.0))
        host.on_speed(0.2, 0.0, 0.15)
        self.assertEqual(host.setpoint(0.2, False), (0.2, 0.0))


class DriveReportTest(unittest.TestCase):
    def test_rpm_by_side(self):
        drive = ft.Drive(0.40, 0.40)
        drive.move(0.0, 0.25)
        left, right = drive.rpm()
        self.assertLess(left, 0)
        self.assertGreater(right, 0)
        drive.move(0.05, 0.0)
        self.assertEqual(drive.rpm(), (5, 5))
        self.assertFalse(drive.standing())
        drive.move(0.0, 0.0)
        self.assertTrue(drive.standing())

    def test_standstill_noise_like_the_tron(self):
        rng = random.Random(3)
        samples = [ft.standstill_noise(rng) for _ in range(5000)]
        self.assertLessEqual(max(abs(v) for v, _ in samples), 0.0034)
        self.assertLessEqual(max(abs(w) for _, w in samples), 0.02)
        fastest = max(max(abs(v - w * 0.2), abs(v + w * 0.2)) for v, w in samples)
        self.assertLess(fastest, 0.011)
        self.assertGreater(fastest, 0.005)


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

    def test_gga_age_of_corrections(self):
        def fields(**kw):
            return self.sentences(**kw)[0].split("*")[0].split(",")
        self.assertEqual(fields()[13:15], ["1.0", "0000"])
        self.assertEqual(fields(age=45.0)[13:15], ["45.0", "0000"])
        self.assertEqual(fields(age=None)[13:15], ["", ""])
        self.assertTrue(all(nmea_ok(l) for l in self.sentences(age=None)))

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

    def test_tilted_base(self):
        # Upright, a roll of the base is the roll of the sensor, and gravity leans with it.
        frames = parse_wit(ft.wit_frames(ft.Pose(), (0.0, 0.0, 0.0), tilt_deg=(20.0, 0.0)))
        acc = [v / 32768.0 * 16.0 for v in frames[0][1][:3]]
        self.assertAlmostEqual(acc[1], math.sin(math.radians(20.0)), places=3)
        self.assertAlmostEqual(acc[2], math.cos(math.radians(20.0)), places=3)
        self.assertAlmostEqual(frames[2][1][0] / 32768.0 * 180.0, 20.0, delta=0.01)
        # Upside down, gravity is off the axis of the base by the whole tilt of roll and pitch.
        frames = parse_wit(ft.wit_frames(ft.Pose(yaw=1.0), (180.0, 0.0, 0.0), tilt_deg=(10.0, 10.0)))
        acc = [v / 32768.0 * 16.0 for v in frames[0][1][:3]]
        expected = math.degrees(math.acos(math.cos(math.radians(10.0)) ** 2))
        self.assertAlmostEqual(math.degrees(math.acos(-acc[2])), expected, delta=0.05)


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


def rtcm_frame(message, length=19):
    """A RTCM 3 frame of that message number with a valid CRC."""
    payload = bytes([message >> 4, (message & 0x0F) << 4]) + bytes(range(length - 2))
    frame = bytes([0xD3, length >> 8, length & 0xFF]) + payload
    return frame + ft.crc24q(frame).to_bytes(3, "big")


class BoardTest(unittest.TestCase):
    def test_crc24q_check_value(self):
        self.assertEqual(ft.crc24q(b"123456789"), 0xCDE703)

    def test_off_until_the_exact_command(self):
        board = ft.Board()
        self.assertFalse(board.on)
        self.assertEqual(board.report(), b"$GNRTK,OFF*27\r\n")
        board.feed(0.0, b"$GNRTK,ON*70\r\n$GNRTK,O")
        self.assertFalse(board.on)
        board.feed(0.1, b"N*69\r\n")
        self.assertTrue(board.on)
        self.assertEqual(board.report(), ft.BOARD_ON)

    def test_rtcm_only_while_on(self):
        board = ft.Board()
        entries = board.feed(0.0, rtcm_frame(1005))
        self.assertEqual(entries, [{"board": "rtcm_dropped"}])
        self.assertEqual(board.fix(0.0), (1, None))
        board.feed(0.5, ft.BOARD_ON)
        frame = rtcm_frame(1077, 120)
        self.assertEqual(board.feed(1.0, frame[:50]), [])
        self.assertEqual(board.feed(1.0, frame[50:] + b"\x00junk"), [{"rtcm": 1077}])
        self.assertEqual(board.fix(3.0), (4, 2.0))
        self.assertEqual(board.fix(121.5), (1, None))
        bad = bytearray(rtcm_frame(1005))
        bad[6] ^= 1
        self.assertEqual(board.feed(2.0, bytes(bad)), [{"board": "bad_rtcm"}])

    def test_falls_back_without_host_bytes(self):
        board = ft.Board()
        board.feed(0.0, ft.BOARD_ON)
        self.assertFalse(board.tick(5.9))
        # RTCM alone keeps it on, like the keep-alive.
        board.feed(5.9, rtcm_frame(1005))
        self.assertFalse(board.tick(11.8))
        self.assertTrue(board.tick(12.0))
        self.assertFalse(board.on)
        self.assertFalse(board.tick(13.0))

    def test_reset_drops_the_corrections(self):
        board = ft.Board()
        board.feed(0.0, ft.BOARD_ON + rtcm_frame(1005))
        self.assertEqual(board.fix(1.0)[0], 4)
        self.assertEqual(board.feed(2.0, ft.BOARD_RESET + ft.BOARD_ON), [{"board": "reset"}, {"board": "on_command"}])
        self.assertEqual(board.fix(2.0), (1, None))
        self.assertEqual(board.resets, 1)


class DockTest(unittest.TestCase):
    """The charging station: 1 m behind a mower at the origin facing east, it backs in going west."""

    def setUp(self):
        self.dock = ft.Dock(-1.0, 0.0, math.pi, -0.10)
        self.t = 0.0

    def run_for(self, pose, seconds, linear, angular=0.0, dt=0.01):
        """Steps like the loop of fake_tron; returns how many steps the station stopped."""
        stopped = 0
        for _ in range(int(round(seconds / dt))):
            self.t += dt
            before = ft.Pose(pose.x, pose.y, pose.yaw)
            pose.step(dt, linear, angular)
            pushing = self.dock.stop(before, pose)
            stopped += pushing
            self.dock.update(self.t, pose, pushing)
        return stopped

    def test_backing_in_stops_on_the_contacts(self):
        pose = ft.Pose()
        self.assertGreater(self.run_for(pose, 15.0, -0.1), 0)
        self.assertAlmostEqual(pose.x, -1.0, places=9)
        self.assertAlmostEqual(pose.y, 0.0, places=9)

    def test_contact_after_a_second_of_pushing(self):
        pose = ft.Pose()
        self.run_for(pose, 10.0, -0.1)
        self.assertFalse(self.dock.contact)
        self.run_for(pose, 0.9, -0.1)
        self.assertFalse(self.dock.contact)
        self.run_for(pose, 0.3, -0.1)
        self.assertTrue(self.dock.contact)

    def test_a_short_push_and_a_pause_do_not_add_up(self):
        pose = ft.Pose()
        self.run_for(pose, 10.6, -0.1)
        self.run_for(pose, 0.5, 0.0)
        self.run_for(pose, 0.6, -0.1)
        self.assertFalse(self.dock.contact)

    def test_contact_holds_standing_and_ends_driving_off(self):
        pose = ft.Pose()
        self.run_for(pose, 12.5, -0.1)
        self.assertTrue(self.dock.contact)
        self.run_for(pose, 5.0, 0.0)
        self.assertTrue(self.dock.contact)
        self.run_for(pose, 0.2, 0.1)
        self.assertFalse(self.dock.contact)

    def test_off_across_the_housing_stops_without_contact(self):
        pose = ft.Pose(0.0, 0.05, 0.0)
        self.assertGreater(self.run_for(pose, 15.0, -0.1), 0)
        self.assertAlmostEqual(pose.x, -1.0, places=9)
        self.assertFalse(self.dock.contact)

    def test_beside_the_housing_no_stop(self):
        pose = ft.Pose(0.0, 0.15, 0.0)
        self.assertEqual(self.run_for(pose, 15.0, -0.1), 0)
        self.assertAlmostEqual(pose.x, -1.5, places=6)

    def test_turned_on_the_station_contact_only_within_five_degrees(self):
        self.assertTrue(self.dock.settle(ft.Pose(-1.0, 0.0, math.radians(4.0))))
        self.assertFalse(self.dock.settle(ft.Pose(-1.0, 0.0, math.radians(6.0))))

    def test_backing_in_at_an_angle_misses_the_contacts(self):
        # 4 degrees off over 1 m: the contacts meet the plane about 0.08 m to the side, the housing still stops it.
        pose = ft.Pose(0.0, 0.0, math.radians(4.0))
        self.assertGreater(self.run_for(pose, 15.0, -0.1), 0)
        along, across, _ = self.dock.place(pose)
        self.assertAlmostEqual(along, 0.0, places=9)
        self.assertAlmostEqual(abs(across), 0.0769, delta=0.002)
        self.assertFalse(self.dock.contact)

    def test_placed_on_the_station(self):
        self.assertTrue(self.dock.settle(ft.Pose(-1.0, 0.0, 0.0)))
        self.assertFalse(ft.Dock(-1.0, 0.0, math.pi, -0.10).settle(ft.Pose(-0.9, 0.0, 0.0)))

    def test_moving_the_station_takes_a_docked_mower_along(self):
        pose = ft.Pose(-1.0, 0.0, 0.0)
        self.dock.settle(pose)
        self.dock.move((0.2, 0.1), pose)
        self.assertAlmostEqual(pose.x, -0.8, places=9)
        self.assertAlmostEqual(pose.y, 0.1, places=9)
        self.assertTrue(self.dock.update(1.0, pose, False))
        # Without contact the station moves alone.
        free = ft.Pose(0.0, 0.0, 0.0)
        other = ft.Dock(-1.0, 0.0, math.pi, -0.10)
        other.move((0.3, 0.0), free)
        self.assertEqual((free.x, free.y), (0.0, 0.0))
        self.run_for_dock(other, free)
        self.assertAlmostEqual(free.x, -0.7, places=9)

    def run_for_dock(self, dock, pose):
        self.dock = dock
        self.run_for(pose, 15.0, -0.1)

    def test_contacts_ahead_drive_in_forwards(self):
        self.dock = ft.Dock(2.0, 0.0, 0.0, 0.3)
        pose = ft.Pose()
        self.run_for(pose, 25.0, 0.1)
        self.assertAlmostEqual(pose.x, 2.0, places=9)
        self.assertTrue(self.dock.contact)

    def leave(self, pose, seconds, linear, dt=0.01):
        """Drives straight like the loop of fake_tron, both wheels at linear; returns the steps the hollows held."""
        held = 0
        for _ in range(int(round(seconds / dt))):
            self.t += dt
            before = ft.Pose(pose.x, pose.y, pose.yaw)
            pose.step(dt, linear, 0.0)
            held += self.dock.hold(before, pose, abs(linear))
            self.dock.update(self.t, pose, False)
        return held

    def test_the_hollows_hold_a_crawling_mower(self):
        pose = ft.Pose(-1.0, 0.0, 0.0)
        self.assertTrue(self.dock.settle(pose))
        self.assertEqual(self.leave(pose, 3.0, 0.06), 300)
        self.assertAlmostEqual(pose.x, -1.0, places=9)
        self.assertTrue(self.dock.contact)

    def test_out_of_the_hollows_fast_enough(self):
        pose = ft.Pose(-1.0, 0.0, 0.0)
        self.assertTrue(self.dock.settle(pose))
        self.assertEqual(self.leave(pose, 3.0, ft.DOCK_HOLLOW_SPEED + 0.05), 0)
        self.assertAlmostEqual(pose.x, -0.55, places=6)
        self.assertFalse(self.dock.contact)

    def test_into_the_hollows_freely_and_off_beyond_them(self):
        # Backing in drops into them.
        pose = ft.Pose(-0.98, 0.0, 0.0)
        self.assertEqual(self.leave(pose, 0.2, -0.05), 0)
        self.assertAlmostEqual(pose.x, -0.99, places=6)
        # 0.1 m out of the station the wheels are on the plate, a crawl drives on.
        pose = ft.Pose(-0.9, 0.0, 0.0)
        self.assertEqual(self.leave(pose, 1.0, 0.03), 0)
        self.assertAlmostEqual(pose.x, -0.87, places=6)
        # Beside the station there are no hollows.
        pose = ft.Pose(-1.0, 0.2, 0.0)
        self.assertEqual(self.leave(pose, 1.0, 0.03), 0)


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
        write_flags(self.paths["flags"], flags)

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

    def test_1b_stop_latches_until_a_clear_frame(self):
        def stop_bytes():
            read_available(self.fds["mcu"], 0.1)
            return {payload[3] for packets in ft.Decoder().feed(read_available(self.fds["mcu"], 0.3))
                    for t, m, payload in packets or [] if t == ft.FROM_MCU and m == 10}
        clear = ft.encode([(ft.TO_MCU, 10, bytes(8))])
        self.set_flags(stop=1, stop_latches=1)
        time.sleep(0.1)
        self.set_flags(stop_latches=1)
        self.assertEqual(stop_bytes(), {1})
        # Cleared while still pressed, it comes back at once.
        self.set_flags(stop=1, stop_latches=1)
        time.sleep(0.05)
        os.write(self.fds["mcu"], clear)
        time.sleep(0.05)
        self.set_flags(stop_latches=1)
        self.assertEqual(stop_bytes(), {1})
        os.write(self.fds["mcu"], clear)
        self.assertEqual(stop_bytes(), {0})
        # Without stop_latches stop follows the flag.
        self.set_flags(stop=1)
        time.sleep(0.1)
        self.set_flags()
        self.assertEqual(stop_bytes(), {0})

    def test_2_gnss_follows_pose(self):
        self.set_flags()
        read_available(self.fds["gnss"], 0.1)
        self.drive(0.0, 0.0, 0.3)
        before = self.state()
        lines = read_available(self.fds["gnss"], 0.6).decode().splitlines()
        self.assertTrue(lines and all(nmea_ok(l) for l in lines))
        # Besides the receiver's sentences the board's report.
        self.assertEqual({l[3:6] for l in lines} - {"RTK"}, {"GGA", "RMC", "VTG"})
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

    def test_2b_gnss_noise_flag(self):
        def latitudes():
            read_available(self.fds["gnss"], 0.1)
            lines = read_available(self.fds["gnss"], 1.2).decode().splitlines()
            return {l.split(",")[2] for l in lines if l[3:6] == "GGA"}
        self.set_flags()
        self.assertEqual(len(latitudes()), 1)
        self.set_flags(gnss_noise=0.5)
        self.assertGreater(len(latitudes()), 3)
        self.set_flags()

    def test_2c_wheel_odometry_and_delay_flags(self):
        self.set_flags(odom_scale=0.94)
        speeds = self.speed_frames(self.drive(0.2, 0.0, 0.5))
        self.assertAlmostEqual(speeds[-1][0], 0.188, places=5)
        # A slow pivot below the deadband: the wheels stand, the report says so.
        self.set_flags(wheel_deadband=0.06)
        start = self.state()
        speeds = self.speed_frames(self.drive(0.0, 0.25, 0.6))
        self.assertEqual(speeds[-1], (0.0, 0.0))
        self.assertAlmostEqual(self.state()["yaw"], start["yaw"], places=9)
        # The position is sent 0.5 s late with the time of its epoch.
        self.set_flags(gnss_delay=0.5)
        read_available(self.fds["gnss"], 0.3)
        lines = read_available(self.fds["gnss"], 0.5).decode().splitlines()
        now = datetime.datetime.now(datetime.timezone.utc)
        hhmmss = [l for l in lines if l[3:6] == "GGA"][-1].split(",")[1]
        sent = now.replace(hour=int(hhmmss[:2]), minute=int(hhmmss[2:4]), second=0, microsecond=0)
        late = (now - sent).total_seconds() - float(hhmmss[4:])
        self.assertAlmostEqual(late, 0.5, delta=0.3)
        self.set_flags()
        read_available(self.fds["mcu"], 0.9)

    def test_2d_board_and_rtcm_fix(self):
        self.set_flags(gnss_rtcm=1)
        read_available(self.fds["gnss"], 0.3)
        lines = read_available(self.fds["gnss"], 1.2).decode().splitlines()
        self.assertIn("$GNRTK,OFF*27", lines)
        self.assertEqual([l for l in lines if l[3:6] == "GGA"][-1].split(",")[6], "1")
        os.write(self.fds["gnss"], ft.BOARD_ON)
        time.sleep(0.2)
        os.write(self.fds["gnss"], rtcm_frame(1005))
        read_available(self.fds["gnss"], 0.3)
        # The report comes once a second.
        lines = read_available(self.fds["gnss"], 1.2).decode(errors="replace").splitlines()
        self.assertIn("$GNRTK,ON*69", lines)
        gga = [l for l in lines if l[3:6] == "GGA"][-1].split(",")
        self.assertEqual(gga[6], "4")
        # The last GGA of the window comes up to 1.5 s after the frame.
        self.assertLess(float(gga[13]), 2.0)
        self.set_flags()

    def test_3_imu_rate_and_content(self):
        read_available(self.fds["imu"], 0.2)
        data = read_available(self.fds["imu"], 1.0)
        frames = parse_wit(data[:len(data) // 11 * 11])
        types = [t for t, _ in frames]
        self.assertGreater(types.count(0x53), 35)
        self.assertLess(types.count(0x53), 65)
        # Upright like the sensor of the Tron: level, gravity along +z.
        angle = [v for t, v in frames if t == 0x53][-1]
        self.assertAlmostEqual(angle[0] / 32768.0 * 180.0, 0.0, delta=0.1)
        acc = [v for t, v in frames if t == 0x51][-1]
        self.assertAlmostEqual(acc[2] / 32768.0 * 16.0, 1.0, delta=0.02)

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

    def test_5_a_port_is_free_again_after_its_driver_died(self):
        # A driver killed with TIOCEXCL set would leave the pty busy for every later open.
        d = tempfile.mkdtemp(prefix="fake_tron_renew_")
        links = [os.path.join(d, n) for n in ("a", "b", "c")]
        log = os.path.join(d, "log")
        p = subprocess.Popen([sys.executable, "-I", SCRIPT, "--link", links[0], "--gnss-link", links[1],
                              "--imu-link", links[2], "--log", log], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
        try:
            for _ in range(100):
                if all(os.path.lexists(l) for l in links):
                    break
                time.sleep(0.05)
            driver = subprocess.Popen([sys.executable, "-I", "-c", HOLD_EXCLUSIVE, links[2]])
            time.sleep(0.5)
            driver.kill()
            driver.wait(timeout=5)
            time.sleep(0.5)
            fd = os.open(links[2], os.O_RDWR | os.O_NOCTTY)
            os.close(fd)
            with open(log) as f:
                self.assertIn("imu", [json.loads(line).get("renewed") for line in f])
        finally:
            p.terminate()
            p.wait(timeout=5)
            for name in os.listdir(d):
                os.remove(os.path.join(d, name))
            os.rmdir(d)


def write_flags(path, flags):
    """Replaces the flags file in one step and waits until fake_tron, which reads it every FILES_PERIOD s on a thread of
    its own, has taken it."""
    with open(path + ".tmp", "w") as f:
        json.dump(flags, f)
    os.replace(path + ".tmp", path)
    time.sleep(5 * ft.FILES_PERIOD)


class DockProcessTest(unittest.TestCase):
    """fake_tron started on its charging station: contact at once, a charge current only after Charge 1."""

    @classmethod
    def setUpClass(cls):
        cls.dir = tempfile.mkdtemp(prefix="fake_tron_dock_")
        cls.paths = {k: os.path.join(cls.dir, k) for k in ("mcu", "gnss", "imu", "flags", "state", "log")}
        cls.proc = subprocess.Popen(
            [sys.executable, "-I", SCRIPT, "--link", cls.paths["mcu"], "--gnss-link", cls.paths["gnss"],
             "--imu-link", cls.paths["imu"], "--flags", cls.paths["flags"], "--state", cls.paths["state"],
             "--log", cls.paths["log"], "--start", "-1.0", "0.0", "0.0", "--dock", "-1.0", "0.0", "180.0"],
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        for _ in range(100):
            if os.path.lexists(cls.paths["mcu"]):
                break
            time.sleep(0.05)
        cls.fd = os.open(cls.paths["mcu"], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        import tty
        tty.setraw(cls.fd)

    @classmethod
    def tearDownClass(cls):
        os.close(cls.fd)
        cls.proc.terminate()
        cls.proc.wait(timeout=5)
        cls.err = cls.proc.stderr.read()
        cls.proc.stderr.close()

    def batteries(self, seconds):
        """(current A, dock_ok) of the battery reports within seconds."""
        out = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            for packets in ft.Decoder().feed(read_available(self.fd, 0.1)):
                for t, m, payload in packets or []:
                    if t == ft.FROM_MCU and m == 5:
                        current, dock_ok = struct.unpack("<HhBbbI", payload)[1], payload[5]
                        out.append((current / 10.0, dock_ok))
        return out

    def reports(self, seconds):
        """(current A, percentage) of the battery reports within seconds."""
        out = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            for packets in ft.Decoder().feed(read_available(self.fd, 0.1)):
                for t, m, payload in packets or []:
                    if t == ft.FROM_MCU and m == 5:
                        _, current, percentage = struct.unpack("<HhBbbI", payload)[:3]
                        out.append((current / 10.0, percentage))
        return out

    def test_charge_current_and_percentage_from_the_flags(self):
        os.write(self.fd, ft.encode([(ft.TO_MCU, 3, bytes([1]))]))
        self.assertEqual(self.reports(2.2)[-1], (-3.0, 80))
        write_flags(self.paths["flags"], {"charge_current": 1.5, "battery_percentage": 60})
        self.assertEqual(self.reports(2.2)[-1], (-1.5, 60))
        # Contacts that do not charge, and a full battery: no charge current.
        write_flags(self.paths["flags"], {"charge_current": 0.0})
        self.assertEqual(self.reports(2.2)[-1], (1.2, 80))
        write_flags(self.paths["flags"], {"battery_percentage": 100})
        self.assertEqual(self.reports(2.2)[-1], (1.2, 100))
        write_flags(self.paths["flags"], {})
        os.write(self.fd, ft.encode([(ft.TO_MCU, 3, bytes([0]))]))
        self.assertGreater(self.reports(2.2)[-1][0], 0)

    def test_contact_at_once_and_current_only_after_charge_on(self):
        before = self.batteries(2.2)
        self.assertTrue(before and all(ok == 1 for _, ok in before), before)
        self.assertTrue(all(current > 0 for current, _ in before), before)
        os.write(self.fd, ft.encode([(ft.TO_MCU, 3, bytes([1]))]))
        after = self.batteries(2.2)
        self.assertTrue(after and after[-1] == (-3.0, 1), after)
        os.write(self.fd, ft.encode([(ft.TO_MCU, 3, bytes([0]))]))
        self.assertGreater(self.batteries(2.2)[-1][0], 0)
        with open(self.paths["state"]) as f:
            self.assertTrue(json.load(f)["dock_ok"])

    def drive(self, linear, seconds):
        """SpeedData like the driver every 100 ms, which also keeps the host link alive."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            os.write(self.fd, ft.encode([(ft.TO_MCU, 4, struct.pack("<ff", linear, 0.0))]))
            read_available(self.fd, 0.1)

    def state(self):
        time.sleep(0.3)
        with open(self.paths["state"]) as f:
            return json.load(f)

    def test_hollows_hold_a_crawl_and_let_the_undock_speed_out(self):
        self.drive(0.06, 1.0)
        held = self.state()
        self.assertAlmostEqual(held["x"], -1.0, places=3)
        self.assertTrue(held["dock_ok"])
        # Stuck for good with the flag.
        write_flags(self.paths["flags"], {"dock_hollow_speed": 9.0})
        self.drive(0.15, 1.0)
        self.assertAlmostEqual(self.state()["x"], -1.0, places=3)
        write_flags(self.paths["flags"], {})
        self.drive(0.15, 1.5)
        self.drive(0.0, 0.3)
        out = self.state()
        self.assertGreater(out["x"], -0.9)
        self.assertFalse(out["dock_ok"])

    def test_placed_back_on_the_station_by_hand(self):
        write_flags(self.paths["flags"], {"place": [-1.0, 0.0, 0.0]})
        back = self.state()
        self.assertAlmostEqual(back["x"], -1.0, places=6)
        self.assertAlmostEqual(back["y"], 0.0, places=6)
        self.assertTrue(back["dock_ok"])
        # Once per value: driving off again works with the flag still set.
        self.drive(0.15, 1.0)
        self.drive(0.0, 0.3)
        self.assertGreater(self.state()["x"], -0.95)


# Opens the port given like a driver does and holds it.
HOLD_EXCLUSIVE = ("import fcntl, os, sys, termios, time\n"
                  "fd = os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY)\n"
                  "fcntl.ioctl(fd, termios.TIOCEXCL)\n"
                  "time.sleep(30)\n")


if __name__ == "__main__":
    unittest.main()
