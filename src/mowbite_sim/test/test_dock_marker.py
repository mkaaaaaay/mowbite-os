"""The pose at the docking station from the dock marker (localization/dock_pose) against the true pose of fake_tron.

The chain is fake_tron with the station of sim.yaml, tron_mcu, tron_imu, fake_dock_marker and the localization with the
values of the Tron; the test plays the logic: the geometry on bridge/params, the requests on hw/dock_marker_request,
the heading hint and the drive on hw/cmd_vel, steered by the true pose. The mower stands at the approach point, where
the marker has no direction yet, starts from the hint, backs onto the contacts and leaves again. It checks: no start
without a heading, the estimate against the truth, single outliers without a jump, outliers in a row drop it, the
final stretch without the marker stays good for its time, a hidden marker ends good within 0.05 m, the faults of
fake_dock_marker, and that the heading keeps the spread of one image of the marker, whose direction errs by degrees
that stay at one place (M3b), and stays within three times its reported accuracy, also with 6 degrees at one place.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_dock_marker.py
"""
import json
import math
import os
import shutil
import sys
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from launch.actions import TimerAction
from launch_ros.actions import Node
from mowbite_msgs.msg import Camera, DockMarker, RobotPose
from std_msgs.msg import Empty, Float64, String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (LATCHED, SimProbe, TestShutdown, estimate_at, fake_tron, layers, pose_stamp,  # noqa: E402, F401
                        sim_files, sim_links, wrap)

ANTENNA_X = 0.098
CONTACT_X = -0.10
WORK = tempfile.mkdtemp(prefix="mowbite_dock_marker_")
LINKS = sim_links(WORK)
STATE, FLAGS, _, _ = sim_files(WORK)
TRON = layers.files("tron")
SIM = os.path.join(get_package_share_directory("mowbite_sim"), "config", "sim.yaml")
LOCAL = os.path.join(WORK, "local.yaml")
# The station of sim.yaml: the docked mower at (-1, 0), the way in going west; the Tron backs in and faces east there.
STATION = yaml.safe_load(open(SIM))["fake_tron"]["ros__parameters"]["dock"]
STATION_POSE = (STATION[0], STATION[1], math.radians(STATION[2]))
# The approach point 0.8 m before the docked pose, a little off the line.
START = (STATION[0] + 0.8, 0.02, math.radians(2.0))
MOUNT = layers.parameters(TRON, "tron_rear")["mount"]
HINT_ERROR = math.radians(2.0)
NO_ACCURACY = 999.0


def generate_test_description():
    with open(LOCAL, "w") as f:
        f.write(f"tron_mcu:\n  ros__parameters:\n    port: {LINKS['mcu']}\n    safety_measurement_date: simulation\n"
                f"tron_imu:\n  ros__parameters:\n    port: {LINKS['imu']}\n")
    nodes = [
        Node(package="tron_mcu", executable="tron_mcu_node", name="tron_mcu", output="screen",
             parameters=[*TRON, LOCAL]),
        Node(package="tron_imu", executable="tron_imu_node", name="tron_imu", output="screen",
             parameters=[*TRON, LOCAL]),
        Node(package="mowbite_localization", executable="mowbite_localization_node", name="mowbite_localization",
             output="screen", parameters=[*TRON, LOCAL]),
        Node(package="mowbite_sim", executable="fake_dock_marker", name="fake_dock_marker", output="screen",
             parameters=[*TRON, {"state": STATE, "flags": FLAGS, "dock": [float(v) for v in STATION],
                                "dock_contact_x": CONTACT_X, "mount": [float(v) for v in MOUNT]}]),
    ]
    return launch.LaunchDescription([
        fake_tron(LINKS, "--flags", FLAGS, "--state", STATE, "--antenna-offset", str(ANTENNA_X), "0.0",
                  "--dock", *(str(float(v)) for v in STATION), "--dock-contact-x", str(CONTACT_X),
                  "--start", str(START[0]), str(START[1]), str(math.degrees(START[2]))),
        TimerAction(period=1.5, actions=nodes + [launch_testing.actions.ReadyToTest()])]), {}


def in_station(truth):
    """The true pose (x, y, yaw) in the station frame."""
    c, s = math.cos(STATION_POSE[2]), math.sin(STATION_POSE[2])
    dx, dy = truth["x"] - STATION_POSE[0], truth["y"] - STATION_POSE[1]
    return c * dx + s * dy, -s * dx + c * dy, wrap(truth["yaw"] - STATION_POSE[2])


def good(msg):
    return msg.position_accuracy < NO_ACCURACY


class Probe(SimProbe):
    """Drives on hw/cmd_vel, asks for the marker, gives the hint and pairs true poses with dock poses."""
    spin_timeout = 0.02

    def __init__(self):
        super().__init__("dock_marker_probe", state=STATE, flags=FLAGS)
        # (stamp s, x, y, heading) of the good dock poses, and every dock pose with its arrival time
        self.estimates = []
        self.poses = []
        self.markers = []
        self.map_pose = None
        self.setpoint = (0.0, 0.0)
        self.asking = False
        self.hint = False
        # (wall time, truth in the station frame, estimate or None, heading accuracy of the newest dock pose)
        self.samples = []
        self.node.create_subscription(RobotPose, "localization/dock_pose", self._dock, 50)
        self.node.create_subscription(RobotPose, "localization/pose", self._set("map_pose"), 10)
        self.node.create_subscription(DockMarker, "hw/dock_marker", self._marker, 50)
        self.cmd_pub = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        self.request_pub = self.node.create_publisher(Empty, "hw/dock_marker_request", 10)
        self.hint_pub = self.node.create_publisher(Float64, "logic/dock_heading_hint", 10)
        # The logic is not part of this launch: the probe sends the geometry of the mower.
        self.geometry = self.node.create_publisher(String, "bridge/params", LATCHED)
        self.geometry.publish(String(data=json.dumps(
            {"node": "/mowbite_geometry",
             "params": {"antenna_x": ANTENNA_X, "antenna_y": 0.0, "dock_contact_x": CONTACT_X, "version": 0}})))
        self.start()
        for target in (self._record, self._drive, self._ask):
            self.spawn(target)

    def _dock(self, msg):
        with self.lock:
            self.poses.append((time.time(), msg))
            if good(msg):
                stamp = pose_stamp(msg)
                if not self.estimates or stamp > self.estimates[-1][0]:
                    self.estimates.append((stamp, msg.x, msg.y, msg.heading))

    def _marker(self, msg):
        with self.lock:
            self.markers.append((time.time(), msg))

    def _record(self):
        last = None
        while self.running:
            try:
                written = os.stat(STATE).st_mtime
                truth = self.truth()
            except OSError:
                truth = None
            if truth and truth["t"] != last:
                last = truth["t"]
                time.sleep(0.06)
                with self.lock:
                    accuracy = self.poses[-1][1].heading_accuracy if self.poses else None
                    self.samples.append((written, in_station(truth), estimate_at(self.estimates, written), accuracy))
            time.sleep(0.02)

    def _drive(self):
        while self.running:
            msg = Twist()
            msg.linear.x, msg.angular.z = self.setpoint
            self.cmd_pub.publish(msg)
            time.sleep(0.1)

    def _ask(self):
        """A request, and with hint the true heading in the station frame HINT_ERROR off, every second."""
        while self.running:
            if self.asking:
                self.request_pub.publish(Empty())
            truth = self.truth() if self.hint else None
            if truth:
                self.hint_pub.publish(Float64(data=wrap(in_station(truth)[2] + HINT_ERROR)))
            time.sleep(DockMarker.REQUEST_PERIOD_S)

    def station(self):
        truth = self.truth()
        return in_station(truth) if truth else None

    def poses_since(self, since):
        with self.lock:
            return [msg for t, msg in self.poses if t >= since]

    def markers_since(self, since):
        with self.lock:
            return [msg for t, msg in self.markers if t >= since]

    def samples_since(self, since):
        with self.lock:
            return [s for s in self.samples if s[0] >= since and s[2] is not None]

    def back_in(self, speed, until, timeout):
        """Backs along the line of the station at speed (m/s, positive), steered by the truth, until until(station
        pose) or timeout s; True if until came."""
        end = time.time() + timeout
        while time.time() < end:
            x, y, yaw = self.station()
            if until((x, y, yaw)):
                return True
            # Facing out of the station (yaw pi in its frame) and backing: a heading that leans to the line.
            target = math.pi - math.atan(2.0 * y)
            self.setpoint = (-speed, max(-0.3, min(0.3, 1.5 * wrap(target - yaw))))
            time.sleep(0.05)
        return False

    def stop_driving(self, settle=1.0):
        self.setpoint = (0.0, 0.0)
        time.sleep(settle)

    def latest(self):
        with self.lock:
            return self.poses[-1][1] if self.poses else None


def error(sample):
    """(position error m, heading error rad) of one sample."""
    _, (tx, ty, tyaw), (x, y, yaw), _ = sample
    return math.hypot(x - tx, y - ty), wrap(yaw - tyaw)


def heading_ratio(samples):
    """The largest heading error of the samples in units of their reported heading accuracy."""
    return max(abs(error(s)[1]) / s[3] for s in samples)


def yaw_spread(msg):
    """The spread of the direction an image of the marker reports, rad."""
    return math.sqrt(msg.covariance[8])


class TestDockMarker(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        cls.probe.flags(dock_hollow_speed=0.0)

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def test_01_off_until_asked(self):
        p = self.probe
        # The localization publishes once the gyro offset is known.
        self.assertTrue(p.wait(lambda: p.map_pose is not None, 40), "no localization/pose")
        since = time.time()
        time.sleep(2.5)
        states = {m.state for m in p.markers_since(since)}
        self.assertEqual(states, {DockMarker.STATE_OFF})
        self.assertEqual(p.poses_since(0.0), [])

    def test_02_no_start_without_a_heading(self):
        p = self.probe
        since = time.time()
        p.asking = True
        self.assertTrue(p.wait(lambda: any(m.valid for m in p.markers_since(since)), 6), "no measurement")
        markers = p.markers_since(since)
        first = next(i for i, m in enumerate(markers) if m.valid)
        self.assertIn(DockMarker.STATE_STARTING, [m.state for m in markers[:first]])
        # 0.87 m from the camera: the marker has no direction yet.
        self.assertFalse(any(m.yaw_valid for m in markers[first:]))
        time.sleep(3.0)
        self.assertGreater(len([m for m in p.markers_since(since) if m.valid]), 20)
        self.assertEqual(p.poses_since(0.0), [], "a start without a heading")

    def test_03_starts_from_the_hint(self):
        p = self.probe
        p.hint = True
        self.assertTrue(p.wait(lambda: p.latest() is not None and good(p.latest()), 6), "no dock pose")
        time.sleep(1.0)
        msg = p.latest()
        x, y, yaw = p.station()
        self.assertEqual(msg.header.frame_id, "dock")
        self.assertTrue(msg.heading_valid)
        miss = math.hypot(msg.x - x, msg.y - y)
        print(f"\ndock: start miss {miss:.3f} m, accuracy {msg.position_accuracy:.3f} m, heading "
              f"{math.degrees(wrap(msg.heading - yaw)):.1f} deg", flush=True)
        self.assertLess(miss, 0.06)
        self.assertLess(miss, 3.0 * msg.position_accuracy)
        self.assertLess(abs(wrap(msg.heading - yaw)), math.radians(4.0))

    def test_04_single_outliers_do_not_move_it(self):
        p = self.probe
        before = p.latest()
        since = time.time()
        p.flags(dock_hollow_speed=0.0, marker_outlier=[0.3, 0.0, 3])
        time.sleep(3.0)
        poses = p.poses_since(since)
        self.assertTrue(poses and all(good(m) for m in poses), [m.position_accuracy for m in poses][:10])
        steps = [math.hypot(a.x - b.x, a.y - b.y) for a, b in zip([before] + poses, poses)]
        self.assertLess(max(steps), 0.005)
        # Outliers in a row drop the estimate, which starts again from the images after them.
        since = time.time()
        p.flags(dock_hollow_speed=0.0, marker_outlier=[0.3, 0.0, 1])
        self.assertTrue(p.wait(lambda: any(not m.heading_valid for m in p.poses_since(since)), 3), "not dropped")
        dropped = next(m for m in p.poses_since(since) if not m.heading_valid)
        self.assertEqual(dropped.position_accuracy, NO_ACCURACY)
        p.flags(dock_hollow_speed=0.0)
        since = time.time()
        self.assertTrue(p.wait(lambda: any(good(m) for m in p.poses_since(since + 0.5)), 6), "no new start")
        x, y, _ = p.station()
        self.assertLess(math.hypot(p.latest().x - x, p.latest().y - y), 0.06)

    def test_05_backs_onto_the_station(self):
        p = self.probe
        since = time.time()
        self.assertTrue(p.back_in(0.1, lambda s: s[0] >= -0.3, 15), p.station())
        at_gate = time.time()
        self.assertTrue(p.back_in(0.05, lambda s: s[0] >= -0.07, 10), p.station())
        blind = time.time()
        p.back_in(0.05, lambda s: False, 3.0)
        p.stop_driving(1.0)
        on_station = time.time()
        truth = p.truth()
        x, _, _ = in_station(truth)
        self.assertGreater(x, -0.01, truth)
        samples = p.samples_since(since)
        errors = [error(s)[0] for s in samples]
        near = [error(s)[0] for s in p.samples_since(at_gate)]
        headings = [abs(error(s)[1]) for s in p.samples_since(at_gate)]
        ratio = heading_ratio(samples)
        print(f"\ndock: way in {len(errors)} samples, worst {max(errors):.3f} m; from 0.3 m {len(near)} samples, worst "
              f"{max(near):.3f} m, heading {math.degrees(max(headings)):.1f} deg; heading error at most {ratio:.2f} "
              "times its accuracy", flush=True)
        self.assertGreater(len(errors), 40)
        # From the hint (5 degrees) at 0.87 m until the marker gives the direction, as at the start.
        self.assertLess(max(errors), 0.06)
        self.assertLess(max(near), 0.02)
        # The direction of the marker errs up to 3.5 degrees at a place: the heading within its reported accuracy.
        self.assertLessEqual(ratio, 3.0)
        # Good all the way, the blind stretch included.
        poses = p.poses_since(since)
        self.assertTrue(all(good(m) for m in poses), [m.position_accuracy for m in poses if not good(m)][:5])
        self.assertFalse([m for m in p.markers_since(blind + 0.5) if m.valid], "the marker in the blind distance")
        final = p.latest()
        self.assertTrue(good(final))
        self.assertLess(abs(final.x - x), 0.03)
        # On the contacts it stays good without a fix, also past contact_timeout + 2 s after the last one.
        time.sleep(13.0)
        self.assertTrue(good(p.latest()), p.latest().position_accuracy)
        self.assertGreater(time.time() - on_station, 12.0)

    def test_06_a_hidden_marker_ends_good(self):
        p = self.probe
        since = time.time()
        p.setpoint = (0.1, 0.0)
        self.assertTrue(p.wait(lambda: any(good(m) for m in p.poses_since(since)), 8), "the marker does not return")
        p.flags(dock_hollow_speed=0.0, marker_hidden=1)
        hidden = time.time()
        time.sleep(2.0)
        p.stop_driving(0.5)
        p.flags(dock_hollow_speed=0.0)
        with p.lock:
            after = [(t, m) for t, m in p.poses if t >= hidden]
        lost = next((t for t, m in after if not good(m)), None)
        self.assertIsNotNone(lost)
        # From the contacts: the blind distance and 0.05 m past it at 0.1 m/s, the delay of the camera and a message
        # on its way.
        self.assertLess(lost - hidden, 1.6)
        self.assertFalse([m for t, m in after if t > lost and t < hidden + 2.0 and good(m)])

    def test_07_faults_of_the_camera(self):
        p = self.probe
        since = time.time()
        p.flags(dock_hollow_speed=0.0, marker_dark=1)
        self.assertTrue(p.wait(lambda: any(m.reason == DockMarker.REASON_TOO_DARK
                                           for m in p.markers_since(since)), 3))
        since = time.time()
        p.flags(dock_hollow_speed=0.0, camera_off=1)
        fault = (DockMarker.STATE_FAULT, Camera.REASON_NO_SIGNAL)
        self.assertTrue(p.wait(lambda: any((m.state, m.reason) == fault for m in p.markers_since(since)), 3))
        p.asking = False
        since = time.time()
        self.assertTrue(p.wait(lambda: any(m.state == DockMarker.STATE_OFF for m in p.markers_since(since)),
                               DockMarker.REQUEST_HOLD_S + 2))
        self.assertEqual(p.markers_since(since)[-1].reason, Camera.REASON_NO_SIGNAL)
        p.flags(dock_hollow_speed=0.0)

    def test_08_six_degrees_at_one_place(self):
        p = self.probe
        p.asking = True
        p.hint = True
        # Outliers in a row drop the estimate, which starts again at rest from images whose direction errs 6 degrees.
        since = time.time()
        p.flags(dock_hollow_speed=0.0, marker_yaw_bias=6.0, marker_outlier=[0.3, 0.0, 1])
        self.assertTrue(p.wait(lambda: any(not m.heading_valid for m in p.poses_since(since)), 6), "not dropped")
        p.flags(dock_hollow_speed=0.0, marker_yaw_bias=6.0)
        started = time.time()
        self.assertTrue(p.wait(lambda: any(good(m) for m in p.poses_since(started + 0.3)), 6), "no new start")
        time.sleep(4.0)
        _, _, yaw = p.station()
        poses = [m for m in p.poses_since(started + 0.3) if good(m)]
        spread = min(yaw_spread(m) for m in p.markers_since(started) if m.valid and m.yaw_valid)
        miss = abs(wrap(poses[-1].heading - yaw))
        print(f"\ndock: 6 deg at one place: heading error {math.degrees(miss):.1f} deg, accuracy "
              f"{math.degrees(min(m.heading_accuracy for m in poses)):.1f} to "
              f"{math.degrees(max(m.heading_accuracy for m in poses)):.1f} deg, one image {math.degrees(spread):.1f} "
              "deg", flush=True)
        # Standing, more images of the place do not make the heading surer than one of them.
        self.assertGreaterEqual(min(m.heading_accuracy for m in poses), 0.999 * spread)
        self.assertGreater(miss, math.radians(4.0))
        self.assertLessEqual(miss, 3.0 * poses[-1].heading_accuracy)
        # Into the station, the error of the direction changing with the place again.
        p.flags(dock_hollow_speed=0.0)
        since = time.time()
        self.assertTrue(p.back_in(0.05, lambda s: s[0] >= -0.12, 10), p.station())
        p.stop_driving(0.5)
        samples = p.samples_since(since)
        self.assertGreater(len(samples), 10)
        self.assertLessEqual(heading_ratio(samples), 3.0)
