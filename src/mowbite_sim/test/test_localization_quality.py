"""How good the pose of mowbite_localization is against the true pose of fake_tron, with the faults of a real Tron.

The chain is fake_tron, tron_mcu, tron_gnss, tron_imu and the localization with the values of the Tron; the test plays
the logic (the geometry on bridge/params) and drives on hw/cmd_vel, steered by the true pose, so every localization
drives the same way. It measures: the heading alignment from the start heading and from a heading 90 degrees off (time
and distance until the heading is good), the position and heading error on a square, the error after 8 s without GNSS
at 0.3 m/s, the heading drift standing for 60 s after the gyro bias changed, and the error on a 10 degree side slope
with the antenna at the height of geometry.yaml of the Tron (at rest and across the way while driving). Every value
is printed as one line "quality: ...".

MOWBITE_GNSS_LATENCY=<s> gives the localization that gnss_latency.

Run in a sourced workspace: launch_test /ws/src/mowbite_sim/test/test_localization_quality.py
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
from geometry_msgs.msg import Twist
from launch.actions import TimerAction
from launch_ros.actions import Node
from mowbite_msgs.msg import RobotPose
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sim_common import (LATCHED, REAL_FAULTS, SimProbe, TestShutdown, estimate_at, fake_tron,  # noqa: E402, F401
                        layers, pose_stamp, quantile, sim_files, sim_links, wrap)

ANTENNA_X = 0.098
WORK = tempfile.mkdtemp(prefix="mowbite_quality_")
LINKS = sim_links(WORK)
STATE, FLAGS, _, _ = sim_files(WORK)
TRON = layers.files("tron")
GEOMETRY = layers.geometry("tron")
# The ports of the simulation: a section of the node wins over values for all nodes, so these come as one too.
LOCAL = os.path.join(WORK, "local.yaml")
GOOD_HEADING = math.radians(5.0)
SPEED = 0.3
# A gyro bias that changes by this much (deg/s) while the mower stands, e.g. as the sensor warms up.
BIAS_STEP_DPS = 0.3
# Limits: the targets of O19a (alignment within 3 s of driving from any heading, less than 0.5 deg/min of drift at
# rest) and room above what the localization reaches.
LIMITS = {
    "align_s": 3.0,
    "align_wrong_s": 3.0,
    "square_position_p95_m": 0.08,
    "square_heading_p95_deg": 1.5,
    "outage_position_m": 0.3,
    "outage_heading_deg": 1.0,
    "drift_deg_per_min": 0.5,
    # Level the antenna would put the pose 0.30 m * sin(10 deg) = 5.2 cm downhill.
    "slope_still_median_m": 0.015,
    "slope_across_driving_median_m": 0.015,
}


def generate_test_description():
    with open(LOCAL, "w") as f:
        f.write(f"tron_mcu:\n  ros__parameters:\n    port: {LINKS['mcu']}\n    safety_measurement_date: simulation\n"
                f"tron_gnss:\n  ros__parameters:\n    port: {LINKS['gnss']}\n    data_dir: {WORK}\n"
                f"tron_imu:\n  ros__parameters:\n    port: {LINKS['imu']}\n")
        if "MOWBITE_GNSS_LATENCY" in os.environ:
            f.write(f"mowbite_localization:\n  ros__parameters:\n"
                    f"    gnss_latency: {float(os.environ['MOWBITE_GNSS_LATENCY'])}\n")
    nodes = [
        Node(package="tron_mcu", executable="tron_mcu_node", name="tron_mcu", output="screen",
             parameters=[*TRON, LOCAL]),
        Node(package="tron_gnss", executable="tron_gnss_node", name="tron_gnss", output="screen",
             parameters=[*TRON, LOCAL, {"datum_lat": 52.0, "datum_long": 9.0, "datum_height": 50.0}]),
        Node(package="tron_imu", executable="tron_imu_node", name="tron_imu", output="screen",
             parameters=[*TRON, LOCAL]),
        Node(package="mowbite_localization", executable="mowbite_localization_node", name="mowbite_localization",
             output="screen", parameters=[GEOMETRY, *TRON, LOCAL]),
    ]
    # The ptys have to exist before the drivers look for them.
    return launch.LaunchDescription([
        fake_tron(LINKS, "--flags", FLAGS, "--state", STATE, "--antenna-offset", str(ANTENNA_X), "0.0"),
        TimerAction(period=1.5, actions=nodes + [launch_testing.actions.ReadyToTest()])]), {}


def report(name, value, unit):
    print(f"\nquality: {name} {'none' if value is None else f'{value:.3f}'} {unit}", flush=True)


class Probe(SimProbe):
    """Drives on hw/cmd_vel and pairs every true pose of fake_tron with the estimate of the same moment."""
    spin_timeout = 0.02
    faults = REAL_FAULTS

    def __init__(self):
        super().__init__("localization_quality_probe", state=STATE, flags=FLAGS)
        # (stamp s, x, y, yaw) of localization/pose, and the newest message
        self.estimates = []
        self.pose = None
        # (wall time, truth dict, estimate (x, y, yaw) or None, the RobotPose of that moment)
        self.samples = []
        self.setpoint = (0.0, 0.0)
        self.node.create_subscription(RobotPose, "localization/pose", self._pose, 50)
        self.cmd_pub = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        # The logic is not part of this launch: the probe sends the antenna offset of the mower.
        self.geometry = self.node.create_publisher(String, "bridge/params", LATCHED)
        self.geometry.publish(String(data=json.dumps(
            {"node": "/mowbite_geometry", "params": {"antenna_x": ANTENNA_X, "antenna_y": 0.0, "version": 0}})))
        self.start()
        for target in (self._record, self._drive):
            self.spawn(target)

    def _pose(self, msg):
        stamp = pose_stamp(msg)
        yaw = msg.heading
        with self.lock:
            self.pose = msg
            if not self.estimates or stamp > self.estimates[-1][0]:
                self.estimates.append((stamp, msg.x, msg.y, yaw))

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
                # The newest estimate has to be past the moment of the truth before the two are paired.
                time.sleep(0.06)
                with self.lock:
                    self.samples.append((written, truth, estimate_at(self.estimates, written), self.pose))
            time.sleep(0.02)

    def _drive(self):
        while self.running:
            msg = Twist()
            msg.linear.x, msg.angular.z = self.setpoint
            self.cmd_pub.publish(msg)
            time.sleep(0.1)

    def stop_driving(self, settle=1.5):
        self.setpoint = (0.0, 0.0)
        time.sleep(settle)

    def straight(self, seconds, until=None):
        """Drives at SPEED for seconds, holding the true heading of the start; until(sample) ends it early. Returns
        the first sample until accepts, or None."""
        heading = self.truth()["yaw"]
        checked = len(self.samples)
        end = time.time() + seconds
        while time.time() < end:
            error = wrap(heading - self.truth()["yaw"])
            self.setpoint = (SPEED, max(-0.3, min(0.3, 1.5 * error)))
            if until:
                with self.lock:
                    new, checked = self.samples[checked:], len(self.samples)
                found = next((s for s in new if until(s)), None)
                if found:
                    return found
            time.sleep(0.05)
        return None

    def turn(self, angle):
        """Turns on the spot by angle (rad), by the true heading."""
        target = self.truth()["yaw"] + angle
        end = time.time() + 30.0
        while time.time() < end:
            error = wrap(target - self.truth()["yaw"])
            if abs(error) < 0.02:
                break
            self.setpoint = (0.0, math.copysign(min(0.3, max(0.25, 2.0 * abs(error))), error))
            time.sleep(0.02)
        self.stop_driving(0.5)

    def samples_since(self, since):
        with self.lock:
            return [s for s in self.samples[since:] if s[2] is not None]


def errors(sample):
    """(position error m, heading error rad) of one sample."""
    _, truth, (x, y, yaw), _ = sample
    return math.hypot(x - truth["x"], y - truth["y"]), wrap(yaw - truth["yaw"])


class TestLocalizationQuality(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()
        cls.probe.flags()

    @classmethod
    def tearDownClass(cls):
        cls.probe.stop()
        rclpy.shutdown()
        shutil.rmtree(WORK, ignore_errors=True)

    def check(self, name, value):
        limit = LIMITS[name]
        if limit is not None:
            self.assertIsNotNone(value, name)
            self.assertLess(value, limit, name)

    def test_01_a_fix_and_no_heading_at_rest(self):
        p = self.probe
        good = p.wait(lambda: p.pose if p.pose is not None and p.pose.position_accuracy < 0.1 else None, 60)
        self.assertIsNotNone(good, "no localization/pose with a fix")
        self.assertFalse(good.heading_valid)

    def test_02_alignment_from_the_start_heading(self):
        p = self.probe
        start = p.truth()
        t0 = time.time()
        found = p.straight(30.0, until=lambda s: s[3] is not None and s[3].heading_valid)
        aligned = found[0] - t0 if found else None
        distance = math.hypot(found[1]["x"] - start["x"], found[1]["y"] - start["y"]) if found else None
        heading = math.degrees(abs(errors(found)[1])) if found and found[2] else None
        p.stop_driving()
        report("align_s", aligned, "s")
        report("align_m", distance, "m")
        report("align_heading_error_deg", heading, "deg")
        self.check("align_s", aligned)

    def test_03_errors_on_a_square(self):
        p = self.probe
        since = len(p.samples)
        for _ in range(4):
            p.straight(10.0)
            p.stop_driving(0.5)
            p.turn(math.pi / 2)
        samples = p.samples_since(since)
        self.assertGreater(len(samples), 100)
        position = [errors(s)[0] for s in samples]
        heading = [math.degrees(abs(errors(s)[1])) for s in samples]
        for name, values, unit in (("square_position", position, "m"), ("square_heading", heading, "deg")):
            report(f"{name}_median", quantile(values, 0.5), unit)
            report(f"{name}_p95", quantile(values, 0.95), unit)
            report(f"{name}_max", max(values), unit)
        # Driving or not, and how far the estimate is ahead (+) of the truth along the way while driving: a lag shows
        # there, noise does not.
        driving = [s for s in samples if abs(s[1]["linear"]) > 0.1]
        still = [errors(s)[0] for s in samples if abs(s[1]["linear"]) < 0.01]
        along = [(s[2][0] - s[1]["x"]) * math.cos(s[1]["yaw"]) + (s[2][1] - s[1]["y"]) * math.sin(s[1]["yaw"])
                 for s in driving]
        report("square_position_driving_median", quantile([errors(s)[0] for s in driving], 0.5), "m")
        report("square_position_still_median", quantile(still, 0.5) if still else None, "m")
        report("square_along_driving_median", quantile(along, 0.5), "m")
        self.check("square_position_p95_m", quantile(position, 0.95))
        self.check("square_heading_p95_deg", quantile(heading, 0.95))

    def test_04_alignment_from_a_heading_90_degrees_off(self):
        p = self.probe
        truth = p.truth()
        # Somebody turns the mower by hand: the gyro does not see it, the estimate keeps the old heading.
        p.flags(place=[truth["x"], truth["y"], math.degrees(truth["yaw"]) + 90.0])
        time.sleep(1.5)
        start = p.truth()
        before = p.samples_since(len(p.samples) - 3)
        report("wrong_heading_before_deg", math.degrees(abs(errors(before[-1])[1])) if before else None, "deg")
        t0 = time.time()
        # Good from the first of 5 true poses in a row (1 s) with a heading error under GOOD_HEADING.
        run = []

        def good(sample):
            if sample[2] is None:
                return False
            if abs(errors(sample)[1]) < GOOD_HEADING:
                run.append(sample)
            else:
                run.clear()
            return len(run) >= 5

        found = p.straight(40.0, until=good) and run[0]
        aligned = found[0] - t0 if found else None
        distance = math.hypot(found[1]["x"] - start["x"], found[1]["y"] - start["y"]) if found else None
        valid = p.pose.heading_valid
        p.stop_driving()
        report("align_wrong_s", aligned, "s")
        report("align_wrong_m", distance, "m")
        report("align_wrong_valid_meanwhile", float(valid), "")
        self.check("align_wrong_s", aligned)

    def test_05_eight_seconds_without_gnss(self):
        p = self.probe
        p.wait(lambda: p.pose.position_accuracy < 0.1, 30)
        p.straight(2.0)
        p.flags(gnss_quality=0)
        p.straight(8.0)
        last = p.samples_since(len(p.samples) - 2)
        accuracy = p.pose.position_accuracy
        p.flags()
        p.straight(3.0)
        p.stop_driving()
        self.assertTrue(last, "no sample at the end of the outage")
        position, heading = errors(last[-1])
        report("outage_position_m", position, "m")
        report("outage_heading_deg", math.degrees(abs(heading)), "deg")
        report("outage_reported_accuracy_m", accuracy, "m")
        self.check("outage_position_m", position)
        self.check("outage_heading_deg", math.degrees(abs(heading)))
        self.assertTrue(p.wait(lambda: p.pose.position_accuracy < 0.1, 30), "no fix after the outage")

    def test_06_heading_drift_at_rest_after_the_gyro_bias_changes(self):
        p = self.probe
        p.stop_driving(3.0)
        p.flags(gyro_bias_dps=BIAS_STEP_DPS)
        time.sleep(1.0)
        first = p.samples_since(len(p.samples) - 2)[-1]
        time.sleep(60.0)
        last = p.samples_since(len(p.samples) - 2)[-1]
        drift = math.degrees(abs(wrap(errors(last)[1] - errors(first)[1]))) * 60.0 / (last[0] - first[0])
        report("drift_deg_per_min", drift, "deg/min")
        since = len(p.samples)
        p.straight(5.0)
        p.stop_driving()
        heading = [math.degrees(abs(errors(s)[1])) for s in p.samples_since(since)]
        report("after_drift_heading_max_deg", max(heading) if heading else None, "deg")
        p.flags()
        self.check("drift_deg_per_min", drift)

    def test_07_on_a_side_slope(self):
        p = self.probe
        p.stop_driving(1.0)
        p.flags(tilt_roll=10.0)
        p.stop_driving(3.0)
        since = len(p.samples)
        p.stop_driving(3.0)
        p.straight(6.0)
        p.stop_driving(2.0)
        samples = p.samples_since(since)
        p.flags()
        still = [errors(s)[0] for s in samples if abs(s[1]["linear"]) < 0.01]
        across = [abs(-(s[2][0] - s[1]["x"]) * math.sin(s[1]["yaw"]) + (s[2][1] - s[1]["y"]) * math.cos(s[1]["yaw"]))
                  for s in samples if abs(s[1]["linear"]) > 0.1]
        self.assertTrue(still and across, "no samples at rest or driving")
        report("slope_still_median", quantile(still, 0.5), "m")
        report("slope_across_driving_median", quantile(across, 0.5), "m")
        self.check("slope_still_median_m", quantile(still, 0.5))
        self.check("slope_across_driving_median_m", quantile(across, 0.5))
