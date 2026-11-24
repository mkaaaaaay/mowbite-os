#!/usr/bin/env python3
"""Safety measurement of a mower behind hw/*: programme M1-M8 of the NX60i (xcore_driver, next to the minimal launch)
and T1-T7 of the Tron (--platform tron: this programme starts tron_imu and tron_mcu itself, see TronStart).

Asks before every step and shows what it will send. In the air the wheels get at most 0.1 m/s each, on the ground in
M8 (only with --ground) 0.3 m/s, in the fast runs of M8 (only with --fast as well, and only up to the limit the driver
reports) 0.5 m/s. The fast runs need the raised start of the measurement image (speed_limit of measure.launch.py);
a driver reporting more than 0.3 m/s takes nothing else. They go in stages, 0.4 m/s first, and stop at their limits:
a wheel above 0.6 m/s for 0.2 s, the wheels standing late or after too long a way, runs of a case too far apart.
Any key while a step runs aborts it: three stops on hw/cmd_vel, the blade off, the emergency latched
over hw/emergency; keys within 0.4 s after an answer are dropped, so a double tap does not abort the step it started.
Before the first step that can turn a motor (any step but STANDSTILL_STEPS) comes the STOP check: each STOP button
pressed and let go once, hw/status has to show stop with its emergency, after the release neither (the release goes
out once stop is free, on the Tron, whose MCU keeps stop until it, a few seconds after the request to let go). Only
what this run saw counts and nothing switches it off: before it this programme sends no wheel command and no blade on.
Records hw/status, hw/wheel_odom, imu/data, hw/capabilities and its own commands as JSON lines with a size and a time
limit. The capture of the inner network (tcpdump on the host) is the second record; the report on both
comes from xcore_capture.py.

With --fake-flags the actions of the person (press STOP, lift, tilt, turn a wheel) go to fake_xcore's or fake_tron's
flag file instead, for the rehearsal against the simulated board.

On the Tron the wheels turn at most 0.1 m/s, in the air only; the first turn is at 0.05 m/s (0.1 m/s if no wheel turns
at 0.05), followed at once by a check of STOP. For the time the MCU holds a setpoint without its host (T2) tron_mcu is
frozen (SIGSTOP) or killed; only while it is stopped or gone this programme reads the MCU port, read only and without
touching its settings, into a capture file (at most capture_limit bytes each), which tron_mcu_capture decodes with the
codec of tron_mcu. The values for the parameter file of the Tron come out as a proposal only; nothing is written there.
T6 (only with --blade, the blade disc off) asks the same of the blade motor: tron_mcu runs with the blade allowed for
that step alone, the motor turns free on the lease of mowing, and the host goes the same two ways.
T7 (also only with --blade) lifts the mower by hand: without a motor each side of --lift-sides (default the front) in
that order until hw/status shows lift (with the tilt of imu/data then), the whole mower level, then the front while the
blade motor turns. The Tron's MCU shows lift only with both front wheels off the ground: no lift at another side is a
note, not a finding. Where the person has to act while the blade motor turns, --input-wait gives the time; lifting
without a motor waits for lift. On the Tron as root: mowbite-measure --steps T6 --blade, or --steps T7 --blade
(optionally --input-wait 30, --lift-sides left,right).
T8 asks how the MCU keeps STOP, bumper and lift and what a clear frame drops (G37), without a motor and without a
wheel command: tron_mcu sends the clear frame with each release while stop or lift shows, at most one a second. A short
press of STOP, STOP held through releases, a bumper, the front lifted with a release while it is up and once it is
down, then tron_mcu killed with the port read until the MCU shows stop or lift on its own: mowbite-measure --steps T8.
T9 asks how long the MCU keeps a setpoint once the host's frames end, by what came last (H3): for each case of --hold
tron_mcu starts again with measurement settings, the wheels turn in the air and hw/silence begins right after the
chosen frame; the driver stops them when the silence ends. G adds a long silence with heartbeats alone afterwards, B
(with --blade, the disc off) the blade on frame last: mowbite-measure --steps T9 [--hold V0,V1,V2,V3,G].
T10 (only with --blade, the disc off) asks what the cutting height does, which the MCU does not report: with the blade
off another height in the off frames, the deck down with the start of the blade motor and up with its off, then new
heights while it turns. The person reads the time the deck moved off a video; the programme records the motor's
rpm and status meanwhile: mowbite-measure --steps T10 --blade.
"""
import argparse
import collections
import datetime
import json
import math
import os
import pwd
import select
import signal
import statistics
import subprocess
import sys
import termios
import threading
import time
import tty

import yaml

import rclpy
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from mowbite_msgs.msg import HardwareCapabilities, HardwareStatus
from mowbite_msgs.srv import SetBlade, SetEmergency, SilenceDriver
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu

AIR_WHEEL_SPEED = 0.1
GROUND_WHEEL_SPEED = 0.3
FAST_WHEEL_SPEED = 0.5
# Fast runs of M8: first at FAST_FIRST_SPEED; the top speed only if the first one went slow enough that the top one, as
# much faster as its command, stays at FAST_ABORT_SPEED or below. A wheel above FAST_ABORT_SPEED for FAST_ABORT_S aborts.
FAST_FIRST_SPEED = 0.4
FAST_ABORT_SPEED = 0.6
FAST_ABORT_S = 0.2
FAST_REPEATS = 3
# Limits of a fast run: the wheels stand within FAST_STAND_S of the cause (STOP pressed, or the last command), after at
# most FAST_DISTANCE_M in the driver's odometry; the runs of one case lie within FAST_SPREAD_M of each other.
FAST_STAND_S = {"stop": 0.5, "quiet": 1.0}
FAST_DISTANCE_M = {"stop": 0.30, "quiet": 0.60}
FAST_SPREAD_M = 0.15
# A wheel this slow stands; the Tron reports up to 0.011 m/s for one standing wheel (07.11.).
STILL_SPEED = 0.005
TRON_STILL_SPEED = 0.015
# Keys this soon after an answer belong to it (a double tap on a phone), not to the step it starts.
KEY_SETTLE_S = 0.4
# STOP check: hw/status has to show a STOP button within STOP_CHECK_S of the request to press it, the emergency within
# STOP_EMERGENCY_S of that. The release goes out once the button is free, within STOP_CHECK_S, or on a board that keeps
# stop until the release STOP_LET_GO_S after the request to let go; stop and the emergency then stay off STOP_FREE_S.
STOP_CHECK_S = 60.0
STOP_EMERGENCY_S = 1.0
STOP_LET_GO_S = 3.0
STOP_FREE_S = 2.0
# A driver frozen longer than the limit of its watch ends itself this soon after SIGCONT.
WATCH_END_S = 2.0
# Emergency bits of hw/status by bit number, named as in the catalog of mowbite_api (x-emergency-bits);
# test/test_names.py holds them against the catalog, the message and the board's own names.
EMERGENCY_BITS = {0: "LATCH", 1: "TIMEOUT_INPUTS", 2: "STOP", 3: "LIFT", 4: "LIFT_MULTIPLE", 5: "COLLISION",
                  6: "TIMEOUT_HIGH_LEVEL", 7: "HIGH_LEVEL", 8: "SERVICE_NOT_READY", 9: "COLLISION_MULTIPLE",
                  10: "MOWER_RPM_TIMEOUT", 11: "MOWER_RPM_LIMIT", 12: "FIRMWARE_INCOMPATIBLE", 13: "TILT",
                  14: "FENCE", 15: "STUCK", 16: "IMU_TIMEOUT", 17: "MOWER_NOT_STOPPED", 18: "WHEEL_RUNAWAY",
                  19: "FOREIGN_HOST", 20: "DRIVER_STALL", 21: "UNEXPECTED_RESTART",
                  22: "INPUTS_LEARNING", 23: "BOARD_NOT_HEARD"}
DRIVER = "xcore_driver_node"
SERVICE_NOT_READY = 256

# The Tron (T1-T7). The firmware this programme was written against, read on the Tron 07.11.; another one ends the run.
TRON_FIRMWARE = {"cutter": "0.6.36", "chassis": "0.6.34"}
TRON_FIRST_SPEED = 0.05
# Runs of heartbeat alone, of each input in T4, of the driver's watchdog in T5 and of the front lift in T7 (host gone:
# --repeats).
TRON_SHORT_REPEATS = 3
# Host gone (T2): the programme reads the port for HOST_GONE_WINDOW_S after the signal. In every run the wheels have to
# stand within HOST_GONE_LIMIT_S of it, the runs of one way within HOST_GONE_SPREAD_S of each other.
HOST_GONE_WINDOW_S = 5.0
HOST_GONE_LIMIT_S = 3.0
HOST_GONE_SPREAD_S = 0.3
# Heartbeat alone (T2): tron_mcu of that start sends each command once and stops the wheels only after this long
# without hw/cmd_vel.
HEARTBEAT_ONLY_TIMEOUT_S = 10.0
HEARTBEAT_ONLY = {"cmd_vel_timeout": HEARTBEAT_ONLY_TIMEOUT_S, "speed_repeat_window": 0.0,
                  "heartbeat_only_measurement": True}
# Hold after the last frame (T9): each case starts tron_mcu again with its measurement settings and silence_max; the
# wheels turn HOLD_DRIVE_S, then hw/silence starts right after the chosen frame. A stand later than HOLD_HELD_S after
# it counts as held. G: a long silence, then heartbeats alone for HOLD_QUIET_S before the stops. B: the blade on frame
# last (a new speed for the turning motor), with --blade. Case: (text, overrides, silence s, runs or None = --repeats).
HOLD_DRIVE_S = 1.0
HOLD_HELD_S = 1.0
HOLD_QUIET_S = 3.0
HOLD_ARM_S = 0.5
# The driver latches LATCH|HIGH_LEVEL at the end of a silence; a start this much before the frame still counts.
HOLD_END_REASON = 1 << 0 | 1 << 7
HOLD_SLACK_S = 0.05
HOLD_CASES = {
    "V0": ("the heartbeat ahead of the setpoint only", {"speed_heartbeat": "", "silence_after": "speed"}, 3.0, None),
    "V1": ("a heartbeat right after each setpoint, in one write", {"speed_heartbeat": "after",
                                                                    "silence_after": "speed"}, 3.0, None),
    "V2": ("a heartbeat in the frame of each setpoint", {"speed_heartbeat": "frame", "silence_after": "speed"}, 3.0,
           None),
    "V3": ("a heartbeat 25 ms after each setpoint, as the Tron runs", {"speed_heartbeat": "later",
                                                                       "silence_after": "speed"}, 3.0, None),
    "G": (f"as V0 with a long silence, then heartbeats alone for {HOLD_QUIET_S:.0f} s before the stops",
          {"speed_heartbeat": "", "silence_after": "speed", "silence_stop_delay": HOLD_QUIET_S}, 10.0, 2),
    "B": ("the blade on frame last, the disc off", {"silence_after": "cutter", "allow_blade": True}, 3.0, 3),
}
HOLD_DEFAULT = ("V0", "V1", "V2")
# The wheels stand this soon after an emergency (T2, T4), after the driver's watchdog (T5). The driver reports
# moving false once its watchdog fired: T5 looks at it WATCHDOG_MARGIN_S after that.
EMERGENCY_STAND_S = 1.0
WATCHDOG_MARGIN_S = 0.5
# Blade without host (T6), the disc off: the motor turns at each of --blade-percent (default TRON_BLADE_PERCENT, at
# most BLADE_PERCENT_COUNT values) for BLADE_RUN_S, then the host goes and the programme reads the port for
# --blade-window s (default BLADE_GONE_WINDOW_S, within BLADE_WINDOW_RANGE_S); the MCU reports the motors about once a
# second. The height stays where tron_mcu leaves a blade that is off, so the height motor does not move.
TRON_BLADE_PERCENT = (30, 100)
BLADE_PERCENT_COUNT = 3
TRON_BLADE_HEIGHT_MM = 90
BLADE_RUN_S = 4.0
BLADE_GONE_WINDOW_S = 10.0
BLADE_WINDOW_RANGE_S = (5, 120)
# The person acts within --input-wait s (default BLADE_INPUT_WAIT_S) once the blade motor turns (T6, T7).
BLADE_INPUT_WAIT_S = 10.0
INPUT_WAIT_RANGE_S = (5, 60)
# Lift (T7), the disc off: by hand each side of --lift-sides goes up to LIFT_MAX_DEG, the programme waits up to
# LIFT_WAIT_S for lift and as long for the set-down; on the blade motor at LIFT_BLADE_PERCENT LIFT_SHOWN_SIDE goes up to
# TRON_SHORT_REPEATS times. Per side: how the person reads it and the tilt of fake_tron (degrees; the front up is
# pitch < 0). The Tron's MCU shows lift only with both front wheels off the ground, so only at LIFT_SHOWN_SIDE.
LIFT_MAX_DEG = 45
# An imu/data sample off 1 g by more than this is a knock or a jolt of the hands, not a tilt; the tilt is the median
# of the last TILT_MEDIAN samples left (about 0.1 s), since a knock also brings single samples near 1 g far off.
TILT_SHOCK_MS2 = 1.5
TILT_MEDIAN = 9
LIFT_WAIT_S = 60.0
LIFT_BLADE_PERCENT = 100
# Cutting height (T10), the disc off: the deck from TRON_BLADE_HEIGHT_MM down to HEIGHT_LOW_MM as the blade motor
# starts and back up as it goes off, then to each of HEIGHT_WHILE_TURNING while it turns, HEIGHT_RUN_S a height on a
# renewed lease. The speed has no effect on the Tron (T6.1).
HEIGHT_LOW_MM = 30
# Before that, with the blade off, tron_mcu starts with its off frames at HEIGHT_PARKED_MM, then again as shipped.
HEIGHT_PARKED_MM = 40
HEIGHT_WHILE_TURNING = (60, 30, 90)
HEIGHT_RUN_S = 10.0
HEIGHT_BLADE_PERCENT = 30
LIFT_SIDES = {"front": ("the front", {"tilt_pitch": -38.0}), "rear": ("the rear", {"tilt_pitch": 38.0}),
              "left": ("the left side", {"tilt_roll": 38.0}), "right": ("the right side", {"tilt_roll": -38.0})}
LIFT_SHOWN_SIDE = "front"
# A tilt of the rehearsal goes up this long before the lift it brings.
LEAD_S = 0.5
# Limit of a capture: CAPTURE_MAX_BYTES, for a long window CAPTURE_BYTES_PER_S where that is more (capture_limit). The
# Tron's MCU filled 8.4 kB a second (T2, 08.11.). A capture that reached its limit ends the step.
CAPTURE_MAX_BYTES = 1024 * 1024
CAPTURE_BYTES_PER_S = 32 * 1024
NODE_LOG_BYTES = 5 * 1024 * 1024
# STOP latch of the MCU (T8, G37), no motor: a press is let go within G37_LET_GO_S; a held STOP gets a release every
# G37_RELEASE_EVERY_S for G37_HOLD_S; a release counts as dropping a flag within G37_CLEAR_WAIT_S; the front stays up
# G37_LIFT_HOLD_S in the rehearsal; with tron_mcu killed the port is read for G37_GONE_WINDOW_S.
G37_LET_GO_S = 2.0
G37_HOLD_S = 6.0
G37_RELEASE_EVERY_S = 1.0
G37_CLEAR_WAIT_S = 1.5
G37_LIFT_HOLD_S = 4.0
G37_GONE_WINDOW_S = 20.0


class Abort(Exception):
    pass


class Quit(Exception):
    pass


def reason_text(bits):
    """Names of the set bits in their order; set bits without a name add UNKNOWN once at the end."""
    names = [name for bit, name in sorted(EMERGENCY_BITS.items()) if bits >> bit & 1]
    if bits & ~sum(1 << bit for bit in EMERGENCY_BITS):
        names.append("UNKNOWN")
    return "|".join(names) if names else "0"


def fastest_wheel(v, w, wheel_distance):
    return max(abs(v - w * wheel_distance / 2), abs(v + w * wheel_distance / 2))


def tilt_of(acc):
    """Degrees from upright of one accelerometer sample (m/s2); None during a knock, when |a| is off 1 g."""
    if abs(math.hypot(*acc) - 9.81) > TILT_SHOCK_MS2:
        return None
    return math.degrees(math.atan2(math.hypot(acc[0], acc[1]), acc[2]))


def steady_speed(log, start, end, wheel_distance):
    """Mean speed of the faster wheel over the samples (t, v, w) from start to end; None without one."""
    speeds = [fastest_wheel(v, w, wheel_distance) for t, v, w in log if start <= t <= end]
    return sum(speeds) / len(speeds) if speeds else None


def still_after(log, since, wheel_distance, still=STILL_SPEED):
    """Time of the first sample (t, v, w) at or after `since` with both wheels standing; None if there is none."""
    for t, v, w in log:
        if t >= since and fastest_wheel(v, w, wheel_distance) <= still:
            return t
    return None


def fast_run_problems(kind, stand_s, metres):
    """What a fast run of kind "stop" or "quiet" did beyond its limits, as texts."""
    problems = []
    if stand_s is None or stand_s > FAST_STAND_S[kind]:
        shown = "never" if stand_s is None else f"after {stand_s:.2f} s"
        problems.append(f"the wheels stood {shown}, limit {FAST_STAND_S[kind]:.1f} s")
    if metres > FAST_DISTANCE_M[kind]:
        problems.append(f"{metres:.2f} m until they stood, limit {FAST_DISTANCE_M[kind]:.2f} m")
    return problems


def spread_problem(metres):
    """The runs of one case further apart than FAST_SPREAD_M, as a text; None otherwise."""
    if len(metres) < 2 or max(metres) - min(metres) <= FAST_SPREAD_M:
        return None
    return f"stopping distances {min(metres):.2f}-{max(metres):.2f} m, spread above {FAST_SPREAD_M:.2f} m"


def top_speed_allowed(first_steady, top):
    """The top speed runs only if the speed at FAST_FIRST_SPEED, scaled like the command, stays within the abort."""
    return first_steady is not None and first_steady * top / FAST_FIRST_SPEED <= FAST_ABORT_SPEED + 1e-9


def host_gone_result(packets, since, wheel_distance):
    """What the MCU reported after its host went at `since`, from the decoded capture: when the wheels stood for good
    (None while a wheel still turned at the end), whether they turned at the first sample, when lift and stop came up,
    the last report of a drive motor turning, and the number of speed samples."""
    speeds = [(p["t"], fastest_wheel(*p["speed"], wheel_distance)) for p in packets if "speed" in p and p["t"] >= since]
    stood = None
    for t, speed in speeds:
        if speed > TRON_STILL_SPEED:
            stood = None
        elif stood is None:
            stood = t

    def first(flag):
        return next((p["t"] - since for p in packets if p.get("sensor", {}).get(flag) and p["t"] >= since), None)
    turning = [p["t"] for p in packets
               if "motors" in p and p["t"] >= since and (p["motors"]["left"] or p["motors"]["right"])]
    return {"stand_s": None if stood is None else stood - since,
            "turning_at_start": bool(speeds) and speeds[0][1] > TRON_STILL_SPEED, "lift_s": first("lift"),
            "stop_s": first("stop"), "rpm_last_s": turning[-1] - since if turning else None, "samples": len(speeds)}


def blade_gone_result(packets, since, still_rpm):
    """What the MCU reported of the blade motor after its host went at `since`, from the decoded capture: when it stood
    for good (None while it still turned at the end), whether it turned at the first report, the last rpm and the
    number of reports."""
    reports = [(p["t"], abs(p["motors"]["cutter"])) for p in packets if "motors" in p and p["t"] >= since]
    stood = None
    for t, rpm in reports:
        if rpm >= still_rpm:
            stood = None
        elif stood is None:
            stood = t
    return {"still_s": None if stood is None else stood - since,
            "turning_at_start": bool(reports) and reports[0][1] >= still_rpm,
            "last_rpm": reports[-1][1] if reports else None, "samples": len(reports)}


def whole_number(text, low, high):
    """`text` as a whole number from `low` to `high`; None if it is none."""
    item = text.strip()
    return int(item) if item.isascii() and item.isdigit() and low <= int(item) <= high else None


def blade_percents(text):
    """The values of --blade-percent: whole numbers 1-100, comma separated, at most BLADE_PERCENT_COUNT, each once (the
    captures of a run are named by its percent). ValueError with the reason otherwise."""
    values = [whole_number(item, 1, 100) for item in text.split(",")]
    if len(values) > BLADE_PERCENT_COUNT:
        raise ValueError(f"at most {BLADE_PERCENT_COUNT} values, got {len(values)}")
    if None in values:
        raise ValueError(f"each value a whole number from 1 to 100, got {text!r}")
    if len(set(values)) < len(values):
        raise ValueError(f"each value once, got {text!r}")
    return tuple(values)


def whole_seconds(text, bounds):
    """The value of --blade-window or --input-wait: whole seconds within `bounds`, as the questions show them.
    ValueError with the reason otherwise."""
    low, high = bounds
    seconds = whole_number(text, low, high)
    if seconds is None:
        raise ValueError(f"a whole number from {low} to {high}, got {text!r}")
    return float(seconds)


def lift_sides(text):
    """The value of --lift-sides: sides of LIFT_SIDES, comma separated, each once, in the order given. ValueError with
    the reason otherwise."""
    sides = [item.strip() for item in text.split(",")]
    unknown = [side for side in sides if side not in LIFT_SIDES]
    if unknown:
        raise ValueError(f"each side one of {', '.join(LIFT_SIDES)}, got {text!r}")
    if len(set(sides)) < len(sides):
        raise ValueError(f"each side once, got {text!r}")
    return tuple(sides)


def hold_cases(text):
    """The value of --hold: cases of HOLD_CASES, comma separated, each once, in the order given. ValueError with the
    reason otherwise."""
    cases = [item.strip().upper() for item in text.split(",")]
    unknown = [case for case in cases if case not in HOLD_CASES]
    if unknown:
        raise ValueError(f"each case one of {', '.join(HOLD_CASES)}, got {text!r}")
    if len(set(cases)) < len(cases):
        raise ValueError(f"each case once, got {text!r}")
    return tuple(cases)


def lift_blade_items(tilts, unlifted, runs):
    """The lifts of T7.3 as inputs of blade_inputs: LIFT_SHOWN_SIDE `runs` times, none if it is in `unlifted` (T7.1 saw
    no lift there); past the tilt at lift T7.1 found (`tilts` by side), until lift shows if that is unknown."""
    side = LIFT_SHOWN_SIDE
    if side in unlifted:
        return []
    name, tilt = LIFT_SIDES[side]
    angle = tilts.get(side)
    how = "until lift shows" if angle is None else f"past {angle:.0f} deg"
    return [(f"lift {name} {how} while the blade motor turns", {"lift": True, **tilt}, runs, f"set {name} down")]


def capture_limit(window):
    """Bytes the capture of a window of `window` s may take."""
    return max(CAPTURE_MAX_BYTES, int(window * CAPTURE_BYTES_PER_S))


def holding_problems(stands):
    """The host-gone runs of one way (stand times in s, None: did not stand) beyond the limits, as texts."""
    problems = []
    if any(s is None for s in stands):
        problems.append(f"in {sum(s is None for s in stands)} of {len(stands)} runs the wheels did not stand within "
                        f"{HOST_GONE_WINDOW_S:.0f} s")
    known = [s for s in stands if s is not None]
    if known and max(known) > HOST_GONE_LIMIT_S:
        problems.append(f"the wheels stood after up to {max(known):.2f} s, limit {HOST_GONE_LIMIT_S:.1f} s")
    if len(known) >= 2 and max(known) - min(known) > HOST_GONE_SPREAD_S + 1e-9:
        problems.append(f"stand times {min(known):.2f}-{max(known):.2f} s, spread above {HOST_GONE_SPREAD_S:.1f} s")
    return problems


def failsafe_proposal(stands):
    """failsafe_without_host from the stand times of all host-gone runs: the longest, rounded up to 0.1 s."""
    known = [s for s in stands if s is not None]
    return math.ceil(max(known) * 10 - 1e-6) / 10 if known else None


def tron_values_text(failsafe, firmware, date):
    """The proposal for the measured values of the Tron, in the form of its parameter file. The firmware lists name only
    the firmware this measurement ran with; a board that did not report one keeps the list of the platform."""
    lines = ["tron_mcu:", "  ros__parameters:", f"    failsafe_without_host: {failsafe:.1f}",
             f'    safety_measurement_date: "{date}"']
    lines += [f'    {board}_firmware: ["{firmware[board]}"]' for board in ("cutter", "chassis") if firmware.get(board)]
    return "\n".join(lines)


def termios_problem(attrs):
    """Why a port with these settings must not be read without changing them; None if it may. Echo and canonical mode
    would answer the MCU, software flow control would send it XON and XOFF."""
    iflag, _, cflag, lflag, ispeed, ospeed, _ = attrs
    if lflag & (termios.ECHO | termios.ECHONL | termios.ICANON):
        return "echo or canonical mode is on"
    if iflag & (termios.IXON | termios.IXOFF):
        return "software flow control is on"
    if not cflag & termios.CREAD:
        return "the receiver is off"
    if ispeed != termios.B115200 or ospeed != termios.B115200:
        return "the speed is not 115200 baud"
    return None


class SpeedGuard:
    """Trips once a wheel stays above `limit` m/s for `hold` s."""

    def __init__(self, limit, hold):
        self.limit, self.hold = limit, hold
        self.since = None
        self.tripped = None

    def update(self, t, fastest):
        if fastest <= self.limit:
            self.since = None
            return
        if self.since is None:
            self.since = t
        if self.tripped is None and t - self.since >= self.hold - 1e-9:
            self.tripped = (fastest, t - self.since)


class Recorder:
    """JSON lines with a limit in bytes; past it nothing more is written and the steps refuse to start."""

    def __init__(self, path, max_bytes, max_seconds, imu_every):
        self.file = open(path, "x")
        self.path = path
        self.left = max_bytes
        self.end = time.monotonic() + max_seconds
        self.imu_every = imu_every
        self.imu_count = 0
        self.full = False
        self.lock = threading.Lock()

    def write(self, kind, **fields):
        line = json.dumps({"t": time.monotonic(), "wall": time.time(), "kind": kind, **fields}) + "\n"
        with self.lock:
            if self.full:
                return
            if len(line) > self.left:
                self.full = True
                self.file.write(json.dumps({"t": time.monotonic(), "kind": "limit", "what": "size"}) + "\n")
                self.file.flush()
                return
            self.left -= len(line)
            self.file.write(line)
            if kind != "imu":
                self.file.flush()

    def imu(self, **fields):
        self.imu_count += 1
        if self.imu_count % self.imu_every == 0:
            self.write("imu", **fields)

    def over(self):
        return self.full or time.monotonic() > self.end

    def close(self):
        with self.lock:
            self.file.flush()
            self.file.close()


class Keys:
    """Any key on the terminal, or the abort file of a rehearsal."""

    def __init__(self, abort_file):
        self.abort_file = abort_file
        self.tty = sys.stdin.isatty()
        self.saved = None
        if self.tty:
            self.saved = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin)

    def restore(self):
        if self.saved is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.saved)
            self.saved = None

    def drain(self):
        while self.tty and select.select([sys.stdin], [], [], 0)[0]:
            os.read(sys.stdin.fileno(), 64)

    def settle(self):
        """Drops the keys of KEY_SETTLE_S after an answer; the wheels stand meanwhile."""
        end = time.monotonic() + KEY_SETTLE_S
        while self.tty and time.monotonic() < end:
            self.drain()
            time.sleep(0.02)

    def pressed(self):
        if self.abort_file and os.path.exists(self.abort_file):
            return True
        if self.tty and select.select([sys.stdin], [], [], 0)[0]:
            os.read(sys.stdin.fileno(), 64)
            return True
        return False

    def read_key(self):
        if not self.tty:
            line = sys.stdin.readline()
            return line[:1] if line else "q"
        self.drain()
        return os.read(sys.stdin.fileno(), 1).decode(errors="replace")

    def read_line(self, prompt):
        self.restore()
        try:
            return input(prompt)
        finally:
            if self.tty:
                self.saved = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin)


class Mower:
    """The driver's topics and services."""

    def __init__(self, rec, wheel_limit, wheel_distance):
        self.rec = rec
        self.wheel_limit = wheel_limit
        self.wheel_distance = wheel_distance
        self.lock = threading.Lock()
        self.status = None
        self.status_at = 0.0
        self.odom = (0.0, 0.0)
        self.odom_at = 0.0
        self.max_tilt = 0.0
        # The tilt of the last imu/data sample; the time and that tilt of the first lift since reset_tilt.
        self.tilt = None
        self.tilts = collections.deque(maxlen=TILT_MEDIAN)
        self.first_lift = None
        self.max_linear_speed = None
        self.emergency_since = None
        self.last_cmd_at = None
        self.guard = None
        # Set by the STOP check of this run alone; before it no wheel command but a stop and no blade on go out.
        self.stop_checked = False
        self.firmware = {}
        self.capability_wheel_distance = None
        # Messages per topic, the inputs hw/status showed since reset_seen, a_z of imu/data.
        self.counts = collections.Counter()
        self.seen = set()
        self.acc_z = collections.deque(maxlen=3000)
        # (t, v, w) of hw/wheel_odom.
        self.odom_log = collections.deque(maxlen=3000)
        self.node = rclpy.create_node("yf_measure")
        self.node.create_subscription(HardwareStatus, "hw/status", self._status, 10)
        self.node.create_subscription(TwistStamped, "hw/wheel_odom", self._odom, 20)
        self.node.create_subscription(Imu, "imu/data", self._imu, qos_profile_sensor_data)
        self.node.create_subscription(HardwareCapabilities, "hw/capabilities", self._capabilities,
                                      QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1))
        self.cmd_vel = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.blade = self.node.create_client(SetBlade, "hw/blade")
        self.silence_client = self.node.create_client(SilenceDriver, "hw/silence")
        self.running = True
        self.spinner = threading.Thread(target=self._spin, daemon=True)
        self.spinner.start()

    def _spin(self):
        while self.running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _status(self, msg):
        with self.lock:
            if msg.emergency and not (self.status is not None and self.status.emergency):
                self.emergency_since = time.monotonic()
            self.status, self.status_at = msg, time.monotonic()
            self.counts["status"] += 1
            self.seen.update(name for name in ("stop", "lift", "bumper_left", "bumper_right") if getattr(msg, name))
            if msg.lift and self.first_lift is None:
                self.first_lift = (self.status_at, self.tilt)
        self.rec.write("status", emergency=msg.emergency, latched=msg.emergency_latched,
                       reason=msg.emergency_reason, telemetry_ok=msg.telemetry_ok, stop=msg.stop, lift=msg.lift,
                       bumper=msg.bumper, moving=msg.moving, blade_on=msg.blade_on, blade_rpm=msg.blade_motor.rpm,
                       blade_off_reason=msg.blade_off_reason, battery=msg.battery_voltage)

    def _odom(self, msg):
        with self.lock:
            self.odom, self.odom_at = (msg.twist.linear.x, msg.twist.angular.z), time.monotonic()
            self.odom_log.append((self.odom_at, msg.twist.linear.x, msg.twist.angular.z))
            self.counts["odom"] += 1
            if self.guard is not None:
                self.guard.update(self.odom_at, fastest_wheel(*self.odom, self.wheel_distance))
        self.rec.write("odom", v=msg.twist.linear.x, w=msg.twist.angular.z)

    def _imu(self, msg):
        acc = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        rate = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        tilt = tilt_of(acc)
        with self.lock:
            if tilt is not None:
                self.tilts.append(tilt)
                self.tilt = statistics.median(self.tilts)
                self.max_tilt = max(self.max_tilt, self.tilt)
            self.counts["imu"] += 1
            self.acc_z.append(acc[2])
        self.rec.imu(a=list(acc), g=list(rate))

    def _capabilities(self, msg):
        with self.lock:
            self.max_linear_speed = msg.max_linear_speed
            self.firmware = {kv.key: kv.value for kv in msg.firmware}
            self.capability_wheel_distance = msg.wheel_distance
        self.rec.write("capabilities", firmware={kv.key: kv.value for kv in msg.firmware},
                       max_linear_speed=msg.max_linear_speed, max_angular_speed=msg.max_angular_speed)

    def travelled(self, since):
        """Metres the odometry went since `since` (monotonic), forward and back counted alike."""
        with self.lock:
            log = [entry for entry in self.odom_log if entry[0] >= since]
        return sum(abs(b[1]) * (b[0] - a[0]) for a, b in zip(log, log[1:]))

    def odometry(self):
        with self.lock:
            return list(self.odom_log)

    def reset_seen(self):
        """Starts a new count of the messages, inputs and a_z; returns the status before it."""
        with self.lock:
            self.counts.clear()
            self.seen.clear()
            self.acc_z.clear()
            return self.status

    def reset_tilt(self):
        """Starts a new largest tilt and a new first lift."""
        with self.lock:
            self.max_tilt = 0.0
            self.first_lift = None

    def set_guard(self, guard):
        with self.lock:
            self.guard = guard

    def tripped(self):
        with self.lock:
            return self.guard.tripped if self.guard is not None else None

    def snapshot(self):
        with self.lock:
            return self.status, time.monotonic() - self.status_at, self.odom, time.monotonic() - self.odom_at

    def wheel_speeds(self, v, w):
        return v - w * self.wheel_distance / 2, v + w * self.wheel_distance / 2

    def clamp(self, v, w):
        """Scales a command so that neither wheel exceeds the limit."""
        top = max(abs(s) for s in self.wheel_speeds(v, w))
        if top > self.wheel_limit:
            v, w = v * self.wheel_limit / top, w * self.wheel_limit / top
        return v, w

    def drive(self, v, w):
        if not self.stop_checked:
            raise Abort("a wheel command before the STOP check")
        v, w = self.clamp(v, w)
        self.cmd_vel.publish(Twist(linear=Vector3(x=v), angular=Vector3(z=w)))
        self.last_cmd_at = time.monotonic()
        self.rec.write("cmd", v=v, w=w)

    def stop_wheels(self):
        for _ in range(3):
            self.cmd_vel.publish(Twist())
            self.rec.write("cmd", v=0.0, w=0.0)
            time.sleep(0.1)

    def call(self, client, request, timeout=3.0):
        if not client.service_is_ready() and not client.wait_for_service(timeout):
            return None
        future = client.call_async(request)
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(0.01)
        return future.result() if future.done() else None

    def latch(self, why):
        answer = self.call(self.emergency, SetEmergency.Request(latch=True, bits=0, reason=why))
        self.rec.write("latch", why=why, success=bool(answer and answer.success))
        return answer

    def release(self):
        answer = self.call(self.emergency, SetEmergency.Request(latch=False, reason="measurement"))
        refusal = {}
        if answer is not None and answer.code:
            refusal = {"code": answer.code, "value": answer.value if math.isfinite(answer.value) else None,
                       "limit": answer.limit if math.isfinite(answer.limit) else None}
        self.rec.write("release", success=bool(answer and answer.success), message=answer.message if answer else "",
                       **refusal)
        return answer

    def set_blade(self, on, lease_s=2.0, percent=0, height_mm=0):
        """percent and height_mm 0: the driver's defaults."""
        if on and not self.stop_checked:
            raise Abort("blade on before the STOP check")
        answer = self.call(self.blade, SetBlade.Request(on=on, speed_percent=percent, height_mm=height_mm,
                                                        lease_s=lease_s if on else 0.0))
        self.rec.write("blade", on=on, lease_s=lease_s, percent=percent, height_mm=height_mm,
                       success=bool(answer and answer.success), message=answer.message if answer else "")
        return answer

    def silence(self, seconds, blade=False):
        answer = self.call(self.silence_client, SilenceDriver.Request(seconds=seconds, blade=blade))
        self.rec.write("silence", seconds=seconds, blade=blade, success=bool(answer and answer.success),
                       code=answer.code if answer else "")
        return answer

    def close(self):
        self.running = False
        self.spinner.join(2.0)
        self.node.destroy_node()


class Person:
    """What the person does; in a rehearsal the same goes to fake_xcore's flags."""

    def __init__(self, keys, flags_path, assume_yes):
        self.keys = keys
        self.flags_path = flags_path
        self.assume_yes = assume_yes or bool(flags_path)
        # The flags of the rehearsal that are not the person's (e.g. host_timeout of fake_tron) stay as they are.
        self.flags = {}
        if flags_path and os.path.exists(flags_path):
            with open(flags_path) as f:
                self.flags = json.load(f)
        self.timers = []

    @property
    def fake(self):
        return self.flags_path is not None

    def write_flags(self):
        with open(self.flags_path + ".tmp", "w") as f:
            json.dump(self.flags, f)
        os.replace(self.flags_path + ".tmp", self.flags_path)

    def set(self, flags):
        self.flags.update(flags)
        self.write_flags()

    def clear(self, flags):
        for key in flags:
            self.flags.pop(key, None)
        self.write_flags()

    def ask(self, text):
        """y runs, s skips, q quits."""
        print(f"\n{text}\n  [y] run  [s] skip  [q] quit: ", end="", flush=True)
        if self.assume_yes:
            print("y")
            return True
        while True:
            key = self.keys.read_key().lower()
            if key in ("y", "s", "q"):
                print(key)
                if key == "q":
                    raise Quit()
                self.keys.settle()
                return key == "y"

    def act(self, text, fake=None, hold=0.0):
        """The person does something with the wheels standing; Enter when done."""
        print(f"  -> {text}")
        if self.fake:
            if fake is None:
                print("     (not in the rehearsal)")
                return False
            self.set(fake)
            time.sleep(hold)
            self.clear(fake)
            return True
        print("     Enter when done, q to abort: ", end="", flush=True)
        key = self.keys.read_key()
        print()
        if key.lower() == "q":
            raise Abort()
        self.keys.settle()
        return True

    def act_later(self, text, fake=None, after=1.0, hold=1.0, lead=None):
        """The person acts while a step runs; in a rehearsal the flags go up after `after` s for `hold` s, those of
        `lead` LEAD_S before them."""
        print(f"  -> {text}")
        if not self.fake or fake is None:
            return

        def run():
            time.sleep(after)
            if lead:
                self.set(lead)
                time.sleep(LEAD_S)
            self.set(fake)
            time.sleep(hold)
            self.clear({**(lead or {}), **fake})
        timer = threading.Thread(target=run, daemon=True)
        timer.start()
        self.timers.append(timer)

    def done(self):
        """Until the rehearsal's actions of act_later are over."""
        for timer in self.timers:
            timer.join()

    def number(self, text):
        if self.assume_yes:
            print(f"  {text}: (skipped)")
            return None
        value = self.keys.read_line(f"  {text} (empty skips): ").strip().replace(",", ".")
        try:
            return float(value) if value else None
        except ValueError:
            return None


def driver_pids():
    out = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                argv = f.read().split(b"\0")
        except OSError:
            continue
        if argv and os.path.basename(argv[0].decode(errors="replace")) == DRIVER:
            out.append(int(pid))
    return out


class Measurement:
    still_speed = STILL_SPEED
    # Steps that send no motor command; every other step waits for the STOP check.
    STANDSTILL_STEPS = ("M1", "M2")
    # The STOP buttons, each with its flag of the rehearsal.
    STOP_BUTTONS = (("STOP top", {"stop": True}), ("STOP rear", {"rear_stop": True}))
    # True: the board reports a pressed STOP until the release, also once it is let go.
    STOP_LATCHES = False

    def __init__(self, args, rec, mower, person, keys):
        self.args, self.rec, self.mower, self.person, self.keys = args, rec, mower, person, keys
        self.blade_used = False
        self.findings = []

    # Building blocks -------------------------------------------------------------------------------------------------

    def ask(self, text):
        if self.rec.over():
            print("\nThe recording reached its limit (size or time): no further step.")
            raise Quit()
        return self.person.ask(text)

    def mark(self, step, text):
        print(f"[{step}] {text}")
        self.rec.write("mark", step=step, text=text)

    def finding(self, step, text):
        print(f"  !! {text}")
        self.findings.append(f"{step}: {text}")
        self.rec.write("finding", step=step, text=text)

    def check_keys(self):
        if self.keys.pressed():
            raise Abort()
        tripped = self.mower.tripped()
        if tripped is not None:
            self.finding("M8", f"a wheel at {tripped[0]:.2f} m/s for {tripped[1]:.2f} s, above {FAST_ABORT_SPEED} m/s")
            self.mower.set_guard(None)
            raise Abort()

    def wait(self, seconds, until=None):
        """Waits, records, aborts on a key; True when `until` came true."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.check_keys()
            if until and until():
                return True
            time.sleep(0.02)
        return False

    def status(self):
        return self.mower.snapshot()[0]

    def healthy(self):
        s, age, _, _ = self.mower.snapshot()
        return s is not None and age < 0.5 and s.telemetry_ok and not s.emergency

    def in_emergency(self):
        s = self.status()
        return s is not None and s.emergency

    def wheels_turn(self, limit=0.02):
        _, _, (v, w), age = self.mower.snapshot()
        speeds = self.mower.wheel_speeds(v, w)
        return age < 0.5 and max(abs(x) for x in speeds) > limit

    def show(self, label):
        s = self.status()
        if s is None:
            print(f"     {label}: no hw/status")
            return
        print(f"     {label}: emergency {s.emergency}, latched {s.emergency_latched}, reason "
              f"{reason_text(s.emergency_reason)}, stop {s.stop}, lift {s.lift}, bumper {s.bumper}, "
              f"blade {s.blade_motor.rpm} rpm")

    def drive(self, v, w, seconds, until=None):
        """Commands (v, w) at 10 Hz; ends early when `until` holds. Shows what it sends."""
        cv, cw = self.mower.clamp(v, w)
        left, right = self.mower.wheel_speeds(cv, cw)
        print(f"     sending hw/cmd_vel v={cv:.3f} m/s w={cw:.3f} rad/s (wheels {left:.3f}/{right:.3f}) at 10 Hz "
              f"for up to {seconds:.1f} s")
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.check_keys()
            if until and until():
                return True
            self.mower.drive(v, w)
            time.sleep(0.1)
        return False

    def release(self, step):
        """Releases the emergency once inputs are free; a refusal is shown."""
        if not self.in_emergency():
            return True
        print("     sending hw/emergency release")
        answer = self.mower.release()
        if answer is None:
            print("     release refused: no answer")
        elif not answer.success:
            print(f"     {answer.message}" + (f" [{answer.code}]" if answer.code else ""))
        if self.wait(5.0, self.healthy):
            return True
        self.show("still in emergency")
        self.finding(step, "no release within 5 s")
        return False

    def no_motion(self, step, seconds, why):
        if self.wait(seconds, self.wheels_turn):
            self.finding(step, f"wheels turned {why}")
            raise Abort()

    def ready(self, step):
        if self.rec.over():
            raise Quit()
        if not self.wait(10.0, lambda: self.status() is not None and self.mower.snapshot()[1] < 0.5):
            self.finding(step, "no hw/status from the driver")
            raise Quit()
        if not self.healthy():
            self.show("before the step")
            self.release(step)
        # Each step starts with the wheels standing for 0.5 s, so that its runs stay apart from the last step's.
        still_since = None
        end = time.monotonic() + 5.0
        while time.monotonic() < end:
            self.check_keys()
            now = time.monotonic()
            still_since = None if self.wheels_turn(self.still_speed) else (still_since or now)
            if still_since is not None and now - still_since >= 0.5:
                return
            time.sleep(0.02)
        self.finding(step, "the wheels do not stand still before the step")
        raise Abort()

    def shown(self, name):
        """The input `name` (stop, lift) of a fresh hw/status; None without one."""
        s, age, _, _ = self.mower.snapshot()
        return bool(getattr(s, name)) if s is not None and age < 0.5 else None

    def stop_shown(self):
        return self.shown("stop")

    def stop_check(self, before):
        """Before the first step that can turn a motor: each STOP button pressed and let go once, seen in hw/status
        with its emergency, then released, after which hw/status has to show neither for STOP_FREE_S; before each press
        hw/status has to show neither. False if the person skipped it. A button hw/status did not show that way ends
        the step; the motors stay locked until a check of this run passed."""
        if self.mower.stop_checked:
            return True
        step = "STOP"
        buttons = ", ".join(button for button, _ in self.STOP_BUTTONS)
        each = ", one at a time" if len(self.STOP_BUTTONS) > 1 else ""
        if not self.ask(f"{step} check before {before}, the first step that can turn a motor: press and let go "
                        f"{buttons}{each}; sends nothing but the release"):
            print(f"     {before} skipped: no motor step without the STOP check")
            return False
        for button, fake in self.STOP_BUTTONS:
            self.ready(step)
            shown = self.stop_shown()
            if shown is not False or self.in_emergency():
                self.show("before the press")
                what = "an emergency" if shown is False else "stop" if shown else "nothing"
                self.finding(step, f"hw/status shows {what} before {button} was pressed")
                raise Abort()
            self.mark(step, button)
            asked = time.monotonic()
            self.person.act_later(f"press {button} now, hold it 2 s, let go (unlock it if it stays down)", fake)
            if not self.wait(STOP_CHECK_S, until=self.stop_shown):
                self.finding(step, f"hw/status did not show stop within {STOP_CHECK_S:.0f} s for {button}: is the "
                                   "button plugged in? No motor step")
                raise Abort()
            pressed = time.monotonic()
            if not self.wait(STOP_EMERGENCY_S, until=self.in_emergency):
                self.show("pressed")
                self.finding(step, f"no emergency within {STOP_EMERGENCY_S:.0f} s of {button}")
                raise Abort()
            s = self.status()
            print(f"     {button} seen after {pressed - asked:.1f} s, reason {reason_text(s.emergency_reason)}")
            let_go = self.let_go_of(step, button)
            held = time.monotonic() - pressed
            released = let_go is not False and self.release(step)
            free = released and not self.wait(STOP_FREE_S, until=lambda: self.stop_shown() is not False
                                              or self.in_emergency())
            if let_go is not False and not free:
                self.show("after the release")
                self.finding(step, f"stop or the emergency stays after the release: {button} stuck or held")
            self.rec.write("stop_check", button=button, pressed_after_s=pressed - asked, let_go=let_go, held_s=held,
                           latched=s.emergency_latched, reason=s.emergency_reason, released=released, free=free)
            if not free:
                raise Abort()
            print(f"     {button} released {held:.1f} s after the press, stop and the emergency stayed off")
        self.mower.stop_checked = True
        return True

    def let_go_of(self, step, button):
        """Until the person let go of a pressed button: True once hw/status shows it free, False if it does not within
        STOP_CHECK_S; on a board that keeps stop until the release None, after STOP_LET_GO_S."""
        if self.STOP_LATCHES:
            print(f"  -> let go of {button} now (unlock it if it stays down); the release follows in "
                  f"{STOP_LET_GO_S:.0f} s")
            self.wait(STOP_LET_GO_S)
            return None
        print("     waiting for it to be free")
        if self.wait(STOP_CHECK_S, until=lambda: self.stop_shown() is False):
            return True
        self.finding(step, f"hw/status still shows stop {STOP_CHECK_S:.0f} s after {button} was pressed")
        return False

    def safe_stop(self, why):
        """The abort: three stops, blade off, emergency latched."""
        print(f"\n  ABORT ({why}): sending 3 stops, blade off, latch")
        self.rec.write("abort", why=why)
        self.mower.stop_wheels()
        if self.blade_used:
            self.mower.set_blade(False)
        self.mower.latch("measurement aborted")
        self.show("after the abort")

    def signal_driver(self, step, sig):
        pids = driver_pids()
        if len(pids) != 1:
            self.finding(step, f"found {len(pids)} driver processes, expected 1")
            raise Abort()
        print(f"     sending {signal.Signals(sig).name} to {DRIVER} (pid {pids[0]})")
        self.rec.write("signal", signal=signal.Signals(sig).name, pid=pids[0])
        os.kill(pids[0], sig)
        return pids[0]

    def thaw(self, step, pid):
        """SIGCONT; a driver whose watch ends it after the freeze is waited for like a killed one."""
        os.kill(pid, signal.SIGCONT)
        self.rec.write("signal", signal="SIGCONT", pid=pid)
        if self.wait(WATCH_END_S, until=lambda: pid not in driver_pids()):
            print(f"     the watch ended {DRIVER} (pid {pid}) after the freeze")
            self.rec.write("watch_end", pid=pid)
            self.wait_for_driver(step, pid, self.args.restart_wait)

    def wait_for_driver(self, step, old_pid, seconds):
        print(f"     waiting up to {seconds:.0f} s for a new {DRIVER} (restart it if the launch does not)")
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.check_keys()
            pids = driver_pids()
            status, age, _, _ = self.mower.snapshot()
            # Back means: a new process whose board runs every service again (claims and configuration take s).
            if (pids and pids != [old_pid] and status is not None and age < 0.3 and status.telemetry_ok
                    and not status.emergency_reason & SERVICE_NOT_READY):
                return True
            time.sleep(0.1)
        self.finding(step, "the driver did not come back")
        raise Quit()

    # Steps ------------------------------------------------------------------------------------------------------------

    def m1(self):
        step = "M1"
        items = [("STOP top: press and hold 2 s, release", {"stop": True}, 2.0),
                 ("STOP rear: press and hold 2 s, release", {"rear_stop": True}, 2.0),
                 ("lift front left: lift 4 s, set down", {"lift_left": True}, 4.0),
                 ("lift front left only 1.5 s (must not stop)", {"lift_left": True}, 1.5),
                 ("lift front right: lift 4 s, set down", {"lift_right": True}, 4.0),
                 ("both lifts: lift the front 2 s, set down", {"lift": True}, 2.0),
                 ("bumper rear left: push 2 s, let go", {"bumper_left": True}, 2.0),
                 ("bumper rear right: push 2 s, let go", {"bumper_right": True}, 2.0),
                 ("both bumpers: push the rear hood 2 s, let go", {"bumper": True}, 2.0)]
        for text, fake, hold in items:
            if not self.ask(f"{step} inputs at standstill, sends nothing but a release afterwards: {text}"):
                continue
            self.ready(step)
            self.mark(step, text)
            self.person.act(text, fake, hold)
            self.wait(1.0)
            self.show("after")
            self.release(step)
        text = "STOP top held while the release is sent: the emergency must stay"
        if self.ask(f"{step} release with a pressed input: {text}"):
            self.ready(step)
            self.mark(step, text)
            if self.person.fake:
                self.person.set({"stop": True})
                self.wait(0.5)
            else:
                self.person.act("press and keep holding STOP top, then Enter")
            self.mower.release()
            self.wait(1.0)
            if not self.in_emergency():
                self.finding(step, "released while STOP was pressed")
            self.show("after the release while held")
            if self.person.fake:
                self.person.clear({"stop": True})
            else:
                self.person.act("let go of STOP top")
            self.wait(0.5)
            self.release(step)

    def m2(self):
        step = "M2"
        items = [("leave the mower still and level for 5 s", {}, 5.0),
                 ("turn the mower by hand to the left (counterclockwise seen from above) about 90 deg within 3 s",
                  {"yaw_rate": 0.5}, 3.0),
                 ("tilt the front up beyond 35 deg and hold 3 s, set down", {"pitch": -38}, 3.0),
                 ("tilt the left side up beyond 35 deg and hold 3 s, set down", {"roll": 38}, 3.0)]
        for text, fake, hold in items:
            if not self.ask(f"{step} IMU at standstill, sends nothing but a release afterwards: {text}"):
                continue
            self.ready(step)
            self.mark(step, text)
            with self.mower.lock:
                self.mower.max_tilt = 0.0
            self.person.act(text, fake, hold)
            self.wait(0.5)
            with self.mower.lock:
                print(f"     largest tilt seen: {self.mower.max_tilt:.1f} deg")
            self.release(step)

    def m3(self):
        step = "M3"
        if not self.ask(f"{step} wheels in the air (mower jacked up, wheels 2 cm free, blade disc off, battery "
                               "full, OpenMower and the updater stopped as agreed): start?"):
            return
        d = self.mower.wheel_distance
        # Long enough to see the direction (3 s at 0.05 m/s is a quarter turn of the wheel).
        items = [("forward 0.05 m/s for 8 s: do both wheels turn forward?", 0.05, 0.0, 8.0, True),
                 ("turn left: left wheel back, right wheel forward, 0.05 m/s each, 8 s", 0.0, 0.1 / d, 8.0, True)]
        items += [(f"straight at {v:.2f} m/s for 3 s (speed per duty)", v, 0.0, 3.0, False)
                  for v in (0.02, 0.03, 0.05, 0.07, 0.1)]
        for text, v, w, seconds, question in items:
            if not self.ask(f"{step} {text}"):
                continue
            self.ready(step)
            self.mark(step, text)
            self.drive(v, w, seconds, until=self.in_emergency)
            self.mower.stop_wheels()
            self.wait(1.0)
            if question and not self.person.fake and not self.ask(f"{step} did the wheels turn as said?"):
                self.finding(step, f"wheels not as commanded: {text}")
        text = "0.05 m/s for 20 s: count the turns of the left wheel"
        if self.ask(f"{step} {text}"):
            self.ready(step)
            self.mark(step, text)
            self.drive(0.05, 0.0, 20.0, until=self.in_emergency)
            self.mower.stop_wheels()
            turns = self.person.number("turns of the left wheel")
            circumference = self.person.number("circumference of the wheel in m")
            self.rec.write("wheel_turns", turns=turns, circumference=circumference)

    def m4(self):
        step = "M4"
        modes = [("frozen 3 s (SIGSTOP, then SIGCONT)", "freeze"), ("killed (SIGKILL)", "kill"),
                 ("inner network down (typed by the person on the CM4)", "link"),
                 ("ended normally (SIGINT)", "sigint")]
        for text, mode in modes:
            for n in range(self.args.repeats):
                if not self.ask(f"{step} failsafe without host, run {n + 1}/{self.args.repeats}: wheels at "
                                       f"0.1 m/s, then the driver {text}; no command afterwards"):
                    break
                self.ready(step)
                self.mark(step, f"{text}, run {n + 1}")
                if mode == "link":
                    self.link_down(step)
                    continue
                self.drive(0.1, 0.0, 2.0, until=self.in_emergency)
                if mode == "freeze":
                    pid = self.signal_driver(step, signal.SIGSTOP)
                    try:
                        self.wait(3.0)
                    except BaseException:
                        os.kill(pid, signal.SIGCONT)
                        self.rec.write("signal", signal="SIGCONT", pid=pid)
                        raise
                    self.thaw(step, pid)
                    self.wait(1.0)
                elif mode in ("kill", "sigint"):
                    pid = self.signal_driver(step, signal.SIGKILL if mode == "kill" else signal.SIGINT)
                    self.wait(3.0)
                    self.wait_for_driver(step, pid, self.args.restart_wait)
                self.no_motion(step, 3.0, "after the driver came back without a new command")
                self.show("after")
                self.release(step)
                self.no_motion(step, 2.0, "after the release without a new command")

    def link_down(self, step):
        """M4 with the inner network down: the commands go on until the driver loses the board, so the board's own
        timeout is what stops the wheels; then the link comes back without a new command."""
        print("  -> in a second terminal on the CM4, while the wheels turn: "
              "sudo ip link set eth0 down; sleep 5; sudo ip link set eth0 up")
        print("     (the wheels turn at 0.1 m/s until the driver reports the loss, at most 120 s)")
        if self.person.fake:
            self.person.act_later("inner network down", {"link_down": True}, after=2.0, hold=5.0)
        started = time.monotonic()
        if not self.drive(0.1, 0.0, 120.0, until=lambda: not self.healthy() and time.monotonic() - started > 0.5):
            self.finding(step, "the driver reported no loss of the board within 120 s")
            self.mower.stop_wheels()
            return
        self.rec.write("link_lost", after_s=time.monotonic() - started)
        self.show("board lost")
        print("     no more commands; waiting for the board to come back (eth0 up)")
        if not self.wait(60.0, lambda: (s := self.status()) is not None and s.telemetry_ok
                         and not s.emergency_reason & SERVICE_NOT_READY):
            self.finding(step, "the board did not come back within 60 s")
            raise Quit()
        self.no_motion(step, 3.0, "after the link came back without a new command")
        self.show("after")
        self.release(step)
        self.no_motion(step, 2.0, "after the release without a new command")

    def m5(self):
        step = "M5"
        items = [("press STOP top while the wheels turn", {"stop": True}, 0.5),
                 ("lift the front left while the wheels turn (the board waits 2.5 s)", {"lift_left": True}, 3.5),
                 ("lift the whole front while the wheels turn", {"lift": True}, 1.0),
                 ("push the rear hood (both bumpers) while the wheels turn", {"bumper": True}, 1.0),
                 ("push bumper rear left while the wheels turn", {"bumper_left": True}, 1.0),
                 ("nothing: the script latches over hw/emergency (the way of the safety layer)", None, 0.0)]
        for text, fake, hold in items:
            for n in range(self.args.repeats):
                if not self.ask(f"{step} reason while turning, run {n + 1}/{self.args.repeats}: wheels at "
                                       f"0.1 m/s for up to 10 s; {text}; stop on the emergency"):
                    break
                self.ready(step)
                self.mark(step, f"{text}, run {n + 1}")
                if fake is None:
                    threading.Timer(1.0, self.mower.latch, args=("measurement M5",)).start()
                else:
                    self.person.act_later(text, fake, after=1.0, hold=hold)
                if not self.drive(0.1, 0.0, 10.0, until=self.in_emergency):
                    self.finding(step, f"no emergency: {text}")
                    self.mower.stop_wheels()
                self.show("emergency")
                self.wait(max(hold, 0.5) + 1.0)
                if not self.person.fake and fake is not None:
                    self.person.act("let go of the input")
                self.no_motion(step, 2.0, "with the input free and no new command")
                self.release(step)
                self.no_motion(step, 2.0, "after the release without a new command")
                self.drive(0.05, 0.0, 1.0, until=self.in_emergency)
                self.mower.stop_wheels()

    def m6(self):
        step = "M6"
        if self.ask(f"{step} the command stops: 0.1 m/s for 2 s, then nothing (the driver sends 3 stops after "
                           "0.5 s)"):
            self.ready(step)
            self.mark(step, "command stops")
            self.drive(0.1, 0.0, 2.0, until=self.in_emergency)
            self.wait(2.0)
            self.no_motion(step, 1.0, "after the driver's stop")
        if self.ask(f"{step} a wheel turned by hand after a command: 0.1 m/s for 2 s, nothing, after 1.5 s turn "
                           "the left wheel by hand quickly for 1 s; the driver must latch"):
            self.ready(step)
            self.mark(step, "wheel by hand after a command")
            self.drive(0.1, 0.0, 2.0, until=self.in_emergency)
            self.person.act_later("turn the left wheel by hand quickly for 1 s", {"wheel_speed": 0.2}, 1.5, 1.0)
            if not self.wait(4.0, self.in_emergency):
                self.finding(step, "no latch for a wheel turning without a command")
            self.show("after")
            self.wait(1.0)
            self.release(step)
        if self.ask(f"{step} tilt beyond 40 deg for 1 s at standstill; the driver must latch TILT"):
            self.ready(step)
            self.mark(step, "tilt beyond 40 deg")
            self.person.act("tilt the left side up beyond 45 deg for 1 s, set down", {"roll": 45}, 1.2)
            self.wait(0.5)
            s = self.status()
            if s is None or not s.emergency_reason & 8192:
                self.finding(step, "no TILT latch")
            self.show("after")
            self.release(step)

    def m7(self):
        step = "M7"
        if not self.args.blade:
            print(f"\n{step} needs --blade (own word of the user, blade disc off first).")
            return
        self.blade_used = True
        items = [("blade on for 5 s with a renewed lease, then off", "off", None),
                 ("blade on for 5 s, then the lease runs out", "lease", None),
                 ("blade on, press STOP top", "input", {"stop": True}),
                 ("blade on, lift the front left (the board waits 2.5 s)", "input", {"lift_left": True}),
                 ("blade on, push the rear hood", "input", {"bumper": True}),
                 ("blade on, driver frozen 3 s", "freeze", None),
                 ("blade on, driver killed", "kill", None)]
        for text, how, fake in items:
            if not self.ask(f"{step} blade (hw/blade on, lease 2 s renewed every 0.5 s): {text}"):
                continue
            self.ready(step)
            self.mark(step, text)
            print("     sending hw/blade on, lease 2.0 s, every 0.5 s")
            end = time.monotonic() + 5.0
            if how == "input":
                self.person.act_later(text, fake, after=3.0, hold=3.5)
            while time.monotonic() < end and not self.in_emergency():
                self.check_keys()
                self.mower.set_blade(True, 2.0)
                self.wait(0.5)
            if how == "off":
                print("     sending hw/blade off")
                self.mower.set_blade(False)
            elif how == "freeze":
                pid = self.signal_driver(step, signal.SIGSTOP)
                try:
                    self.wait(3.0)
                except BaseException:
                    os.kill(pid, signal.SIGCONT)
                    self.rec.write("signal", signal="SIGCONT", pid=pid)
                    raise
                self.thaw(step, pid)
            elif how == "kill":
                pid = self.signal_driver(step, signal.SIGKILL)
                self.wait(3.0)
                self.wait_for_driver(step, pid, self.args.restart_wait)
            self.wait(6.0)
            self.show("after")
            if how == "input" and not self.person.fake:
                self.person.act("let go of the input; STOP top locks: unlock it by hand")
            self.release(step)
            self.wait(1.0)
            s = self.status()
            if s is not None and abs(s.blade_motor.rpm) >= 100:
                self.finding(step, f"blade still at {s.blade_motor.rpm} rpm: {text}")

    def m8(self):
        step = "M8"
        if not self.args.ground:
            print(f"\n{step} needs --ground (own word of the user, on the ground, blade disc off).")
            return
        limit = self.mower.max_linear_speed
        raised = limit is not None and limit > GROUND_WHEEL_SPEED + 1e-6
        items = [("0.3 m/s straight, press STOP top after about 2 s", 0.3, 0.0, {"stop": True}, None),
                 ("0.3 m/s straight for 3 s, then no command (driver stops)", 0.3, 0.0, None, "quiet"),
                 ("0.3 m/s straight for 2 s, then the driver killed", 0.3, 0.0, None, "kill"),
                 ("0.1 m/s backward against a soft obstacle for up to 10 s", -0.1, 0.0, {"bumper": True}, None),
                 ("turn on the spot at 0.3 rad/s for 5 s", 0.0, 0.3, None, "quiet")]
        if raised:
            # Its max_duty may be raised as well: 0.3 m/s could then turn the wheels faster than its word says.
            print(f"\n{step} at 0.3 m/s: skipped, the driver runs in the raised start ({limit:.2f} m/s); run it in "
                  "the normal start.")
        else:
            for text, v, w, fake, how in items:
                if self.ask(f"{step} on the ground: {text}"):
                    self.ground_run(step, text, v, w, fake, how)
        if not self.args.fast:
            return
        if limit is None or limit < FAST_FIRST_SPEED - 1e-6:
            shown = "no" if limit is None else f"{limit:.2f}"
            print(f"\n{step} fast runs: the driver allows {shown} m/s; they need its limits raised to 0.4 m/s or more "
                  "(own word of the user) and are skipped.")
            return
        top = min(FAST_WHEEL_SPEED, limit)
        self.mower.wheel_limit = top
        self.mower.set_guard(SpeedGuard(FAST_ABORT_SPEED, FAST_ABORT_S))
        try:
            self.fast_runs(step, top)
        finally:
            self.mower.set_guard(None)
            self.mower.wheel_limit = GROUND_WHEEL_SPEED

    def fast_runs(self, step, top):
        """Speed per duty at high duty and the stopping distance at 0.4-0.5 m/s (own word, raised start), in stages."""
        n = max(1, min(self.args.repeats, FAST_REPEATS))
        print(f"\n{step} fast runs, {n}x each, {FAST_FIRST_SPEED} m/s first, then up to {top:.2f} m/s: a straight "
              f"lane of 5 m or more, nobody in it; a wheel above {FAST_ABORT_SPEED} m/s for {FAST_ABORT_S} s aborts")
        first = self.fast_case(step, f"{FAST_FIRST_SPEED:.2f} m/s straight, press STOP top after about 2 s",
                               FAST_FIRST_SPEED, "stop", n)
        steady = [r["steady"] for r in first if r["steady"] is not None]
        first_steady = max(steady) if steady else None
        if not top_speed_allowed(first_steady, top):
            shown = "unknown" if first_steady is None else f"{first_steady:.2f} m/s"
            print(f"\n{step} at {FAST_FIRST_SPEED:.2f} m/s the faster wheel went {shown}: {top:.2f} m/s would go "
                  f"above {FAST_ABORT_SPEED} m/s, the top speed is skipped.")
            self.rec.write("fast_top_skipped", step=step, first_steady=first_steady, top=top)
            return
        for text, how in ((f"{top:.2f} m/s straight, press STOP top after about 2 s", "stop"),
                          (f"{top:.2f} m/s straight for 3 s, then no command (driver stops)", "quiet"),
                          (f"{top:.2f} m/s straight for 2 s, then the driver killed", "kill")):
            self.fast_case(step, text, top, how, n)

    def fast_case(self, step, text, v, how, n):
        """Up to n runs of one fast case, each checked against its limits, then their spread; the results of the runs."""
        results = []
        for i in range(n):
            label = f"fast {text}" + (f", run {i + 1} of {n}" if n > 1 else "")
            if not self.ask(f"{step} on the ground, {label}"):
                continue
            results.append(self.ground_run(step, label, v, 0.0, {"stop": True} if how == "stop" else None,
                                           None if how == "stop" else how, fast=True))
        problem = spread_problem([r["metres"] for r in results if r.get("metres") is not None])
        if problem:
            self.finding(step, f"fast {text}: {problem}")
            raise Abort()
        return results

    def ground_run(self, step, text, v, w, fake, how, fast=False):
        """One run of M8: drive, the cause, the distance until the wheels stood (odometry), then the release. A fast
        run beyond its limits aborts, before the release."""
        self.ready(step)
        self.mark(step, text)
        if fake is not None:
            self.person.act_later(text, fake, after=2.0, hold=1.0)
            started = time.monotonic()
            if not self.drive(v, w, 10.0, until=self.in_emergency):
                self.finding(step, f"no emergency: {text}")
                self.mower.stop_wheels()
            # The script sees the emergency up to 0.1 s late; the distance counts from its first status.
            since = self.mower.emergency_since
            cause = since if since is not None and since >= started else time.monotonic()
        elif how == "kill":
            self.drive(v, w, 2.0, until=self.in_emergency)
            cause = time.monotonic()
            pid = self.signal_driver(step, signal.SIGKILL)
            self.wait(3.0)
            self.wait_for_driver(step, pid, self.args.restart_wait)
        else:
            self.drive(v, w, 3.0 if w == 0 else 5.0, until=self.in_emergency)
            cause = self.mower.last_cmd_at or time.monotonic()
        self.wait(2.0)
        log = self.mower.odometry()
        result = {"steady": steady_speed(log, cause - 0.6, cause - 0.1, self.mower.wheel_distance)}
        if w == 0:
            # A killed driver sees nothing until its successor runs; the counters in the capture tell that distance.
            if how != "kill":
                still = still_after(log, cause, self.mower.wheel_distance)
                result.update(metres=self.mower.travelled(cause), stand_s=None if still is None else still - cause)
                stood = "never" if still is None else f"after {still - cause:.2f} s"
                steady = "-" if result["steady"] is None else f"{result['steady']:.2f}"
                print(f"     odometry (the driver's view): {result['metres']:.3f} m from the "
                      f"{'emergency' if fake else 'last command'} until it stood ({stood}); before: {steady} m/s")
                self.rec.write("stopping", step=step, text=text, odometry_m=result["metres"],
                               stand_s=result["stand_s"], steady=result["steady"])
            chalk = self.person.number("stopping distance at the chalk line in m")
            if chalk is not None:
                self.rec.write("stopping_chalk", step=step, text=text, metres=chalk)
        self.no_motion(step, 1.0, "after the stop")
        if fast and how != "kill":
            problems = fast_run_problems("stop" if fake else how, result["stand_s"], result["metres"])
            for problem in problems:
                self.finding(step, f"{text}: {problem}")
            if problems:
                raise Abort()
        if not self.person.fake and fake is not None:
            self.person.act("let go; STOP top locks: unlock it by hand" if fake.get("stop") else "let go")
        self.release(step)
        return result

    STEPS = {"M1": m1, "M2": m2, "M3": m3, "M4": m4, "M5": m5, "M6": m6, "M7": m7, "M8": m8}
    # A limit above GROUND_WHEEL_SPEED means the raised start of the measurement image, only for M8 --fast.
    RAISED_START_LIMITED = True

    def capabilities(self, seconds=10.0):
        """The driver's limit for driving from hw/capabilities, waiting for them up to `seconds`; None without them."""
        end = time.monotonic() + seconds
        while self.mower.max_linear_speed is None and time.monotonic() < end:
            time.sleep(0.05)
        return self.mower.max_linear_speed

    def run(self, steps):
        limit = self.capabilities()
        if limit is None:
            self.finding("start", "no hw/capabilities from the driver within 10 s")
            return False
        raised = limit > GROUND_WHEEL_SPEED + 1e-6 and not (steps == ["M8"] and self.args.fast)
        if self.RAISED_START_LIMITED and raised:
            print(f"\nThe driver allows {limit:.2f} m/s: the raised start is only for --steps M8 --ground --fast. Start "
                  "it again without speed_limit and max_duty for any other step.")
            self.findings.append(f"start: raised start ({limit:.2f} m/s) for {','.join(steps)}")
            return False
        for name in steps:
            self.mower.wheel_limit = GROUND_WHEEL_SPEED if name == "M8" else AIR_WHEEL_SPEED
            print(f"\n== {name}, wheels at most {self.mower.wheel_limit} m/s ==")
            try:
                if name in self.STANDSTILL_STEPS or self.stop_check(name):
                    self.STEPS[name](self)
            except Abort as abort:
                if abort.args:
                    self.finding(name, abort.args[0])
                self.safe_stop("key or failed check")
                if self.person.assume_yes or not self.person.ask("continue with the next step?"):
                    return False
            except Quit:
                return True
        return True


# The Tron ----------------------------------------------------------------------------------------------------------


class PortRefused(Exception):
    pass


def port_holders(path):
    """Pids other than this one that have the device behind `path` open (all of them when run as root)."""
    device = os.path.realpath(path)
    pids = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            fds = os.listdir(f"/proc/{pid}/fd")
        except OSError:
            continue
        for fd in fds:
            try:
                if os.readlink(f"/proc/{pid}/fd/{fd}") == device:
                    pids.append(int(pid))
                    break
            except OSError:
                continue
    return sorted(pids)


def process_state(pid):
    """The state letter of a process (R, S, T, ...); None once it is gone."""
    try:
        with open(f"/proc/{pid}/stat") as f:
            return f.read().rsplit(")", 1)[1].split()[0]
    except (OSError, IndexError):
        return None


class PortReader:
    """Reads a serial port without writing to it or changing its settings: one line "<monotonic s> <hex>" per chunk
    into a capture file of at most max_bytes; `full` once a chunk found no room, the capture ends there. Refuses a port
    whose settings could answer the other side."""

    def __init__(self, path, capture, max_bytes=CAPTURE_MAX_BYTES):
        self.fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC)
        try:
            problem = termios_problem(termios.tcgetattr(self.fd))
            if problem:
                raise PortRefused(f"{path}: {problem}; not read")
            self.file = open(capture, "x")
        except BaseException:
            os.close(self.fd)
            raise
        self.left = max_bytes
        self.full = False
        self.running = True
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        while self.running:
            if not select.select([self.fd], [], [], 0.02)[0]:
                continue
            try:
                data = os.read(self.fd, 4096)
            except BlockingIOError:
                continue
            except OSError:
                return
            line = f"{time.monotonic():.6f} {data.hex()}\n"
            if len(line) > self.left:
                self.full = True
                return
            self.left -= len(line)
            self.file.write(line)

    def close(self):
        self.running = False
        self.thread.join(1.0)
        os.close(self.fd)
        self.file.close()


def capped_copy(stream, path, max_bytes):
    """Copies a process's output into path until max_bytes, then drops the rest."""
    with open(path, "ab") as f:
        for line in stream:
            if len(line) <= max_bytes:
                max_bytes -= len(line)
                f.write(line)
                f.flush()


class TronStart:
    """The minimal start of the Tron: tron_imu and tron_mcu as processes of this programme, nothing else (no GNSS,
    logic, bridge or MQTT). Parameters: stack.yaml and the file of the Tron, then its measured values in data_dir,
    then the overrides of this start (only here, e.g. the long cmd_vel_timeout of heartbeat alone). The output of each
    process goes to a file next to the recording, at most NODE_LOG_BYTES."""

    def __init__(self, args, rec):
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        share = get_package_share_directory("mowbite_bringup")
        sys.path.insert(0, os.path.join(share, "launch"))
        import data_store
        import layers
        self.files = layers.files("tron")
        mower = layers.device_file(args.data_dir, "tron")
        if os.path.isfile(mower):
            self.files.append(mower)
        self.common = {"hold_file": os.path.join(args.data_dir, data_store.HOLD_FILE), "allow_blade": False}
        if args.journal_dir:
            self.common["journal_dir"] = args.journal_dir
        self.ports = {"tron_mcu": args.mcu_port or self.param("tron_mcu", "port"),
                      "tron_imu": args.imu_port or self.param("tron_imu", "port")}
        self.executables = {node: os.path.join(get_package_prefix(node), "lib", node, f"{node}_node")
                            for node in self.ports}
        self.capture_tool = os.path.join(get_package_prefix("tron_mcu"), "lib", "tron_mcu", "tron_mcu_capture")
        self.user = pwd.getpwnam(args.node_user) if args.node_user else None
        self.rec = rec
        self.log_stem = rec.path[:-len(".jsonl")]
        self.processes = {}
        self.overrides = {}
        self.started = collections.Counter()

    @property
    def mcu_port(self):
        return self.ports["tron_mcu"]

    def param(self, node, name):
        """A parameter of a node as the files give it (the last file wins, its own section over /**)."""
        value = None
        for path in self.files:
            with open(path) as f:
                content = yaml.safe_load(f) or {}
            for section in ("/**", node):
                value = content.get(section, {}).get("ros__parameters", {}).get(name, value)
        return value

    def watchdog(self):
        """s without hw/cmd_vel until tron_mcu sends its stops, with the overrides of the running start."""
        def value(name):
            return float(self.overrides.get(name, self.param("tron_mcu", name) or 0.0))
        timeout, window = value("cmd_vel_timeout"), value("speed_repeat_window")
        return min(window, timeout) if window > 0 else timeout

    def imu_start(self):
        """s tron_mcu gives its IMU for the first sample after the start."""
        return max(float(self.param("tron_mcu", name)) for name in ("imu_timeout", "imu_start_timeout"))

    def start(self, node, **overrides):
        params = {**self.common, "port": self.ports[node], **overrides}
        # In a file of its own: a section of the node in a file wins over -p.
        self.started[node] += 1
        override = f"{self.log_stem}-{node}-{self.started[node]}.yaml"
        with open(override, "w") as f:
            yaml.safe_dump({node: {"ros__parameters": params}}, f)
        argv = [self.executables[node], "--ros-args"]
        for path in self.files + [override]:
            argv += ["--params-file", path]
        user = {}
        if self.user is not None:
            user = {"user": self.user.pw_uid, "group": self.user.pw_gid,
                    "extra_groups": os.getgrouplist(self.user.pw_name, self.user.pw_gid)}
        process = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   start_new_session=True, **user)
        threading.Thread(target=capped_copy, args=(process.stdout, f"{self.log_stem}-{node}.log", NODE_LOG_BYTES),
                         daemon=True).start()
        self.processes[node] = process
        self.rec.write("process", node=node, pid=process.pid, params=params)
        print(f"     started {node} (pid {process.pid})" + (f" with {overrides}" if overrides else ""))
        return process

    def start_driver(self, **overrides):
        """tron_mcu with the overrides of this start; a start after a kill keeps those of the last one."""
        self.overrides = overrides
        return self.start("tron_mcu", **overrides)

    @property
    def driver_pid(self):
        process = self.processes.get("tron_mcu")
        return process.pid if process is not None and process.poll() is None else None

    def signal_driver(self, sig):
        pid = self.driver_pid
        os.kill(pid, sig)
        self.rec.write("signal", signal=signal.Signals(sig).name, pid=pid)
        return pid

    def stop(self, node, timeout=5.0):
        """SIGINT (tron_mcu stops moving wheels and the blade on the way out), SIGKILL after the timeout."""
        process = self.processes.pop(node, None)
        if process is None or process.poll() is not None:
            return
        os.kill(process.pid, signal.SIGCONT)
        os.kill(process.pid, signal.SIGINT)
        try:
            process.wait(timeout)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        self.rec.write("process_end", node=node, pid=process.pid, code=process.returncode)

    def restart_driver(self, **overrides):
        self.stop("tron_mcu")
        return self.start_driver(**overrides)

    def decode(self, capture):
        """The decoded capture (tron_mcu_capture) next to it; its lines and the summary."""
        with open(capture + ".jsonl", "w") as out:
            done = subprocess.run([self.capture_tool, capture], stdout=out, stderr=subprocess.PIPE, text=True)
        with open(capture + ".jsonl") as f:
            lines = [json.loads(line) for line in f if line.strip()]
        summary = lines.pop()["summary"] if lines and "summary" in lines[-1] else {}
        return lines, summary, done.returncode

    def close(self):
        self.stop("tron_mcu")
        self.stop("tron_imu")


class TronMeasurement(Measurement):
    """T1-T7 of phase 6 on the Tron, in the air: the driver is this programme's process (TronStart)."""
    still_speed = TRON_STILL_SPEED
    STANDSTILL_STEPS = ("T1", "T8")
    STOP_BUTTONS = (("STOP", {"stop": True}),)
    STOP_LATCHES = True
    # tron_mcu reports the mower's own maximum; the T steps keep to AIR_WHEEL_SPEED (run).
    RAISED_START_LIMITED = False

    def __init__(self, args, rec, mower, person, keys, start):
        super().__init__(args, rec, mower, person, keys)
        self.start = start
        self.turn_speed = None
        # Stand times of the host-gone runs by way (freeze, kill), the firmware the driver reported.
        self.host_gone = {}
        self.firmware = {}
        # T6 and T7, from the parameter files of the start: a blade motor below blade_still_rpm stands, within
        # blade_stop_s of going off; the lease and its renewal are those of mowing.
        self.blade_still_rpm = self.blade_stop_s = self.blade_lease_s = self.blade_renew_s = None
        self.blade_renewed = None

    @property
    def speed(self):
        return self.turn_speed or TRON_FIRST_SPEED

    def safe_stop(self, why):
        super().safe_stop(why)
        print("  If a wheel still turns: press STOP, or take the battery out.")

    def capture_path(self, label):
        return f"{self.start.log_stem}-{label}.capture"

    def stand_after(self, since, seconds=EMERGENCY_STAND_S):
        """s from `since` (monotonic) until the wheels stood, waiting up to `seconds` for it; None if they did not."""
        self.wait(seconds, until=lambda: not self.wheels_turn(self.still_speed))
        self.wait(0.2)
        still = still_after(self.mower.odometry(), since, self.mower.wheel_distance, self.still_speed)
        return None if still is None else still - since

    def wait_driver_back(self, step, old_pid):
        """Until the new tron_mcu reports, then through the time it gives its IMU: before the first sample it refuses
        motion and blade without an emergency in hw/status, after that time it reports one."""
        end = time.monotonic() + self.args.restart_wait
        while time.monotonic() < end:
            self.check_keys()
            status, age, _, _ = self.mower.snapshot()
            pid = self.start.driver_pid
            if pid is not None and pid != old_pid and status is not None and age < 0.3 and status.telemetry_ok:
                self.wait(self.start.imu_start())
                return
            time.sleep(0.1)
        self.finding(step, "tron_mcu did not come back")
        raise Quit()

    def emergency_run(self, step, text, fake, hold):
        """Wheels at the turning speed until the person's input brings the emergency; the wheels have to stand within
        EMERGENCY_STAND_S of it. Returns that time."""
        self.person.act_later(text, fake, after=1.0, hold=hold)
        started = time.monotonic()
        if not self.drive(self.speed, 0.0, 10.0, until=self.in_emergency):
            self.mower.stop_wheels()
            self.finding(step, f"no emergency within 10 s: {text}")
            raise Abort()
        since = self.mower.emergency_since
        since = since if since is not None and since >= started else time.monotonic()
        stand = self.stand_after(since)
        self.rec.write("emergency_stop", step=step, text=text, stand_s=stand)
        if stand is None:
            self.finding(step, f"the wheels did not stand within {EMERGENCY_STAND_S:.1f} s of the emergency: {text}")
            raise Abort()
        print(f"     the wheels stood {stand:.2f} s after the emergency")
        self.show("emergency")
        return stand

    def let_go(self, text, then=""):
        if self.person.fake:
            self.person.done()
        else:
            self.person.act(f"let go ({text}); unlock STOP if it stays down" + (f". {then}" if then else ""))

    # Steps ------------------------------------------------------------------------------------------------------------

    def t1(self):
        step = "T1"
        if self.ask(f"{step} the driver at standstill, no wheel command: 10 s of telemetry (rates, checksum errors, "
                    "flags, IMU)"):
            self.ready(step)
            self.mark(step, "telemetry at standstill")
            before = self.mower.reset_seen()
            self.wait(10.0)
            after = self.status()
            with self.mower.lock:
                counts = dict(self.mower.counts)
                seen = set(self.mower.seen)
                acc_z = list(self.mower.acc_z)
            frames = (after.link_messages_ok - before.link_messages_ok) / 10.0
            errors = after.link_errors - before.link_errors
            a_z = sum(acc_z) / len(acc_z) if acc_z else None
            rates = {topic: counts.get(topic, 0) / 10.0 for topic in ("status", "odom", "imu")}
            self.rec.write("telemetry", frames_per_s=frames, link_errors=errors, rates=rates, a_z=a_z,
                           seen=sorted(seen), link_total=after.link_messages_ok, errors_total=after.link_errors)
            shown = "-" if a_z is None else f"{a_z:.2f}"
            print(f"     MCU frames {frames:.0f}/s, checksum and length errors {errors}; hw/wheel_odom "
                  f"{rates['odom']:.0f}/s, imu/data {rates['imu']:.0f}/s, a_z {shown} m/s2")
            if errors:
                self.finding(step, f"{errors} link errors in 10 s at standstill")
            if a_z is None or not 8.8 <= a_z <= 10.8:
                self.finding(step, f"a_z at standstill {shown} m/s2, expected about +9.81 (mount of the IMU)")
            if rates["imu"] < 20 or rates["odom"] < 20:
                self.finding(step, "imu/data or hw/wheel_odom below 20/s")
            if seen:
                self.finding(step, f"inputs at standstill without a hand on them: {', '.join(sorted(seen))}")
        items = [("STOP: press, hold 2 s, let go", {"stop": True}, 2.0, "stop"),
                 ("lift: lift the front 4 s, set it down", {"lift": True}, 4.0, "lift"),
                 ("bumper left: push 2 s, let go", {"bumper_left": True}, 2.0, "bumper_left"),
                 ("bumper right: push 2 s, let go", {"bumper_right": True}, 2.0, "bumper_right")]
        for text, fake, hold, flag in items:
            if not self.ask(f"{step} input at standstill, sends nothing but the release (clear frame) afterwards: "
                            f"{text}"):
                continue
            self.ready(step)
            self.mark(step, text)
            self.mower.reset_seen()
            self.person.act(f"{text}; unlock STOP if it stays down", fake, hold)
            self.wait(1.0)
            self.show("after")
            if flag not in self.mower.seen:
                self.finding(step, f"hw/status never showed {flag}: {text}")
            self.release(step)
        text = "STOP held while the release is sent: the emergency must stay"
        if self.ask(f"{step} release with a pressed input: {text}"):
            self.ready(step)
            self.mark(step, text)
            if self.person.fake:
                self.person.set({"stop": True})
                self.wait(0.5)
            else:
                self.person.act("press and keep holding STOP, then Enter")
            self.mower.release()
            self.wait(1.5)
            if not self.in_emergency():
                self.finding(step, "released while STOP was pressed")
            self.show("after the release while held")
            if self.person.fake:
                self.person.clear({"stop": True})
            else:
                self.person.act("let go of STOP, unlock it if it stays down")
            self.wait(0.5)
            self.release(step)

    def t2(self):
        step = "T2"
        if not self.ask(f"{step} wheels in the air (jacked up, wheels free, blade disc off, you at STOP and battery): "
                        f"first turn at {TRON_FIRST_SPEED} m/s ({AIR_WHEEL_SPEED} m/s if no wheel turns), then press "
                        "STOP once while they turn"):
            return
        self.ready(step)
        self.first_turn(step)
        self.ready(step)
        self.mark(step, "STOP while turning")
        self.emergency_run(step, "press STOP while the wheels turn", {"stop": True}, 1.0)
        self.let_go("STOP")
        self.release(step)
        self.no_motion(step, 2.0, "after the release without a new command")
        restart = "blade off, the height motor may move"
        ways = [("freeze", signal.SIGSTOP, f"frozen (SIGSTOP), killed after the window and started again: {restart}"),
                ("kill", signal.SIGKILL, f"killed (SIGKILL), started again after the window: {restart}")]
        for way, sig, text in ways:
            stands = []
            for n in range(self.args.repeats):
                if not self.ask(f"{step} host gone, run {n + 1}/{self.args.repeats}: wheels at {self.speed} m/s "
                                f"for 2 s, then tron_mcu {text}; meanwhile this programme only reads the port, "
                                f"{HOST_GONE_WINDOW_S:.0f} s"):
                    break
                stands.append(self.host_gone_run(step, way, sig, n + 1))
            self.host_gone[way] = stands
            problems = holding_problems(stands)
            for problem in problems:
                self.finding(step, f"host gone ({way}): {problem}")
            if problems:
                raise Abort()
        self.heartbeat_only(step)

    def first_turn(self, step):
        for v in (TRON_FIRST_SPEED, AIR_WHEEL_SPEED):
            self.mark(step, f"first turn at {v} m/s")
            if self.drive(v, 0.0, 3.0, until=self.in_emergency):
                self.mower.stop_wheels()
                self.finding(step, f"emergency during the first turn at {v} m/s")
                raise Abort()
            turned = self.wheels_turn()
            self.mower.stop_wheels()
            self.wait(1.0)
            if turned:
                self.turn_speed = v
                print(f"     the wheels turn at {v} m/s")
                self.rec.write("turn_speed", speed=v)
                return
            print(f"     no wheel turned at {v} m/s")
        self.finding(step, f"no wheel turned at {AIR_WHEEL_SPEED} m/s")
        raise Abort()

    def host_gone_run(self, step, way, sig, n):
        """One run without host: the stand time from the signal, from the MCU's own reports on the port."""
        self.ready(step)
        self.mark(step, f"host gone ({way}), run {n}")
        if self.drive(self.speed, 0.0, 2.0, until=self.in_emergency) or not self.wheels_turn():
            self.mower.stop_wheels()
            self.finding(step, f"host gone ({way}) run {n}: the wheels did not turn, or an emergency came first")
            raise Abort()
        since, capture = self.without_host(step, way, sig, f"{way}-{n}", HOST_GONE_WINDOW_S)
        packets, summary, code = self.start.decode(capture)
        result = host_gone_result(packets, since, self.mower.wheel_distance)
        self.rec.write("host_gone", step=step, way=way, run=n, capture=capture, summary=summary, **result)
        if code != 0 or not result["samples"]:
            self.finding(step, f"the capture {capture} has no speed reports (tron_mcu_capture exit {code})")
            raise Abort()
        stand = result["stand_s"]
        flags = ", ".join(f"{name} up after {result[name + '_s']:.2f} s" for name in ("lift", "stop")
                          if result[name + "_s"] is not None)
        print(f"     the wheels stood {'never' if stand is None else f'{stand:.2f} s'} after the signal"
              + (f"; {flags}" if flags else "") + f" ({summary.get('frames', 0)} frames, "
              f"{summary.get('checksum_errors', 0)} checksum errors)")
        if not result["turning_at_start"]:
            print("     (the first report after the signal already stood)")
        if stand is None:
            self.finding(step, f"host gone ({way}) run {n}: the wheels still turned {HOST_GONE_WINDOW_S:.0f} s after "
                               "the signal")
            if way == "kill" and not self.person.fake:
                self.person.act("the wheels may still turn without a stop from the host: press STOP now")
            raise Abort()
        self.no_motion(step, 3.0, "after tron_mcu came back without a new command")
        self.show("after")
        self.release(step)
        self.no_motion(step, 2.0, "after the release without a new command")
        return stand

    def without_host(self, step, way, sig, label, window):
        """Sends `sig` to tron_mcu and reads the port for `window` s; then the driver starts again and is waited for.
        Returns the time of the signal and the capture. A capture that reached its limit does not hold the whole window
        and ends the step."""
        capture = self.capture_path(f"{step}-{label}")
        limit = capture_limit(window)
        since = time.monotonic()
        pid = self.start.signal_driver(sig)
        print(f"     sent {signal.Signals(sig).name} to tron_mcu (pid {pid}), reading {self.start.mcu_port}")
        reader = None
        try:
            self.driver_off(step, way, pid)
            reader = PortReader(self.start.mcu_port, capture, limit)
            self.wait(window)
        except PortRefused as error:
            self.finding(step, str(error))
            raise Abort()
        finally:
            if reader is not None:
                reader.close()
            if way == "freeze":
                # Stopped longer than its watch limit it would end itself once continued; it is ended here instead.
                os.kill(pid, signal.SIGKILL)
                self.start.processes["tron_mcu"].wait()
                self.rec.write("signal", signal="SIGKILL", pid=pid)
            if self.start.driver_pid is None:
                self.start.start_driver(**self.start.overrides)
        self.wait_driver_back(step, pid)
        if reader.full:
            self.finding(step, f"the capture {capture} reached its limit of {limit} bytes before the window ended")
            raise Abort()
        return since, capture

    def driver_off(self, step, way, pid):
        """Waits until tron_mcu reads no more (stopped, or ended and reaped) and nobody else holds the port."""
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            state = process_state(pid)
            if way == "freeze" and state in ("T", "t"):
                break
            if way == "kill" and self.start.processes["tron_mcu"].poll() is not None:
                break
            time.sleep(0.01)
        else:
            self.finding(step, f"tron_mcu (pid {pid}) is still running after the signal")
            raise Abort()
        holders = port_holders(self.start.mcu_port)
        if holders != ([pid] if way == "freeze" else []):
            self.finding(step, f"{self.start.mcu_port} is open in {holders}, expected only the stopped tron_mcu")
            raise Abort()

    def heartbeat_only(self, step):
        n = min(self.args.repeats, TRON_SHORT_REPEATS)
        if not self.ask(f"{step} heartbeat alone, {n} runs: tron_mcu starts again sending each command once, with a "
                        f"watchdog of {HEARTBEAT_ONLY_TIMEOUT_S:.0f} s (this start only); wheels at {self.speed} m/s "
                        f"for 2 s, then no command: they may turn {HEARTBEAT_ONLY_TIMEOUT_S:.0f} s more"):
            return
        old = self.start.driver_pid
        self.start.restart_driver(**HEARTBEAT_ONLY)
        try:
            self.wait_driver_back(step, old)
            for i in range(n):
                if i and not self.ask(f"{step} heartbeat alone, run {i + 1}/{n}"):
                    break
                self.ready(step)
                self.mark(step, f"heartbeat alone, run {i + 1}")
                if self.drive(self.speed, 0.0, 2.0, until=self.in_emergency):
                    self.finding(step, "emergency while driving (heartbeat alone)")
                    raise Abort()
                last = self.mower.last_cmd_at
                stand = self.stand_after(last, HEARTBEAT_ONLY_TIMEOUT_S + 2.0)
                self.rec.write("heartbeat_only", step=step, run=i + 1, stand_s=stand)
                if stand is None:
                    self.finding(step, "the wheels did not stand after the driver's watchdog (heartbeat alone)")
                    raise Abort()
                held = stand >= HEARTBEAT_ONLY_TIMEOUT_S - WATCHDOG_MARGIN_S
                print(f"     the wheels stood {stand:.2f} s after the last command: "
                      + ("the MCU held the setpoint with the heartbeat alone" if held else
                         "the MCU stopped with the heartbeat alone"))
                self.no_motion(step, 2.0, "after the driver's stop")
        finally:
            old = self.start.driver_pid
            self.start.restart_driver()
        self.wait_driver_back(step, old)

    def t3(self):
        step = "T3"
        if not self.ask(f"{step} driving in the air, at most {AIR_WHEEL_SPEED} m/s a wheel (jacked up, wheels free, "
                        "blade disc off): start?"):
            return
        d = self.mower.wheel_distance
        items = [("forward 0.05 m/s for 8 s: do both wheels turn forward?", 0.05, 0.0),
                 ("backward 0.05 m/s for 8 s: do both wheels turn backward?", -0.05, 0.0),
                 ("turn left, 8 s: left wheel back, right wheel forward, 0.05 m/s each", 0.0, 0.1 / d),
                 ("turn right, 8 s: left wheel forward, right wheel back, 0.05 m/s each", 0.0, -0.1 / d)]
        for text, v, w in items:
            if not self.ask(f"{step} {text}"):
                continue
            self.ready(step)
            self.mark(step, text)
            started = time.monotonic()
            self.drive(v, w, 8.0, until=self.in_emergency)
            status = self.status()
            ended = time.monotonic()
            self.mower.stop_wheels()
            self.direction(step, text, v, w, started + 2.0, ended, status)
            self.wait(1.0)
            if not self.person.fake and not self.ask(f"{step} did the wheels turn as said?"):
                self.finding(step, f"wheels not as commanded: {text}")
        points = []
        for v in (0.02, 0.03, 0.05, 0.07, 0.1):
            text = f"straight at {v:.2f} m/s for 3 s (reported speed per command)"
            if not self.ask(f"{step} {text}"):
                continue
            self.ready(step)
            self.mark(step, text)
            self.drive(v, 0.0, 3.0, until=self.in_emergency)
            ended = time.monotonic()
            self.mower.stop_wheels()
            reported = self.mean_odometry(ended - 1.5, ended)[0]
            points.append((v, reported))
            self.rec.write("speed_point", step=step, command=v, reported=reported)
            print(f"     command {v:.2f} m/s, reported {reported:.3f} m/s")
            self.wait(1.0)
        moving = [(v, r) for v, r in points if abs(r) > self.still_speed]
        if moving:
            factor = sum(r / v for v, r in moving) / len(moving)
            standing = [v for v, r in points if abs(r) <= self.still_speed]
            print(f"     reported/command {factor:.3f} on average; no motion at {standing or 'none'}")
            self.rec.write("speed_factor", step=step, factor=factor, standing=standing)
        text = "0.05 m/s for 20 s: count the turns of the left wheel"
        if self.ask(f"{step} {text}"):
            self.ready(step)
            self.mark(step, text)
            self.drive(0.05, 0.0, 20.0, until=self.in_emergency)
            self.mower.stop_wheels()
            turns = self.person.number("turns of the left wheel")
            circumference = self.person.number("circumference of the wheel in m")
            self.rec.write("wheel_turns", turns=turns, circumference=circumference)

    def mean_odometry(self, start, end):
        samples = [(v, w) for t, v, w in self.mower.odometry() if start <= t <= end]
        if not samples:
            return 0.0, 0.0
        return sum(v for v, _ in samples) / len(samples), sum(w for _, w in samples) / len(samples)

    def direction(self, step, text, v, w, start, end, status):
        """Signs of the reported motion against the command, and the labels of the drive motors on a turn."""
        rv, rw = self.mean_odometry(start, end)
        left = right = None
        if status is not None and status.left_motor.ok and status.right_motor.ok:
            left, right = status.left_motor.rpm, status.right_motor.rpm
        self.rec.write("direction", step=step, text=text, v=v, w=w, reported_v=rv, reported_w=rw, left_rpm=left,
                       right_rpm=right)
        print(f"     reported v {rv:.3f} m/s, w {rw:.3f} rad/s; motors left {left}, right {right} rpm")
        command, reported = (v, rv) if w == 0 else (w, rw)
        if command * reported <= 0:
            self.finding(step, f"reported motion not in the commanded direction: {text}")
        if w != 0 and left and right:
            if (left > 0) == (w > 0) and (right < 0) == (w > 0):
                self.finding(step, f"labels left/right swapped (left motor {left}, right motor {right} rpm): {text}")

    def t4(self):
        step = "T4"
        items = [("press STOP while the wheels turn", {"stop": True}, 1.0),
                 ("lift the front while the wheels turn", {"lift": True}, 1.0),
                 ("push the bumper left while the wheels turn", {"bumper_left": True}, 1.0),
                 ("push the bumper right while the wheels turn", {"bumper_right": True}, 1.0)]
        n = min(self.args.repeats, TRON_SHORT_REPEATS)
        for text, fake, hold in items:
            for i in range(n):
                if not self.ask(f"{step} {text}, run {i + 1}/{n}: wheels at {self.speed} m/s for up to 10 s, stop on "
                                "the emergency, then the release and a new command"):
                    break
                self.ready(step)
                self.mark(step, f"{text}, run {i + 1}")
                self.emergency_run(step, text, fake, hold)
                self.wait(hold + 0.5)
                self.let_go(text, "Then hands off: after the release the wheels turn 1 s on a new command")
                self.no_motion(step, 2.0, "with the input free and no new command")
                self.release(step)
                self.no_motion(step, 2.0, "after the release without a new command")
                self.drive(self.speed, 0.0, 1.0, until=self.in_emergency)
                if not self.wheels_turn():
                    self.finding(step, f"no turn on a new command after the release: {text}")
                self.mower.stop_wheels()

    def t5(self):
        step = "T5"
        timeout = self.start.watchdog()
        n = min(self.args.repeats, TRON_SHORT_REPEATS)
        for i in range(n):
            if not self.ask(f"{step} the driver's watchdog, run {i + 1}/{n}: wheels at {self.speed} m/s for 2 s, then "
                            f"no command; tron_mcu has to stop them after {timeout:.2f} s with three stops, then stay "
                            "silent"):
                break
            self.ready(step)
            self.mark(step, f"watchdog, run {i + 1}")
            if self.drive(self.speed, 0.0, 2.0, until=self.in_emergency):
                self.finding(step, "emergency while driving")
                raise Abort()
            last = self.mower.last_cmd_at
            stand = self.stand_after(last, timeout + EMERGENCY_STAND_S + WATCHDOG_MARGIN_S)
            # The MCU may stop the wheels before the watchdog fires (it drops a setpoint ~0.2 s after the last one).
            self.wait(max(0.0, last + timeout + WATCHDOG_MARGIN_S - time.monotonic()))
            moving = self.status().moving
            self.rec.write("watchdog", step=step, run=i + 1, stand_s=stand, moving=moving)
            if stand is None or stand > timeout + EMERGENCY_STAND_S or moving:
                shown = "never" if stand is None else f"after {stand:.2f} s"
                self.finding(step, f"the wheels stood {shown} without a command (watchdog {timeout:.2f} s), "
                                   f"driver moving {moving}")
                raise Abort()
            print(f"     the wheels stood {stand:.2f} s after the last command")
            self.no_motion(step, 2.0, "after the driver's stop")

    def t6(self):
        step = "T6"
        self.blade_setup()
        longest = BLADE_RUN_S + self.args.blade_window + self.blade_stop_s
        if not self.ask(f"{step} blade motor with the disc OFF (jacked up, you at STOP and battery): tron_mcu starts "
                        f"again with the blade allowed, for this step only; the motor turns up to {longest:.0f} s a "
                        f"run at {TRON_BLADE_HEIGHT_MM} mm, the wheels get no command"):
            return
        items = [("press STOP while the blade motor turns", {"stop": True}, 1,
                  "let go of STOP, unlock it if it stays down"),
                 ("push the bumper left while the blade motor turns", {"bumper_left": True}, 1, "let go of the bumper"),
                 ("push the bumper right while the blade motor turns", {"bumper_right": True}, 1,
                  "let go of the bumper")]

        def parts():
            self.blade_basics(step)
            self.blade_inputs(step, items, self.args.blade_percent[0])
            self.blade_host_gone(step)
        self.blade_allowed(step, parts)

    def blade_setup(self):
        self.blade_still_rpm = int(self.start.param("tron_mcu", "blade_still_rpm"))
        self.blade_stop_s = float(self.start.param("tron_mcu", "blade_stop_time"))
        self.blade_lease_s = float(self.start.param("mowbite_logic", "blade_lease"))
        self.blade_renew_s = float(self.start.param("mowbite_logic", "blade_renew"))

    def blade_allowed(self, step, parts):
        """`parts` with tron_mcu started again with the blade allowed; after them, also on an abort, it starts again
        without."""
        self.blade_used = True
        old = self.start.driver_pid
        self.start.restart_driver(allow_blade=True)
        try:
            self.wait_driver_back(step, old)
            parts()
        finally:
            # tron_mcu switches the blade off on its way out; the next start does not allow it.
            old = self.start.driver_pid
            self.start.restart_driver()
        self.wait_driver_back(step, old)

    def blade_rpm(self):
        """The rpm of the blade motor in hw/status; None without a fresh report of the motors."""
        s, age, _, _ = self.mower.snapshot()
        return abs(s.blade_motor.rpm) if s is not None and age < 0.5 and s.blade_motor.ok else None

    def blade_turns(self):
        rpm = self.blade_rpm()
        return rpm is not None and rpm >= self.blade_still_rpm

    def blade_run(self, step, percent, seconds, until=None, height_mm=TRON_BLADE_HEIGHT_MM):
        """The blade motor on at `percent` on a renewed lease for `seconds`, or until `until` holds: True then. A
        refused request ends the step. blade_renewed is when the last accepted request went out."""
        print(f"     sending hw/blade on at {percent} %, {height_mm} mm, lease {self.blade_lease_s} s, "
              f"every {self.blade_renew_s} s, for up to {seconds:.0f} s")
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if until and until():
                return True
            asked = time.monotonic()
            answer = self.mower.set_blade(True, self.blade_lease_s, percent, height_mm)
            if answer is None or not answer.success:
                if until and until():
                    return True
                self.finding(step, f"hw/blade refused: {answer.message if answer else 'no answer'}")
                raise Abort()
            self.blade_renewed = asked
            if self.wait(self.blade_renew_s, until):
                return True
        return False

    def blade_turning(self, step, percent):
        """`percent` for BLADE_RUN_S; the rpm the MCU reports then. A motor that does not turn ends the step."""
        self.blade_run(step, percent, BLADE_RUN_S)
        rpm = self.blade_rpm()
        if self.in_emergency() or not self.blade_turns():
            self.mower.set_blade(False)
            self.show("blade")
            self.finding(step, f"the blade motor reports {rpm} rpm after {BLADE_RUN_S:.0f} s at {percent} %")
            raise Abort()
        print(f"     the blade motor turns at {rpm} rpm")
        return rpm

    def blade_stands(self, step, since, what, seconds=None):
        """s from `since` until hw/status showed the blade motor standing, waiting up to `seconds` (blade_stop_s) for
        it. A motor that still turns ends the step."""
        limit = self.blade_stop_s if seconds is None else seconds
        rpm = self.blade_rpm
        if not self.wait(max(0.0, since + limit - time.monotonic()),
                         until=lambda: rpm() is not None and rpm() < self.blade_still_rpm):
            self.finding(step, f"the blade motor reports {rpm()} rpm {limit:.0f} s after {what}")
            if not self.person.fake:
                self.person.act("the blade motor may still turn: press STOP now")
            raise Abort()
        still = time.monotonic() - since
        print(f"     the blade motor stood {still:.1f} s after {what} (the MCU reports it about once a second)")
        return still

    def blade_basics(self, step):
        for percent in self.args.blade_percent:
            text = f"on at {percent} % for {BLADE_RUN_S:.0f} s, then off"
            if not self.ask(f"{step} blade motor {text}"):
                continue
            self.ready(step)
            self.mark(step, text)
            rpm = self.blade_turning(step, percent)
            print("     sending hw/blade off")
            since = time.monotonic()
            self.mower.set_blade(False)
            still = self.blade_stands(step, since, "the off request")
            self.rec.write("blade_off", step=step, percent=percent, rpm=rpm, still_s=still)
        percent = self.args.blade_percent[0]
        text = f"on at {percent} %, then no renewal: tron_mcu switches it off when the lease runs out"
        if self.ask(f"{step} blade motor {text}"):
            self.ready(step)
            self.mark(step, text)
            rpm = self.blade_turning(step, percent)
            still = self.blade_stands(step, self.blade_renewed, "the last renewal",
                                      self.blade_lease_s + self.blade_stop_s)
            self.rec.write("blade_lease", step=step, percent=percent, rpm=rpm, still_s=still)

    def blade_inputs(self, step, items, percent):
        """Each input (text, flags of the rehearsal, runs, how it goes free) while the blade motor turns at `percent`:
        the emergency within --input-wait s, the motor standing within blade_stop_s of it, still standing once the input
        is free and after the release."""
        wait = self.args.input_wait
        for text, fake, runs, free in items:
            for i in range(runs):
                run = f", run {i + 1}/{runs}" if runs > 1 else ""
                if not self.ask(f"{step} {text}{run}: on at {percent} % for up to {BLADE_RUN_S + wait:.0f} s, it has "
                                "to stand on the emergency and stay so after the release"):
                    break
                self.ready(step)
                self.mark(step, text + run)
                rpm = self.blade_turning(step, percent)
                self.person.act_later(f"{step} now, within {wait:.0f} s: {text}", fake, after=1.0, hold=1.0)
                started = time.monotonic()
                if not self.blade_run(step, percent, wait, until=self.in_emergency):
                    self.mower.set_blade(False)
                    self.finding(step, f"no emergency within {wait:.0f} s: {text}")
                    raise Abort()
                since = self.mower.emergency_since
                since = since if since is not None and since >= started else time.monotonic()
                still = self.blade_stands(step, since, "the emergency")
                self.rec.write("blade_emergency", step=step, text=text, run=i + 1, rpm=rpm, still_s=still)
                self.show("emergency")
                self.wait(1.5)
                if not self.person.fake:
                    self.person.act(f"{step}: the blade motor stands; {free}, hands off")
                if self.wait(1.0, until=self.blade_turns):
                    self.finding(step, f"the blade motor turned with the input free, before the release: {text}")
                    raise Abort()
                self.release(step)
                if self.wait(2.0, until=self.blade_turns):
                    self.finding(step, f"the blade motor turned after the release without a request: {text}")
                    raise Abort()

    def t7(self):
        tilts, unlifted = self.lift_angles("T7.1")
        self.lift_level("T7.2")
        step = "T7.3"
        items = lift_blade_items(tilts, unlifted, min(self.args.repeats, TRON_SHORT_REPEATS))
        if not items:
            print(f"\n{step} skipped: T7.1 saw no lift at {LIFT_SIDES[LIFT_SHOWN_SIDE][0]}")
            return
        self.blade_setup()
        longest = BLADE_RUN_S + self.args.input_wait + self.blade_stop_s
        if not self.ask(f"{step} lift on the blade motor with the disc OFF (jacked up, you at STOP and battery, hands "
                        "only at the top edge of the hood): tron_mcu starts again with the blade allowed, for this "
                        f"part only; the motor turns at {LIFT_BLADE_PERCENT} % up to {longest:.0f} s a run at "
                        f"{TRON_BLADE_HEIGHT_MM} mm, the wheels get no command"):
            return
        self.blade_allowed(step, lambda: self.blade_inputs(step, items, LIFT_BLADE_PERCENT))

    def lift_angles(self, step):
        """Each side of --lift-sides lifted by hand without a motor until hw/status shows lift: the tilt at the first
        lift by side, and the sides without lift. No lift is a finding only at LIFT_SHOWN_SIDE (T7.3 leaves it out
        then); at another side it is expected and only noted."""
        tilts, unlifted = {}, set()
        for side in self.args.lift_sides:
            name, tilt = LIFT_SIDES[side]
            label = f"{step} {side}"
            expected = side == LIFT_SHOWN_SIDE
            note = "" if expected else "; the MCU is not expected to show lift here"
            if not self.ask(f"{label} no motor, sends nothing but the release (clear frame) afterwards: lift {name} "
                            f"slowly until lift shows, at most {LIFT_MAX_DEG} deg, hold 2 s, set it down{note}"):
                continue
            result = self.lift_by_hand(label, f"lift {name} slowly by the top edge of the hood until lift shows, at "
                                              f"most {LIFT_MAX_DEG} deg", f"set {name} down", 2.0, tilt)
            self.rec.write("lift_angle", step=step, side=side, expected_lift=expected, **result)
            if result["lift"]:
                tilts[side] = result["tilt_at_lift"]
                if not expected:
                    print(f"     note: lift lifting {name}, where the MCU is not expected to show it")
                continue
            unlifted.add(side)
            largest = f"largest tilt {result['max_tilt']:.1f} deg"
            if expected:
                self.finding(label, f"no lift in hw/status lifting {name} ({largest}): T7.3 leaves it out")
            else:
                print(f"     note: no lift lifting {name} ({largest}), as expected: the MCU shows lift only with "
                      "both front wheels off the ground")
        return tilts, unlifted

    def lift_level(self, step):
        text = "lift the whole mower level with both hands at the top edge of the hood about 5 cm off the stand"
        if not self.ask(f"{step} no motor, sends nothing but the release (clear frame) afterwards: {text}, hold 3 s, "
                        "set it down; lift or no lift is the result"):
            return
        result = self.lift_by_hand(step, text, "set it down on the stand", 3.0)
        self.rec.write("lift_level", step=step, **result)
        if not result["lift"]:
            self.finding(step, "no lift in hw/status with the mower lifted level (safety finding G1: lift does not "
                               "cover a Tron lifted level)")

    def lift_by_hand(self, label, text, down, hold, tilt=None):
        """The person lifts the mower, no motor runs. Waits up to LIFT_WAIT_S for lift in hw/status (or a tilt past
        LIFT_MAX_DEG), then as long for lift free; the emergency has to stay latched until the release. Returns what
        it saw: lift, the tilt at the first lift, the largest tilt, latched, released."""
        self.ready(label)
        self.mark(label, text)
        self.mower.reset_tilt()
        self.person.act_later(f"{label} now: {text}, hold {hold:.0f} s, {down}; this programme waits up to "
                              f"{LIFT_WAIT_S:.0f} s for lift", {"lift": True}, after=1.0, hold=hold, lead=tilt)

        def steep():
            return (self.mower.tilt or 0.0) > LIFT_MAX_DEG
        self.wait(LIFT_WAIT_S, until=lambda: self.shown("lift") or steep())
        with self.mower.lock:
            first = self.mower.first_lift
        result = {"lift": first is not None, "tilt_at_lift": first[1] if first else None, "latched": None}
        if first is None:
            why = f"up to {LIFT_MAX_DEG} deg" if steep() else f"within {LIFT_WAIT_S:.0f} s"
            print(f"     no lift {why}")
            if not self.person.fake:
                self.person.act(f"{label}: no lift {why}; {down} now")
        else:
            shown = "no tilt from imu/data" if first[1] is None else f"{first[1]:.1f} deg"
            self.person.act_later(f"{label}: lift seen at {shown}; hold {hold:.0f} s, then {down}")
            if not self.wait(LIFT_WAIT_S, until=lambda: self.shown("lift") is False):
                self.finding(label, f"hw/status still shows lift {LIFT_WAIT_S:.0f} s after it came")
                raise Abort()
            self.wait(0.5)
            result["latched"] = self.in_emergency()
            if not result["latched"]:
                self.finding(label, "the emergency of the lift did not stay after setting down")
        with self.mower.lock:
            result["max_tilt"] = self.mower.max_tilt
        self.show("after")
        result["released"] = self.release(label)
        return result

    def blade_host_gone(self, step):
        ways = [("freeze", signal.SIGSTOP, "frozen (SIGSTOP), killed after the window and started again"),
                ("kill", signal.SIGKILL, "killed (SIGKILL), started again after the window")]
        n = min(self.args.repeats, TRON_SHORT_REPEATS)
        for way, sig, text in ways:
            for percent in self.args.blade_percent:
                stills = []
                for i in range(n):
                    if not self.ask(f"{step} blade without host, {percent} %, run {i + 1}/{n}: the motor on for "
                                    f"{BLADE_RUN_S:.0f} s, then tron_mcu {text}; meanwhile this programme only reads "
                                    f"the port, {self.args.blade_window:.0f} s"):
                        break
                    stills.append(self.blade_gone_run(step, way, sig, percent, i + 1))
                if stills:
                    print(f"     {way}, {percent} %: the blade motor stood {min(stills):.1f}-{max(stills):.1f} s after "
                          "the signal")

    def blade_gone_run(self, step, way, sig, percent, n):
        """One run of the blade motor without host: the time from the signal until the MCU reported it standing. A
        motor the MCU kept turning through the window is the finding of T6 and ends it."""
        self.ready(step)
        self.mark(step, f"blade without host ({way}), {percent} %, run {n}")
        rpm = self.blade_turning(step, percent)
        since, capture = self.without_host(step, way, sig, f"{way}-{percent}-{n}", self.args.blade_window)
        back = time.monotonic()
        packets, summary, code = self.start.decode(capture)
        result = blade_gone_result(packets, since, self.blade_still_rpm)
        self.rec.write("blade_gone", step=step, way=way, percent=percent, run=n, rpm=rpm, capture=capture,
                       summary=summary, **result)
        if code != 0 or not result["samples"]:
            self.finding(step, f"the capture {capture} has no reports of the motors (tron_mcu_capture exit {code})")
            raise Abort()
        still = result["still_s"]
        print(f"     the MCU reported the blade motor standing {'never' if still is None else f'{still:.1f} s'} after "
              f"the signal, last {result['last_rpm']} rpm ({result['samples']} reports, "
              f"{summary.get('checksum_errors', 0)} checksum errors)")
        # With its host back the blade goes off: the lease ran out, and a new tron_mcu sends off when it connects.
        self.blade_stands(step, back, "tron_mcu came back")
        if still is None:
            self.finding(step, f"blade without host ({way}), {percent} %, run {n}: the MCU kept the motor turning for "
                               f"{self.args.blade_window:.0f} s without its host")
            raise Abort()
        self.show("after")
        self.release(step)
        return still

    def t8(self):
        parts = (("T8.1", "a short press of STOP, then one release", self.g37_short),
                 ("T8.2", f"STOP held about {G37_HOLD_S:.0f} s with a release every {G37_RELEASE_EVERY_S:.0f} s, "
                          "then let go and released", self.g37_held),
                 ("T8.3", "a short press of a bumper; if the MCU keeps it, a short press of STOP and one release",
                  self.g37_bumper),
                 ("T8.4", f"{LIFT_SIDES[LIFT_SHOWN_SIDE][0]} lifted until lift shows, one release while it is up, "
                          "one once it is down", self.g37_lift),
                 ("T8.5", f"tron_mcu killed (SIGKILL) with stop and lift free, the port read for "
                          f"{G37_GONE_WINDOW_S:.0f} s, then tron_mcu starts again", self.g37_gone))
        for step, text, run in parts:
            if self.ask(f"{step} no motor, no wheel command, sends nothing but the release (clear frame): {text}"):
                result = run(step)
                self.rec.write("g37", step=step, **result)
                print(f"     {step}: {result}")

    def g37_press(self, step, text, fake, name, hold=0.5):
        """The person presses something and lets go; waits until hw/status shows `name` with the emergency."""
        self.ready(step)
        self.mark(step, text)
        self.person.act_later(f"{step} now: {text}", fake, after=1.0, hold=hold)
        if not self.wait(STOP_CHECK_S, until=lambda: self.shown(name) and self.in_emergency()):
            self.finding(step, f"hw/status did not show {name} with the emergency within {STOP_CHECK_S:.0f} s")
            raise Abort()

    def g37_clear(self, name, wait_s=G37_CLEAR_WAIT_S):
        """One release (tron_mcu sends the clear frame with it while stop or lift shows): s until hw/status shows
        `name` free, None if it does not within `wait_s`."""
        sent = time.monotonic()
        self.mower.release()
        if self.wait(max(0.0, sent + wait_s - time.monotonic()), until=lambda: self.shown(name) is False):
            return round(time.monotonic() - sent, 2)
        return None

    def g37_short(self, step):
        self.g37_press(step, "press STOP briefly (under 1 s) and let go", {"stop": True}, "stop")
        self.wait(G37_LET_GO_S)
        self.person.done()
        kept = self.shown("stop")
        if not kept:
            print("     note: the MCU did not keep stop after it was let go")
        free_s = self.g37_clear("stop")
        if kept and free_s is None:
            self.finding(step, "stop stayed after the clear frame with STOP let go")
        return {"kept": kept, "free_s": free_s, "released": self.release(step)}

    def g37_held(self, step):
        hold = G37_HOLD_S + 2 * G37_RELEASE_EVERY_S
        self.g37_press(step, f"press STOP and keep it pressed until this programme says let go (about {hold:.0f} s)",
                       {"stop": True}, "stop", hold=hold)
        releases, dropped_s = 0, None
        end = time.monotonic() + G37_HOLD_S
        while time.monotonic() < end and dropped_s is None:
            releases += 1
            dropped_s = self.g37_clear("stop", G37_RELEASE_EVERY_S)
        self.let_go("STOP")
        self.wait(G37_RELEASE_EVERY_S)
        free_s = self.g37_clear("stop")
        if free_s is None:
            free_s = self.g37_clear("stop")
        held = None
        if dropped_s is not None:
            held = self.ask(f"{step}: stop fell {dropped_s:.2f} s after release {releases}. Was STOP pressed all the "
                            "time until 'let go'? y = yes, s = no")
            if held:
                self.finding(step, "stop fell after a clear frame while STOP was held: a held STOP is not seen this "
                                   "way (G37 draft)")
        if free_s is None:
            self.finding(step, "stop stayed after the clear frame with STOP let go")
        return {"releases": releases, "dropped_s": dropped_s, "held_confirmed": held, "free_s": free_s,
                "released": self.release(step)}

    def g37_bumper(self, step):
        self.g37_press(step, "press a bumper briefly and let go", {"bumper_left": True}, "bumper")
        self.wait(G37_LET_GO_S)
        self.person.done()
        result = {"kept": self.shown("bumper")}
        if result["kept"]:
            print("     the MCU keeps the bumper after it was let go; a short press of STOP brings the clear frame")
            self.person.act_later(f"{step} now: press STOP briefly (under 1 s) and let go", {"stop": True}, after=0.5,
                                  hold=0.5)
            if not self.wait(STOP_CHECK_S, until=lambda: self.shown("stop")):
                self.finding(step, f"hw/status did not show stop within {STOP_CHECK_S:.0f} s")
                raise Abort()
            self.wait(G37_LET_GO_S)
            self.person.done()
            result["stop_free_s"] = self.g37_clear("stop")
            self.wait(0.5)
            result["kept_after_clear"] = self.shown("bumper")
        result["released"] = self.release(step)
        return result

    def g37_lift(self, step):
        name, tilt = LIFT_SIDES[LIFT_SHOWN_SIDE]
        self.ready(step)
        self.mark(step, f"lift {name}")
        self.person.act_later(f"{step} now: lift {name} slowly by the top edge of the hood until lift shows, at most "
                              f"{LIFT_MAX_DEG} deg, and keep it up until this programme says set it down",
                              {"lift": True}, after=1.0, hold=G37_LIFT_HOLD_S, lead=tilt)
        if not self.wait(LIFT_WAIT_S, until=lambda: self.shown("lift")):
            self.finding(step, f"no lift in hw/status within {LIFT_WAIT_S:.0f} s")
            raise Abort()
        self.wait(0.5)
        free_up_s = self.g37_clear("lift")
        if free_up_s is not None:
            self.finding(step, f"lift fell {free_up_s:.2f} s after a clear frame with {name} still up")
        if self.person.fake:
            self.person.done()
        else:
            self.person.act(f"set {name} down now")
        self.wait(G37_LET_GO_S)
        kept = self.shown("lift")
        free_down_s = self.g37_clear("lift")
        if free_down_s is None:
            self.finding(step, f"lift stayed after the clear frame with {name} down")
        return {"free_up_s": free_up_s, "kept_after_down": kept, "free_down_s": free_down_s,
                "released": self.release(step)}

    def g37_gone(self, step):
        """What the MCU shows on its own once its host is gone after a clear: when stop and lift come up."""
        self.ready(step)
        before = {"stop": self.shown("stop"), "lift": self.shown("lift")}
        self.mark(step, "tron_mcu killed")
        since, capture = self.without_host(step, "kill", signal.SIGKILL, "g37", G37_GONE_WINDOW_S)
        packets, summary, code = self.start.decode(capture)
        gone = host_gone_result(packets, since, self.mower.wheel_distance)
        if code != 0 or not gone["samples"]:
            self.finding(step, f"the capture {capture} has no reports of the MCU (tron_mcu_capture exit {code})")
            raise Abort()
        after = self.status()
        if gone["stop_s"] is None and gone["lift_s"] is None:
            print(f"     note: the MCU showed neither stop nor lift within {G37_GONE_WINDOW_S:.0f} s without its host")
        self.show("tron_mcu back")
        return {"before": before, "stop_s": gone["stop_s"], "lift_s": gone["lift_s"], "capture": capture,
                "checksum_errors": summary.get("checksum_errors", 0),
                "restart_emergency": bool(after and after.emergency),
                "restart_reason": reason_text(after.emergency_reason) if after else None,
                "released": self.release(step)}

    def t9(self):
        step = "T9"
        if not self.ask(f"{step} hold after the last frame (jacked up, wheels free, disc off, you at STOP and "
                        f"battery): cases {','.join(self.args.hold)}; each starts tron_mcu again with its settings; wheels at "
                        f"{TRON_FIRST_SPEED} m/s for {HOLD_DRIVE_S:.0f} s, then nothing to the MCU right after the "
                        "chosen frame: they may turn until the silence ends"):
            return
        for case in self.args.hold:
            text, overrides, silence, runs = HOLD_CASES[case]
            runs = runs or self.args.repeats
            if not self.ask(f"{step}.{case} {text}: {runs} runs, silence {silence:.0f} s"):
                continue
            old = self.start.driver_pid
            self.start.restart_driver(silence_max=silence, **overrides)
            try:
                self.wait_driver_back(step, old)
                if case == "B":
                    self.blade_used = True
                    self.blade_setup()
                times = []
                for n in range(runs):
                    if n and not self.ask(f"{step}.{case} run {n + 1}/{runs}"):
                        break
                    self.release(step)
                    run = self.hold_blade_run if case == "B" else self.hold_run
                    times.append(run(step, case, silence, n + 1))
            finally:
                old = self.start.driver_pid
                self.start.restart_driver()
            self.wait_driver_back(step, old)
            held = sum(1 for t in times if t > HOLD_HELD_S)
            if times:
                print(f"     {case}: stood {min(times):.2f}-{max(times):.2f} s after the silence began, held in "
                      f"{held} of {len(times)} runs")
            self.rec.write("hold_case", step=step, case=case, stand_s=times, held=held)

    def hold_start(self, step, case, n, silence, ask_silence, frame):
        """hw/silence asked, then one `frame`, after which the driver goes silent: the time it began. hw/status shows
        no emergency while the driver is silent, only the HIGH_LEVEL latch at its end, so this returns after the
        silence with that time less `silence`."""
        answer = ask_silence()
        if answer is None or not answer.success:
            self.finding(step, f"{case} run {n}: hw/silence refused: {answer.code if answer else 'no answer'}")
            raise Abort()
        sent = time.monotonic()
        frame()
        self.wait(HOLD_ARM_S + silence + 1.0, until=self.in_emergency)
        s, latched = self.status(), self.mower.emergency_since
        if not self.in_emergency() or latched is None or latched < sent:
            self.finding(step, f"{case} run {n}: no latch at the end of a silence after the frame")
            raise Abort()
        if s.emergency_reason != HOLD_END_REASON:
            self.finding(step, f"{case} run {n}: emergency {reason_text(s.emergency_reason)} instead of the end "
                               "of the silence")
            raise Abort()
        began = latched - silence
        if not sent - HOLD_SLACK_S <= began <= sent + HOLD_ARM_S:
            self.finding(step, f"{case} run {n}: the silence began {began - sent:+.2f} s after the frame")
            raise Abort()
        return began

    def hold_run(self, step, case, silence, n):
        """Wheels at TRON_FIRST_SPEED, then silent right after a setpoint: s from then until they stood."""
        self.mark(step, f"{case} run {n}")
        if self.drive(TRON_FIRST_SPEED, 0.0, HOLD_DRIVE_S, until=self.in_emergency) or not self.wheels_turn():
            self.mower.stop_wheels()
            self.finding(step, f"{case} run {n}: the wheels did not turn, or an emergency came first")
            raise Abort()
        since = self.hold_start(step, case, n, silence, lambda: self.mower.silence(silence),
                                lambda: self.mower.drive(TRON_FIRST_SPEED, 0.0))
        quiet = HOLD_CASES[case][1].get("silence_stop_delay", 0.0)
        stand = self.stand_after(since, silence + quiet + 1.0)
        restarted = None
        if stand is not None and stand < silence and quiet:
            # Heartbeats alone after the silence: does the MCU take up the old setpoint again?
            self.wait(max(0.0, since + silence - time.monotonic()))
            restarted = self.wait(quiet, self.wheels_turn)
        self.rec.write("hold", step=step, case=case, run=n, since=since, stand_s=stand, restarted=restarted)
        if stand is None:
            self.finding(step, f"{case} run {n}: the wheels still turned after the silence and its stops")
            if not self.person.fake:
                self.person.act("the wheels may still turn: press STOP now")
            raise Abort()
        print(f"     the wheels stood {stand:.2f} s after the silence began"
              + ("" if restarted is None else
                 ", turned again with heartbeats alone" if restarted else ", stayed with heartbeats alone"))
        self.wait(max(0.0, since + silence + quiet + 0.5 - time.monotonic()))
        self.no_motion(step, 1.0, "after the silence and its stops")
        return stand

    def hold_blade_run(self, step, case, silence, n):
        """The blade motor turning, then a new speed for it and silent right after that frame: s from then until the
        MCU reported it standing (about once a second)."""
        self.mark(step, f"{case} run {n}")
        first = self.args.blade_percent[0]
        second = first - 10 if first > 20 else first + 10
        self.blade_turning(step, first)
        since = self.hold_start(step, case, n, silence, lambda: self.mower.silence(silence, blade=True),
                                lambda: self.mower.set_blade(True, self.blade_lease_s, second, TRON_BLADE_HEIGHT_MM))
        still = self.blade_stands(step, since, "the silence began", silence + self.blade_stop_s)
        self.rec.write("hold", step=step, case=case, run=n, since=since, stand_s=still)
        self.wait(max(0.0, since + silence + 0.5 - time.monotonic()))
        return still

    def t10(self):
        step = "T10"
        self.blade_setup()
        if not self.ask(f"{step} cutting height with the disc OFF (jacked up, you at STOP and battery, hands away from "
                        "the deck, a mark on it filmed against a ruler): tron_mcu starts again with the blade allowed, "
                        f"for this step only; the motor turns at {HEIGHT_BLADE_PERCENT} % up to "
                        f"{HEIGHT_RUN_S * (1 + len(HEIGHT_WHILE_TURNING)) + BLADE_RUN_S:.0f} s a run, the deck goes "
                        f"between {HEIGHT_LOW_MM} and {TRON_BLADE_HEIGHT_MM} mm, the wheels get no command"):
            return
        self.height_parked(f"{step}.0")
        self.blade_allowed(step, lambda: self.height_runs(step))

    def height_parked(self, step):
        """No motor: tron_mcu starts again with its off frames at HEIGHT_PARKED_MM, then again as shipped."""
        text = (f"tron_mcu starts again with the off frames at {HEIGHT_PARKED_MM} mm instead of "
                f"{TRON_BLADE_HEIGHT_MM} mm, then again as shipped")
        if not self.ask(f"{step} no motor, no wheel command: {text}"):
            return
        self.mark(step, text)
        try:
            for from_mm, to_mm in ((TRON_BLADE_HEIGHT_MM, HEIGHT_PARKED_MM), (HEIGHT_PARKED_MM, TRON_BLADE_HEIGHT_MM)):
                old = self.start.driver_pid
                since = time.monotonic()
                if to_mm == TRON_BLADE_HEIGHT_MM:
                    self.start.restart_driver()
                else:
                    self.start.restart_driver(blade_off_height_mm=to_mm)
                self.wait_driver_back(step, old)
                moved = self.person.number(f"s the deck took from {from_mm} to {to_mm} mm with the blade off "
                                           "(0: it did not move)")
                self.rec.write("height", step=step, way="parked", from_mm=from_mm, to_mm=to_mm, since=since,
                               moved_s=moved)
        finally:
            if self.start.overrides:
                old = self.start.driver_pid
                self.start.restart_driver()
                self.wait_driver_back(step, old)

    def height_runs(self, step):
        text = (f"on at {HEIGHT_LOW_MM} mm (the deck at {TRON_BLADE_HEIGHT_MM} mm), then off, which sends "
                f"{TRON_BLADE_HEIGHT_MM} mm")
        if self.ask(f"{step}.1 blade motor {text}"):
            self.ready(step)
            self.mark(f"{step}.1", text)
            changes = [self.height_change(f"{step}.1", TRON_BLADE_HEIGHT_MM, HEIGHT_LOW_MM)]
            changes.append(self.height_off(f"{step}.1", HEIGHT_LOW_MM))
            self.heights_moved(changes)
        heights = ", ".join(str(h) for h in HEIGHT_WHILE_TURNING)
        text = f"on at {TRON_BLADE_HEIGHT_MM} mm, then {heights} mm while it turns, {HEIGHT_RUN_S:.0f} s each, then off"
        if self.ask(f"{step}.2 blade motor {text}"):
            self.ready(step)
            self.mark(f"{step}.2", text)
            self.blade_turning(f"{step}.2", HEIGHT_BLADE_PERCENT)
            changes, last = [], TRON_BLADE_HEIGHT_MM
            for height in HEIGHT_WHILE_TURNING:
                changes.append(self.height_change(f"{step}.2", last, height))
                last = height
            changes.append(self.height_off(f"{step}.2", last))
            self.heights_moved(changes)

    def height_change(self, step, from_mm, to_mm):
        """The blade on at `to_mm` for HEIGHT_RUN_S (a new on frame), the motor's rpm and status meanwhile. Nothing
        is asked here: a question between two changes would let the lease run out and the blade stop."""
        rpms, faults = [], set()

        def sample():
            s, age, _, _ = self.mower.snapshot()
            if s is not None and age < 0.5 and s.blade_motor.ok:
                rpms.append(abs(s.blade_motor.rpm))
                if s.blade_motor.status < 0:
                    faults.add(s.blade_motor.status)
            return False
        since = time.monotonic()
        self.blade_run(step, HEIGHT_BLADE_PERCENT, HEIGHT_RUN_S, until=sample, height_mm=to_mm)
        turning = [i for i, rpm in enumerate(rpms) if rpm >= self.blade_still_rpm]
        # A motor that turned and then reported standing stopped for the height.
        stopped = bool(turning) and min(rpms[turning[0]:]) < self.blade_still_rpm
        print(f"     {from_mm} -> {to_mm} mm: blade motor {min(rpms, default=None)}..{max(rpms, default=None)} rpm"
              f"{', stopped meanwhile' if stopped else ''}{f', faults {sorted(faults)}' if faults else ''}")
        return dict(step=step, way="on", from_mm=from_mm, to_mm=to_mm, since=since, rpm_min=min(rpms, default=None),
                    rpm_max=max(rpms, default=None), stopped=stopped, faults=sorted(faults))

    def height_off(self, step, from_mm):
        print("     sending hw/blade off")
        since = time.monotonic()
        self.mower.set_blade(False)
        still = self.blade_stands(step, since, "the off request")
        return dict(step=step, way="off", from_mm=from_mm, to_mm=TRON_BLADE_HEIGHT_MM, since=since, still_s=still)

    def heights_moved(self, changes):
        """After the run, with the blade off: how long the deck took for each change (from the video)."""
        for change in changes:
            after = " after the off" if change["way"] == "off" else ""
            change["moved_s"] = self.person.number(f"s the deck took from {change['from_mm']} to {change['to_mm']} "
                                                   f"mm{after} (0: it did not move)")
            self.rec.write("height", **change)

    STEPS = {"T1": t1, "T2": t2, "T3": t3, "T4": t4, "T5": t5, "T6": t6, "T7": t7, "T8": t8, "T9": t9, "T10": t10}

    def run(self, steps):
        """The firmware and the first IMU sample first: another firmware than TRON_FIRMWARE ends the run before any
        step. tron_mcu releases nothing before that sample, and the MCU reports stop from the start until a release."""
        end = time.monotonic() + 20.0
        while time.monotonic() < end and (not all(self.mower.firmware.get(b) for b in TRON_FIRMWARE)
                                          or self.mower.tilt is None):
            time.sleep(0.1)
        self.firmware = dict(self.mower.firmware)
        print(f"Firmware: {self.firmware}")
        self.rec.write("firmware", **self.firmware)
        wrong = {b: self.firmware.get(b, "") for b, v in TRON_FIRMWARE.items() if self.firmware.get(b) != v}
        if wrong:
            self.finding("start", f"firmware {wrong}, this programme is for {TRON_FIRMWARE}")
            return False
        if self.mower.capability_wheel_distance:
            self.mower.wheel_distance = self.mower.capability_wheel_distance
        return super().run(steps)

    def proposal(self):
        """The values for the parameter file of the Tron from T2, as a proposal; nothing is written."""
        stands = [s for way in ("freeze", "kill") for s in self.host_gone.get(way, [])]
        failsafe = failsafe_proposal(stands)
        if failsafe is None or any(holding_problems(self.host_gone.get(way, [])) for way in self.host_gone):
            return None
        return tron_values_text(failsafe, self.firmware, datetime.date.today().isoformat())


def interrupted(*_):
    raise KeyboardInterrupt()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--platform", choices=["nx60i", "tron"], default="nx60i")
    parser.add_argument("--steps", help="comma separated, of M1..M8 (nx60i, default M1-M6) or T1..T10 (tron, default "
                                        "T1-T5)")
    parser.add_argument("--record-dir", default=".", help="directory of the recording")
    parser.add_argument("--max-mb", type=float, default=100.0, help="size limit of the recording")
    parser.add_argument("--max-minutes", type=float, default=120.0, help="no step starts after this time")
    parser.add_argument("--imu-every", type=int, default=1, help="record every n-th IMU sample")
    parser.add_argument("--repeats", type=int, default=5, help="runs per case in M4 and M5, host-gone runs in T2")
    parser.add_argument("--wheel-distance", type=float, default=0.3556, help="nx60i; the Tron's driver reports its own")
    parser.add_argument("--restart-wait", type=float, default=120.0, help="s to wait for a restarted driver")
    parser.add_argument("--blade", action="store_true", help="allow M7, on the Tron T6 and T7 (blade, the disc off)")
    parser.add_argument("--blade-percent",
                        help=f"tron T6: up to {BLADE_PERCENT_COUNT} whole numbers from 1 to 100, comma separated, "
                             f"default {','.join(map(str, TRON_BLADE_PERCENT))}; the lease and the inputs at the first")
    parser.add_argument("--blade-window", help=f"tron T6: whole s without host, {BLADE_WINDOW_RANGE_S[0]} to "
                                               f"{BLADE_WINDOW_RANGE_S[1]}, default {BLADE_GONE_WINDOW_S:g}")
    parser.add_argument("--input-wait", help=f"tron T6 and T7: whole s to act in once the blade motor turns, "
                                             f"{INPUT_WAIT_RANGE_S[0]} to {INPUT_WAIT_RANGE_S[1]}, default "
                                             f"{BLADE_INPUT_WAIT_S:g}")
    parser.add_argument("--lift-sides", help=f"tron T7: sides T7.1 lifts, in this order, each once, comma separated "
                                             f"of {','.join(LIFT_SIDES)}, default {LIFT_SHOWN_SIDE}; the MCU shows "
                                             f"lift only at {LIFT_SHOWN_SIDE}, T7.3 lifts {LIFT_SHOWN_SIDE}")
    parser.add_argument("--hold", help=f"tron T9: cases of {','.join(HOLD_CASES)}, comma separated, each once, default "
                                       f"{','.join(HOLD_DEFAULT)}; B needs --blade")
    parser.add_argument("--ground", action="store_true", help="allow M8 on the ground, 0.3 m/s per wheel")
    parser.add_argument("--fast", action="store_true",
                        help="with --ground: the fast runs of M8 up to 0.5 m/s in stages, in the raised start of the "
                             "measurement image only (speed_limit of measure.launch.py)")
    parser.add_argument("--data-dir", default="/userdata/mowbite",
                        help="tron: data_dir of MowBite OS (hold file, measured values of this Tron)")
    parser.add_argument("--journal-dir", default=os.environ.get("MOWBITE_JOURNAL_DIR", ""),
                        help="tron: journal of tron_mcu, empty: none")
    parser.add_argument("--node-user", help="tron: user of tron_mcu and tron_imu (this programme runs as root)")
    parser.add_argument("--mcu-port", help="tron: port of the MCU instead of the parameter files (rehearsal)")
    parser.add_argument("--imu-port", help="tron: port of the IMU instead of the parameter files (rehearsal)")
    parser.add_argument("--yes", action="store_true", help="answer every question with yes (rehearsal)")
    parser.add_argument("--fake-flags", help="flag file of fake_xcore or fake_tron: the person's actions go there")
    parser.add_argument("--abort-file", help="abort the running step once this file exists (rehearsal)")
    args = parser.parse_args(argv)
    tron = args.platform == "tron"
    programme = TronMeasurement if tron else Measurement
    default = "T1,T2,T3,T4,T5" if tron else "M1,M2,M3,M4,M5,M6"
    steps = [s.strip().upper() for s in (args.steps or default).split(",") if s.strip()]
    unknown = [s for s in steps if s not in programme.STEPS]
    if unknown:
        sys.exit(f"unknown steps for {args.platform}: {unknown}")
    if tron and (args.ground or args.fast):
        sys.exit("--ground and --fast are not for the Tron")
    if args.hold is not None and not (tron and "T9" in steps):
        sys.exit("--hold is for T9 on the Tron")
    try:
        args.hold = HOLD_DEFAULT if args.hold is None else hold_cases(args.hold)
    except ValueError as error:
        sys.exit(f"--hold: {error}")
    blade_steps = tron and bool({"T6", "T7", "T10"} & set(steps) or ("T9" in steps and "B" in args.hold))
    if tron and args.blade != blade_steps:
        sys.exit("on the Tron --blade and T6, T7, T10 or T9 with B go together")
    if (args.blade_percent is not None or args.blade_window is not None) and not (tron and "T6" in steps):
        sys.exit("--blade-percent and --blade-window are for T6 on the Tron")
    try:
        args.blade_percent = TRON_BLADE_PERCENT if args.blade_percent is None else blade_percents(args.blade_percent)
    except ValueError as error:
        sys.exit(f"--blade-percent: {error}")
    if args.input_wait is not None and not blade_steps:
        sys.exit("--input-wait is for T6 and T7 on the Tron")
    if args.lift_sides is not None and not (tron and "T7" in steps):
        sys.exit("--lift-sides is for T7 on the Tron")
    try:
        args.lift_sides = (LIFT_SHOWN_SIDE,) if args.lift_sides is None else lift_sides(args.lift_sides)
    except ValueError as error:
        sys.exit(f"--lift-sides: {error}")
    for option, default, bounds in (("blade_window", BLADE_GONE_WINDOW_S, BLADE_WINDOW_RANGE_S),
                                    ("input_wait", BLADE_INPUT_WAIT_S, INPUT_WAIT_RANGE_S)):
        text = getattr(args, option)
        try:
            setattr(args, option, default if text is None else whole_seconds(text, bounds))
        except ValueError as error:
            sys.exit(f"--{option.replace('_', '-')}: {error}")
    if "M8" in steps and not args.ground:
        sys.exit("M8 needs --ground")
    if args.fast and not args.ground:
        sys.exit("--fast needs --ground")
    os.makedirs(args.record_dir, exist_ok=True)
    path = os.path.join(args.record_dir, time.strftime("measure-%Y%m%d-%H%M%S.jsonl"))
    rec = Recorder(path, int(args.max_mb * 1024 * 1024), args.max_minutes * 60, max(1, args.imu_every))
    rec.write("start", steps=steps, argv=sys.argv[1:])
    start = TronStart(args, rec) if tron else None
    if start is not None:
        busy = {port: port_holders(port) for port in start.ports.values()}
        if any(busy.values()):
            rec.write("end", complete=False, findings=[f"ports in use: {busy}"])
            rec.close()
            sys.exit(f"ports in use, nothing started: {busy}")
    print(f"Recording to {path} (at most {args.max_mb:g} MB, {args.max_minutes:g} min); wheels at most "
          f"{AIR_WHEEL_SPEED} m/s in the air" + ("" if tron else f", {GROUND_WHEEL_SPEED} m/s in M8")
          + "; any key aborts a running step.")
    keys = Keys(args.abort_file)
    rclpy.init()
    mower = Mower(rec, AIR_WHEEL_SPEED, args.wheel_distance)
    person = Person(keys, args.fake_flags, args.yes)
    measurement = TronMeasurement(args, rec, mower, person, keys, start) if tron else Measurement(args, rec, mower,
                                                                                                     person, keys)
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    complete = False
    try:
        if start is not None:
            if not person.ask("T0 start tron_imu and tron_mcu alone: heartbeat, IMU, the blade off at 90 mm (the "
                              "height motor may move), no wheel command"):
                raise Quit()
            start.start("tron_imu")
            start.start_driver()
        complete = measurement.run(steps)
    except Quit:
        pass
    except KeyboardInterrupt:
        measurement.safe_stop("interrupted")
    finally:
        try:
            mower.stop_wheels()
            if measurement.blade_used:
                mower.set_blade(False)
        finally:
            if start is not None:
                start.close()
        proposal = measurement.proposal() if tron else None
        rec.write("end", complete=complete, findings=measurement.findings, proposal=proposal)
        keys.restore()
        mower.close()
        rclpy.shutdown()
        rec.close()
    print(f"\nRecording: {path}" + (" (limit reached)" if rec.full else ""))
    for f in measurement.findings:
        print(f"Finding {f}")
    if proposal:
        print(f"\nProposal for {os.path.join(args.data_dir, 'tron.yaml')} (nothing written; only on the user's word "
              f"after T9):\n{proposal}")
    return 0 if complete and not measurement.findings else 1


if __name__ == "__main__":
    sys.exit(main())
