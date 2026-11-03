#!/usr/bin/env python3
"""Safety measurement of a mower with xcore_driver (programme M1-M8 of the NX60i), next to the minimal launch.

Asks before every step and shows what it will send. In the air the wheels get at most 0.1 m/s each, on the ground in
M8 (only with --ground) 0.3 m/s. Any key while a step runs aborts it: three stops on hw/cmd_vel, the blade off, the emergency latched
over hw/emergency. Records hw/status, hw/wheel_odom, imu/data, hw/capabilities and its own commands as JSON lines with a
size and a time limit. The capture of the inner network (tcpdump on the host) is the second record; the report on both
comes from xcore_capture.py.

With --fake-flags the actions of the person (press STOP, lift, tilt, turn a wheel) go to fake_xcore's flag file
instead, for the rehearsal against the simulated board.
"""
import argparse
import json
import math
import os
import select
import signal
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from mowbite_msgs.msg import HardwareCapabilities, HardwareStatus
from mowbite_msgs.srv import SetBlade, SetEmergency
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu

AIR_WHEEL_SPEED = 0.1
GROUND_WHEEL_SPEED = 0.3
REASONS = {1: "LATCH", 2: "TIMEOUT_INPUTS", 4: "STOP", 8: "LIFT", 16: "LIFT_MULTIPLE", 32: "COLLISION",
           64: "TIMEOUT_HIGH_LEVEL", 128: "HIGH_LEVEL", 256: "SERVICE_NOT_READY", 512: "COLLISION_MULTIPLE",
           4096: "FIRMWARE_INCOMPATIBLE", 8192: "TILT", 16384: "FENCE", 32768: "STUCK", 65536: "IMU_TIMEOUT",
           131072: "MOWER_NOT_STOPPED"}
DRIVER = "xcore_driver_node"
SERVICE_NOT_READY = 256


class Abort(Exception):
    pass


class Quit(Exception):
    pass


def reason_text(bits):
    names = [name for bit, name in REASONS.items() if bits & bit]
    return "|".join(names) if names else "0"


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
        self.node = rclpy.create_node("yf_measure")
        self.node.create_subscription(HardwareStatus, "hw/status", self._status, 10)
        self.node.create_subscription(TwistStamped, "hw/wheel_odom", self._odom, 20)
        self.node.create_subscription(Imu, "imu/data", self._imu, qos_profile_sensor_data)
        self.node.create_subscription(HardwareCapabilities, "hw/capabilities", self._capabilities,
                                      QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1))
        self.cmd_vel = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        self.emergency = self.node.create_client(SetEmergency, "hw/emergency")
        self.blade = self.node.create_client(SetBlade, "hw/blade")
        self.running = True
        self.spinner = threading.Thread(target=self._spin, daemon=True)
        self.spinner.start()

    def _spin(self):
        while self.running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _status(self, msg):
        with self.lock:
            self.status, self.status_at = msg, time.monotonic()
        self.rec.write("status", emergency=msg.emergency, latched=msg.emergency_latched,
                       reason=msg.emergency_reason, telemetry_ok=msg.telemetry_ok, stop=msg.stop, lift=msg.lift,
                       bumper=msg.bumper, moving=msg.moving, blade_on=msg.blade_on, blade_rpm=msg.blade_motor.rpm,
                       blade_off_reason=msg.blade_off_reason, battery=msg.battery_voltage)

    def _odom(self, msg):
        with self.lock:
            self.odom, self.odom_at = (msg.twist.linear.x, msg.twist.angular.z), time.monotonic()
        self.rec.write("odom", v=msg.twist.linear.x, w=msg.twist.angular.z)

    def _imu(self, msg):
        acc = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        rate = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        tilt = math.degrees(math.atan2(math.hypot(acc[0], acc[1]), acc[2]))
        with self.lock:
            self.max_tilt = max(self.max_tilt, tilt)
        self.rec.imu(a=list(acc), g=list(rate))

    def _capabilities(self, msg):
        self.rec.write("capabilities", firmware={kv.key: kv.value for kv in msg.firmware})

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
        v, w = self.clamp(v, w)
        self.cmd_vel.publish(Twist(linear=Vector3(x=v), angular=Vector3(z=w)))
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
        self.rec.write("release", success=bool(answer and answer.success), message=answer.message if answer else "")
        return answer

    def set_blade(self, on, lease_s=2.0):
        answer = self.call(self.blade, SetBlade.Request(on=on, lease_s=lease_s if on else 0.0))
        self.rec.write("blade", on=on, lease_s=lease_s, success=bool(answer and answer.success),
                       message=answer.message if answer else "")
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
        self.flags = {}
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
        return True

    def act_later(self, text, fake=None, after=1.0, hold=1.0):
        """The person acts while a step runs; in a rehearsal the flags go up after `after` s for `hold` s."""
        print(f"  -> {text}")
        if not self.fake or fake is None:
            return

        def run():
            time.sleep(after)
            self.set(fake)
            time.sleep(hold)
            self.clear(fake)
        timer = threading.Thread(target=run, daemon=True)
        timer.start()
        self.timers.append(timer)

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
        if answer is None or not answer.success:
            print(f"     release refused: {answer.message if answer else 'no answer'}")
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
            still_since = None if self.wheels_turn(0.005) else (still_since or now)
            if still_since is not None and now - still_since >= 0.5:
                return
            time.sleep(0.02)
        self.finding(step, "the wheels do not stand still before the step")
        raise Abort()

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
        items = [("forward 0.05 m/s for 3 s: do both wheels turn forward?", 0.05, 0.0, 3.0, True),
                 ("turn left: left wheel back, right wheel forward, 0.05 m/s each, 3 s", 0.0, 0.1 / d, 3.0, True)]
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
            if mode == "link" and self.person.fake:
                continue
            for n in range(self.args.repeats):
                if not self.ask(f"{step} failsafe without host, run {n + 1}/{self.args.repeats}: wheels at "
                                       f"0.1 m/s, then the driver {text}; no command afterwards"):
                    break
                self.ready(step)
                self.mark(step, f"{text}, run {n + 1}")
                self.drive(0.1, 0.0, 2.0, until=self.in_emergency)
                if mode == "freeze":
                    pid = self.signal_driver(step, signal.SIGSTOP)
                    try:
                        self.wait(3.0)
                    finally:
                        os.kill(pid, signal.SIGCONT)
                        self.rec.write("signal", signal="SIGCONT", pid=pid)
                    self.wait(1.0)
                elif mode in ("kill", "sigint"):
                    pid = self.signal_driver(step, signal.SIGKILL if mode == "kill" else signal.SIGINT)
                    self.wait(3.0)
                    self.wait_for_driver(step, pid, self.args.restart_wait)
                else:
                    self.person.act("on the CM4: sudo ip link set eth0 down; sleep 3; sudo ip link set eth0 up")
                    self.wait(3.0)
                self.no_motion(step, 3.0, "after the driver came back without a new command")
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
                finally:
                    os.kill(pid, signal.SIGCONT)
                    self.rec.write("signal", signal="SIGCONT", pid=pid)
            elif how == "kill":
                pid = self.signal_driver(step, signal.SIGKILL)
                self.wait(3.0)
                self.wait_for_driver(step, pid, self.args.restart_wait)
            self.wait(6.0)
            self.show("after")
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
        items = [("0.3 m/s straight, press STOP top after about 2 s", 0.3, 0.0, {"stop": True}, None),
                 ("0.3 m/s straight for 3 s, then no command (driver stops)", 0.3, 0.0, None, "quiet"),
                 ("0.3 m/s straight for 2 s, then the driver killed", 0.3, 0.0, None, "kill"),
                 ("0.1 m/s backward against a soft obstacle for up to 10 s", -0.1, 0.0, {"bumper": True}, None),
                 ("turn on the spot at 0.3 rad/s for 5 s", 0.0, 0.3, None, "quiet")]
        for text, v, w, fake, how in items:
            if not self.ask(f"{step} on the ground: {text}"):
                continue
            self.ready(step)
            self.mark(step, text)
            if fake is not None:
                self.person.act_later(text, fake, after=2.0, hold=1.0)
                if not self.drive(v, w, 10.0, until=self.in_emergency):
                    self.finding(step, f"no emergency: {text}")
                    self.mower.stop_wheels()
            elif how == "kill":
                self.drive(v, w, 2.0, until=self.in_emergency)
                pid = self.signal_driver(step, signal.SIGKILL)
                self.wait(3.0)
                self.wait_for_driver(step, pid, self.args.restart_wait)
            else:
                self.drive(v, w, 3.0 if w == 0 else 5.0, until=self.in_emergency)
            self.wait(2.0)
            self.no_motion(step, 1.0, "after the stop")
            if not self.person.fake and fake is not None:
                self.person.act("let go")
            self.release(step)

    STEPS = {"M1": m1, "M2": m2, "M3": m3, "M4": m4, "M5": m5, "M6": m6, "M7": m7, "M8": m8}

    def run(self, steps):
        for name in steps:
            self.mower.wheel_limit = GROUND_WHEEL_SPEED if name == "M8" else AIR_WHEEL_SPEED
            print(f"\n== {name}, wheels at most {self.mower.wheel_limit} m/s ==")
            try:
                self.STEPS[name](self)
            except Abort:
                self.safe_stop("key or failed check")
                if self.person.assume_yes or not self.person.ask("continue with the next step?"):
                    return False
            except Quit:
                return True
        return True


def interrupted(*_):
    raise KeyboardInterrupt()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--steps", default="M1,M2,M3,M4,M5,M6", help="comma separated, of M1..M8")
    parser.add_argument("--record-dir", default=".", help="directory of the recording")
    parser.add_argument("--max-mb", type=float, default=100.0, help="size limit of the recording")
    parser.add_argument("--max-minutes", type=float, default=120.0, help="no step starts after this time")
    parser.add_argument("--imu-every", type=int, default=1, help="record every n-th IMU sample")
    parser.add_argument("--repeats", type=int, default=5, help="runs per case in M4 and M5")
    parser.add_argument("--wheel-distance", type=float, default=0.3556)
    parser.add_argument("--restart-wait", type=float, default=120.0, help="s to wait for a restarted driver")
    parser.add_argument("--blade", action="store_true", help="allow M7 (blade)")
    parser.add_argument("--ground", action="store_true", help="allow M8 on the ground, 0.3 m/s per wheel")
    parser.add_argument("--yes", action="store_true", help="answer every question with yes (rehearsal)")
    parser.add_argument("--fake-flags", help="flag file of fake_xcore: the person's actions go there")
    parser.add_argument("--abort-file", help="abort the running step once this file exists (rehearsal)")
    args = parser.parse_args(argv)
    steps = [s.strip().upper() for s in args.steps.split(",") if s.strip()]
    unknown = [s for s in steps if s not in Measurement.STEPS]
    if unknown:
        sys.exit(f"unknown steps: {unknown}")
    if "M8" in steps and not args.ground:
        sys.exit("M8 needs --ground")
    os.makedirs(args.record_dir, exist_ok=True)
    path = os.path.join(args.record_dir, time.strftime("measure-%Y%m%d-%H%M%S.jsonl"))
    rec = Recorder(path, int(args.max_mb * 1024 * 1024), args.max_minutes * 60, max(1, args.imu_every))
    rec.write("start", steps=steps, argv=sys.argv[1:])
    print(f"Recording to {path} (at most {args.max_mb:g} MB, {args.max_minutes:g} min); wheels at most "
          f"{AIR_WHEEL_SPEED} m/s in the air, {GROUND_WHEEL_SPEED} m/s in M8; any key aborts a running step.")
    keys = Keys(args.abort_file)
    rclpy.init()
    mower = Mower(rec, AIR_WHEEL_SPEED, args.wheel_distance)
    person = Person(keys, args.fake_flags, args.yes)
    measurement = Measurement(args, rec, mower, person, keys)
    signal.signal(signal.SIGTERM, interrupted)
    complete = False
    try:
        complete = measurement.run(steps)
    except KeyboardInterrupt:
        measurement.safe_stop("interrupted")
    finally:
        mower.stop_wheels()
        if measurement.blade_used:
            mower.set_blade(False)
        rec.write("end", complete=complete, findings=measurement.findings)
        keys.restore()
        mower.close()
        rclpy.shutdown()
        rec.close()
    print(f"\nRecording: {path}" + (" (limit reached)" if rec.full else ""))
    for f in measurement.findings:
        print(f"Finding {f}")
    return 0 if complete and not measurement.findings else 1


if __name__ == "__main__":
    sys.exit(main())
