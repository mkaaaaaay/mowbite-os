"""Shared parts of the launch tests of the simulation: the stack, a probe node beside it, MQTT and RPC, the files of
the simulated boards.

A test imports it after `sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))`.
"""
import bisect
import ctypes
import json
import math
import os
import random
import signal
import subprocess
import sys
import threading
import time
import unittest

import launch
import launch_testing.actions
import launch_testing.asserts
import paho.mqtt.client as mqtt
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, Vector3
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.events.process import ProcessStarted
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from mowbite_msgs.msg import HardwareCapabilities, HardwareStatus, RobotPose, RpcRequest
from mowbite_msgs.srv import SetBlade, SetEmergency
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# layers.py of mowbite_bringup: the parameter files of a platform as its start loads them.
sys.path.insert(0, os.path.join(get_package_share_directory("mowbite_bringup"), "launch"))
import layers  # noqa: E402

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
# Latched topics that keep a history (methods, actions, params), and /rosout from the start of the stack.
HISTORY = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
ROSOUT = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
# A node stopped by launch ends with 0 or by its signal (SIGINT, SIGTERM; negative or 128 + n).
EXIT_CODES = [0, -2, -15, 130, 143]

LIBC = ctypes.CDLL(None, use_errno=True)
LIBC.ptrace.argtypes = [ctypes.c_long, ctypes.c_long, ctypes.c_void_p, ctypes.c_void_p]
PTRACE_DETACH = 0x11
PTRACE_SEIZE = 0x4206
PTRACE_INTERRUPT = 0x4207
# waitpid: also for threads that are not children (tracees).
WALL = 0x40000000


def stop_thread(pid, name):
    """Stops the thread of pid with that name (the watch names it after its node) and only it."""
    for tid in os.listdir(f"/proc/{pid}/task"):
        with open(f"/proc/{pid}/task/{tid}/comm") as f:
            if f.read().strip() != name:
                continue
        for request in (PTRACE_SEIZE, PTRACE_INTERRUPT):
            if LIBC.ptrace(request, int(tid), None, None) != 0:
                raise OSError(ctypes.get_errno(), f"ptrace {request:#x} of {name} ({tid})")
        return int(tid)
    raise LookupError(f"no thread {name} in {pid}")


def stall_thread(pid, name, seconds):
    """Holds the thread of pid with that name for seconds, as a long write would, then lets it go on."""
    tid = stop_thread(pid, name)
    try:
        os.waitpid(tid, WALL)
        time.sleep(seconds)
    finally:
        if LIBC.ptrace(PTRACE_DETACH, tid, None, None) != 0:
            raise OSError(ctypes.get_errno(), f"ptrace detach of {name} ({tid})")


def thread_owner(name):
    """The pid of the component container that has a thread with that name; None without one."""
    for pid in filter(str.isdigit, os.listdir("/proc")):
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                if b"component_container_isolated" not in f.read():
                    continue
            for tid in os.listdir(f"/proc/{pid}/task"):
                with open(f"/proc/{pid}/task/{tid}/comm") as f:
                    if f.read().strip() == name:
                        return int(pid)
        except OSError:
            continue
    return None
# Modules of the Tron MCU as fake_tron numbers them.
TRON_SPEED, TRON_CUTTER = 4, 11
# Services and message types of the xCore as fake_xcore numbers them.
XCORE_EMERGENCY, XCORE_DIFF_DRIVE, XCORE_MOWER = 1, 2, 3
XCORE_DATA, XCORE_CLAIM = 1, 3


def follower_param(name):
    """A parameter of mowbite_follower on the Tron."""
    return layers.parameters(layers.files("tron"), "mowbite_follower")[name]


def follower_max_linear():
    """max_linear (m/s) of mowbite_follower on the Tron: no drive command of a sim goes faster."""
    return float(follower_param("max_linear"))


def tron_geometry(name):
    """A value of the geometry of the Tron (geometry.yaml)."""
    with open(layers.geometry("tron")) as f:
        return yaml.safe_load(f)["mowbite_logic"]["ros__parameters"]["geometry"][name]


# fake_tron flags of a real Tron: RTK at 10 Hz with 2 mm per fix (measured 15.11. at rest: 1.0 mm east, 1.7 mm north,
# no drift to be seen up to 95 s) and 3 mm drifting over 60 s for multipath (assumed), the fix 0.1 s late, wheels report
# 6 % too little, a wheel below 0.05 m/s stands, the base turns on 0.395 m while the MCU counts 0.40 m, 0.01 rad of yaw
# per metre from slip.
REAL_FAULTS = {"gnss_noise": 0.002, "gnss_drift": 0.003, "gnss_noise_tau": 60.0, "gnss_period": 0.1, "gnss_delay": 0.1,
               "odom_scale": 0.94, "wheel_deadband": 0.05, "track": 0.395, "yaw_drift": 0.01}


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=EXIT_CODES)


# --- the stack


def journal_dir(work):
    """The journal of ctest (MOWBITE_JOURNAL_DIR), else one in work."""
    return os.environ.get("MOWBITE_JOURNAL_DIR") or os.path.join(work, "journal")


def sim_files(work):
    """Where sim_stack puts the files of fake_tron: (state, flags, rx log, data dir)."""
    return (os.path.join(work, "state.json"), os.path.join(work, "flags.json"), os.path.join(work, "rx.jsonl"),
            os.path.join(work, "data"))


def schedule_allowed(work):
    """sim.yaml with the schedule allowed for this platform, written into work, for extra_config of sim_stack."""
    path = os.path.join(work, "schedule.yaml")
    with open(os.path.join(get_package_share_directory("mowbite_sim"), "config", "sim.yaml")) as f:
        text = f.read()
    with open(path, "w") as f:
        f.write(text + "\nmowbite_schedule:\n  ros__parameters:\n    allowed: true\n")
    return path


def without_sunset(data_dir, **settings):
    """Settings of params.set in data_dir with dock_at_sunset 0 and the given ones: a run in the simulation keeps mowing
    past the real sunset at its datum, which falls into working hours in winter."""
    os.makedirs(os.path.join(data_dir, "params"), exist_ok=True)
    write_json(os.path.join(data_dir, "params", "mower_logic.json"), {"dock_at_sunset": 0, **settings})


def sim_stack(work, mqtt_port, sunset=False, settings=None, **arguments):
    """sim.launch.py with its links and files in work; arguments add to or replace the launch arguments. Without sunset
    the run ignores the sunset (without_sunset), with settings of params.set besides."""
    state, flags, log, data = sim_files(work)
    values = {"link": os.path.join(work, "mcu"), "gnss_link": os.path.join(work, "gnss"),
              "imu_link": os.path.join(work, "imu"), "state": state, "flags": flags, "log": log, "data_dir": data,
              "mqtt_port": str(mqtt_port), **arguments}
    if not sunset:
        without_sunset(values["data_dir"], **(settings or {}))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("mowbite_sim").find("mowbite_sim"), "launch", "sim.launch.py")),
        launch_arguments=values.items())


def sim_description(work, mqtt_port, **arguments):
    """The return value of generate_test_description for the stack of sim.launch.py alone."""
    return launch.LaunchDescription([sim_stack(work, mqtt_port, **arguments),
                                     launch_testing.actions.ReadyToTest()]), {}


def package_program(package, name):
    """Path of a program a package installs in its lib directory."""
    return os.path.join(FindPackagePrefix(package).find(package), "lib", package, name)


def sim_program(name):
    """Path of a program of mowbite_sim (fake_tron, fake_xcore, ...)."""
    return package_program("mowbite_sim", name)


def bridge_node(work, port):
    """mowbite_bridge alone on the mosquitto at port, its data in work/bridge."""
    return Node(package="mowbite_bridge", executable="mowbite_bridge_node", output="screen",
                parameters=[{"mqtt_host": "127.0.0.1", "mqtt_port": port, "data_dir": os.path.join(work, "bridge")}])


def sim_links(work):
    """The ptys of fake_tron in work, by name."""
    return {name: os.path.join(work, name) for name in ("mcu", "gnss", "imu")}


def antenna_height():
    """antenna_height (m) of the Tron in its geometry.yaml."""
    with open(layers.geometry("tron")) as f:
        return float(yaml.safe_load(f)["mowbite_localization"]["ros__parameters"]["antenna_height"])


def fake_tron(links, *args):
    """fake_tron on the ptys of sim_links with the antenna as high as on the Tron; args add its other options."""
    return ExecuteProcess(cmd=[sim_program("fake_tron"), "--link", links["mcu"], "--gnss-link", links["gnss"],
                               "--imu-link", links["imu"], "--antenna-height", str(antenna_height()), *args],
                          output="screen")


def loopback_group():
    """A group port and a driver address of their own, so that tests side by side neither hear each other's board nor
    meet another host at the bind_ip: (port, ip)."""
    port = random.randint(42000, 48000)
    ip = f"127.{random.randint(1, 250)}.{random.randint(1, 250)}.{random.randint(2, 250)}"
    return port, ip


def fake_xcore(group_port, *args):
    """fake_xcore with the services of the nx60i on that group port; args add its other options."""
    services = os.path.join(FindPackageShare("mowbite_sim").find("mowbite_sim"), "config", "xcore_nx60i_services.json")
    return ExecuteProcess(cmd=[sim_program("fake_xcore"), "--services", services, "--group-port", str(group_port),
                               *args], output="screen")


def xcore_battery():
    """The battery voltages of the logic on the NX60i as its platform.py hands them to the driver (logic_params.py of
    xcore_driver)."""
    launch_dir = os.path.join(get_package_share_directory("xcore_driver"), "launch")
    if launch_dir not in sys.path:
        sys.path.insert(0, launch_dir)
    import logic_params
    return logic_params.battery_voltages(layers.parameters(layers.files("nx60i"), "mowbite_logic"))


def xcore_config():
    """nx60i.yaml of xcore_driver."""
    return os.path.join(get_package_share_directory("xcore_driver"), "config", "nx60i.yaml")


def xcore_driver(bind_ip, group_port, config=None, node=None, **params):
    """xcore_driver on the loopback with config (its nx60i.yaml by default), the battery voltages of the nx60i and
    params; node adds arguments of the Node (name, namespace)."""
    if config is None:
        config = xcore_config()
    return Node(package="xcore_driver", executable="xcore_driver_node", output="screen", **(node or {}),
                parameters=[config, xcore_battery(),
                            {"interface": "lo", "bind_ip": bind_ip, "announcement_port": group_port, **params}])


def stop_process(proc, sig=signal.SIGTERM, group=False):
    """Stops a process (with group its whole process group) by sig, after 10 s by SIGKILL; its exit code."""
    def send(s):
        if group:
            os.killpg(proc.pid, s)
        else:
            proc.send_signal(s)
    send(sig)
    try:
        proc.wait(10)
    except subprocess.TimeoutExpired:
        send(signal.SIGKILL)
        proc.wait(5)
    return proc.returncode


def pids(proc_info, name):
    """Pids of the processes with that name that started, oldest first."""
    out = []
    for action in proc_info.processes():
        event = proc_info[action]
        if name in action.process_details["name"] and isinstance(event, ProcessStarted):
            out.append(event.pid)
    return out


def alive(pid):
    try:
        with open(f"/proc/{pid}/status") as f:
            return "State:\tZ" not in f.read()
    except OSError:
        return False


def running(proc_info, name):
    """Pids of the processes with that name that run now."""
    return [pid for pid in pids(proc_info, name) if alive(pid)]


def container_pid(proc_info):
    """The pid of the container that runs now; the start makes a new one after each crash."""
    for action in proc_info.processes():
        event = proc_info[action]
        if "component_container_isolated" in action.process_details["name"] and isinstance(event, ProcessStarted):
            return event.pid
    return None


def restarted(proc_info, pid, timeout):
    """The pid of the container once one runs again after pid, else None."""
    end = time.time() + timeout
    while time.time() < end:
        now = container_pid(proc_info)
        if now is not None and now != pid:
            return now
        time.sleep(0.1)
    return None


def mosquitto(work, port, extra=""):
    """A private mosquitto on 127.0.0.1:port without login; extra lines go into its configuration."""
    conf = os.path.join(work, "mosquitto.conf")
    with open(conf, "w") as f:
        f.write(f"listener {port} 127.0.0.1\nallow_anonymous true\n{extra}")
    return ExecuteProcess(cmd=["mosquitto", "-c", conf], output="screen")


# --- files of the simulated boards


def write_json(path, values):
    """Replaces the file in one step, so the reader never sees half of it."""
    with open(path + ".tmp", "w") as f:
        json.dump(values, f)
    os.replace(path + ".tmp", path)


def read_json(path, tries=1):
    """The JSON in the file, None if it is missing or half written after that many reads 50 ms apart."""
    for attempt in range(tries):
        try:
            with open(path) as f:
                return json.load(f)
        except (OSError, ValueError):
            if attempt + 1 < tries:
                time.sleep(0.05)
    return None


def iter_entries(path, since=0.0):
    """The entries of a JSON lines log of a simulated board from its time since on; none without the file."""
    try:
        with open(path) as f:
            for line in f:
                entry = json.loads(line)
                if entry.get("t", 0) >= since:
                    yield entry
    except OSError:
        return


def log_entries(path, since=0.0):
    """iter_entries as a list."""
    return list(iter_entries(path, since))


def host_packets(path, since=0.0):
    """What the simulated board got from the host (packets and charge frames) since its time since."""
    return [e for e in iter_entries(path, since) if "mod" in e]


def sub_packets(path, since=0.0, module=None):
    """The sub packets fake_tron got from the driver since its time since, of one module if given."""
    return [e for e in iter_entries(path, since) if e.get("type") == 0 and (module is None or e["mod"] == module)]


def xcore_datagrams(log, since=0.0, sid=None, msg_type=None, arg2=None):
    """The datagrams fake_xcore logged since its time since, of that service, message type and arg2 if given."""
    return [e for e in iter_entries(log, since) if "sid" in e and (sid is None or e["sid"] == sid)
            and (msg_type is None or e.get("type") == msg_type) and (arg2 is None or e.get("arg2") == arg2)]


def moving(entry):
    """Whether a speed packet of the log sets a wheel turning."""
    return bool(entry.get("speed")) and any(v != 0 for v in entry["speed"])


def wheel_speeds(path, since, module):
    """The speed packets (of that module) since its time since that move a wheel."""
    return [e for e in sub_packets(path, since, module) if moving(e)]


def last_time(path, size=65536):
    """The time of the newest entry of a JSON lines log of a simulated board, 0 without one; reads only its end."""
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            cut = f.tell() > size
            f.seek(max(0, f.tell() - size))
            lines = f.read().decode().splitlines()
    except OSError:
        return 0.0
    lines = [line for line in lines[1 if cut else 0:] if line.strip()]
    return json.loads(lines[-1])["t"] if lines else 0.0


def recent_entries(path, size=65536):
    """The last entries of a JSON lines log, without reading all of it."""
    with open(path, "rb") as f:
        f.seek(0, os.SEEK_END)
        f.seek(max(0, f.tell() - size))
        lines = f.read().decode().splitlines()[1:]
    return [json.loads(line) for line in lines]


# --- small helpers


def wait_until(getter, timeout, lock=None, poll=0.05):
    """The first true value of getter() within timeout s (under lock if given), else None."""
    end = time.time() + timeout
    while time.time() < end:
        if lock is None:
            value = getter()
        else:
            with lock:
                value = getter()
        if value:
            return value
        time.sleep(poll)
    return None


def inside(ring, x, y):
    result = False
    for i in range(len(ring)):
        (ax, ay), (bx, by) = ring[i], ring[i - 1]
        if (ay > y) != (by > y) and x < (bx - ax) * (y - ay) / (by - ay) + ax:
            result = not result
    return result


def distance_to(ring, x, y):
    """m from (x, y) to the closest edge of the ring."""
    def to_edge(p, q):
        ex, ey = q[0] - p[0], q[1] - p[1]
        along = min(1.0, max(0.0, ((x - p[0]) * ex + (y - p[1]) * ey) / (ex * ex + ey * ey)))
        return math.dist((x, y), (p[0] + along * ex, p[1] + along * ey))
    return min(to_edge(ring[i - 1], ring[i]) for i in range(len(ring)))


def quantile(values, q):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


def crc24q(data):
    """CRC-24Q of RTCM 3."""
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def rtcm_frame(message, length=19, filler=lambda i: (i * 7) & 0xFF):
    """A RTCM 3 frame of that message number with a valid CRC; filler(i) gives the payload bytes after the number."""
    payload = bytes([message >> 4, (message & 0x0F) << 4]) + bytes(filler(i) for i in range(length - 2))
    frame = bytes([0xD3, length >> 8, length & 0xFF]) + payload
    return frame + crc24q(frame).to_bytes(3, "big")


def station_frame():
    """An RTCM 3 frame of message 1005 (reference station), all fields 0: corrections the board counts for RTK fixed."""
    return rtcm_frame(1005, filler=lambda i: 0)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_stamp(msg):
    """The stamp of a pose in s; a stamp far from the arrival is no clock of this host, so the arrival stands in."""
    now = time.time()
    stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
    return now if abs(stamp - now) > 0.5 else stamp


def estimate_at(estimates, t):
    """The pose at time t from (stamp, x, y, yaw) sorted by stamp, linear between the two around it; None outside them
    or across a gap of more than 0.2 s."""
    i = bisect.bisect_left(estimates, (t,))
    if i == 0 or i >= len(estimates):
        return None
    a, b = estimates[i - 1], estimates[i]
    if b[0] - a[0] > 0.2:
        return None
    f = (t - a[0]) / (b[0] - a[0])
    return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * wrap(b[3] - a[3])


# --- MQTT, the side of the app


def mqtt_connect(name, port, on_message, subscribe, timeout=15.0, **kwargs):
    """A client of paho 1 or 2 with the callbacks of paho 1, connected (retried until the broker is up), subscribed to
    the topic (or list of (topic, qos)) and looping in its own thread."""
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, name, **kwargs)
    except AttributeError:
        client = mqtt.Client(name, **kwargs)
    client.on_message = on_message
    end = time.time() + timeout
    while True:
        try:
            client.connect("127.0.0.1", port)
            break
        except OSError:
            if time.time() > end:
                raise
            time.sleep(0.2)
    client.subscribe(subscribe)
    client.loop_start()
    return client


def store_answer(answers, lock, payload):
    """Keeps a JSON-RPC answer from rpc/response by its id; anything else is ignored."""
    try:
        answer = json.loads(payload)
    except ValueError:
        return
    with lock:
        answers[answer.get("id")] = answer


def mqtt_rpc(client, answers, lock, method, params=None, timeout=10.0):
    """Asks over rpc/request as the app does: the answer kept by store_answer, None without one in time."""
    rid = f"{method}-{time.time_ns()}"
    request = {"jsonrpc": "2.0", "method": method, "id": rid}
    if params is not None:
        request["params"] = params
    client.publish("rpc/request", json.dumps(request))
    end = time.time() + timeout
    while time.time() < end:
        with lock:
            if rid in answers:
                return answers.pop(rid)
        time.sleep(0.05)
    return None


class MqttBroker:
    """A client on every topic of the broker of the OS that keeps what comes as (topic, payload) in msgs."""

    def __init__(self, name, port):
        self.msgs = []
        self.client = mqtt_connect(name, port, lambda _c, _u, m: self.msgs.append((m.topic, m.payload)), "#",
                                   timeout=30.0)

    def latest(self, topic):
        for top, payload in reversed(self.msgs):
            if top == topic:
                return payload
        return None

    def wait(self, pred, timeout=10.0):
        return wait_until(pred, timeout)

    def events(self, kind, since):
        """The events of that type on events/json from msgs[since] on."""
        return [json.loads(p) for top, p in self.msgs[since:] if top == "events/json" and json.loads(p)["type"] == kind]

    def rpc(self, method, params=None, rid="r"):
        """The answer to a request on rpc/request, None after 5 s."""
        start = len(self.msgs)
        self.client.publish("rpc/request", json.dumps({"jsonrpc": "2.0", "method": method, "params": params or {},
                                                       "id": rid}))

        def answer():
            for top, p in self.msgs[start:]:
                if top == "rpc/response" and json.loads(p).get("id") == rid:
                    return json.loads(p)
            return None
        return self.wait(answer, 5)

    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()


# --- the probe node beside the stack


class SimProbe:
    """A node of the test, spun in a thread of its own; the callbacks change its values under self.lock.

    A test derives from it, creates its subscriptions, publishers and clients and then calls start(). The class
    attributes set how often it looks and how long it waits."""
    poll = 0.05
    spin_timeout = 0.05
    offer_timeout = 20.0
    call_timeout = 60.0
    truth_tries = 1
    # How often call looks for the answer.
    call_poll = 0.05
    # Flags written with every flags().
    faults = {}
    # The reason set_emergency gives the driver.
    emergency_reason = "test"

    def __init__(self, name, state=None, flags=None, log=None):
        # Re-entrant: a getter of wait() may call a method that takes it again.
        self.lock = threading.RLock()
        self.node = rclpy.create_node(name)
        self.state_file, self.flags_file, self.log_file = state, flags, log
        self.events = []
        self.logs = []
        self.track = []
        self.answers = {}
        self.responses = {}
        self.errors = {}
        self.blade_on = False
        self.map = None
        self.hw = None
        self.methods = {}
        self.actions = {}
        self.params = {}
        self.rpc_count = 0
        self.running = False
        self.spinner = None
        self.threads = []

    def start(self):
        self.running = True
        self.spinner = threading.Thread(target=self._spin, daemon=True)
        self.spinner.start()

    def spawn(self, target):
        """Runs target in a daemon thread of its own until stop()."""
        thread = threading.Thread(target=target, daemon=True)
        thread.start()
        self.threads.append(thread)

    def _spin(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=self.spin_timeout)

    def stop(self):
        self.running = False
        if self.spinner is not None:
            self.spinner.join(2.0)
        for thread in self.threads:
            thread.join(0.5)
        self.node.destroy_node()

    def wait(self, getter, timeout=10.0):
        """The first true value of getter() within timeout s, else None."""
        return wait_until(getter, timeout, self.lock, self.poll)

    def wait_for(self, pred, timeout=10.0):
        """As wait, with the probe as the argument of pred."""
        return wait_until(lambda: pred(self), timeout, self.lock, self.poll)

    def call(self, client, request, timeout=None):
        """The answer of a ROS service; fails the test if it is not offered or does not answer in time."""
        assert client.wait_for_service(timeout_sec=self.offer_timeout), f"{client.srv_name} not offered"
        future = client.call_async(request)
        end = time.time() + (self.call_timeout if timeout is None else timeout)
        while not future.done() and time.time() < end:
            time.sleep(self.call_poll)
        assert future.done(), f"no answer from {client.srv_name}"
        return future.result()

    # callbacks

    def _set(self, name):
        """A callback that keeps the latest message as attribute name."""
        def store(msg):
            with self.lock:
                setattr(self, name, msg)
        return store

    def _publish(self, msg):
        """bridge/publish: keeps the events."""
        if msg.topic != "events/json":
            return
        with self.lock:
            self.events.append(json.loads(msg.payload))

    def _log(self, msg):
        """/rosout: keeps the texts."""
        with self.lock:
            self.logs.append(msg.msg)

    def _map(self, msg):
        with self.lock:
            self.map = json.loads(msg.data)

    def _response(self, msg):
        with self.lock:
            self.answers[msg.id] = {"result": json.loads(msg.result)}

    def _error(self, msg):
        with self.lock:
            self.answers[msg.id] = {"error": {"code": msg.code, "message": msg.message,
                                              "data": json.loads(msg.data) if msg.data else None}}

    def _result(self, msg):
        """bridge/rpc/response for rpc_outcome."""
        with self.lock:
            self.responses[msg.id] = json.loads(msg.result)

    def _failure(self, msg):
        """bridge/rpc/error for rpc_outcome."""
        with self.lock:
            self.errors[msg.id] = (msg.code, msg.message, json.loads(msg.data) if msg.data else None)

    def _hw(self, msg):
        """hw/status: keeps the latest and whether the blade ever ran."""
        with self.lock:
            self.hw = msg
            self.blade_on = self.blade_on or msg.blade_on

    def _methods(self, msg):
        with self.lock:
            self.methods[msg.node_id] = list(msg.methods)

    def _actions(self, msg):
        with self.lock:
            for a in msg.actions:
                self.actions[msg.node_prefix + "/" + a.id] = a.enabled

    def _params(self, msg):
        with self.lock:
            data = json.loads(msg.data)
            for name, value in data["params"].items():
                self.params[data["node"] + "/" + name] = value

    # what the test asks

    def event(self, since, type_, timeout, **match):
        """The first event of that type since index since whose fields match."""
        return self.wait(lambda: next((e for e in self.events[since:] if e["type"] == type_
                                       and all(e.get(k) == v for k, v in match.items())), None), timeout)

    def crash_trigger(self, since, process):
        """The trigger CRASH named after the process in an incident told since index since, else None."""
        for event in [e for e in self.events[since:] if e["type"] == "INCIDENT"]:
            answer = self.rpc("incidents.get", {"id": event["incident"]}, 5) or {}
            for trigger in answer.get("result", {}).get("manifest", {}).get("triggers", []):
                if (trigger["kind"], trigger["reason"]) == ("CRASH", process):
                    return trigger
        return None

    def logged(self, text, timeout=10.0, since=0):
        """Whether a log line with that text came within timeout s, among the lines from index since on."""
        return self.wait(lambda: any(text in line for line in self.logs[since:]), timeout)

    def act(self, action):
        """An action of the app on mower_logic/actions (self.action_pub)."""
        self.wait(lambda: self.action_pub.get_subscription_count() > 0)
        self.action_pub.publish(String(data=action))

    def _send(self, method, params, raw, rid):
        """Hands a command to the nodes as the bridge does (self.rpc_pub) once they listen: raw or params as JSON as
        its params, rid or a new id as its id; the id."""
        self.wait(lambda: self.rpc_pub.get_subscription_count() > 0)
        if rid is None:
            with self.lock:
                self.rpc_count += 1
                rid = f"{self.node.get_name()}-{self.rpc_count}"
        text = raw if raw is not None else ("" if params is None else json.dumps(params))
        self.rpc_pub.publish(RpcRequest(method=method, params=text, id=rid))
        return rid

    def rpc(self, method, params=None, timeout=10.0, raw=None, rid=None):
        """A command as the bridge hands it to the nodes: {"result": ...} or {"error": {...}} as the bridge would pass
        it on, None without an answer in time."""
        rid = self._send(method, params, raw, rid)
        return self.wait(lambda: self.answers.get(rid), timeout)

    def rpc_outcome(self, method, params=None, timeout=5.0, raw=None, rid=None):
        """As rpc, with _result and _failure as the callbacks: ("ok", result) or ("error", (code, message, data))."""
        rid = self._send(method, params, raw, rid)
        return self.wait(lambda: (("ok", self.responses[rid]) if rid in self.responses else None)
                         or (("error", self.errors[rid]) if rid in self.errors else None), timeout)

    # the driver

    def set_blade(self, on, lease_s=0.0, speed=0, height=0):
        """hw/blade (self.blade)."""
        return self.call(self.blade, SetBlade.Request(on=on, lease_s=lease_s, speed_percent=speed, height_mm=height))

    def set_emergency(self, latch, bits=0, latch_count=0):
        """hw/emergency (self.emergency)."""
        return self.call(self.emergency, SetEmergency.Request(latch=latch, bits=bits, reason=self.emergency_reason,
                                                              latch_count=latch_count))

    def cmd_vel_for(self, linear, seconds):
        """cmd_vel (self.cmd_vel) at 10 Hz for that long; the time of the board when it began, if it has a log."""
        began = last_time(self.log_file) if self.log_file else None
        end = time.time() + seconds
        while time.time() < end:
            self.cmd_vel.publish(Twist(linear=Vector3(x=linear)))
            time.sleep(0.1)
        return began

    def _keepalive(self):
        """A healthy driver (self.hw_pub) and a good pose (self.pose_pub) five times a second while running."""
        while self.running:
            hw = HardwareStatus()
            hw.telemetry_ok = True
            hw.battery_source = HardwareStatus.BATTERY_SOURCE_BMS
            hw.battery_voltage = 24.5
            hw.battery_percentage = 80
            self.hw_pub.publish(hw)
            pose = RobotPose()
            pose.position_accuracy = 0.02
            pose.heading_accuracy = 0.05
            pose.heading_valid = True
            self.pose_pub.publish(pose)
            time.sleep(0.2)

    # the simulated mower

    def truth(self):
        """The state file of the simulated board, None if it cannot be read."""
        return read_json(self.state_file, self.truth_tries)

    def flags(self, **values):
        """Writes the flags file: faults, then values."""
        write_json(self.flags_file, {**self.faults, **values})

    def standing(self, tolerance=0.02):
        """Whether the simulated mower moved less than tolerance m within a second, and the two states."""
        a = self.truth()
        time.sleep(1.0)
        b = self.truth()
        return math.hypot(a["x"] - b["x"], a["y"] - b["y"]) < tolerance, (a, b)

    def received(self, since=0.0):
        return host_packets(self.log_file, since)

    def cutter_frames(self):
        """The cutter frames the simulated board got."""
        return [e for e in log_entries(self.log_file) if "cutter" in e]

    def _record(self):
        """Keeps the true pose of the simulated mower in self.track, five times a second while running."""
        while self.running:
            truth = self.truth()
            if truth:
                with self.lock:
                    self.track.append(truth)
            time.sleep(0.2)

    def recent(self, size=65536):
        return recent_entries(self.log_file, size)

    def clock_offset(self, reads=40):
        """time.monotonic() minus the time of fake_tron, from the read that found its newest entry the freshest."""
        offsets = []
        for _ in range(reads):
            now = time.monotonic()
            offsets.append(now - self.recent(4096)[-1]["t"])
            time.sleep(0.01)
        return min(offsets)


class DriverProbe(SimProbe):
    """Short waits for the tests of a driver alone."""
    poll = 0.02
    offer_timeout = 5.0
    call_timeout = 5.0
    call_poll = 0.01


class HwProbe(DriverProbe):
    """A driver probe on hw/status, hw/capabilities, hw/cmd_vel and hw/blade; a test adds its own and calls start()."""

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        self.status = None
        self.capabilities = None
        self.node.create_subscription(HardwareStatus, "hw/status", self._set("status"), 10)
        self.node.create_subscription(HardwareCapabilities, "hw/capabilities", self._set("capabilities"), LATCHED)
        self.cmd_vel = self.node.create_publisher(Twist, "hw/cmd_vel", 10)
        self.blade = self.node.create_client(SetBlade, "hw/blade")
