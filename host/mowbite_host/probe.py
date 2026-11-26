"""What mowbite-host asks the OS over the broker: system.hold for the updater, and whether a started OS answers healthy.

Answers over the broker prove nothing about the gate (no login); they only tell whether the OS works.
"""
import json
import secrets
import threading
import time

# Bits of emergency_reason that fail an update when they were off before it and still stand STABLE s after the first
# healthy answer (rule M5): TIMEOUT_INPUTS, SERVICE_NOT_READY, FIRMWARE_INCOMPATIBLE.
FAIL_BITS = (1 << 1) | (1 << 8) | (1 << 12)
# Without hardware no driver starts and these stand always: they fail nothing then.
DRIVER_BITS = (1 << 1) | (1 << 12)
# robot_state older than this does not count, an answer later than this is none.
STATE_FRESH = 5.0
ANSWER_WAIT = 3.0
PLANNER_TRIES = 3


class OsRpc:
    """Calls into the OS over broker (broker.Broker); answers arrive through on_answer."""

    def __init__(self, broker):
        self.broker = broker
        self._lock = threading.Lock()
        self._waiting = {}

    def _send(self, method, params=None):
        call_id = "host-" + secrets.token_hex(6)
        request = {"jsonrpc": "2.0", "id": call_id, "method": method}
        if params is not None:
            request["params"] = params
        return call_id, self.broker.request(json.dumps(request))

    # The result of method, None without an answer in time or with an error.
    def call(self, method, params=None, wait=ANSWER_WAIT):
        done = threading.Event()
        box = {}
        with self._lock:
            call_id, sent = self._send(method, params)
            if not sent:
                return None
            self._waiting[call_id] = (done, box)
        try:
            done.wait(wait)
        finally:
            with self._lock:
                self._waiting.pop(call_id, None)
        return box.get("result")

    def on_answer(self, payload):
        try:
            answer = json.loads(payload)
        except (ValueError, RecursionError):
            return
        if not isinstance(answer, dict):
            return
        with self._lock:
            waiting = self._waiting.get(answer.get("id"))
        if waiting:
            waiting[1]["result"] = answer.get("result")
            waiting[0].set()

    # The answer of the OS, None without one in time; the gate reads hold.json, the answer only tells why not.
    def request_hold(self, seconds):
        return self.call("system.hold", {"seconds": int(seconds)})

    def release_hold(self):
        self._send("system.hold", {"seconds": 0})


class Probe:
    """Health over the broker for installer.Health: robot_state comes, rpc.methods answers with system.hold, system.info
    with the api the bundle names; settled() at the end of the stable time judges the bits, planner() the planner.
    hardware() tells whether the OS starts its drivers (MOWBITE_HARDWARE)."""

    def __init__(self, rpc, api_of_running=lambda: None, hardware=lambda: True):
        self.rpc, self.api_of_running, self.hardware = rpc, api_of_running, hardware
        self._lock = threading.Lock()
        self._state = None
        self._state_t = None
        self._bits_before = 0

    def on_robot_state(self, payload):
        try:
            state = json.loads(payload)
        except (ValueError, RecursionError):
            return
        if isinstance(state, dict):
            with self._lock:
                self._state, self._state_t = state, time.time()

    def bits(self):
        with self._lock:
            reason = (self._state or {}).get("emergency_reason")
        return reason if isinstance(reason, int) else 0

    # Before the switch: the bits that stand now do not fail it.
    def begin(self):
        with self._lock:
            fresh = self._state_t is not None and time.time() - self._state_t < STATE_FRESH
        self._bits_before = self.bits() if fresh else 0

    # Whether the OS that started at wall time since answers; a robot_state from before is the old one.
    def __call__(self, since):
        with self._lock:
            state, state_t = self._state, self._state_t
        if state_t is None or state_t < since or time.time() - state_t > STATE_FRESH:
            return False, "no robot_state from the new OS"
        methods = self.rpc.call("rpc.methods")
        if not isinstance(methods, list) or "system.hold" not in methods:
            return False, "rpc.methods does not answer with system.hold"
        info = self.rpc.call("system.info")
        if not isinstance(info, dict):
            return False, "system.info does not answer"
        api = self.api_of_running()
        if api is not None and info.get("api") != api:
            return False, f"system.info says api {info.get('api')}, the bundle {api}"
        return True, f"api {info.get('api')}, {state.get('current_state')}"

    # For a switch of the planner: whether the planner answers through the OS.
    def planner(self):
        for _ in range(PLANNER_TRIES):
            if isinstance(self.rpc.call("planner.settings"), dict):
                return True, ""
        return False, "planner.settings does not answer"

    def settled(self):
        judged = FAIL_BITS if self.hardware() else FAIL_BITS & ~DRIVER_BITS
        new = self.bits() & judged & ~self._bits_before
        if new:
            names = [name for bit, name in ((1, "TIMEOUT_INPUTS"), (8, "SERVICE_NOT_READY"),
                                            (12, "FIRMWARE_INCOMPATIBLE")) if new & (1 << bit)]
            return False, "emergency since the update: " + ", ".join(names)
        return True, ""
