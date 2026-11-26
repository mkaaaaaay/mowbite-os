"""The service mowbite-host: answers update.* and host.* on the broker of the OS, also while the OS is off, and keeps
mowbite/host/status and mowbite/update/status retained.

Only the methods of METHODS are answered, each request once: the bridge stays silent on the methods the host lists in
its status. A request with a response topic the bridge refuses (not rpc/response/<token>) is left to the bridge.
Refusals are -32600 with data {reason, message}, wrong params -32602 with reason INVALID.
"""
import json
import os
import queue
import re
import threading
import time

from mowbite_host import Refused, command, gate, gesture, ssh, trial, wifi
from mowbite_host.updater import COMPONENTS

SCHEMA = 1
HOST_STATUS = "mowbite/host/status"
UPDATE_STATUS = "mowbite/update/status"
METHODS = ("host.confirm_start", "host.restart_os", "host.ssh", "host.usb_mode", "host.wifi_scan", "host.wifi_set",
           "update.abort", "update.check", "update.rollback", "update.set_auto", "update.set_channel", "update.start",
           "update.status")
OFFLINE = json.dumps({"schema": SCHEMA, "online": False})
TICK_EVERY = 5 * 60.0
CHECK_EVERY = 6 * 3600.0
# How often the state of the OS is read from its files.
LOOK_EVERY = 2.0
# How often the trial period is held while the first installation runs; the tool lets it go after 30 min.
HOLD_EVERY = 60.0
# host.usb_mode answers first and reboots this much later.
REBOOT_AFTER = 2.0
LOADER_REBOOT = ("systemctl", "--no-block", "--reboot-argument=loader", "reboot")
RESPONSE_TOPIC = re.compile(r"rpc/response/[A-Za-z0-9._-]{1,64}")
MAX_CORRELATION = 1024
# Requests beyond this many waiting are dropped.
MAX_WAITING = 64
INVALID_REQUEST = -32600
INVALID_PARAMS = -32602
INTERNAL_ERROR = -32603


CHALLENGE = re.compile(r"[0-9a-f]{32,128}")


class Invalid(Exception):
    pass


def _text(params, key, required=True):
    value = params.get(key)
    if value is None and not required:
        return None
    if not isinstance(value, str) or not value:
        raise Invalid(f"{key} has to be a text")
    return value


def _component(params):
    component = _text(params, "component")
    if component not in COMPONENTS:
        raise Invalid(f"component is one of {', '.join(COMPONENTS)}")
    return component


# reset_jobs: only for the planner, whose update and rollback wait for the end of the mowing job without it.
def _reset_jobs(params):
    reset = params.get("reset_jobs", False)
    if not isinstance(reset, bool) or (reset and params.get("component") != "planner"):
        raise Invalid("reset_jobs has to be true or false, and true only for the planner")
    return reset


class Host:
    """updater: updater.Updater; watch: os_state.OsWatch; carrier: what runs the OS; publish(topic, text, retain,
    correlation=None) sends when connected; run: command.run."""

    def __init__(self, paths, updater, watch, carrier, publish, run=command.run):
        self.paths, self.updater, self.watch, self.carrier, self.publish = paths, updater, watch, carrier, publish
        self.run_command = run
        self.wifi = wifi.Wifi(paths, run)
        self.gesture = gesture.Gesture(paths, run)
        self.ssh = ssh.Ssh(paths, run)
        self._setup_until = None
        self.requests = queue.Queue(maxsize=MAX_WAITING)
        self.rebooting = False
        # The last restart into the USB mode that failed: {t, error}, None after a new try.
        self.usb_mode_failed = None
        self._os = None
        self._shown = None
        self._update_text = None
        self._complaints = {}
        # Monotonic time of the last hold of the trial period, None when this host holds none.
        self._trial_held = None
        # None: the answer is update.status.
        self._methods = {
            "update.status": lambda p: None,
            "update.start": lambda p: self.updater.start(_component(p), _text(p, "revision", required=False),
                                                         reset_jobs=_reset_jobs(p)),
            "update.abort": lambda p: self.updater.abort(),
            "update.rollback": lambda p: self.updater.rollback(_component(p), reset_jobs=_reset_jobs(p)),
            "update.check": lambda p: self.updater.check(fetch=True),
            "update.set_auto": self._set_auto,
            "update.set_channel": lambda p: self.updater.set_channel(_text(p, "channel")),
            "host.restart_os": self._restart_os,
            "host.confirm_start": self._confirm_start,
            "host.wifi_scan": lambda p: self.wifi.scan(),
            "host.wifi_set": lambda p: self.wifi.set(p.get("ssid"), p.get("password"), p.get("hidden", False)),
            "host.usb_mode": self._usb_mode,
            "host.ssh": self._ssh,
        }

    def host_status(self):
        return {"schema": SCHEMA, "online": True, "t": int(time.time()), "methods": list(METHODS), "os": self._os,
                "trial": trial.active(self.paths), "wifi": self.wifi.status(),
                "setup_wifi": {"until": self._setup_until} if self._setup_until else None,
                "setup_wifi_gesture": self.gesture.describe(), "ssh": self.ssh.status(),
                "usb_mode_failed": self.usb_mode_failed}

    # Sends mowbite/host/status when the OS, the trial period, the WLAN change, the setup WLAN, a failed restart into
    # the USB mode or SSH changed.
    def look(self):
        self._os = self.watch.read()
        if self.gesture.take():
            self._on_gesture()
        self._setup_until = self.gesture.until()
        shown = (self._os, trial.active(self.paths), self.wifi.status(), self._setup_until, self.usb_mode_failed,
                 self.ssh.status())
        if shown != self._shown:
            self._shown = shown
            self.publish(HOST_STATUS, json.dumps(self.host_status()), True)

    # The gesture of the buttons: the setup WLAN, with a tone whether it opened.
    def _on_gesture(self):
        why = self.gesture.refusal(self._os) or self.gesture.open()
        print(f"setup WLAN not opened: {why}" if why else "setup WLAN opened by the buttons", flush=True)
        self.gesture.tone(why is None)

    def update_changed(self, status):
        text = json.dumps(status)
        if text != self._update_text:
            self._update_text = text
            self.publish(UPDATE_STATUS, text, True)

    # After every connect: both statuses again, the broker may have lost them.
    def connected(self):
        if self._os is None:
            self._os = self.watch.read()
        self.publish(HOST_STATUS, json.dumps(self.host_status()), True)
        self.publish(UPDATE_STATUS, self._update_text or json.dumps(self.updater.status()), True)

    # From the thread of the broker client: requests for the host go to the queue of the main loop.
    def on_request(self, message):
        if message.retained:
            return
        # Whoever sends a request reaches the mower, over the network it is on now.
        self.wifi.reached()
        try:
            request = json.loads(message.payload)
        except (ValueError, UnicodeDecodeError, RecursionError):
            return
        if not isinstance(request, dict) or request.get("method") not in METHODS:
            return
        to, correlation = "rpc/response", None
        if message.response_topic is not None:
            if not RESPONSE_TOPIC.fullmatch(message.response_topic):
                return
            if message.correlation is not None and len(message.correlation) > MAX_CORRELATION:
                return
            to, correlation = message.response_topic, message.correlation
        call_id = request.get("id")
        if call_id is None and to == "rpc/response":
            to = None
        try:
            self.requests.put_nowait((request, to, correlation, call_id))
        except queue.Full:
            pass

    def handle(self, request, to, correlation, call_id):
        method = request["method"]
        params = request.get("params")
        if params is None:
            params = {}
        try:
            if not isinstance(params, dict):
                raise Invalid("params has to be an object")
            if self.rebooting:
                raise Refused("BUSY", "the mower reboots into the USB mode")
            if method.startswith("update.") and isinstance(params.get("app_version"), str):
                self.updater.remember_app(params["app_version"])
            result = self._methods[method](params)
            reply = {"jsonrpc": "2.0", "id": call_id, "result": self.updater.status() if result is None else result}
        except (Invalid, wifi.Invalid) as why:
            reply = self._error(call_id, INVALID_PARAMS, str(why), "INVALID")
        except Refused as why:
            code = INVALID_PARAMS if why.reason == "INVALID" else INVALID_REQUEST
            reply = self._error(call_id, code, str(why), why.reason)
        except Exception as error:  # noqa: BLE001 - the caller gets an answer, the host goes on
            reply = self._error(call_id, INTERNAL_ERROR, f"{type(error).__name__}: {error}", "INTERNAL")
        if to is not None:
            self.publish(to, json.dumps(reply), False, correlation)

    @staticmethod
    def _error(call_id, code, message, reason):
        return {"jsonrpc": "2.0", "id": call_id,
                "error": {"code": code, "message": message,
                          "data": {"event": "UPDATE_REFUSED", "reason": reason, "message": message}}}

    def _set_auto(self, params):
        auto = self.updater.state["auto"]
        enabled = params.get("enabled", auto["enabled"])
        self.updater.set_auto(enabled, params.get("from", auto["from"]), params.get("to", auto["to"]))

    # Decision (N): stopped, down or halted always; a running OS only in IDLE without an emergency or restart hold;
    # never during an update.
    def _restart_os(self, params):
        if self.updater.busy():
            raise Refused("HOLD", "an update runs")
        state = self.watch.read()["state"]
        if state not in ("stopped", "down", "halted"):
            if os.path.lexists(self.paths.restart_hold):
                raise Refused("EMERGENCY", "the restart hold stands: release it first")
            why = gate.unrest(gate.session_of_this_boot(self.paths))
            if why is not None:
                raise Refused(why.code, why.why)
            if state != "running":
                raise Refused("BUSY", f"MowBite OS is {state}")
        if not self.carrier.restart():
            raise Refused("INTERNAL", "the OS did not start")
        self.look()
        return self._os

    def _confirm_start(self, params):
        challenge = _text(params, "challenge", required=False)
        if challenge is not None and not CHALLENGE.fullmatch(challenge):
            raise Invalid("challenge has to be 32 to 128 hex digits")
        result = trial.confirm(self.paths, self.run_command, self.watch.read(),
                               self.updater.state.get("first") is True, challenge)
        self.look()
        return result

    # SSH of the owner on or off (ssh.py); host/status shows it at once.
    def _ssh(self, params):
        result = self.ssh.set(params)
        self.look()
        return result

    # Tron only: through the gate (hold_for_reboot), then a reboot into the loader mode of U-Boot, where the PC program
    # reaches the mower over USB. The answer goes out before the reboot.
    def _usb_mode(self, params):
        try:
            with open(self.paths.device_tree, "rb") as f:
                rockchip = b"rockchip,rk3588" in f.read()
        except OSError:
            rockchip = False
        if not rockchip or self.updater.platform() != "tron":
            raise Refused("UNSUPPORTED", "only a Tron has the USB mode")
        self.updater.give_way()
        self.updater.hold_for_reboot()
        self.usb_mode_failed = None
        self.rebooting = True
        timer = threading.Timer(REBOOT_AFTER, self._reboot_loader)
        timer.daemon = True
        timer.start()
        return {"rebooting": True}

    def _reboot_loader(self):
        try:
            code, _, err = self.run_command(list(LOADER_REBOOT))
        except OSError as error:
            code, err = 1, str(error)
        if code != 0:
            print(f"usb_mode: the reboot failed: {err or code}", flush=True)
            self.updater.release_hold()
            # The next look of the main loop sends it in mowbite/host/status.
            self.usb_mode_failed = {"t": int(time.time()), "error": str(err or f"exit code {code}")}
            self.rebooting = False

    # A step of the main loop that fails (a full or read-only disk) is logged once per text; the host goes on.
    def _step(self, name, call):
        try:
            call()
            self._complaints.pop(name, None)
        except Exception as error:  # noqa: BLE001
            text = f"{type(error).__name__}: {error}"
            if self._complaints.get(name) != text:
                self._complaints[name] = text
                print(f"{name}: {text}", flush=True)

    # The trial period stands still while the first installation runs; ended without a running OS, it runs again.
    def _hold_trial(self):
        if not trial.active(self.paths):
            self._trial_held = None
            return
        now = time.monotonic()
        if self.updater.state.get("first") is True:
            if self._trial_held is None or now - self._trial_held >= HOLD_EVERY:
                self._trial_held = now
                trial.hold(self.paths, self.run_command)
        elif self._trial_held is not None:
            self._trial_held = None
            if self.watch.read()["state"] != "running":
                trial.again(self.paths, self.run_command)

    def run(self, alive=lambda: None, stop=lambda: False):
        self.updater.on_change = self.update_changed
        self.gesture.start()
        next_tick = time.monotonic() + TICK_EVERY
        next_check = time.monotonic()
        next_look = 0.0
        while not stop():
            alive()
            now = time.monotonic()
            if self.rebooting:
                next_check = next_tick = next_look = now + LOOK_EVERY
            if now >= next_check:
                next_check = now + CHECK_EVERY
                self._step("check", lambda: self.updater.check(fetch=True))
            if now >= next_tick:
                next_tick = now + TICK_EVERY
                self._step("tick", self.updater.tick)
            if now >= next_look:
                next_look = now + LOOK_EVERY
                self._step("look", self.look)
                self._step("recover", self.updater.recover_due)
                self._step("net", self.updater.poll)
                self._step("first", self.updater.first_install)
                self._step("trial", self._hold_trial)
                self._step("wifi", self.wifi.look)
            try:
                item = self.requests.get(timeout=max(0.05, min(next_look, next_tick) - time.monotonic()))
            except queue.Empty:
                continue
            self.handle(*item)
