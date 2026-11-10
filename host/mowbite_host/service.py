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
import time

from mowbite_host import Refused, gate
from mowbite_host.updater import COMPONENTS

SCHEMA = 1
HOST_STATUS = "mowbite/host/status"
UPDATE_STATUS = "mowbite/update/status"
METHODS = ("host.restart_os", "update.abort", "update.check", "update.rollback", "update.set_auto", "update.start",
           "update.status")
OFFLINE = json.dumps({"schema": SCHEMA, "online": False})
TICK_EVERY = 5 * 60.0
CHECK_EVERY = 6 * 3600.0
# How often the state of the OS is read from its files.
LOOK_EVERY = 2.0
RESPONSE_TOPIC = re.compile(r"rpc/response/[A-Za-z0-9._-]{1,64}")
MAX_CORRELATION = 1024
# Requests beyond this many waiting are dropped.
MAX_WAITING = 64
INVALID_REQUEST = -32600
INVALID_PARAMS = -32602
INTERNAL_ERROR = -32603


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


class Host:
    """updater: updater.Updater; watch: os_state.OsWatch; carrier: what runs the OS; publish(topic, text, retain,
    correlation=None) sends when connected."""

    def __init__(self, paths, updater, watch, carrier, publish):
        self.paths, self.updater, self.watch, self.carrier, self.publish = paths, updater, watch, carrier, publish
        self.requests = queue.Queue(maxsize=MAX_WAITING)
        self._os = None
        self._update_text = None
        # None: the answer is update.status.
        self._methods = {
            "update.status": lambda p: None,
            "update.start": lambda p: self.updater.start(_component(p), _text(p, "revision", required=False)),
            "update.abort": lambda p: self.updater.abort(),
            "update.rollback": lambda p: self.updater.rollback(_component(p)),
            "update.check": lambda p: self.updater.check(),
            "update.set_auto": self._set_auto,
            "host.restart_os": self._restart_os,
        }

    def host_status(self):
        return {"schema": SCHEMA, "online": True, "t": int(time.time()), "methods": list(METHODS), "os": self._os}

    def look(self):
        os_now = self.watch.read()
        if os_now != self._os:
            self._os = os_now
            self.publish(HOST_STATUS, json.dumps(self.host_status()), True)

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
        try:
            request = json.loads(message.payload)
        except (ValueError, UnicodeDecodeError):
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
            if method.startswith("update.") and isinstance(params.get("app_version"), str):
                self.updater.remember_app(params["app_version"])
            result = self._methods[method](params)
            reply = {"jsonrpc": "2.0", "id": call_id, "result": self.updater.status() if result is None else result}
        except Invalid as why:
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
            session = gate.session_of_this_boot(self.paths) or {}
            if session.get("emergency") is True or os.path.lexists(self.paths.restart_hold):
                raise Refused("EMERGENCY", "an emergency or the restart hold stands: release it first")
            if state != "running" or session.get("state") != "IDLE":
                raise Refused("BUSY", f"MowBite OS is {session.get('state') or state}")
        if not self.carrier.restart():
            raise Refused("INTERNAL", "the OS did not start")
        self.look()
        return self._os

    def run(self, alive=lambda: None, stop=lambda: False):
        self.updater.on_change = self.update_changed
        next_tick = time.monotonic() + TICK_EVERY
        next_check = time.monotonic()
        next_look = 0.0
        while not stop():
            alive()
            now = time.monotonic()
            if now >= next_check:
                next_check = now + CHECK_EVERY
                self.updater.check()
            if now >= next_tick:
                next_tick = now + TICK_EVERY
                self.updater.tick()
            if now >= next_look:
                next_look = now + LOOK_EVERY
                self.look()
            try:
                item = self.requests.get(timeout=max(0.05, min(next_look, next_tick) - time.monotonic()))
            except queue.Empty:
                continue
            self.handle(*item)
