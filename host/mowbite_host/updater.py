"""The updater of MowBite OS for its components (Bau 2: os): what is there, one update at a time, by hand or at night.

Phases: idle, verifying, unpacking, waiting_gate, switching, checking. The last result {result installed|rolled_back|
refused|aborted|failed, reason, message, revision, t} stays until the next one. A revision that ended rolled_back or
failed, that was taken back by hand or that cannot run on this mower is blocked: never installed by itself again, only
by hand (rule P4). By itself a revision is tried once a night, and an OS stopped by hand is left alone.
"""
import os
import re
import shutil
import threading
import time

from mowbite_host import Refused, bundle, files, gate
from mowbite_host.installer import HOLD_SECONDS, Installer, require_previous

COMPONENTS = {"os": "MowBite OS"}
# How long an update waits for the gate before it gives up, and how often it looks.
GATE_WAIT = 30 * 60.0
GATE_EVERY = 5.0
HOLD_WAIT = 10.0
# The automatic tries a revision again at the earliest this long after its last try: once a night.
AUTO_AGAIN = 20 * 3600.0
# States of the service in which the gate judges; in any other it starts or stops, or nobody knows.
SETTLED = ("active", "inactive", "failed")
# A revision cut off this often during its switch (power cut) is blocked.
MAX_ATTEMPTS = 2
AUTO_DEFAULT = {"enabled": False, "from": "02:00", "to": "05:00"}
IDLE = {"phase": "idle", "component": None, "revision": None, "reason": None}
HHMM = re.compile(r"([01][0-9]|2[0-3]):[0-5][0-9]")


def version_tuple(text):
    """1.4.0 -> ((1, 4, 0), 1); a pre-release (1.4.0-beta.2) sorts before its release."""
    if not isinstance(text, str):
        return None
    core, _, pre = text.lstrip("v").partition("-")
    numbers = []
    for part in core.split("."):
        if not part.isdigit():
            return None
        numbers.append(int(part))
    return tuple(numbers), 0 if pre else 1


def app_suffices(app_version, min_app):
    need = version_tuple(min_app)
    if need is None:
        return True
    have = version_tuple(app_version)
    return have is not None and have >= need


def in_window(now_local, start, end):
    minute = now_local.tm_hour * 60 + now_local.tm_min
    a = int(start[:2]) * 60 + int(start[3:])
    b = int(end[:2]) * 60 + int(end[3:])
    return a <= minute < b if a <= b else minute >= a or minute < b


class Aborted(Exception):
    pass


class Updater:
    """os_rpc: request_hold(seconds) and release_hold() over the broker; their answers are not trusted, the gate reads
    the files. on_change(status) after every change of the status."""

    def __init__(self, paths, carrier, health, inbox, os_rpc, on_change=None):
        self.paths, self.carrier, self.inbox, self.os_rpc = paths, carrier, inbox, os_rpc
        self.installer = Installer(paths, carrier, health)
        self.on_change = on_change or (lambda status: None)
        self._lock = threading.RLock()
        self._abort = threading.Event()
        self._worker = None
        self._phase = dict(IDLE)
        self._candidates = []
        # Whether a hold of this updater may stand: asked for at the gate or written by the switch.
        self._holding = False
        self.gate_wait = GATE_WAIT
        self.state = self._load()

    # update.json: {auto, app_version, last, components {id: {serial, installed_at, crashes_since_install, last_crash,
    # blocked {revision: {reason, result?, t}}, attempts {revision: n}, auto_tried {revision, t}}}}.
    def _load(self):
        state = files.read_json(self.paths.state) or {}
        state.setdefault("auto", dict(AUTO_DEFAULT))
        state.setdefault("app_version", None)
        state.setdefault("last", None)
        components = state.setdefault("components", {})
        for component in COMPONENTS:
            entry = components.setdefault(component, {})
            for key, value in (("installed_at", None), ("crashes_since_install", 0), ("last_crash", None),
                               ("blocked", {}), ("attempts", {}), ("auto_tried", None)):
                entry.setdefault(key, value)
        return state

    def _save(self):
        os.makedirs(self.paths.var, mode=0o700, exist_ok=True)
        files.write_json(self.paths.state, self.state)

    def _changed(self):
        self.on_change(self.status())

    def _entry(self, component):
        return self.state["components"][component]

    def _running_meta(self):
        return self.installer.meta_of(self.installer.current()) or {}

    # The api the running installation names, None without one.
    def running_api(self):
        return self._running_meta().get("api")

    def serial_of(self, component):
        meta = self._running_meta()
        serial = meta.get("serial") if meta.get("format") == bundle.FORMAT else 0
        return serial if isinstance(serial, int) else 0

    def _revision_info(self, revision):
        if not revision:
            return None
        meta = self.installer.meta_of(revision) or {}
        return {"revision": revision, "date": meta.get("date", meta.get("serial"))}

    def _available(self, component):
        have = self.serial_of(component)
        newer = [c for c in self._candidates if c["component"] == component and c["serial"] > have]
        if not newer:
            return None
        best = max(newer, key=lambda c: c["serial"])
        blocked = self._entry(component)["blocked"].get(best["revision"])
        return {"revision": best["revision"], "date": best.get("date", best["serial"]), "min_app": best.get("min_app"),
                "blocked": blocked}

    def status(self):
        with self._lock:
            components = []
            for component, name in COMPONENTS.items():
                entry = self._entry(component)
                components.append({
                    "id": component, "name": name, "running": self._revision_info(self.installer.current()),
                    "available": self._available(component), "previous": self._revision_info(self.installer.previous()),
                    "installed_at": entry["installed_at"], "crashes_since_install": entry["crashes_since_install"],
                    "auto": self.state["auto"]["enabled"]})
            return {**self._phase, "components": components, "auto": dict(self.state["auto"]),
                    "last": self.state["last"]}

    def _set_phase(self, phase, reason=None, **extra):
        with self._lock:
            self._phase = {**self._phase, "phase": phase, "reason": reason, **extra}
        self._changed()

    # Blocks revision: with result when an attempt ended so, without one for an obstacle like BASE_TOO_OLD.
    def _block(self, component, revision, reason, result=None):
        with self._lock:
            blocked = {"reason": reason, "t": int(time.time())}
            if result:
                blocked["result"] = result
            self._entry(component)["blocked"][revision] = blocked
            self._save()

    def _finish(self, result, reason, message, component, revision, block=False):
        if block and revision:
            self._block(component, revision, reason, result)
        with self._lock:
            self.state["last"] = {"result": result, "reason": reason, "message": message[:500],
                                  "revision": revision, "component": component, "t": int(time.time())}
            self._save()
            self._phase = dict(IDLE)
        self._changed()

    def phase(self):
        return self._phase["phase"]

    def last_crash(self, component):
        return self._entry(component)["last_crash"]

    # An end of the component at unix time t, from its exit note: counted once, since its install.
    def note_crash(self, component, t):
        with self._lock:
            entry = self._entry(component)
            if entry["last_crash"] is not None and t <= entry["last_crash"]:
                return
            entry["last_crash"] = t
            if entry["installed_at"] is None or t >= entry["installed_at"]:
                entry["crashes_since_install"] += 1
            # Counted in memory when the disk does not take it: the state of the OS stays visible.
            try:
                self._save()
            except OSError as error:
                print(f"update.json: {error}", flush=True)
        self._changed()

    def remember_app(self, app_version):
        if isinstance(app_version, str) and app_version != self.state["app_version"]:
            with self._lock:
                self.state["app_version"] = app_version
                self._save()

    # A candidate from elsewhere than the inbox (mowbite-update install <file>).
    def offer(self, candidate):
        with self._lock:
            self._candidates = [candidate]

    # Waits for the update or rollback that runs; the last result.
    def join(self):
        if self._worker is not None:
            self._worker.join()
        return self.state["last"]

    def busy(self):
        return self._worker is not None and self._worker.is_alive()

    # Reads the inbox again; the candidates and the inbox within its limits.
    def check(self):
        try:
            candidates = self.inbox.candidates()
        except Refused as why:
            print(f"inbox: {why}", flush=True)
            candidates = []
        with self._lock:
            self._candidates = self.inbox.prune(candidates, self.serial_of)
        self._changed()

    def set_auto(self, enabled, start, end):
        if not isinstance(enabled, bool) or not all(isinstance(t, str) and HHMM.fullmatch(t) for t in (start, end)):
            raise Refused("INVALID", "set_auto needs enabled true|false and from, to as HH:MM")
        with self._lock:
            self.state["auto"] = {"enabled": enabled, "from": start, "to": end}
            self._save()
        self._changed()

    # The candidate to install: the given revision, or the newest one that is newer than what runs.
    def _pick(self, component, revision):
        if component not in COMPONENTS:
            raise Refused("INVALID", f"no component {component}")
        if revision is None:
            available = self._available(component)
            if available is None:
                raise Refused("NO_NEWER", "no newer revision found; scp it into the inbox and check again")
            revision = available["revision"]
        for candidate in self._candidates:
            if candidate["component"] == component and candidate["revision"] == revision:
                return candidate
        raise Refused("UNKNOWN_REVISION", f"revision {revision} of {component} was not found")

    def _run(self, name, work):
        if self.busy():
            raise Refused("BUSY", "an update runs")
        self._abort.clear()
        self._worker = threading.Thread(target=work, name=name, daemon=True)
        self._worker.start()

    # update.start: runs in its own thread; Refused right away when it cannot begin.
    def start(self, component, revision=None, by_hand=True):
        with self._lock:
            candidate = self._pick(component, revision)
            blocked = self._entry(component)["blocked"].get(candidate["revision"])
            if blocked and not by_hand:
                raise Refused("BLOCKED", f"{candidate['revision']} ended {blocked.get('result')} before")
            self._run("update", lambda: self._install(candidate))

    # Under the lock the switch begins under: an abort that is accepted is never followed by the switch.
    def abort(self):
        with self._lock:
            if not self.busy():
                raise Refused("NOT_UPDATING", "no update runs")
            if self._phase["phase"] in ("switching", "checking"):
                raise Refused("BUSY", "the switch has begun; its health check decides")
            self._abort.set()

    def rollback(self, component):
        with self._lock:
            if component not in COMPONENTS:
                raise Refused("INVALID", f"no component {component}")
            previous = require_previous(self.installer)
            self._run("rollback", lambda: self._roll_back(component, previous))

    def _check_abort(self):
        if self._abort.is_set():
            raise Aborted()

    # Waits until the gate opens: an OS that moves the mower granted its hold, any other needs none.
    def _wait_gate(self):
        deadline = time.monotonic() + self.gate_wait
        while True:
            self._check_abort()
            state, _, started = self.carrier.show()
            if state not in SETTLED:
                why = ("BUSY", "MowBite OS is starting or stopping")
            else:
                active = state == "active"
                still = active and gate.off(self.paths, started) is not None
                why = gate.before_hold(self.paths, active, still)
                if why is None and (still or not active):
                    return
                if why is None:
                    why = self._ask_hold()
                    if why is None:
                        return
            self._set_phase("waiting_gate", why[0])
            if time.monotonic() >= deadline:
                raise Refused(why[0], f"the gate stayed closed: {why[1]}")
            self._abort.wait(GATE_EVERY)

    # Asks the OS for its hold: None once hold.json shows it and the files still open the gate, else why not, with
    # the hold given back.
    def _ask_hold(self):
        asked = time.time()
        self.os_rpc.request_hold(HOLD_SECONDS)
        self._holding = True
        why = ("NO_HOLD", "MowBite OS did not grant the hold of the update")
        until = time.monotonic() + HOLD_WAIT
        while time.monotonic() < until:
            if gate.hold_granted(self.paths, asked, time.time(), HOLD_SECONDS - 60):
                # Granted only in IDLE; the files must still say so after it.
                why = gate.before_hold(self.paths, self.carrier.active())
                if why is None:
                    return None
                break
            if self._abort.wait(0.5):
                break
        # No switch now: the mower is not held for nothing.
        self._release_hold()
        self._check_abort()
        return why

    def _release_hold(self):
        if not self._holding:
            return
        self._holding = False
        if self.carrier.active():
            self.os_rpc.release_hold()
        else:
            files.remove(self.paths.hold)

    def _switch(self, component, revision):
        entry = self._entry(component)
        with self._lock:
            # An abort accepted until here wins; from here on the health check decides.
            self._check_abort()
            entry["attempts"][revision] = entry["attempts"].get(revision, 0) + 1
            self._save()
            self._phase = {**self._phase, "phase": "switching", "reason": None, "component": component,
                           "revision": revision}
        self._changed()
        # The switch writes the hold itself once the OS is stopped.
        self._holding = True
        try:
            ok, line = self.installer.switch(revision, on_started=lambda: self._set_phase("checking"))
        except Exception:
            self._switched(entry, revision, False)
            raise
        self._switched(entry, revision, ok)
        return ok, line

    # The switch ended by itself, healthy or back at the one before: no cut to count.
    def _switched(self, entry, revision, ok):
        with self._lock:
            entry["attempts"].pop(revision, None)
            if ok:
                entry["installed_at"] = int(time.time())
                entry["crashes_since_install"] = 0
            self._save()

    def _install(self, candidate):
        component, revision = candidate["component"], candidate["revision"]
        try:
            self._set_phase("verifying", component=component, revision=revision)
            staged = self.inbox.stage(candidate)
            if staged.serial <= self.serial_of(component):
                raise Refused("OLD_SERIAL", f"serial {staged.serial} is not newer than the running "
                                            f"{self.serial_of(component)}")
            try:
                bundle.check_base(staged, self.paths.base)
            except Refused as why:
                self._block(component, revision, why.reason)
                raise
            target = self.paths.opt_of(revision)
            if not os.path.isdir(target):
                self._check_abort()
                self._set_phase("unpacking")
                bundle.check_space(staged, self.paths.opt)
                partial = bundle.unpack(staged, self.paths.opt)
                problems = bundle.unresolved_libraries(os.path.join(partial, revision), self.paths.os_setup,
                                                       self._env())
                if problems:
                    shutil.rmtree(partial, ignore_errors=True)
                    self._block(component, revision, "LIBRARY_MISSING")
                    raise Refused("LIBRARY_MISSING", "libraries not found:\n  " + "\n  ".join(problems[:20]))
                os.rename(os.path.join(partial, revision), target)
                os.rmdir(partial)
                files.sync_dir(self.paths.opt)
            self._set_phase("waiting_gate")
            self._wait_gate()
            ok, line = self._switch(component, revision)
            self.installer.prune()
            if ok:
                self._finish("installed", None, line, component, revision)
            elif self.installer.current():
                self._finish("rolled_back", "UNHEALTHY", line, component, revision, block=True)
            else:
                self._finish("failed", "UNHEALTHY", line, component, revision, block=True)
        except Aborted:
            self._finish("aborted", None, "aborted before the switch", component, revision)
        except Refused as why:
            self._finish("refused", why.reason, str(why), component, revision)
        except Exception as error:  # noqa: BLE001 - the result has to say it, the host goes on
            self._finish("failed", "INTERNAL", f"{type(error).__name__}: {error}", component, revision)
        finally:
            shutil.rmtree(self.inbox.install_stage(), ignore_errors=True)
            self._release_hold()

    def _roll_back(self, component, previous):
        current = self.installer.current()
        try:
            self._set_phase("waiting_gate", component=component, revision=previous)
            self._wait_gate()
            ok, line = self._switch(component, previous)
            if ok:
                # Taken back by hand: the newer one does not come back by itself.
                if current:
                    self._block(component, current, "ROLLED_BACK_BY_HAND", "rolled_back")
                self._finish("installed", None, line, component, previous)
            else:
                self._finish("failed", "UNHEALTHY", line, component, previous)
        except Aborted:
            self._finish("aborted", None, "aborted before the switch", component, previous)
        except Refused as why:
            self._finish("refused", why.reason, str(why), component, previous)
        except Exception as error:  # noqa: BLE001
            self._finish("failed", "INTERNAL", f"{type(error).__name__}: {error}", component, previous)
        finally:
            self._release_hold()

    def _env(self):
        return {**self.paths.env(), "PATH": os.environ.get("PATH", "/usr/sbin:/usr/bin:/sbin:/bin"),
                "HOME": "/nonexistent"}

    # At the start of the host: back from a switch that was cut off; a revision cut off MAX_ATTEMPTS times is blocked.
    def recover(self, start_stopped=False):
        pending = self.installer.recover(start_stopped)
        if pending is None:
            return
        revision = pending.get("to")
        attempts = self._entry("os")["attempts"]
        message = f"the switch to {revision} was cut off; back to {pending.get('from') or 'none'}"
        blocked = bool(revision) and attempts.get(revision, 0) >= MAX_ATTEMPTS
        if blocked:
            attempts.pop(revision)
        self._finish("failed", "INTERRUPTED", message, "os", revision, block=blocked)

    def clock_ok(self):
        return os.path.exists(self.paths.prefix + "/run/systemd/timesync/synchronized")

    # Called every few minutes: starts the available update at night when auto is on, the clock is sure and the app
    # the user has is new enough for it; each revision once a night.
    def tick(self, now=None):
        auto = self.state["auto"]
        if not auto["enabled"] or self.busy() or not self.clock_ok():
            return
        now = time.time() if now is None else now
        if not in_window(time.localtime(now), auto["from"], auto["to"]):
            return
        # An OS stopped by hand stays as it is; one that gave up after crashes or was never installed gets its update.
        if self.installer.current() and self.carrier.show()[0] not in ("active", "failed"):
            return
        self.check()
        for component in COMPONENTS:
            available = self._available(component)
            if (available and not available["blocked"] and
                    app_suffices(self.state["app_version"], available.get("min_app"))):
                entry = self._entry(component)
                tried = entry["auto_tried"] or {}
                if tried.get("revision") == available["revision"] and 0 <= now - tried.get("t", 0) < AUTO_AGAIN:
                    return
                with self._lock:
                    entry["auto_tried"] = {"revision": available["revision"], "t": int(now)}
                    self._save()
                try:
                    self.start(component, available["revision"], by_hand=False)
                except Refused as why:
                    print(f"auto: {why}", flush=True)
                return
