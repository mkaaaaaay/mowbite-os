"""The updater of MowBite OS for its components os, planner and app: what is there, one update at a time, by hand or
at night.

Phases: idle, verifying, unpacking, waiting_gate, switching, checking. The last result {result installed|rolled_back|
refused|aborted|failed, reason, message, revision, t} stays until the next one. A revision that ended rolled_back or
failed, that was taken back by hand or that cannot run on this mower is blocked: never installed by itself again, only
by hand (rule P4). By itself a revision is tried once a night, and an OS stopped by hand is left alone.

The OS and the planner switch through the gate of the OS and restart it, the planner only once the mowing job is
finished. The app switches at any time: only files change, nothing that runs is touched. The host switches at any
time as well and ends; its result comes from the new one once it confirmed itself, or from the note of mowbite_boot
when it went back. An own planner or app that
does not go with the installed OS is not installed; one already installed is left aside (installer.settle).
"""
import os
import re
import shutil
import threading
import time

import mowbite_host
from mowbite_host import Refused, bundle, carrier as carriers, compat, files, gate, net, origin
from mowbite_host.compat import app_suffices
from mowbite_host.installer import HOLD_SECONDS, AppFiles, HostInstaller, Installer, WithPlanner, os_meta, skipped

COMPONENTS = {"os": "MowBite OS", "planner": "MowBite Planner", "app": "MowBite App", "host": "MowBite Host"}
# The order the automatic takes them in: a new host may be needed for the bundles after it.
AUTO_ORDER = ("host", "app", "planner", "os")
# The folder of the library this process runs: an installation of the host or its seed.
RUNNING_HOST = os.path.dirname(os.path.dirname(os.path.realpath(mowbite_host.__file__)))
# How long an update waits for the gate before it gives up, and how often it looks.
GATE_WAIT = 30 * 60.0
GATE_EVERY = 5.0
HOLD_WAIT = 10.0
# The OS grants no hold this long before a planned start (system.hold); the slack covers the clocks.
SCHEDULE_HOLD_S = 1800 + 60
# A hold the OS still shows in hold.json is given back this often, each time waiting this long for the file to go.
RELEASE_TRIES = 3
RELEASE_WAIT = 3.0
# The automatic tries a revision again at the earliest this long after its last try: once a night.
AUTO_AGAIN = 20 * 3600.0
# States of the service in which the gate judges; in any other it starts or stops, or nobody knows.
SETTLED = ("active", "inactive", "failed")
# A revision cut off this often during its switch (power cut) is blocked.
MAX_ATTEMPTS = 2
# An unfinished switch whose hold the OS refused is undone at the earliest this long after.
RECOVER_RETRY = 5 * 60.0
AUTO_DEFAULT = {"enabled": False, "from": "02:00", "to": "05:00"}
ORIGIN_DEFAULT = {"channel": "stable", "index_serial": 0, "last": None}
# What each new phase starts without.
FRESH = {"reason": None, "at": None, "percent": None, "bytes": None}
IDLE = {"phase": "idle", "component": None, "revision": None, **FRESH}
# percent and bytes of verifying and unpacking are told at most this often.
PROGRESS_EVERY = 1.0
HHMM = re.compile(r"([01][0-9]|2[0-3]):[0-5][0-9]")
MAX_APP_VERSION = 64
# The line of the health check for an OS that halted since data_dir has no room for its backup.
NO_SPACE = "halted: DATA_NO_SPACE"
# After such a halt the automatic leaves the OS and its planner alone until data_dir has this much more room.
NO_SPACE_MORE = 256 * 1024 * 1024


# Why the OS gave no hold, from its answer to system.hold (None without one): a planned start soon, with its time.
def _why_no_hold(answer, now):
    start = answer.get("next_start") if isinstance(answer, dict) else None
    if (type(start) is int and answer.get("hold_until") == 0 and answer.get("idle") is False
            and 0 <= start - now <= SCHEDULE_HOLD_S):
        return "SCHEDULED_START", f"a plan of the schedule starts at {start} (unix s)", start
    return "NO_HOLD", "MowBite OS did not grant the hold of the update"


# Minute of the day of HH:MM.
def _minutes(hhmm):
    return int(hhmm[:2]) * 60 + int(hhmm[3:])


def in_window(now_local, start, end):
    minute = now_local.tm_hour * 60 + now_local.tm_min
    a, b = _minutes(start), _minutes(end)
    return a <= minute < b if a <= b else minute >= a or minute < b


# Seconds from now_local until the window ends at end.
def window_left(now_local, end):
    left = (_minutes(end) - now_local.tm_hour * 60 - now_local.tm_min) % (24 * 60)
    return left * 60 - now_local.tm_sec


class Aborted(Exception):
    pass


class Restarting(Exception):
    """The host switched itself and ends; the new one gives the result."""


class Updater:
    """os_rpc: request_hold(seconds) and release_hold() over the broker; the gate reads the files, the answer of
    request_hold (None without one) only tells why no hold came. on_change(status) after every change of the
    status. net: net.Net, None without bundles from the net."""

    def __init__(self, paths, carrier, health, inbox, os_rpc, on_change=None, net=None):
        self.paths, self.carrier, self.inbox, self.os_rpc, self.net = paths, carrier, inbox, os_rpc, net
        places = paths.places
        self.installers = {"os": Installer(paths, places["os"], carrier, health),
                           "planner": Installer(paths, places["planner"], carrier, WithPlanner(health)),
                           "app": Installer(paths, places["app"], carriers.Nothing(), AppFiles(places["app"])),
                           "host": HostInstaller(paths)}
        self.installers["app"].hold = False
        self.installer = self.installers["os"]
        self.on_change = on_change or (lambda status: None)
        # Ends this process after a switch of the host; whether the broker is connected, for its confirmation.
        self.restart_self = lambda: None
        self.connected = lambda: False
        # Whether the health of a switch is judged over the broker (mowbite-host): then no OS switches without it.
        self.judges_by_broker = False
        self._lock = threading.RLock()
        self._abort = threading.Event()
        self._worker = None
        # From the start of a worker until it gave back its hold, before it tells idle.
        self._working = False
        self._phase = dict(IDLE)
        self._candidates = []
        # Whether a hold of this updater may stand: asked for at the gate or written by the switch.
        self._holding = False
        # Monotonic time before which recover_due does not ask the OS for its hold again; why each component waits.
        self._recover_after = 0.0
        self._recover_waits = {}
        # The running OS while an unfinished switch of it or its planner waits: started of its service, since when it
        # is looked at (monotonic), whether it answered healthy once and the last line.
        self._judged = None
        self._bad_app_logged = False
        self.gate_wait = GATE_WAIT
        self.state = self._load()

    # update.json: {auto, origin {channel, index_serial, last}, app_version, last, components {id: {serial,
    # installed_at, crashes_since_install, last_crash, blocked {revision: {reason, result?, t}}, attempts {revision: n},
    # auto_tried {revision, t}}}, no_space {free, t} | None}; no_space: room in data_dir when an OS halted for the
    # want of it in a switch.
    def _load(self):
        state = files.read_json(self.paths.state) or {}
        state.setdefault("auto", dict(AUTO_DEFAULT))
        state["origin"] = {**ORIGIN_DEFAULT, **state.get("origin", {})}
        if not self._app_ok(state.setdefault("app_version", None)):
            state["app_version"] = None
        state.setdefault("last", None)
        state.setdefault("no_space", None)
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
        return os_meta(self.paths)

    # The platform of this mower: MOWBITE_PLATFORM of the device, else the one its running OS is built for alone; None
    # when neither names one.
    def _platform(self):
        named = self.paths.env().get("MOWBITE_PLATFORM")
        if named:
            return named
        platforms = self._running_meta().get("platforms")
        return platforms[0] if isinstance(platforms, list) and len(platforms) == 1 else None

    # The api the running installation names, None without one.
    def running_api(self):
        return self._running_meta().get("api")

    # Whether a switch of component stops and starts the OS.
    def _stops_os(self, component):
        return self.installers[component].carrier is self.carrier

    # Whether the OS is in a switch, its own or the one of its planner.
    def os_switching(self):
        phase, component = self._phase["phase"], self._phase["component"]
        return phase in ("switching", "checking") and component in self.installers and self._stops_os(component)

    # The serial a bundle of component has to be newer than: the one of its installation, for the planner and the app
    # also the one the OS brings along.
    def serial_of(self, component):
        installer = self.installers[component]
        meta = installer.meta_of(installer.current()) or {}
        serial = compat.serial(meta) if meta.get("format") == bundle.FORMAT else 0
        return max(serial, compat.serial(installer.bundled()))

    @staticmethod
    def _info(revision, meta):
        return {"revision": revision, "date": meta.get("date", meta.get("serial")), "version": meta.get("version")}

    def _revision_info(self, component, revision):
        if not revision:
            return None
        return self._info(revision, self.installers[component].meta_of(revision) or {})

    # The planner or the app the OS brings along, the seed of the host; None when there is none.
    def _bundled_info(self, component):
        return self._meta_info(self.installers[component].bundled())

    def _meta_info(self, meta):
        meta = meta or {}
        return self._info(meta["revision"], meta) if isinstance(meta.get("revision"), str) else None

    def _available(self, component, platform):
        have = self.serial_of(component)
        newer = [c for c in self._candidates if c["component"] == component and c["serial"] > have and
                 bundle.fits_platform(c, platform)]
        if not newer:
            return None
        best = max(newer, key=lambda c: c["serial"])
        blocked = self._entry(component)["blocked"].get(best["revision"])
        if blocked is None:
            # Not kept: after an update of the OS it may fit.
            why = compat.reason(component, best, self._running_meta())
            blocked = {"reason": why, "t": None} if why else None
        available = {"revision": best["revision"], "date": best.get("date", best["serial"]),
                     "min_app": best.get("min_app"), "version": best.get("version"), "blocked": blocked,
                     "from": best.get("from", "inbox")}
        if component == "os":
            available.update(api=best.get("api"), api_break=best.get("api_break"))
        return available

    # running, previous (where a rollback goes) and for the planner and the app source (own while active names their
    # installation, else bundled) and own_skipped {revision, reason} for an own installation left aside.
    def _installed(self, component):
        installer = self.installers[component]
        place, own = installer.place, installer.current()
        try:
            back = installer.back_target()
        except Refused:
            previous = None
        else:
            previous = self._revision_info(component, back) if back else self._bundled_info(component)
        if component == "host":
            return {"running": self._meta_info(files.read_json(os.path.join(RUNNING_HOST, "bundle.json"))),
                    "previous": previous}
        if not place.overlay:
            return {"running": self._revision_info(component, own), "previous": previous}
        used = own is not None and files.link_target(place.active) == own
        return {"running": self._revision_info(component, own) if used else self._bundled_info(component),
                "previous": previous, "source": "own" if used else "bundled",
                "own_skipped": None if used or own is None else {"revision": own,
                                                                 "reason": skipped(self.paths, place)}}

    def status(self):
        with self._lock:
            components = []
            platform = self._platform()
            for component, name in COMPONENTS.items():
                entry = self._entry(component)
                components.append({
                    "id": component, "name": name, **self._installed(component),
                    "available": self._available(component, platform),
                    "installed_at": entry["installed_at"], "crashes_since_install": entry["crashes_since_install"],
                    "auto": self.state["auto"]["enabled"]})
            origin = self.state["origin"]
            return {**self._phase, "components": components, "auto": dict(self.state["auto"]),
                    "last": self.state["last"],
                    "origin": {"channel": origin["channel"], "url": net.INDEX_URL if self.net else None,
                               "downloading": bool(self.net and self.net.busy()), **(origin["last"] or {})}}

    def _set_phase(self, phase, reason=None, at=None, **extra):
        with self._lock:
            self._phase = {**self._phase, **FRESH, "phase": phase, "reason": reason, "at": at, **extra}
        self._changed()

    # Blocks revision: with result when an attempt ended so, without one for an obstacle like BASE_TOO_OLD.
    def _block(self, component, revision, reason, result=None):
        with self._lock:
            blocked = {"reason": reason, "t": int(time.time())}
            if result:
                blocked["result"] = result
            self._entry(component)["blocked"][revision] = blocked
            self._save_or_log()

    # The state kept in memory when it cannot be written (a full disk); the next save takes it along.
    def _save_or_log(self):
        try:
            self._save()
        except OSError as error:
            print(f"{self.paths.state} cannot be written: {error}", flush=True)

    def _finish(self, result, reason, message, component, revision, block=False):
        if block and revision:
            self._block(component, revision, reason, result)
        with self._lock:
            self.state["last"] = {"result": result, "reason": reason, "message": message[:500],
                                  "revision": revision, "component": component, "t": int(time.time())}
            self._save_or_log()
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

    # None, or a version of up to MAX_APP_VERSION characters that compares; another is logged once.
    def _app_ok(self, app_version):
        if app_version is None:
            return True
        if isinstance(app_version, str) and len(app_version) <= MAX_APP_VERSION and compat.version_tuple(app_version):
            return True
        if not self._bad_app_logged:
            self._bad_app_logged = True
            print(f"app_version {str(app_version)[:MAX_APP_VERSION]!r} is no version: ignored", flush=True)
        return False

    def remember_app(self, app_version):
        if not isinstance(app_version, str) or not self._app_ok(app_version):
            return
        if app_version != self.state["app_version"]:
            with self._lock:
                self.state["app_version"] = app_version
                self._save_or_log()

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
        return self._working

    # Reads the inbox and the net folder again; the candidates and both folders within their limits. fetch: also
    # asks the net for a newer index in the background, poll takes its result.
    def check(self, fetch=False):
        candidates = []
        for name, source, read in (("inbox", self.inbox, self.inbox.candidates),
                                   ("net", self.net, lambda: self.net.candidates(self.state["origin"]["channel"]))):
            if source is None:
                continue
            try:
                found = read()
            except Refused as why:
                print(f"{name}: {why}", flush=True)
                found = []
            with self._lock:
                candidates += source.prune(found, self.serial_of)
        with self._lock:
            self._candidates = candidates
        if fetch and self.net:
            self.net.start(self.state["origin"]["channel"], self.state["origin"]["index_serial"], self.serial_of,
                           self._platform())
        self._changed()

    # From the main loop: the result of a refresh from the net that ended, then its bundles as candidates.
    def poll(self):
        if not self.net or not self.net.done.is_set():
            return
        self.net.done.clear()
        result = dict(self.net.result)
        with self._lock:
            origin = self.state["origin"]
            if isinstance(result.get("index_serial"), int):
                origin["index_serial"] = max(origin["index_serial"], result["index_serial"])
            origin["last"] = result
            self._save_or_log()
        if result["result"] != "ok":
            print(f"net: {result['result']}: {result.get('message')}", flush=True)
        self.check()

    def set_channel(self, channel):
        if channel not in net.CHANNELS:
            raise Refused("INVALID", f"channel is one of {', '.join(net.CHANNELS)}")
        with self._lock:
            self.state["origin"]["channel"] = channel
            self._save()
        self.check(fetch=True)

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
            available = self._available(component, self._platform())
            if available is None:
                raise Refused("NO_NEWER", "no newer revision found in the inbox or from the net")
            revision = available["revision"]
        for candidate in self._candidates:
            if candidate["component"] == component and candidate["revision"] == revision:
                return candidate
        raise Refused("UNKNOWN_REVISION", f"revision {revision} of {component} was not found")

    def _run(self, name, work):
        if self.busy():
            raise Refused("BUSY", "an update runs")
        # A host on trial installs nothing: going back would cut it off.
        if os.path.lexists(self.paths.host.pending):
            raise Refused("BUSY", "the new host has not confirmed itself yet")
        self._abort.clear()
        self._working = True
        self._worker = threading.Thread(target=self._work, args=(work,), name=name, daemon=True)
        try:
            self._worker.start()
        except BaseException:
            self._working = False
            raise

    # Idle only once the worker has given back its hold and ended: from then on another update can begin. A switch of
    # the host stays checking, the new host gives the result.
    def _work(self, work):
        try:
            work()
        finally:
            with self._lock:
                restarting = self._phase["phase"] == "checking" and self._phase["component"] == "host"
                if not restarting:
                    self._phase = dict(IDLE)
                self._working = False
            if not restarting:
                self._changed()

    # update.start: runs in its own thread; Refused right away when it cannot begin. reset_jobs: the planner is
    # installed although a mowing job is unfinished. gate_until: the monotonic time the gate is given up at the latest.
    def start(self, component, revision=None, by_hand=True, reset_jobs=False, gate_until=None):
        with self._lock:
            candidate = self._pick(component, revision)
            blocked = self._entry(component)["blocked"].get(candidate["revision"])
            if blocked and not by_hand:
                raise Refused("BLOCKED", f"{candidate['revision']} ended {blocked.get('result')} before")
            self._run("update", lambda: self._install(candidate, reset_jobs, gate_until))

    # Under the lock the switch begins under: an abort that is accepted is never followed by the switch.
    def abort(self):
        with self._lock:
            if not self.busy():
                raise Refused("NOT_UPDATING", "no update runs")
            if self._phase["phase"] in ("switching", "checking"):
                raise Refused("BUSY", "the switch has begun; its health check decides")
            self._abort.set()

    # Back to the previous installation; the planner and the app without one go back to the one of the OS.
    def rollback(self, component, reset_jobs=False):
        with self._lock:
            if component not in COMPONENTS:
                raise Refused("INVALID", f"no component {component}")
            previous = self.installers[component].back_target()
            self._run("rollback", lambda: self._roll_back(component, previous, reset_jobs))

    def _check_abort(self):
        if self._abort.is_set():
            raise Aborted()

    # gate.before_hold; with jobs an unfinished mowing job closes the gate as well.
    def _closed(self, jobs, active, still=False, recovering=False):
        why = gate.before_hold(self.paths, active, still, recovering)
        if why is None and jobs and gate.job_open(self.paths):
            why = "JOB_OPEN", "a mowing job is unfinished: finish or reset it, or update with reset_jobs"
        return why

    # Waits until the gate opens: an OS that moves the mower granted its hold, any other needs none. The planner
    # waits for the end of the mowing job as well unless reset_jobs. The app needs no gate, only no unfinished switch.
    def _wait_gate(self, component, reset_jobs=False, until=None):
        if not self._stops_os(component):
            if gate.unfinished(self.paths):
                raise Refused("UNFINISHED", "an update did not finish: recover first")
            return
        jobs = component == "planner" and not reset_jobs
        deadline = min(time.monotonic() + self.gate_wait, until or float("inf"))
        while True:
            self._check_abort()
            state, _, started = self.carrier.show()
            if state not in SETTLED:
                why = ("BUSY", "MowBite OS is starting or stopping")
            else:
                active = state == "active"
                # Given up by systemd after its crashes it is down as well: nobody could release a restart hold.
                still = state == "failed" or (active and gate.off(self.paths, started) is not None)
                why = self._closed(jobs, active, still)
                if why is None and active and not still:
                    why = self._ask_hold(jobs)
                if why is None:
                    if until is not None and time.monotonic() >= until:
                        raise Refused("BUSY", "the window of the automatic update ended before the switch")
                    return
            self._set_phase("waiting_gate", why[0], at=why[2] if len(why) > 2 else None)
            if time.monotonic() >= deadline:
                raise Refused(why[0], f"the gate stayed closed: {why[1]}")
            self._abort.wait(GATE_EVERY)

    # Asks the OS for its hold: None once hold.json shows it and the files still open the gate, else why not, with
    # the hold given back.
    def _ask_hold(self, jobs, recovering=False):
        asked = time.time()
        why = _why_no_hold(self.os_rpc.request_hold(HOLD_SECONDS), asked)
        self._holding = True
        until = time.monotonic() + HOLD_WAIT
        while time.monotonic() < until:
            if gate.hold_granted(self.paths, asked, time.time(), HOLD_SECONDS - 60):
                # Granted only in IDLE; the files must still say so after it.
                why = self._closed(jobs, self.carrier.active(), recovering=recovering)
                if why is None:
                    return None
                break
            if why[0] == "SCHEDULED_START" or self._abort.wait(0.5):
                break
        # No switch now: the mower is not held for nothing.
        self._release_hold()
        self._check_abort()
        return why

    # An OS that runs ends its hold itself and keeps it in memory: it is asked until hold.json is gone, the file is
    # not taken from under it. A hold it never gave back ends in the OS when its time runs out.
    def _release_hold(self):
        if not self._holding:
            return
        self._holding = False
        for _ in range(RELEASE_TRIES):
            if not self.carrier.active():
                files.remove(self.paths.hold)
                return
            self.os_rpc.release_hold()
            until = time.monotonic() + RELEASE_WAIT
            while os.path.lexists(self.paths.hold) and time.monotonic() < until:
                time.sleep(0.05)
            if not os.path.lexists(self.paths.hold):
                return
        print("hold: MowBite OS did not give it back, it ends there when its time runs out", flush=True)

    # revision None: to the planner or the app the OS brings along, to the seed of the host. by_hand_from: a rollback
    # by hand away from that revision.
    def _switch(self, component, revision, by_hand_from=None):
        entry = self._entry(component)
        if component == "host":
            self._switch_host(revision, by_hand_from)
        with self._lock:
            # An abort accepted until here wins; from here on the health check decides.
            self._check_abort()
            if self.judges_by_broker and self._stops_os(component) and not self.connected():
                raise Refused("BUSY", "the broker of the OS is not connected: the health of a switch cannot be judged")
            if revision:
                entry["attempts"][revision] = entry["attempts"].get(revision, 0) + 1
                self._save()
            self._phase = {**self._phase, **FRESH, "phase": "switching", "component": component,
                           "revision": revision}
        self._changed()
        # The switch writes the hold itself once the OS is stopped; a switch of the app holds nothing.
        if self._stops_os(component):
            self._holding = True
        try:
            ok, line = self.installers[component].switch(revision, on_started=lambda: self._set_phase("checking"))
        except Exception:
            self._switched(entry, revision, False)
            raise
        self._switched(entry, revision, ok)
        return ok, line

    def _switch_host(self, revision, by_hand_from):
        with self._lock:
            self._check_abort()
            self.installers["host"].switch(revision, broker=self.connected(), by_hand_from=by_hand_from)
            self._phase = {**self._phase, **FRESH, "phase": "checking", "component": "host",
                           "revision": revision}
        self._changed()
        self.restart_self()
        raise Restarting()

    # At the start: the result of a switch of the host that mowbite_boot took back.
    def host_went_back(self):
        note = files.read_json(self.paths.host_back)
        if note is None:
            return
        revision = note.get("to") if isinstance(note.get("to"), str) else None
        message = f"{note.get('message', '')}; back to {note.get('from') or 'the seed'}"
        self._finish("rolled_back", "UNHEALTHY", message, "host", revision, block=bool(revision))
        files.remove(self.paths.host_back)

    # Whether the new host waits for the broker before it confirms itself: the one before it was connected.
    def host_needs_broker(self):
        return bool((self.installers["host"].pending() or {}).get("broker"))

    # Called once the host ran long enough: ends the switch to it as installed. True when there was one.
    def confirm_host(self):
        installer = self.installers["host"]
        pending = installer.confirm(RUNNING_HOST)
        if pending is None:
            return False
        revision = pending.get("to") or None
        with self._lock:
            entry = self._entry("host")
            entry["installed_at"] = int(time.time())
            entry["crashes_since_install"] = 0
        if pending.get("by_hand_from"):
            self._block("host", pending["by_hand_from"], "ROLLED_BACK_BY_HAND", "rolled_back")
        self._finish("installed", None, "the new host confirmed itself", "host", revision)
        installer.prune()
        return True

    # The switch ended by itself, healthy or back at the one before: no cut to count.
    def _switched(self, entry, revision, ok):
        with self._lock:
            entry["attempts"].pop(revision, None)
            if ok:
                entry["installed_at"] = int(time.time())
                entry["crashes_since_install"] = 0
            # A full disk does not turn a switch that ended into a failed one.
            self._save_or_log()

    # Refused when the staged bundle does not go with the installed OS (not kept: another OS may take it) or with
    # this mower and its rootfs (blocked).
    def _check_fits(self, component, staged):
        why = compat.reason(component, staged.meta, self._running_meta())
        if why:
            raise Refused(why, compat.EXPLAIN[why])
        try:
            bundle.check_platform(staged, self._platform())
            bundle.check_base(staged, self.paths.base)
            bundle.check_requires(staged, self._env())
        except Refused as refused:
            self._block(component, staged.revision, refused.reason)
            raise

    # Why the tree unpacked from a bundle of component is not installed: (reason, text), None when it is good. The OS
    # has to find its libraries; the planner and the app bring no machine code.
    def _tree_problem(self, component, tree):
        if component == "os":
            problems = bundle.unresolved_libraries(tree, self.paths.os_setup, self._env())
            return ("LIBRARY_MISSING", "libraries not found:\n  " + "\n  ".join(problems[:20])) if problems else None
        machine = [os.path.relpath(path, tree) for path in bundle.elf_files(tree)]
        if machine:
            return "BAD_BUNDLE", f"machine code in a bundle of {component}: " + ", ".join(machine[:20])
        broken = bundle.unparsable_python(tree, self._env()) if component == "host" else []
        if broken:
            return "BAD_BUNDLE", "python that does not parse:\n  " + "\n  ".join(broken[:20])
        return None

    # A folder of the revision that the bundle did not give (an earlier, broken unpack) goes, unless it is in use.
    def _replace_stale(self, installer, revision):
        if revision in installer.in_use():
            raise Refused("BAD_BUNDLE", f"{revision} is installed with another bundle.json than this bundle")
        shutil.rmtree(installer.place.of(revision))
        print(f"{installer.place.component}: removed {revision}, it did not match its bundle", flush=True)

    # progress(done, total) for the phase that runs: percent and bytes, told at most every PROGRESS_EVERY s and at the
    # end.
    def _progress(self):
        told = -PROGRESS_EVERY

        def tell(done, total):
            nonlocal told
            now = time.monotonic()
            if done < total and now - told < PROGRESS_EVERY:
                return
            told = now
            with self._lock:
                self._phase = {**self._phase, "percent": min(100, done * 100 // total) if total > 0 else None,
                               "bytes": done}
            self._changed()
        return tell

    # Unpacks next to the installations and renames when the tree is good; nothing half stays behind.
    def _unpack(self, component, staged, place):
        partial = bundle.unpack(staged, place.opt, self._progress())
        try:
            problem = self._tree_problem(component, os.path.join(partial, staged.revision))
            if problem:
                self._block(component, staged.revision, problem[0])
                raise Refused(*problem)
            os.rename(os.path.join(partial, staged.revision), place.of(staged.revision))
        finally:
            shutil.rmtree(partial, ignore_errors=True)
        files.sync_dir(place.opt)

    def _install(self, candidate, reset_jobs=False, gate_until=None):
        component, revision = candidate["component"], candidate["revision"]
        installer = self.installers[component]
        place = installer.place
        try:
            self._set_phase("verifying", component=component, revision=revision)
            staged = (self.net if candidate.get("from") == "net" else self.inbox).stage(candidate, self._progress())
            if staged.serial <= self.serial_of(component):
                raise Refused("OLD_SERIAL", f"serial {staged.serial} is not newer than the installed "
                                            f"{self.serial_of(component)}")
            self._check_fits(component, staged)
            target = place.of(revision)
            if os.path.isdir(target) and installer.meta_of(revision) != staged.meta:
                self._replace_stale(installer, revision)
            if not os.path.isdir(target):
                self._check_abort()
                self._set_phase("unpacking")
                installer.prune()
                bundle.check_space(staged, place.opt)
                self._unpack(component, staged, place)
            self._set_phase("waiting_gate")
            self._wait_gate(component, reset_jobs, gate_until)
            ok, line = self._switch(component, revision)
            installer.prune()
            # An OS that halted for want of room in data_dir says nothing about the revision.
            block = not line.startswith(NO_SPACE)
            with self._lock:
                self.state["no_space"] = None if block else {"free": self._data_free(), "t": int(time.time())}
            if ok:
                self._finish("installed", None, line, component, revision)
            elif installer.current() or place.overlay:
                self._finish("rolled_back", "UNHEALTHY", line, component, revision, block=block)
            else:
                self._finish("failed", "UNHEALTHY", line, component, revision, block=block)
        except Aborted:
            self._finish("aborted", None, "aborted before the switch", component, revision)
        except Restarting:
            pass
        except Refused as why:
            self._finish("refused", why.reason, str(why), component, revision)
        except Exception as error:  # noqa: BLE001 - the result has to say it, the host goes on
            self._finish("failed", "INTERNAL", f"{type(error).__name__}: {error}", component, revision)
        finally:
            shutil.rmtree(self.inbox.install_stage(), ignore_errors=True)
            self._release_hold()

    def _roll_back(self, component, previous, reset_jobs=False):
        current = self.installers[component].current()
        try:
            self._set_phase("waiting_gate", component=component, revision=previous)
            self._wait_gate(component, reset_jobs)
            ok, line = self._switch(component, previous, by_hand_from=current)
            if ok:
                # Taken back by hand: the newer one does not come back by itself.
                if current:
                    self._block(component, current, "ROLLED_BACK_BY_HAND", "rolled_back")
                self._finish("installed", None, line, component, previous)
            else:
                self._finish("failed", "UNHEALTHY", line, component, previous)
        except Aborted:
            self._finish("aborted", None, "aborted before the switch", component, previous)
        except Restarting:
            pass
        except Refused as why:
            self._finish("refused", why.reason, str(why), component, previous)
        except Exception as error:  # noqa: BLE001
            self._finish("failed", "INTERNAL", f"{type(error).__name__}: {error}", component, previous)
        finally:
            self._release_hold()

    def _env(self):
        return {**self.paths.env(), "PATH": os.environ.get("PATH", "/usr/sbin:/usr/bin:/sbin:/bin"),
                "HOME": "/nonexistent"}

    # Whether an unfinished switch of component may be undone now, None when it may, else why not: an OS that runs may
    # mow by now, so it is stopped only at rest on the station with the hold granted, as for a switch.
    def _recover_gate(self, component):
        if not self._stops_os(component):
            return None
        state, _, started = self.carrier.show()
        if state not in SETTLED:
            return "BUSY", "MowBite OS is starting or stopping"
        if state != "active" or gate.off(self.paths, started) is not None:
            return None
        broken = self._broken(started)
        if broken is not None:
            print(f"{component}: the running OS is undone without its gate: {broken}", flush=True)
            return None
        jobs = component == "planner"
        why = self._closed(jobs, True, recovering=True)
        if why is not None:
            return why
        if self.judges_by_broker and not self.connected():
            return "NO_BROKER", "the broker is not connected"
        why = self._ask_hold(jobs, recovering=True)
        if why is not None:
            self._recover_after = time.monotonic() + RECOVER_RETRY
        return why

    # Why the OS that runs while an unfinished switch waits is undone without its gate, None while it is not: it did
    # not answer healthy once in HEALTH_WAIT of looks, or an emergency since the update stands after them (rule M5),
    # and its files say it does not move the mower. Nobody could bring such an OS to its gate.
    def _broken(self, started):
        health = self.installer.health
        judged = self._judged
        if judged is None or judged["started"] != started:
            judged = self._judged = {"started": started, "t": time.monotonic(), "healthy": False, "line": ""}
        if not judged["healthy"]:
            ok, judged["line"], _ = health.once(started or 0.0)
            judged["healthy"] = ok is True
        if time.monotonic() - judged["t"] < health.wait:
            return None
        if judged["healthy"]:
            settled, why = health.probe.settled()
            if settled:
                return None
        else:
            why = f"never healthy: {judged['line'] or 'no answer'}"
        session = gate.session_of_this_boot(self.paths)
        if (session is not None and session.get("clean") is False and session.get("emergency") is not True and
                session.get("state") != "IDLE"):
            return None
        return why

    # _broken for the OS that runs now, also while recover_due does not ask for its hold.
    def _running_broken(self):
        if not any(self._stops_os(component) and os.path.lexists(installer.place.pending)
                   for component, installer in self.installers.items()):
            self._judged = None
            return None
        state, _, started = self.carrier.show()
        if state != "active" or gate.off(self.paths, started) is not None:
            return None
        return self._broken(started)

    # At the start of the host: every component back from a switch that was cut off; a revision cut off MAX_ATTEMPTS
    # times is blocked.
    def recover(self, start_stopped=False):
        for component, installer in self.installers.items():
            if os.path.lexists(installer.place.pending):
                why = self._recover_gate(component)
                if why is not None:
                    if self._recover_waits.get(component) != why:
                        print(f"{component}: an unfinished update waits for the gate: {why[1]}", flush=True)
                    self._recover_waits[component] = why
                    continue
            self._recover_waits.pop(component, None)
            try:
                pending = installer.recover(start_stopped)
            finally:
                self._release_hold()
            installer.prune()
            if pending is None:
                continue
            if self._stops_os(component):
                self._judged = None
            revision = pending.get("to") or None
            attempts = self._entry(component)["attempts"]
            message = f"the switch to {revision} was cut off; back to {pending.get('from') or 'none'}"
            blocked = bool(revision) and attempts.get(revision, 0) >= MAX_ATTEMPTS
            if blocked:
                attempts.pop(revision)
            self._finish("failed", "INTERRUPTED", message, component, revision, block=blocked)

    # From the main loop: an unfinished switch that waits for the gate is undone once it opens or the OS is broken.
    def recover_due(self):
        if self.busy() or not gate.unfinished(self.paths):
            return
        if time.monotonic() < self._recover_after and self._running_broken() is None:
            return
        self.recover()

    def clock_ok(self):
        return os.path.exists(self.paths.prefix + "/run/systemd/timesync/synchronized")

    def _data_free(self):
        try:
            return shutil.disk_usage(self.paths.data).free
        except OSError:
            return 0

    # Whether the automatic leaves a component that restarts the OS alone: the last switch halted for want of room in
    # data_dir and it has not got NO_SPACE_MORE since, else the same stop, switch and way back came every night.
    def _wants_room(self, component):
        no_space = self.state["no_space"]
        return (bool(no_space) and self._stops_os(component) and
                self._data_free() < no_space.get("free", 0) + NO_SPACE_MORE)

    # Called every few minutes: starts one available update at night when auto is on, the clock is sure and the app
    # the user has is new enough for it; each revision once a night, then the next component has its turn.
    def tick(self, now=None):
        auto = self.state["auto"]
        if not auto["enabled"] or self.busy() or not self.clock_ok():
            return
        now = time.time() if now is None else now
        if not in_window(time.localtime(now), auto["from"], auto["to"]):
            return
        # An OS stopped by hand stays as it is, and its planner with it; one that gave up after crashes gets its
        # update, as does one that was never installed. The app and the host do not touch the OS.
        runs = self.carrier.show()[0] in ("active", "failed")
        left_alone = {"os": bool(self.installer.current()) and not runs, "planner": not runs}
        self.check()
        platform = self._platform()
        for component in AUTO_ORDER:
            available = self._available(component, platform)
            if (left_alone.get(component) or not available or available["blocked"] or self._wants_room(component) or
                    not app_suffices(self.state["app_version"], available.get("min_app"))):
                continue
            entry = self._entry(component)
            tried = entry["auto_tried"] or {}
            if tried.get("revision") == available["revision"] and 0 <= now - tried.get("t", 0) < AUTO_AGAIN:
                continue
            with self._lock:
                entry["auto_tried"] = {"revision": available["revision"], "t": int(now)}
                self._save_or_log()
            try:
                # The switch begins within the window or not at all.
                self.start(component, available["revision"], by_hand=False,
                           gate_until=time.monotonic() + window_left(time.localtime(now), auto["to"]))
            except Refused as why:
                print(f"auto: {why}", flush=True)
            return
