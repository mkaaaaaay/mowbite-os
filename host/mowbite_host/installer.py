"""Switching an installed component: <opt>/<revision> with the links current and previous, the OS under /opt/mowbite,
the planner and the app in folders of their own.

A power cut leaves current on the old or the new installation, never half; pending says that a switch has not been
confirmed healthy, and recover goes back to the one before. Two installations are kept. From the stop of the old OS
until the updater lets go, hold.json holds whichever OS runs.

The OS brings a planner and an app along (<os>/planner, <os>/app). An own installation of them is used only while
the link active names it: settle sets the link after every change of a current and takes it away when the own
installation does not go with the OS or is not newer than the one the OS brings along. The readers (os-setup, the
web server) follow active and else take what the OS brings along; without a current, back is that one as well.
"""
import json
import os
import shutil
import stat
import time

from mowbite_host import Refused, compat, files, gate

HEALTH_WAIT = 240.0
# After the first healthy answer the OS has to stay healthy this long (rule M5 of the updater contract).
HEALTH_STABLE = 60.0
HEALTH_EVERY = 2.0
# The hold of a switch covers the stop, the health checks of the new OS and of the one before it when it goes back;
# the OS ends a hold at most after 1800 s.
HOLD_SECONDS = 900
# File times run on a coarse clock, up to a tick behind time.time().
CLOCK_SLACK = 0.1


def log(text):
    print(text, flush=True)


def meta_of(place, revision):
    return files.read_json(os.path.join(place.of(revision), "bundle.json")) if revision else None


# bundle.json of the installed OS, {} without one.
def os_meta(paths):
    return meta_of(paths.os, files.link_target(paths.os.current)) or {}


# Why the own installation at place is not the one to use, None when it is or when there is none.
def skipped(paths, place):
    revision = files.link_target(place.current)
    if revision is None:
        return None
    return compat.skip_own(place.component, meta_of(place, revision) or {}, os_meta(paths))


# active of the planner and the app: on their current while that one is to be used, else gone. Only the host writes
# it, within pending and before the OS starts; the same files give the same links.
def settle(paths):
    for place in paths.places.values():
        if not place.overlay:
            continue
        revision = files.link_target(place.current)
        if revision and skipped(paths, place) is None:
            if files.link_target(place.active) != revision:
                files.set_link(place.active, revision)
                log(f"{place.component}: active -> {revision}")
        elif os.path.lexists(place.active):
            files.remove(place.active)
            files.sync_dir(place.opt)
            log(f"{place.component}: no active, the one of the OS is used")


class FilesOnly:
    """The probe without a broker: the files alone decide."""

    def begin(self):
        pass

    def __call__(self, since):
        return True, ""

    def settled(self):
        return True, ""

    def planner(self):
        return True, ""


class Health:
    """Whether the OS that started at wall time since came up: the service runs, no halt, no crash of the container,
    the logic wrote its session in this boot, and probe() (the answers over the broker) agrees; probe.settled() has the
    last word at the end of the stable time, probe.begin() runs before the switch."""

    def __init__(self, paths, carrier, probe=None, wait=HEALTH_WAIT, stable=HEALTH_STABLE, every=HEALTH_EVERY):
        self.paths, self.carrier = paths, carrier
        self.probe = probe or FilesOnly()
        self.wait, self.stable, self.every = wait, stable, every

    def begin(self):
        self.probe.begin()

    def _since(self, path, since):
        try:
            return os.lstat(path).st_mtime >= since - CLOCK_SLACK
        except OSError:
            return False

    # (False, why) when it failed for sure, (True, line) when it answers healthy, (None, why) while it comes up; with
    # the time the service became active. entered: that time from the first look, a later one is a restart.
    def once(self, since, entered=None):
        state, _, now_entered = self.carrier.show()
        if state != "active":
            return False, "the service does not run", now_entered
        if entered is not None and now_entered != entered:
            return False, "the service started again", now_entered
        return (*self._answers(since), now_entered)

    def _answers(self, since):
        if self._since(self.paths.halt, since):
            return False, f"halted: {gate.halt_reason(self.paths)}"
        note = files.read_json(self.paths.exit_note)
        if note and isinstance(note.get("t"), (int, float)) and note["t"] >= since - CLOCK_SLACK:
            return False, f"the container ended with exit code {note.get('exit_code')}"
        session = gate.session_of_this_boot(self.paths)
        if session is None or session.get("clean") is not False or not self._since(self.paths.session, since):
            return None, "the logic has not started"
        ok, line = self.probe(since)
        return (True, line) if ok else (None, line)

    def wait_healthy(self, since):
        deadline = time.monotonic() + self.wait
        first_ok = None
        line = ""
        entered = None
        while True:
            ok, line, now_entered = self.once(since, entered)
            entered = entered or now_entered
            if ok is False:
                return False, line
            if ok and first_ok is None:
                first_ok = time.monotonic()
            if ok is None and first_ok is not None:
                return False, f"no longer healthy: {line}"
            now = time.monotonic()
            if first_ok is not None and now - first_ok >= self.stable:
                settled, why = self.probe.settled()
                return (True, line) if settled else (False, why)
            if first_ok is None and now >= deadline:
                return False, line or "no answer"
            time.sleep(self.every)


class WithPlanner:
    """Health of the OS for a switch of its planner: at the end the planner has to answer as well."""

    def __init__(self, health):
        self.health = health

    def begin(self):
        self.health.begin()

    def wait_healthy(self, since):
        ok, line = self.health.wait_healthy(since)
        if ok:
            answers, why = self.health.probe.planner()
            if not answers:
                return False, why
        return ok, line


class AppFiles:
    """Health of the app, which nothing on the mower runs: the own installation current names has its page and the
    version of its bundle, and the web server, another user, can read all of it. Without a current the app of the OS
    is served."""

    def __init__(self, place):
        self.place = place

    def begin(self):
        pass

    def wait_healthy(self, since):
        revision = files.link_target(self.place.current)
        if revision is None:
            return True, "the app of the OS"
        root = self.place.of(revision)
        app = os.path.join(root, "app")
        version = (files.read_json(os.path.join(app, "version.json")) or {}).get("version")
        bundled = (meta_of(self.place, revision) or {}).get("version")
        if version is None or version != bundled:
            return False, f"version.json says {version}, the bundle {bundled}"
        if not files.regular(os.path.join(app, "index.html")):
            return False, "index.html is missing or no file"
        closed = _closed_to_others(root, app)
        if closed:
            return False, f"not readable for the web server: {os.path.relpath(closed, root)}"
        return True, f"app {version}"


# The first of root and the folders and files under tree that others cannot read; None when they can read all.
def _closed_to_others(root, tree):
    def closed(path):
        mode = os.lstat(path).st_mode
        need = 0o005 if stat.S_ISDIR(mode) else 0o004
        return not stat.S_ISLNK(mode) and mode & need != need
    if closed(root) or closed(tree):
        return root if closed(root) else tree
    for folder, folders, names in os.walk(tree):
        for name in folders + names:
            if closed(os.path.join(folder, name)):
                return os.path.join(folder, name)
    return None


# Health as the entry points use it; MOWBITE_UPDATE_HEALTH_WAIT and _STABLE (s) shorten it for tests.
def health_of(paths, carrier, probe=None):
    return Health(paths, carrier, probe, wait=float(os.environ.get("MOWBITE_UPDATE_HEALTH_WAIT", HEALTH_WAIT)),
                  stable=float(os.environ.get("MOWBITE_UPDATE_HEALTH_STABLE", HEALTH_STABLE)))


class Installer:
    """The installations at place. carrier is what the switch stops and starts: the OS for itself and for its planner,
    carrier.Nothing for the app. A revision of None is the installation the OS brings along (planner and app only)."""

    def __init__(self, paths, place, carrier, health):
        self.paths, self.place, self.carrier, self.health = paths, place, carrier, health
        # False for a caller without the broker: it could not end the hold in the OS it started. False as well when
        # the switch stops no OS.
        self.hold = True

    def current(self):
        return files.link_target(self.place.current)

    def previous(self):
        return files.link_target(self.place.previous)

    def pending(self):
        return files.read_json(self.place.pending)

    def meta_of(self, revision):
        return meta_of(self.place, revision)

    # ran: whether the OS ran before the switch, so that a host started again after a cut starts it again.
    def _write_pending(self, to, before, ran):
        files.write_json(self.place.pending, {"to": to or "", "from": before or "", "ran": ran})

    def _clear_pending(self):
        files.remove(self.place.pending)
        files.sync_dir(self.place.opt)

    # current -> revision, or no current for None; then active of the planner and the app follows.
    def _set_current(self, revision):
        if revision:
            files.set_link(self.place.current, revision)
        else:
            files.remove(self.place.current)
            files.sync_dir(self.place.opt)
        settle(self.paths)

    def _start(self):
        if not self.carrier.start():
            return False, "the service did not start"
        return None, ""

    # hold.json for the OS that starts next, written while none runs: nothing starts the wheels or the blade until the
    # updater lets go, whatever the broker carried meanwhile. Without the data folder there is no OS to hold yet.
    def _hold(self):
        if not self.hold or not os.path.isdir(os.path.dirname(self.paths.hold)):
            return
        now = int(time.time())
        files.write_new(self.paths.hold, json.dumps({"until": now + HOLD_SECONDS, "set_at": now}) + "\n")

    # current -> revision, previous -> the one before. (True, line) when the OS came up healthy; else it goes back to
    # the one before and gives (False, why). An error on the way goes back as well and is raised after that.
    def switch(self, revision, on_started=lambda: None):
        before = self.current()
        if before == revision:
            before = self.previous()
        self.health.begin()
        ran = self.carrier.active()
        # Written before the stop: a cut while the OS stops is recovered as well.
        self._write_pending(revision, before, ran)
        try:
            self.carrier.stop()
        except Refused:
            self._clear_pending()
            if ran and not self.carrier.active():
                log(f"the stop failed half: {self.place.component} stays on {before or 'the one of the OS'}, "
                    f"starting again")
                self.carrier.start()
            raise
        try:
            self._hold()
            self._set_current(revision)
            log(f"{self.place.component}: current -> {revision or 'none'}, starting")
            since = time.time()
            ok, line = self._start()
            if ok is None:
                on_started()
                ok, line = self.health.wait_healthy(since)
            if ok:
                log(f"healthy: {line}")
                if before:
                    files.set_link(self.place.previous, before)
                elif self.previous() == revision:
                    # Back from the one of the OS to the own one: nothing own lies before it.
                    files.remove(self.place.previous)
                self._clear_pending()
                return True, line
        except Exception as error:
            log(f"error in the switch: {type(error).__name__}: {error}")
            self._back(before)
            raise
        log(f"not healthy: {line}")
        back = self._back(before)
        return False, line if back is None else f"{line}; {back}"

    # Back to the installation before the switch; without one the OS stays off, the planner and the app are those of
    # the OS again. A stop that fails does not keep current on the new one. What went wrong going back, None when the
    # one before came up healthy or there is none to start.
    def _back(self, before):
        try:
            self.carrier.stop()
        except Exception as error:  # noqa: BLE001 - going back goes on
            log(f"stop before going back: {type(error).__name__}: {error}")
        self._set_current(before)
        if before or self.place.overlay:
            log(f"back to {before or 'the one of the OS'}")
            try:
                since = time.time()
                back, back_line = self._start()
                if back is None:
                    back, back_line = self.health.wait_healthy(since)
            except Exception as error:  # noqa: BLE001 - current points back, the record of the switch ends
                back, back_line = False, f"{type(error).__name__}: {error}"
            log(f"{'healthy' if back else 'NOT HEALTHY'} after going back: {back_line}")
            problem = None if back else f"not healthy after going back either: {back_line}"
        else:
            log("no installation before this one: the OS stays off")
            problem = None
        self._clear_pending()
        return problem

    # An update that ended before its health was confirmed (power cut, killed): back to the installation before it,
    # started again when it ran (start_stopped: also when it ran before the switch; never at boot, where the service
    # starts after this). The pending record, None without one.
    def recover(self, start_stopped=False):
        pending = self.pending()
        if pending is None:
            if os.path.lexists(self.place.pending):
                pending = {}
            else:
                return None
        was_active = self.carrier.active()
        if was_active:
            self.carrier.stop()
        before = pending.get("from")
        if not before or not os.path.isdir(self.place.of(before)):
            before = None
        self._set_current(before)
        what = f"{self.place.component}: unfinished update to {pending.get('to') or 'the one of the OS'}"
        if before:
            log(f"{what}: back to {before}")
        elif self.place.overlay:
            log(f"{what} without an installation before it: back to the one of the OS")
        else:
            log(f"{what} without an installation before it: the OS stays off")
        self._clear_pending()
        if (before or self.place.overlay) and (was_active or (start_stopped and pending.get("ran") is True)):
            self.carrier.start()
        return pending

    # Keeps current, previous and what pending names; removes other installations and broken unpacks.
    def prune(self):
        keep = {self.current(), self.previous()}
        keep.update((self.pending() or {}).values())
        for name in os.listdir(self.place.opt):
            path = self.place.of(name)
            if name in ("current", "previous", "pending") or name in keep or os.path.islink(path):
                continue
            if os.path.isdir(path):
                shutil.rmtree(path, ignore_errors=True)
            else:
                files.remove(path)
            log(f"removed {name}")

    def status_lines(self):
        name = self.place.component
        lines = [f"{name} current: {self.current() or '-'}", f"{name} previous: {self.previous() or '-'}"]
        if self.place.overlay:
            why = skipped(self.paths, self.place)
            lines.append(f"{name} in use: {'the one of the OS' if why or not self.current() else 'current'}"
                         f"{f' ({why})' if why else ''}")
        else:
            lines.append(f"service: {'active' if self.carrier.active() else 'inactive'}")
            if os.path.exists(self.paths.crash):
                lines.append(f"crash record: {self.paths.crash}")
            if os.path.lexists(self.paths.restart_hold):
                lines.append(f"restart hold: {self.paths.restart_hold}")
        if os.path.lexists(self.place.pending):
            lines.append(f"{name} unfinished update: {json.dumps(self.pending())}")
        return lines

    # Where a rollback goes: the previous installation, or None for the one of the OS when an own one is installed
    # with none before it.
    def back_target(self):
        previous = self.previous()
        if previous and previous != self.current() and os.path.isdir(self.place.of(previous)):
            return previous
        if self.place.overlay and self.current():
            return None
        raise Refused("NO_PREVIOUS", "no previous installation")
