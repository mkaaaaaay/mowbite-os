"""Switching the installed OS: /opt/mowbite/<revision> with the links current and previous.

A power cut leaves current on the old or the new installation, never half; pending says that a switch has not been
confirmed healthy, and recover goes back to the one before. Two installations are kept. From the stop of the old OS
until the updater lets go, hold.json holds whichever OS runs.
"""
import json
import os
import shutil
import time

from mowbite_host import Refused, files, gate

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


class FilesOnly:
    """The probe without a broker: the files alone decide."""

    def begin(self):
        pass

    def __call__(self):
        return True, ""

    def settled(self):
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

    # (False, why) when it failed for sure, (True, line) when it answers healthy, (None, why) while it comes up.
    def once(self, since):
        if not self.carrier.active():
            return False, "the service does not run"
        if self._since(self.paths.halt, since):
            return False, f"halted: {gate.halt_reason(self.paths)}"
        note = files.read_json(self.paths.exit_note)
        if note and isinstance(note.get("t"), (int, float)) and note["t"] >= since - CLOCK_SLACK:
            return False, f"the container ended with exit code {note.get('exit_code')}"
        session = gate.session_of_this_boot(self.paths)
        if session is None or session.get("clean") is not False or not self._since(self.paths.session, since):
            return None, "the logic has not started"
        ok, line = self.probe()
        return (True, line) if ok else (None, line)

    def wait_healthy(self, since):
        deadline = time.monotonic() + self.wait
        first_ok = None
        line = ""
        while True:
            ok, line = self.once(since)
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


# Health as the entry points use it; MOWBITE_UPDATE_HEALTH_WAIT and _STABLE (s) shorten it for tests.
def health_of(paths, carrier, probe=None):
    return Health(paths, carrier, probe, wait=float(os.environ.get("MOWBITE_UPDATE_HEALTH_WAIT", HEALTH_WAIT)),
                  stable=float(os.environ.get("MOWBITE_UPDATE_HEALTH_STABLE", HEALTH_STABLE)))


class Installer:
    def __init__(self, paths, carrier, health):
        self.paths, self.carrier, self.health = paths, carrier, health
        # False for a caller without the broker: it could not end the hold in the OS it started.
        self.hold = True

    def current(self):
        return files.link_target(self.paths.current)

    def previous(self):
        return files.link_target(self.paths.previous)

    def pending(self):
        return files.read_json(self.paths.pending)

    def meta_of(self, revision):
        return files.read_json(os.path.join(self.paths.opt_of(revision), "bundle.json")) if revision else None

    # ran: whether the OS ran before the switch, so that a host started again after a cut starts it again.
    def _write_pending(self, to, before, ran):
        files.write_json(self.paths.pending, {"to": to, "from": before or "", "ran": ran})

    def _clear_pending(self):
        files.remove(self.paths.pending)
        files.sync_dir(self.paths.opt)

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
            raise
        try:
            self._hold()
            files.set_link(self.paths.current, revision)
            log(f"current -> {revision}, starting")
            since = time.time()
            ok, line = self._start()
            if ok is None:
                on_started()
                ok, line = self.health.wait_healthy(since)
            if ok:
                log(f"healthy: {line}")
                if before:
                    files.set_link(self.paths.previous, before)
                self._clear_pending()
                return True, line
        except Exception as error:
            log(f"error in the switch: {type(error).__name__}: {error}")
            self._back(before)
            raise
        log(f"not healthy: {line}")
        self._back(before)
        return False, line

    # Back to the installation before the switch, or off without one; a stop that fails does not keep current on the
    # new one.
    def _back(self, before):
        try:
            self.carrier.stop()
        except Exception as error:  # noqa: BLE001 - going back goes on
            log(f"stop before going back: {type(error).__name__}: {error}")
        if before:
            files.set_link(self.paths.current, before)
            log(f"back to {before}")
            try:
                since = time.time()
                back, back_line = self._start()
                if back is None:
                    back, back_line = self.health.wait_healthy(since)
            except Exception as error:  # noqa: BLE001 - current points back, the record of the switch ends
                back, back_line = False, f"{type(error).__name__}: {error}"
            log(f"{'healthy' if back else 'NOT HEALTHY'} after going back: {back_line}")
        else:
            files.remove(self.paths.current)
            files.sync_dir(self.paths.opt)
            log("no installation before this one: the OS stays off")
        self._clear_pending()

    # An update that ended before its health was confirmed (power cut, killed): back to the installation before it,
    # started again when it ran (start_stopped: also when it ran before the switch; never at boot, where the service
    # starts after this). The pending record, None without one.
    def recover(self, start_stopped=False):
        pending = self.pending()
        if pending is None:
            if os.path.lexists(self.paths.pending):
                pending = {}
            else:
                return None
        was_active = self.carrier.active()
        if was_active:
            self.carrier.stop()
        before = pending.get("from")
        if before and os.path.isdir(self.paths.opt_of(before)):
            files.set_link(self.paths.current, before)
            log(f"unfinished update to {pending.get('to')}: back to {before}")
        else:
            files.remove(self.paths.current)
            log(f"unfinished update to {pending.get('to')} without an installation before it: the OS stays off")
        self._clear_pending()
        if before and (was_active or (start_stopped and pending.get("ran") is True)):
            self.carrier.start()
        return pending

    # Keeps current, previous and what pending names; removes other installations and broken unpacks.
    def prune(self):
        keep = {self.current(), self.previous()}
        keep.update((self.pending() or {}).values())
        for name in os.listdir(self.paths.opt):
            path = self.paths.opt_of(name)
            if name in ("current", "previous", "pending") or name in keep or os.path.islink(path):
                continue
            if os.path.isdir(path):
                shutil.rmtree(path, ignore_errors=True)
            else:
                files.remove(path)
            log(f"removed {name}")

    def status_lines(self):
        lines = [f"current: {self.current() or '-'}", f"previous: {self.previous() or '-'}",
                 f"service: {'active' if self.carrier.active() else 'inactive'}"]
        if os.path.lexists(self.paths.pending):
            lines.append(f"unfinished update: {json.dumps(self.pending())}")
        if os.path.exists(self.paths.crash):
            lines.append(f"crash record: {self.paths.crash}")
        if os.path.lexists(self.paths.restart_hold):
            lines.append(f"restart hold: {self.paths.restart_hold}")
        return lines


def require_previous(installer):
    previous = installer.previous()
    if not previous or not os.path.isdir(installer.paths.opt_of(previous)):
        raise Refused("NO_PREVIOUS", "no previous installation")
    return previous
