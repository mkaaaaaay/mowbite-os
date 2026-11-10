"""Switching the installed OS: /opt/mowbite/<revision> with the links current and previous.

A power cut leaves current on the old or the new installation, never half; pending says that a switch has not been
confirmed healthy, and recover goes back to the one before. Two installations are kept.
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
# File times run on a coarse clock, up to a tick behind time.time().
CLOCK_SLACK = 0.1


def log(text):
    print(text, flush=True)


class Health:
    """Whether the OS that started at wall time since came up: the service runs, no halt, no crash of the container,
    the logic wrote its session in this boot, and probe() (the answers over the broker) agrees."""

    def __init__(self, paths, carrier, probe=None, wait=HEALTH_WAIT, stable=HEALTH_STABLE, every=HEALTH_EVERY):
        self.paths, self.carrier = paths, carrier
        self.probe = probe or (lambda: (True, ""))
        self.wait, self.stable, self.every = wait, stable, every

    def _since(self, path, since):
        try:
            return os.stat(path).st_mtime >= since - CLOCK_SLACK
        except OSError:
            return False

    # (False, why) when it failed for sure, (True, line) when it answers healthy, (None, why) while it comes up.
    def once(self, since):
        if not self.carrier.active():
            return False, "the service does not run"
        if self._since(self.paths.halt, since):
            try:
                with open(self.paths.halt) as f:
                    return False, f"halted: {f.read().strip()}"
            except OSError:
                return False, "halted"
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
                return True, line
            if first_ok is None and now >= deadline:
                return False, line or "no answer"
            time.sleep(self.every)


class Installer:
    def __init__(self, paths, carrier, health):
        self.paths, self.carrier, self.health = paths, carrier, health

    def current(self):
        return files.link_target(self.paths.current)

    def previous(self):
        return files.link_target(self.paths.previous)

    def pending(self):
        return files.read_json(self.paths.pending)

    def meta_of(self, revision):
        return files.read_json(os.path.join(self.paths.opt_of(revision), "bundle.json")) if revision else None

    def _write_pending(self, to, before):
        files.write_json(self.paths.pending, {"to": to, "from": before or ""})

    def _clear_pending(self):
        files.remove(self.paths.pending)
        files.sync_dir(self.paths.opt)

    def _start(self):
        if not self.carrier.start():
            return False, "the service did not start"
        return None, ""

    # current -> revision, previous -> the one before. (True, line) when the OS came up healthy; else it goes back to
    # the one before and gives (False, why).
    def switch(self, revision, on_started=lambda: None):
        before = self.current()
        if before == revision:
            before = self.previous()
        self.carrier.stop()
        self._write_pending(revision, before)
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
        log(f"not healthy: {line}")
        self.carrier.stop()
        if before:
            files.set_link(self.paths.current, before)
            log(f"back to {before}")
            since = time.time()
            back, back_line = self._start()
            if back is None:
                back, back_line = self.health.wait_healthy(since)
            log(f"{'healthy' if back else 'NOT HEALTHY'} after going back: {back_line}")
        else:
            files.remove(self.paths.current)
            files.sync_dir(self.paths.opt)
            log("no installation before this one: the OS stays off")
        self._clear_pending()
        return False, line

    # An update that ended before its health was confirmed (power cut, killed): back to the installation before it.
    # The pending record, None without one.
    def recover(self):
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
        if was_active and before:
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
