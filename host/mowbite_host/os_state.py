"""How the OS is, from systemd and its files only: os of mowbite/host/status.

state: updating while the updater switches or checks; stopped (inactive by hand); down when it stays off after crashes
(the container after its last restart, or systemd gave up); halted with the reason of the halt line; starting until
the logic wrote its session after the start; then running.
"""
import os
import re
import time

from mowbite_host import files, gate
from mowbite_host.installer import CLOCK_SLACK

HALT_REASON = re.compile(r"reason=([A-Z_]+)")
# A note older than its stable_s does not count as a restart any more (the launch counts the same way).
STABLE_DEFAULT = 600


def halt_reason(paths):
    try:
        with open(paths.halt) as f:
            match = HALT_REASON.search(f.read(4096))
    except OSError:
        return None
    return match.group(1) if match else "DATA_ERROR"


# container_exit.json as written by the launch, None when missing or broken.
def exit_note(paths):
    note = files.read_json(paths.exit_note)
    if note is None or not isinstance(note.get("t"), (int, float)):
        return None
    return note


class OsWatch:
    """read() gives os; a new end of the container counts once into the updater (crashes_since_install, last_crash)."""

    def __init__(self, paths, carrier, updater):
        self.paths, self.carrier, self.updater = paths, carrier, updater
        self._state = None
        self._since = None

    def _base(self, active, active_since, note):
        if self.updater.phase() in ("switching", "checking"):
            return "updating", None
        if active == "failed":
            return "down", "CRASH_LOOP"
        if active in ("inactive", "deactivating", "unknown"):
            return "stopped", None
        if active != "active":
            return "starting", None
        if os.path.exists(self.paths.halt):
            return "halted", halt_reason(self.paths)
        if note and note.get("down") is True:
            return "down", "CRASH_LOOP"
        session = gate.session_of_this_boot(self.paths)
        try:
            written = os.stat(self.paths.session).st_mtime
        except OSError:
            written = 0
        if session is None or session.get("clean") is not False or (active_since and
                                                                     written < active_since - 1 - CLOCK_SLACK):
            return "starting", None
        return "running", None

    def read(self, now=None):
        now = time.time() if now is None else now
        active, changed, active_since = self.carrier.show()
        note = exit_note(self.paths)
        state, reason = self._base(active, active_since, note)
        if state != self._state:
            # The first look takes the time systemd knows; later changes the time they were seen.
            self._since = int(changed if self._state is None and changed else now)
            self._state = state
        if note:
            self.updater.note_crash("os", int(note["t"]))
        restarts = 0
        if note and isinstance(note.get("restart"), int):
            stable = note.get("stable_s") if isinstance(note.get("stable_s"), (int, float)) else STABLE_DEFAULT
            if note.get("down") is True or now - note["t"] < stable:
                restarts = note["restart"]
        stalled = note.get("stalled") if note and isinstance(note.get("stalled"), dict) else None
        return {"state": state, "reason": reason, "since": self._since, "restarts": restarts,
                "last_crash": self.updater.last_crash("os"), "revision": self.updater.installer.current(),
                "stalled": stalled}
