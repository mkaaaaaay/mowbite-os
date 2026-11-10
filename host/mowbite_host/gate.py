"""The gate before a switch, read from the files of the OS only: the broker has no login, so its answers prove nothing.

While the OS runs, it has to idle on the charging contacts without an emergency (os-session.json of this boot) and have
granted the hold the host asked for (hold.json: from then on nothing starts the wheels or the blade). An OS that does
not run moves nothing, nor one that halted before its nodes started or stays down after its crashes. Never with an
unfinished switch of any component, a restart hold or a kernel crash the OS has not judged yet.

A switch of the planner restarts the OS and so passes the same gate; before it the mowing job has to be finished
(checkpoint.json), since the logic begins the area of a job again when its planner changed. The app passes no gate:
its switch touches nothing that runs.
"""
import os
import re

from mowbite_host import files


HALT_REASON = re.compile(r"reason=([A-Z_]+)")


# The reason of the halt line; None without a halt file, DATA_ERROR when it names none.
def halt_reason(paths):
    if not os.path.lexists(paths.halt):
        return None
    match = HALT_REASON.search(files.read_text(paths.halt) or "")
    return match.group(1) if match else "DATA_ERROR"


# How a service that runs keeps the mower still: "halted" before its nodes started, "down" when the container stays
# off after its crashes (the note has to be of this start of the service, started in unix s or None when unknown);
# None when its nodes run.
def off(paths, started):
    if os.path.lexists(paths.halt):
        return "halted"
    note = files.read_json(paths.exit_note)
    if (note and note.get("down") is True and isinstance(note.get("t"), (int, float)) and
            (started is None or note["t"] >= started - 1)):
        return "down"
    return None


def _boot_id(paths):
    try:
        with open(paths.boot_id) as f:
            return f.read().strip()
    except OSError:
        return ""


# os-session.json when the OS wrote it in this boot, else None.
def session_of_this_boot(paths):
    session = files.read_json(paths.session)
    boot = _boot_id(paths)
    if session is None or not boot or session.get("boot_id") != boot:
        return None
    return session


# Whether a switch of some component was not confirmed.
def unfinished(paths):
    return any(os.path.lexists(place.pending) for place in paths.places.values())


# The version of checkpoint.json this host reads (kCheckpointVersion of the logic).
CHECKPOINT_VERSION = 1


# Whether the logic keeps a mowing job it would go on with: a checkpoint beyond its start. One of another version, or
# one that cannot be read, counts as open.
def job_open(paths):
    if not os.path.lexists(paths.checkpoint):
        return False
    checkpoint = files.read_json(paths.checkpoint)
    if checkpoint is None or checkpoint.get("version") != CHECKPOINT_VERSION:
        return True
    progress = [checkpoint.get(key) for key in ("area", "path", "index")]
    return not all(isinstance(value, int) and not isinstance(value, bool) for value in progress) or any(
        value > 0 for value in progress)


# The reason the switch may not happen now, None when it may; active: whether the service of the OS runs, still:
# whether it is halted or down (off).
def before_hold(paths, active, still=False):
    if unfinished(paths):
        return "UNFINISHED", "an update did not finish: recover first"
    # Halted or down nobody answers a release; the file stays and holds the OS that starts next.
    if os.path.lexists(paths.restart_hold) and not still:
        return "EMERGENCY", "the mower is held after an unexpected restart: release it first"
    if os.path.exists(paths.crash) and session_of_this_boot(paths) is None:
        return "KERNEL_CRASH", "a kernel crash in this boot that MowBite OS has not judged yet: start it first"
    if not active or still:
        return None
    session = session_of_this_boot(paths)
    if session is None or session.get("clean") is not False:
        return "BUSY", "MowBite OS is starting or stopping"
    if session.get("emergency") is True:
        return "EMERGENCY", "an emergency stands: release it first"
    if session.get("state") != "IDLE":
        return "BUSY", f"MowBite OS is {session.get('state')}"
    if session.get("charging") is not True:
        return "NO_DOCK_CONTACT", "the mower is not on its charging contacts"
    return None


# Whether the OS granted a hold asked for at wall time since that lasts at least until now + seconds.
def hold_granted(paths, since, now, seconds):
    hold = files.read_json(paths.hold)
    if hold is None:
        return False
    until, set_at = hold.get("until"), hold.get("set_at")
    if not isinstance(until, int) or not isinstance(set_at, int):
        return False
    return set_at >= int(since) - 1 and until >= now + seconds
