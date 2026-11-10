"""The gate before a switch, read from the files of the OS only: the broker has no login, so its answers prove nothing.

While the OS runs, it has to idle on the charging contacts without an emergency (os-session.json of this boot) and have
granted the hold the host asked for (hold.json: from then on nothing starts the wheels or the blade). An OS that does
not run moves nothing. Never with an unfinished switch, a restart hold or a kernel crash the OS has not judged yet.
"""
import os

from mowbite_host import files


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


# The reason the switch may not happen now, None when it may; active: whether the OS runs.
def before_hold(paths, active):
    if os.path.exists(paths.pending):
        return "UNFINISHED", "an update did not finish: recover first"
    if os.path.lexists(paths.restart_hold):
        return "EMERGENCY", "the mower is held after an unexpected restart: release it first"
    if os.path.exists(paths.crash) and session_of_this_boot(paths) is None:
        return "KERNEL_CRASH", "a kernel crash in this boot that MowBite OS has not judged yet: start it first"
    if not active:
        return None
    session = session_of_this_boot(paths)
    if session is None or session.get("clean") is not False:
        return "BUSY", "MowBite OS is starting or stopping"
    if session.get("emergency") is True:
        return "EMERGENCY", "an emergency stands: release it first"
    if session.get("state") != "IDLE":
        return "BUSY", f"MowBite OS is {session.get('state')}"
    if session.get("charging") is not True:
        return "NOT_CHARGING", "the mower is not on its charging contacts"
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
