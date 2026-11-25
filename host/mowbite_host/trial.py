"""The trial period of a newly written rootfs (Tron, /usr/lib/mowbite/probezeit): without a confirmed start the mower
reboots into the loader mode of U-Boot after 10 min or at its third boot. host.confirm_start stands in for the SSH
login of a maintainer, but only while the first installation runs (so the period does not cut it off) or once the
start is good: a caller asking again and again keeps a bad start from going back on its own.
"""
import hashlib
import hmac
import os

from mowbite_host import Refused, wifi


def active(paths):
    return os.path.lexists(paths.trial)


# Units without which the start is bad: they put the pack of the installer (modules, WLAN, bundles) in place. Any other
# failed unit is a note in the answer; the OS tells what it lacks in its health.
COUNTED = ("mowbite-unpack.service", "mowbite-unpack-inbox.service")


# (problems, notes)
def _failed_units(run):
    code, out, err = run(["systemctl", "--failed", "--plain", "--no-legend", "--no-pager"])
    if code != 0:
        return [f"systemctl: {err or code}"], []
    units = [line.split()[0] for line in out.splitlines() if line.split()]
    counted = [unit for unit in units if unit in COUNTED]
    return ([f"failed: {', '.join(counted)}"] if counted else []), [f"failed: {unit}" for unit in units
                                                                      if unit not in COUNTED]


# The proof that the answer comes from the mower the installer wrote: HMAC-SHA256 of the challenge under the install
# id of its pack; None without a challenge or an id.
def proof(paths, challenge):
    if challenge is None:
        return None
    try:
        with open(paths.install_id, "rb") as f:
            key = f.read().strip()
    except OSError:
        return None
    return hmac.new(key, challenge.encode(), hashlib.sha256).hexdigest() if key else None


def _reached(paths, run):
    try:
        code, _, err = run([paths.trial_tool, "ssh"])
    except FileNotFoundError:
        raise Refused("INTERNAL", "the trial period has no tool on this rootfs") from None
    if code != 0:
        raise Refused("INTERNAL", f"the start was not counted as confirmed: {err or code}")


# os_now: os of mowbite/host/status; first_runs: whether the first installation is still at work; challenge: text of
# the caller for proof(). The answer is {"trial": false, proof?, notes?} once the period is over, also when there was
# none.
def confirm(paths, run, os_now, first_runs, challenge=None):
    answer = {"trial": False}
    signed = proof(paths, challenge)
    if signed is not None:
        answer["proof"] = signed
    if not active(paths):
        return answer
    if first_runs:
        _reached(paths, run)
        raise Refused("BUSY", "the first installation runs")
    problems, notes = _failed_units(run)
    problems += wifi.problems(run)
    if os_now["state"] != "running":
        problems.append(f"MowBite OS is {os_now['state']}")
    if problems:
        raise Refused("START_NOT_OK", "; ".join(problems + notes))
    _reached(paths, run)
    code, _, err = run([paths.trial_tool, "end"])
    if code != 0 or active(paths):
        raise Refused("INTERNAL", f"the trial period did not end: {err or code}")
    if notes:
        answer["notes"] = notes
    return answer
