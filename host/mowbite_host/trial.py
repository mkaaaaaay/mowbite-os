"""The trial period of a newly written rootfs (Tron, /usr/lib/mowbite/probezeit): without a confirmed start the mower
reboots into the loader mode of U-Boot after 10 min or at its third boot. host.confirm_start stands in for the SSH
login of a maintainer: every call counts as one, since the mower was reached over the network, and the period ends
once the start is good.
"""
import os

from mowbite_host import Refused
from mowbite_host.command import nmcli_fields


def active(paths):
    return os.path.lexists(paths.trial)


def _failed_units(run):
    code, out, err = run(["systemctl", "--failed", "--plain", "--no-legend", "--no-pager"])
    if code != 0:
        return [f"systemctl: {err or code}"]
    units = [line.split()[0] for line in out.splitlines() if line.split()]
    return [f"failed: {', '.join(units)}"] if units else []


def _wifi(run):
    try:
        code, out, _ = run(["nmcli", "-t", "-f", "TYPE,STATE", "device"])
    except FileNotFoundError:
        return ["no NetworkManager"]
    if code == 0 and any(nmcli_fields(line)[:2] == ["wifi", "connected"] for line in out.splitlines()):
        return []
    return ["no WLAN connection"]


# os_now: os of mowbite/host/status; first_runs: whether the first installation is still at work. The answer is
# {"trial": false} once the period is over, also when there was none.
def confirm(paths, run, os_now, first_runs):
    if not active(paths):
        return {"trial": False}
    try:
        code, _, err = run([paths.trial_tool, "ssh"])
    except FileNotFoundError:
        raise Refused("INTERNAL", "the trial period has no tool on this rootfs") from None
    if code != 0:
        raise Refused("INTERNAL", f"the start was not counted as confirmed: {err or code}")
    if first_runs:
        raise Refused("BUSY", "the first installation runs")
    problems = _failed_units(run) + _wifi(run)
    if os_now["state"] != "running":
        problems.append(f"MowBite OS is {os_now['state']}")
    if problems:
        raise Refused("START_NOT_OK", "; ".join(problems))
    code, _, err = run([paths.trial_tool, "end"])
    if code != 0 or active(paths):
        raise Refused("INTERNAL", f"the trial period did not end: {err or code}")
    return {"trial": False}
