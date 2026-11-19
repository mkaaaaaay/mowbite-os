"""Installs a signed bundle of MowBite OS, its planner, its app or mowbite-host by hand (SSH), with the same checks
as mowbite-host.

  mowbite-update install [--reset-jobs] <bundle.tar.gz> [<signature>]   (signature default <bundle>.sig)
  mowbite-update rollback [--reset-jobs] [os|planner|app|host]          back to the installation before (default os)
  mowbite-update recover                                  after an update that did not finish (at boot)
  mowbite-update status
  mowbite-update unfinished                               0 when no update is unfinished, else 1 and which

A bundle is a tar of one directory <revision>/ with bundle.json (format 2: component, revision, serial, ...) beside
what the component brings; its component is the one whose namespace (mowbite-os, mowbite-planner, mowbite-app,
mowbite-host in /etc/mowbite/allowed_signers) the signature holds for. Before the switch of the OS: a serial newer
than the installed one, the base and runtime packages of this rootfs, every library found (checked as the user
mowbite). The switch only while the OS is stopped, or idles on the charging contacts without an emergency and holds
for the update (system.hold over the broker of the OS, the gate reads the files). After it the OS has to come up
healthy, else it goes back; without the broker its files alone decide. The planner has to go with the installed OS
and switches like the OS, once the mowing job is finished (--reset-jobs: also with an unfinished one, the logic then
begins its area again). The app has to go with the installed OS and switches at any time. Without an installation
before it, the planner and the app go back to the one the OS brings along. A bundle of the host is installed while
mowbite-host is stopped and tried at its next start; without an installation before it, the host goes back to its
seed in the rootfs.

unfinished answers a tool of the rootfs that stops the OS itself (mowbite-measure). Such a tool stops mowbite-host and
holds /run/mowbite-host.lock while it works, which keeps mowbite-host, install, rollback and recover off, and goes on
only after a 0. A bundle of the host does not renew that tool: the command, its answer and the lock stay as they are.
"""
import fcntl
import os
import shutil
import sys
import time

from mowbite_host import Refused, bundle, carrier, gate, installer, origin, paths, updater

# How long install and rollback wait for the broker before they go on without one; 0: never asked.
BROKER_WAIT = float(os.environ.get("MOWBITE_UPDATE_BROKER_WAIT", 5.0))


class NoBroker:
    """Without the broker the OS cannot be asked for its hold: only a stopped OS switches."""

    def request_hold(self, seconds):
        pass

    def release_hold(self):
        pass


# The broker of the OS for system.hold and the health check as mowbite-host does them; None when it does not answer.
def connect(where):
    if BROKER_WAIT <= 0:
        return None, None
    from mowbite_host import broker
    client, rpc, check = broker.attach(where, "mowbite-update", [])
    client.start()
    deadline = time.monotonic() + BROKER_WAIT
    while not client.connected and time.monotonic() < deadline:
        time.sleep(0.1)
    if not client.connected:
        client.stop()
        return None, None
    return rpc, check


def make(prefix, with_broker):
    where = paths.Paths(prefix)
    service = carrier.of(where)
    rpc, check = connect(where) if with_broker else (None, None)
    if with_broker and rpc is None:
        print("no broker: only a stopped OS switches, its health is judged from its files", file=sys.stderr)
    health = installer.health_of(where, service, check)
    tool = updater.Updater(where, service, health, origin.Inbox(where), rpc or NoBroker())
    if rpc is None:
        for one in tool.installers.values():
            one.hold = False
    if check is not None:
        check.api_of_running = tool.running_api
    # By hand the gate is judged once, there is no waiting for it.
    tool.gate_wait = 0
    return tool


# The candidate of a bundle given by path, of the component its signature holds for: read once here, read again
# from its own copy by the install.
def candidate_of(tool, path, signature):
    folder = os.path.join(tool.paths.stage, "cli")
    try:
        staged = bundle.stage_any(path, signature, folder, tool.paths.signers)
    finally:
        shutil.rmtree(folder, ignore_errors=True)
    return {"component": staged.meta["component"], "revision": staged.revision, "serial": staged.serial,
            "path": path, "sig": signature}


def wait_and_report(tool):
    last = tool.join() or {}
    if tool.phase() == "checking":
        print("switched: mowbite-host tries it at its next start (systemctl start mowbite-host)", flush=True)
        return True
    print(f"{last.get('result')}: {last.get('reason') or ''} {last.get('message', '')}".strip(), flush=True)
    return last.get("result") == "installed"


def main(argv):
    prefix = os.environ.get("MOWBITE_UPDATE_PREFIX", "")
    args = [arg for arg in argv[1:] if arg != "--reset-jobs"]
    reset_jobs = len(args) < len(argv) - 1
    command = args[0] if args else None
    if command == "install" and len(args) in (2, 3):
        signature = args[2] if len(args) == 3 else args[1] + ".sig"
    elif command == "rollback" and len(args) <= 2 and (args[1:] or ["os"])[0] in updater.COMPONENTS:
        component = (args[1:] or ["os"])[0]
    elif args not in (["recover"], ["status"], ["unfinished"]) or reset_jobs:
        print(__doc__, file=sys.stderr)
        return 2
    tool = make(prefix, command in ("install", "rollback"))
    if command == "status":
        for one in tool.installers.values():
            print("\n".join(one.status_lines()))
        return 0
    if command == "unfinished":
        lines = [line for one in tool.installers.values() for line in one.unfinished_lines()]
        if lines:
            print("\n".join(lines))
        return 1 if lines else 0
    for place in (*tool.paths.places.values(), tool.paths.host):
        os.makedirs(place.opt, exist_ok=True)
    os.makedirs(os.path.dirname(tool.paths.lock), exist_ok=True)
    with open(tool.paths.lock, "w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("mowbite-host or another mowbite-update runs: stop mowbite-host or update over the app",
                  file=sys.stderr)
            return 1
        try:
            if command == "recover":
                tool.recover()
                installer.settle(tool.paths)
                return 1 if gate.unfinished(tool.paths) else 0
            if command == "rollback":
                tool.rollback(component, reset_jobs)
            else:
                candidate = candidate_of(tool, args[1], signature)
                tool.offer(candidate)
                tool.start(candidate["component"], candidate["revision"], reset_jobs=reset_jobs)
            return 0 if wait_and_report(tool) else 1
        except Refused as why:
            print(f"refused: {why.reason}: {why}", file=sys.stderr)
            return 1
