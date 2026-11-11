"""mowbite-host (mowbite-host.service): updates of MowBite OS, its planner, its app and of itself over the app and how
the OS is, beside the OS as root.

It answers update.* and host.* on the broker of the OS, keeps mowbite/host/status and mowbite/update/status retained,
looks for updates in the inbox every 6 h and on update.check, and installs at night when auto is on. Without the
broker it goes on and reconnects; the gate and the state of the OS come from files only. A new host confirms itself
after mowbite_boot.SETTLE s of its main loop, once connected when the host before it was.
"""
import fcntl
import os
import signal
import socket
import time

import mowbite_boot
from mowbite_host import broker, carrier, installer, origin, os_state, paths, service, updater


# The socket of systemd for this service (Type=notify, WatchdogSec), none outside one. Taken out of the environment:
# a child that inherits NOTIFY_SOCKET reports there too (systemctl its exit status), and systemd logs each message.
NOTIFY_SOCKET = os.environ.pop("NOTIFY_SOCKET", None)


# A message to systemd; nothing without its socket.
def notify(text):
    address = NOTIFY_SOCKET
    if not address:
        return
    if address.startswith("@"):
        address = "\0" + address[1:]
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
        try:
            sock.sendto(text.encode(), address)
        except OSError:
            pass


class Settle:
    """Called from the main loop: after SETTLE s this start counts as good and a new host confirms itself."""

    def __init__(self, tool, client):
        self.tool, self.client = tool, client
        # MOWBITE_HOST_SETTLE (s) shortens it for tests.
        self.at = time.monotonic() + float(os.environ.get("MOWBITE_HOST_SETTLE", mowbite_boot.SETTLE))
        self.done = False

    def __call__(self):
        if self.done or time.monotonic() < self.at:
            return
        if self.tool.host_needs_broker() and not self.client.connected:
            return
        self.done = True
        mowbite_boot.settled()
        if self.tool.confirm_host():
            mowbite_boot.confirmed()


def run():
    stopping = []
    signal.signal(signal.SIGTERM, lambda *_: stopping.append(True))
    signal.signal(signal.SIGINT, lambda *_: stopping.append(True))
    where = paths.Paths(os.environ.get("MOWBITE_UPDATE_PREFIX", ""))
    notify("READY=1")
    # mowbite-update by hand or at boot holds the lock while it works.
    os.makedirs(os.path.dirname(where.lock), exist_ok=True)
    lock = open(where.lock, "w")
    waiting = False
    while True:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            break
        except BlockingIOError:
            if not waiting:
                print("mowbite-update runs: waiting for it", flush=True)
                waiting = True
            if stopping:
                return 0
            notify("WATCHDOG=1")
            time.sleep(1)
    for place in (*where.places.values(), where.host):
        os.makedirs(place.opt, exist_ok=True)

    host = None
    client, rpc, check = broker.attach(where, "mowbite-host", [("rpc/request", 1)],
                                       other=lambda m: host and host.on_request(m),
                                       on_connect=lambda: host and host.connected(),
                                       will=(service.HOST_STATUS, service.OFFLINE))
    service_os = carrier.of(where)
    tool = updater.Updater(where, service_os, installer.health_of(where, service_os, check), origin.Inbox(where), rpc)
    check.api_of_running = tool.running_api
    # A switch of the host ends this process; systemd starts the new one.
    tool.restart_self = lambda: stopping.append(True)
    tool.connected = lambda: client.connected
    tool.host_went_back()
    tool.recover(start_stopped=True)
    installer.settle(where)
    host = service.Host(where, tool, os_state.OsWatch(where, service_os, tool), service_os, client.publish)
    client.start()
    settle = Settle(tool, client)

    def alive():
        notify("WATCHDOG=1")
        settle()
    try:
        host.run(alive=alive, stop=lambda: bool(stopping))
    finally:
        notify("STOPPING=1")
        client.publish(service.HOST_STATUS, service.OFFLINE, True)
        time.sleep(0.2)
        client.stop()
    return 0
