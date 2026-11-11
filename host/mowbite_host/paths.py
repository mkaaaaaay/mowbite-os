"""Where mowbite-host finds and keeps its files; tests give a prefix, on the mower it is empty."""
import os

import mowbite_boot

# The OS reads these files from the environment of its service, the last one winning.
ENV_FILES = ("/usr/lib/mowbite/os.defaults", "/etc/mowbite/os.conf")


# The folder of each component and whether its installations lie over the one the OS brings along.
PLACES = {"os": ("/opt/mowbite", False), "planner": ("/opt/mowbite-planner", True), "app": ("/opt/mowbite-app", True)}


class Place:
    """The installations of a component: <opt>/<revision>, current, previous, pending. overlay: the OS brings one
    along as <os>/<component>, and the link active names the own one while it is the one to use."""

    def __init__(self, component, opt, overlay):
        self.component, self.opt, self.overlay = component, opt, overlay
        self.current = self.of("current")
        self.previous = self.of("previous")
        self.pending = self.of("pending")
        self.active = self.of("active")

    def of(self, name):
        return os.path.join(self.opt, name)


class Paths:
    def __init__(self, prefix="", data_dir="/userdata/mowbite"):
        p = prefix
        self.prefix = prefix
        self.places = {component: Place(component, p + opt, overlay) for component, (opt, overlay) in PLACES.items()}
        self.os = self.places["os"]
        self.signers = p + "/etc/mowbite/allowed_signers"
        # The version of the rootfs or base image, an integer; a bundle names the oldest it runs on.
        self.base = p + "/usr/lib/mowbite/base"
        self.os_setup = p + "/usr/lib/mowbite/os-setup"
        self.env_files = [p + path for path in ENV_FILES]
        self.inbox = p + "/userdata/mowbite-update/inbox"
        # The installations of the host itself and the seed in the rootfs; mowbite_boot chooses between them.
        self.host = Place("host", p + mowbite_boot.OPT, False)
        self.host_seed = p + mowbite_boot.SEED
        self.host_back = p + mowbite_boot.BACK
        # State of the host and the root-owned copies it checks and unpacks.
        self.var = p + mowbite_boot.VAR
        self.state = self.var + "/update.json"
        self.stage = self.var + "/stage"
        # Held by mowbite-host while it runs, by mowbite-update while it works and by a tool of the rootfs that stops
        # the OS itself (mowbite-measure): the path stays.
        self.lock = p + "/run/mowbite-host.lock"
        self.crash = p + "/run/mowbite/crash"
        self.boot_id = p + "/proc/sys/kernel/random/boot_id"
        # Written by MowBite OS in its data_dir (mowbite_hold, stack.py).
        data = p + data_dir
        self.session = data + "/os-session.json"
        self.restart_hold = data + "/restart-hold.json"
        self.hold = data + "/hold.json"
        self.exit_note = data + "/container_exit.json"
        self.checkpoint = data + "/checkpoint.json"
        self.halt = p + (self.env().get("MOWBITE_HALT_FILE") or "/run/mowbite-os/halt")

    # KEY=value of the environment files of the OS service.
    def env(self):
        env = {}
        for path in self.env_files:
            try:
                with open(path) as f:
                    for line in f:
                        line = line.strip()
                        if line and not line.startswith("#") and "=" in line:
                            key, value = line.split("=", 1)
                            env[key] = value
            except FileNotFoundError:
                pass
        return env
