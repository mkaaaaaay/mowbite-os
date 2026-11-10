"""Where mowbite-host finds and keeps its files; tests give a prefix, on the mower it is empty."""
import os

# The OS reads these files from the environment of its service, the last one winning.
ENV_FILES = ("/usr/lib/mowbite/os.defaults", "/etc/mowbite/os.conf")


class Paths:
    def __init__(self, prefix="", data_dir="/userdata/mowbite"):
        p = prefix
        self.prefix = prefix
        # The installations of the OS: <opt>/<revision>, current, previous, pending.
        self.opt = p + "/opt/mowbite"
        self.signers = p + "/etc/mowbite/allowed_signers"
        self.update_conf = p + "/etc/mowbite/update.conf"
        # The version of the rootfs or base image, an integer; a bundle names the oldest it runs on.
        self.base = p + "/usr/lib/mowbite/base"
        self.os_setup = p + "/usr/lib/mowbite/os-setup"
        self.env_files = [p + path for path in ENV_FILES]
        self.inbox = p + "/userdata/mowbite-update/inbox"
        # State of the host and the root-owned copies it checks and unpacks.
        self.var = p + "/var/lib/mowbite-host"
        self.state = self.var + "/update.json"
        self.stage = self.var + "/stage"
        self.lock = p + "/run/mowbite-host.lock"
        self.crash = p + "/run/mowbite/crash"
        self.boot_id = p + "/proc/sys/kernel/random/boot_id"
        # Written by MowBite OS in its data_dir (mowbite_hold, stack.py).
        data = p + data_dir
        self.data_dir = data
        self.session = data + "/os-session.json"
        self.restart_hold = data + "/restart-hold.json"
        self.hold = data + "/hold.json"
        self.exit_note = data + "/container_exit.json"
        self.halt = p + (self.env().get("MOWBITE_HALT_FILE") or "/run/mowbite-os/halt")

    def opt_of(self, name):
        return os.path.join(self.opt, name)

    @property
    def current(self):
        return self.opt_of("current")

    @property
    def previous(self):
        return self.opt_of("previous")

    @property
    def pending(self):
        return self.opt_of("pending")

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
