"""What runs the installed OS: systemd on the Tron; the container of a compose stack comes later. The host only starts,
stops, restarts and asks how it runs."""
import subprocess

from mowbite_host import Refused


class Systemd:
    def __init__(self, service="mowbite-os.service"):
        self.service = service

    def _systemctl(self, *args):
        return subprocess.run(["systemctl", *args, self.service], capture_output=True, text=True)

    def active(self):
        return self._systemctl("is-active", "--quiet").returncode == 0

    def start(self):
        self._systemctl("reset-failed")
        return self._systemctl("start").returncode == 0

    def restart(self):
        self._systemctl("reset-failed")
        return self._systemctl("restart").returncode == 0

    # ActiveState (active, activating, deactivating, inactive, failed), since when in unix s, and since when active.
    def show(self):
        result = self._systemctl("show", "--timestamp=unix", "-p", "ActiveState", "-p", "StateChangeTimestamp",
                                 "-p", "ActiveEnterTimestamp")
        values = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)

        def unix(key):
            value = values.get(key, "")
            return int(value[1:]) if value.startswith("@") and value[1:].isdigit() else None
        return values.get("ActiveState", "unknown"), unix("StateChangeTimestamp"), unix("ActiveEnterTimestamp")

    def stop(self):
        result = self._systemctl("stop")
        if result.returncode != 0:
            raise Refused("BUSY", f"systemctl stop {self.service}: {result.stderr.strip()}")
