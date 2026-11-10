"""What runs the installed OS: systemd on the Tron; the container of a compose stack comes later. The host only starts,
stops and asks whether it runs."""
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

    def stop(self):
        result = self._systemctl("stop")
        if result.returncode != 0:
            raise Refused("BUSY", f"systemctl stop {self.service}: {result.stderr.strip()}")
