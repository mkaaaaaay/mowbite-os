"""What runs the installed OS: systemd on the Tron, a process of its own for tests; the container of a compose stack
comes later. The host only starts, stops, restarts and asks how it runs."""
import os
import signal
import subprocess
import time

from mowbite_host import Refused, bundle


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


def _start_time(pid):
    try:
        with open(f"/proc/{pid}/stat") as f:
            fields = f.read().rsplit(")", 1)[1].split()
    except (OSError, IndexError):
        return None
    # State is the first field after the name, the start time the twentieth.
    return None if fields[0] in ("Z", "X") else fields[19]


class Process:
    """command (run-os) in a session of its own with the environment of the OS, as the user mowbite when root. Its pid
    and start time stay in pid_file, so a host started again finds an OS it did not start; stop sends SIGINT to the
    session as systemd does and kills it after stop_wait s."""

    def __init__(self, command, pid_file, env, stop_wait=20.0):
        self.command, self.pid_file, self.env, self.stop_wait = command, pid_file, env, stop_wait
        self._child = None

    def _pid(self):
        try:
            with open(self.pid_file) as f:
                pid, started = f.read().split()
            pid = int(pid)
        except (OSError, ValueError):
            return None
        if self._child is not None and self._child.pid == pid:
            self._child.poll()
        return pid if _start_time(pid) == started else None

    def active(self):
        return self._pid() is not None

    def show(self):
        if not self.active():
            return "inactive", None, None
        since = int(os.stat(self.pid_file).st_mtime)
        return "active", since, since

    def start(self):
        if self.active():
            return True
        os.makedirs(os.path.dirname(self.pid_file), exist_ok=True)
        self._child = subprocess.Popen(bundle.as_user([self.command]), env=self.env, start_new_session=True,
                                       stdin=subprocess.DEVNULL)
        started = _start_time(self._child.pid)
        if started is None:
            return False
        with open(self.pid_file + ".tmp", "w") as f:
            f.write(f"{self._child.pid} {started}\n")
        os.replace(self.pid_file + ".tmp", self.pid_file)
        return True

    def restart(self):
        self.stop()
        return self.start()

    def stop(self):
        pid = self._pid()
        if pid is not None:
            os.killpg(pid, signal.SIGINT)
            deadline = time.monotonic() + self.stop_wait
            while self._pid() is not None and time.monotonic() < deadline:
                time.sleep(0.1)
            if self._pid() is not None:
                os.killpg(pid, signal.SIGKILL)
                while self._pid() is not None:
                    time.sleep(0.05)
        try:
            os.remove(self.pid_file)
        except FileNotFoundError:
            pass


# The carrier the environment files of the OS name (MOWBITE_HOST_CARRIER systemd | process), systemd without one.
def of(paths):
    env = paths.env()
    if env.get("MOWBITE_HOST_CARRIER", "systemd") == "process":
        return Process(paths.prefix + "/usr/lib/mowbite/run-os", paths.prefix + "/run/mowbite-host/os.pid",
                       {**os.environ, **env})
    return Systemd()
