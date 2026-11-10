"""What runs the installed OS: systemd on the Tron, a process of its own for tests; the container of a compose stack
comes later. The host only starts, stops, restarts and asks how it runs. The planner runs inside the OS and has its
carrier; nothing runs the app."""
import os
import signal
import subprocess
import time

from mowbite_host import Refused, bundle


class Systemd:
    def __init__(self, service="mowbite-os.service"):
        self.service = service

    def _systemctl(self, *args):
        return bundle.run(["systemctl", *args, self.service])

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


class Nothing:
    """The carrier of a component nothing on the mower runs (the app: files a web server reads)."""

    def active(self):
        return False

    def start(self):
        return True

    def show(self):
        return "inactive", None, None

    def stop(self):
        pass


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
    and start time stay in pid_file before it runs, so a host started again finds an OS it did not start; the session
    counts as running while any process of it lives, also after its leader ended. stop sends SIGINT to the session as
    systemd does and kills it after stop_wait s."""

    # The child waits for the pid file and runs nothing when the host ends before writing it.
    GO = 'read -r _ || exit 1; exec "$0" "$@" </dev/null'
    # How long processes have to end after SIGKILL before stop gives up.
    KILL_WAIT = 5.0

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
        now = _start_time(pid)
        if now == started:
            return pid
        # A pid stays taken while a group of that number lives: with the leader gone, a group is what is left of it.
        return pid if now is None and _group_alive(pid) else None

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
        self._child = subprocess.Popen(bundle.as_user(["/bin/sh", "-c", self.GO, self.command]), env=self.env,
                                       start_new_session=True, stdin=subprocess.PIPE, bufsize=0)
        started = _start_time(self._child.pid)
        try:
            if started is None:
                return False
            with open(self.pid_file + ".tmp", "w") as f:
                f.write(f"{self._child.pid} {started}\n")
            os.replace(self.pid_file + ".tmp", self.pid_file)
            self._child.stdin.write(b"go\n")
        except BrokenPipeError:
            return False
        finally:
            self._child.stdin.close()
        return True

    def restart(self):
        self.stop()
        return self.start()

    def _wait_gone(self, seconds):
        deadline = time.monotonic() + seconds
        while self._pid() is not None and time.monotonic() < deadline:
            time.sleep(0.05)
        return self._pid() is None

    def stop(self):
        pid = self._pid()
        if pid is not None:
            _kill_group(pid, signal.SIGINT)
            if not self._wait_gone(self.stop_wait):
                _kill_group(pid, signal.SIGKILL)
                if not self._wait_gone(self.KILL_WAIT):
                    raise Refused("BUSY", f"the session {pid} of the OS does not end")
        try:
            os.remove(self.pid_file)
        except FileNotFoundError:
            pass


# Whether a process of the group pgid lives that is not a zombie.
def _group_alive(pgid):
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open(f"/proc/{name}/stat") as f:
                fields = f.read().rsplit(")", 1)[1].split()
        except (OSError, IndexError):
            continue
        if fields[2] == str(pgid) and fields[0] not in ("Z", "X"):
            return True
    return False


def _kill_group(pgid, sig):
    try:
        os.killpg(pgid, sig)
    except ProcessLookupError:
        pass


# The carrier the environment files of the OS name (MOWBITE_HOST_CARRIER systemd | process), systemd without one.
def of(paths):
    env = paths.env()
    if env.get("MOWBITE_HOST_CARRIER", "systemd") == "process":
        return Process(paths.prefix + "/usr/lib/mowbite/run-os", paths.prefix + "/run/mowbite-host/os.pid",
                       {**os.environ, **env})
    return Systemd()
