"""SSH for the owner, off from the start: host.ssh turns it on with a password of root the owner gives, or off. The
password goes to chpasswd over stdin and is never kept, logged or sent back; a new one needs no old one, the app that
may call the host is the owner's. Only on a rootfs of MowBite (its sshd_config.d/mowbite.conf lets root in with a
password from private networks only), where sshd starts by its socket unit.
"""
import os

from mowbite_host import Refused, files

SOCKET = "ssh.socket"
UNITS = ("ssh.socket", "ssh.service")
MIN_PASSWORD = 8
MAX_PASSWORD = 128


def check_password(password):
    if not isinstance(password, str) or not MIN_PASSWORD <= len(password) <= MAX_PASSWORD:
        raise Refused("INVALID", f"password has to be a text of {MIN_PASSWORD} to {MAX_PASSWORD} characters")
    if any(ord(char) < 32 or ord(char) == 127 for char in password):
        raise Refused("INVALID", "password has to be without control characters")
    return password


class Ssh:
    """run: command.run."""

    def __init__(self, paths, run):
        self.paths, self.run = paths, run
        self._enabled = None

    def supported(self):
        return os.path.lexists(self.paths.sshd_mowbite)

    # {enabled, password} as host/status shows it, None where the host does not handle SSH.
    def status(self):
        if not self.supported():
            return None
        if self._enabled is None:
            self._enabled = self._is_enabled()
        return {"enabled": self._enabled, "password": self._has_password()}

    def set(self, params):
        if not self.supported():
            raise Refused("UNSUPPORTED", "SSH is not handled by the host on this mower")
        enabled = params.get("enabled")
        if not isinstance(enabled, bool):
            raise Refused("INVALID", "enabled has to be true or false")
        password = params.get("password")
        if not enabled:
            if password is not None:
                raise Refused("INVALID", "password only with enabled true")
            self._systemctl("disable", "--now", *UNITS)
        else:
            if password is not None:
                self._set_password(check_password(password))
            elif not self._has_password():
                raise Refused("NO_PASSWORD", "root has no password yet: give one")
            self._systemctl("enable", "--now", SOCKET)
        self._enabled = self._is_enabled()
        return self.status()

    def _is_enabled(self):
        try:
            code, out, _ = self.run(["systemctl", "is-enabled", SOCKET])
        except FileNotFoundError:
            return False
        return code == 0 and out.strip() == "enabled"

    def _has_password(self):
        for line in (files.read_text(self.paths.shadow) or "").splitlines():
            fields = line.split(":")
            if fields[0] == "root":
                return len(fields) > 1 and fields[1] not in ("", "*", "!", "!*") and not fields[1].startswith("!")
        return False

    def _set_password(self, password):
        try:
            code, _, err = self.run(["chpasswd"], input=f"root:{password}\n")
        except FileNotFoundError:
            code, err = 1, "no chpasswd"
        if code != 0:
            raise Refused("FAILED", f"chpasswd: {err or code}")

    def _systemctl(self, *args):
        try:
            code, _, err = self.run(["systemctl", *args])
        except FileNotFoundError:
            code, err = 1, "no systemctl"
        if code != 0:
            raise Refused("FAILED", f"systemctl {args[0]}: {err or code}")
