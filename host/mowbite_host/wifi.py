"""The WLAN of the mower over NetworkManager: host.wifi_scan lists what the mower sees, host.wifi_set tries a network
in the background and keeps it only once it connected and a request reached the host over it within REACH_WAIT (a
guest WLAN may connect and still cut the mower off); else the connection before comes back.

The key of a network goes only into the profile file of NetworkManager (root, 0600), never into an answer, the status,
a log line or the command line of a program.
"""
import glob
import os
import re
import threading
import time
import uuid

from mowbite_host import Refused, files
from mowbite_host.command import nmcli_fields

PREFIX = "mowbite-wifi-"
MAX_NETWORKS = 64
CONNECT_WAIT = 45
REACH_WAIT = 300.0
HEX_KEY = re.compile(r"[0-9A-Fa-f]{64}")


class Invalid(Exception):
    pass


def check_ssid(ssid):
    if not isinstance(ssid, str) or not 1 <= len(ssid.encode()) <= 32 or any(ord(c) < 0x20 for c in ssid):
        raise Invalid("ssid has to be a name of 1 to 32 bytes without control characters")
    return ssid


# None for an open network.
def check_key(key):
    if key is None:
        return None
    if not isinstance(key, str) or not (HEX_KEY.fullmatch(key) or (
            8 <= len(key) <= 63 and all(0x20 <= ord(c) <= 0x7e for c in key))):
        raise Invalid("password has to be 8 to 63 printable ASCII characters or 64 hex digits")
    return key


# A value of a key file of GLib: backslash and a leading blank escaped (control characters never get here).
def _keyfile_text(text):
    text = text.replace("\\", "\\\\")
    return "\\s" + text[1:] if text.startswith(" ") else text


def profile(ssid, key, connection_id, hidden=False):
    # The SSID as its bytes, which a key file of NetworkManager takes for any name.
    lines = ["[connection]", f"id={PREFIX}{connection_id[:8]}", f"uuid={connection_id}", "type=wifi",
             "autoconnect-priority=10", "", "[wifi]", "mode=infrastructure",
             "ssid=" + "".join(f"{byte};" for byte in ssid.encode())]
    lines += ["hidden=true", ""] if hidden else [""]
    if key is not None:
        lines += ["[wifi-security]", "key-mgmt=wpa-psk", "psk=" + _keyfile_text(key), ""]
    # The name servers of the router over IPv4 only, as in every profile of the image (make-image.sh).
    lines += ["[ipv4]", "method=auto", "", "[ipv6]", "method=auto", "ignore-auto-dns=true", ""]
    return "\n".join(lines)


class Wifi:
    """run: command.run; status() is None until the first host.wifi_set of this start of the host."""

    def __init__(self, paths, run):
        self.paths, self.run = paths, run
        self._status = None
        self._worker = None
        self._lock = threading.Lock()
        # A network that connected and waits to be reached: {ssid, before, path, others, until}.
        self._pending = None
        self._reached = threading.Event()

    def status(self):
        with self._lock:
            return dict(self._status) if self._status else None

    def _set(self, state, ssid, reason=None, message=None):
        with self._lock:
            self._status = {"state": state, "ssid": ssid, "reason": reason, "message": message,
                            "t": int(time.time())}

    def _nmcli(self, *args, timeout=30):
        try:
            return self.run(["nmcli", *args], timeout=timeout)
        except FileNotFoundError:
            raise Refused("UNSUPPORTED", "NetworkManager is not on this mower") from None

    def _device(self):
        code, out, err = self._nmcli("-t", "-f", "DEVICE,TYPE", "device")
        if code != 0:
            raise Refused("WIFI_FAILED", f"nmcli: {err or code}")
        for line in out.splitlines():
            fields = nmcli_fields(line)
            if len(fields) >= 2 and fields[1] == "wifi":
                return fields[0]
        raise Refused("UNSUPPORTED", "the mower has no WLAN device")

    def scan(self):
        self._device()
        fields = ("-t", "-f", "IN-USE,SSID,SIGNAL,SECURITY", "device", "wifi", "list")
        code, out, err = self._nmcli(*fields, "--rescan", "yes")
        if code != 0:
            # Right after a scan NetworkManager refuses a new one; its list is fresh then.
            code, out, err = self._nmcli(*fields, "--rescan", "no")
        if code != 0:
            raise Refused("WIFI_FAILED", f"the scan failed: {err or code}")
        networks = {}
        for line in out.splitlines():
            in_use, ssid, signal, security = (nmcli_fields(line) + ["", "", "", ""])[:4]
            if not ssid:
                continue
            seen = networks.get(ssid)
            signal = int(signal) if signal.isdigit() else 0
            if seen is None or signal > seen["signal"]:
                networks[ssid] = {"ssid": ssid, "signal": signal, "secure": security not in ("", "--"),
                                  "in_use": in_use.strip() == "*" or bool(seen and seen["in_use"])}
            elif in_use.strip() == "*":
                seen["in_use"] = True
        ordered = sorted(networks.values(), key=lambda n: (not n["in_use"], -n["signal"], n["ssid"]))
        return {"networks": ordered[:MAX_NETWORKS]}

    # From the thread of the broker client: a request came in, after a change over the new network.
    def reached(self):
        self._reached.set()

    # From the main loop: keeps a network that was reached, or after REACH_WAIT goes back to the one before.
    def look(self, now=None):
        pending = self._pending
        if pending is None or (self._worker is not None and self._worker.is_alive()):
            return
        if self._reached.is_set():
            self._pending = None
            self._drop(pending["others"])
            return
        if (time.monotonic() if now is None else now) >= pending["until"]:
            self._pending = None
            self._worker = threading.Thread(target=self._back, args=(pending,), daemon=True)
            self._worker.start()

    # Starts the change and answers at once: the caller may be on the network the mower leaves.
    def set(self, ssid, key, hidden=False):
        ssid, key = check_ssid(ssid), check_key(key)
        if not isinstance(hidden, bool):
            raise Invalid("hidden has to be true or false")
        if not os.path.isdir(self.paths.nm_connections):
            raise Refused("UNSUPPORTED", "NetworkManager keeps no profiles on this mower")
        self.look()
        if (self._worker is not None and self._worker.is_alive()) or self._pending is not None:
            raise Refused("BUSY", "a change of the WLAN runs")
        device = self._device()
        self._set("trying", ssid)
        self._worker = threading.Thread(target=self._change, args=(device, ssid, key, hidden), daemon=True)
        self._worker.start()
        return self.status()

    def join(self):
        if self._worker is not None:
            self._worker.join()

    def _active_on(self, device):
        code, out, _ = self._nmcli("-t", "-f", "UUID,DEVICE", "connection", "show", "--active")
        if code == 0:
            for line in out.splitlines():
                fields = nmcli_fields(line)
                if len(fields) >= 2 and fields[1] == device:
                    return fields[0]
        return None

    def _own_profiles(self):
        return sorted(glob.glob(os.path.join(self.paths.nm_connections, PREFIX + "*.nmconnection")))

    def _drop(self, paths):
        for path in paths:
            files.remove(path)
        self._nmcli("connection", "reload")

    def _back(self, pending):
        ssid = pending["ssid"]
        try:
            files.remove(pending["path"])
            self._nmcli("connection", "reload")
            again, _, _ = self._nmcli("--wait", str(CONNECT_WAIT), "connection", "up", "uuid", pending["before"],
                                      timeout=CONNECT_WAIT + 15)
            back = "the connection before is back" if again == 0 else "the connection before did not come back"
            self._set("failed", ssid, "WIFI_FAILED", f"nothing reached the mower over {ssid}; {back}")
        except Exception as error:  # noqa: BLE001 - the status tells, the host goes on
            self._set("failed", ssid, "INTERNAL", type(error).__name__)
        print(f"wifi: back from {ssid!r}: {self.status()['message']}", flush=True)

    def _change(self, device, ssid, key, hidden=False):
        try:
            self._try(device, ssid, key, hidden)
        except Refused as why:
            self._set("failed", ssid, why.reason, str(why))
        except Exception as error:  # noqa: BLE001 - the status tells, the host goes on
            self._set("failed", ssid, "INTERNAL", type(error).__name__)
        status = self.status()
        print(f"wifi: {status['state']} for {ssid!r}" + (f": {status['message']}" if status["message"] else ""),
              flush=True)

    def _try(self, device, ssid, key, hidden=False):
        before = self._active_on(device)
        connection_id = str(uuid.uuid4())
        path = os.path.join(self.paths.nm_connections, f"{PREFIX}{connection_id[:8]}.nmconnection")
        others = [p for p in self._own_profiles() if p != path]
        files.write_new(path, profile(ssid, key, connection_id, hidden), 0o600)
        code, _, err = self._nmcli("connection", "load", path)
        if code == 0:
            code, _, err = self._nmcli("--wait", str(CONNECT_WAIT), "connection", "up", "uuid", connection_id,
                                       "ifname", device, timeout=CONNECT_WAIT + 15)
        if code == 0:
            if before is None:
                self._drop(others)
            else:
                # The profiles before stay until the new network has shown it reaches the mower.
                self._reached.clear()
                self._pending = {"ssid": ssid, "before": before, "path": path, "others": others,
                                 "until": time.monotonic() + REACH_WAIT}
            self._set("connected", ssid)
            return
        files.remove(path)
        self._nmcli("connection", "reload")
        back = ""
        if before is not None:
            again, _, _ = self._nmcli("--wait", str(CONNECT_WAIT), "connection", "up", "uuid", before,
                                      timeout=CONNECT_WAIT + 15)
            back = "; the connection before is back" if again == 0 else "; the connection before did not come back"
        raise Refused("WIFI_FAILED", f"no connection to {ssid}: {err or code}{back}")
