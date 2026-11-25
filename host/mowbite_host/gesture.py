"""The setup WLAN on a gesture of the mower's buttons, once the trial period is over: PRESSES short presses of the key
of the platform within WINDOW_S open the access point of the first start (mowbite-setup-ap) for OPEN_S, only at rest
(an OS that runs idles; without a running OS always). A tone tells whether it opened. The access point closes earlier
once the mower's WLAN is connected again, by host.wifi_set or because a known network is back.

The platform names its key in /usr/lib/mowbite/wifi-gesture (KEY=value): device, the name of the input device in
/proc/bus/input/devices; code, the key code of a short press (counted only when it goes down); tone_device, the ALSA
device for aplay. Without the file there is no gesture.
"""
import io
import math
import os
import struct
import threading
import time
import wave

from mowbite_host import files, gate, trial

PRESSES = 5
WINDOW_S = 5.0
OPEN_S = 30 * 60
SETUP_AP_UNIT = "mowbite-setup-ap.service"
# struct input_event of a 64-bit kernel: time (two longs), type, code, value.
EVENT = struct.Struct("llHHi")
EV_KEY = 1
KEY_DOWN = 1
# A device that is missing or cannot be read is looked for again this often.
RETRY_S = 10.0
TONE_RATE = 16000
# (Hz, s) of the tones: rising when the access point opens, low when the gesture is refused.
OPENED = ((660, 0.15), (880, 0.15), (1320, 0.3))
REFUSED = ((440, 0.25), (330, 0.4))


def read_config(path):
    text = files.read_text(path)
    if text is None:
        return None
    config = {}
    for line in text.splitlines():
        key, sep, value = line.partition("=")
        if sep and not line.lstrip().startswith("#"):
            config[key.strip()] = value.strip()
    try:
        code = int(config.get("code", ""))
    except ValueError:
        return None
    if not config.get("device"):
        return None
    return {"device": config["device"], "code": code, "tone_device": config.get("tone_device", "default")}


# /dev/input/eventN of the input device with this name, None when there is none.
def find_device(paths, name):
    text = files.read_text(paths.input_devices) or ""
    for block in text.split("\n\n"):
        lines = block.splitlines()
        if not any(line == f'N: Name="{name}"' for line in lines):
            continue
        for line in lines:
            if line.startswith("H: Handlers="):
                for handler in line.split("=", 1)[1].split():
                    if handler.startswith("event"):
                        return os.path.join(paths.dev_input, handler)
    return None


class Counter:
    """Short presses of one key; True once PRESSES came within WINDOW_S, then it counts anew."""

    def __init__(self):
        self.times = []

    def press(self, now):
        self.times = [t for t in self.times if now - t < WINDOW_S] + [now]
        if len(self.times) < PRESSES:
            return False
        self.times = []
        return True


def tone_wav(notes):
    data = bytearray()
    for frequency, seconds in notes:
        count = int(TONE_RATE * seconds)
        for i in range(count):
            # A short fade at both ends keeps the speaker from clicking.
            fade = min(1.0, i / 200, (count - i) / 200)
            data += struct.pack("<h", int(12000 * fade * math.sin(2 * math.pi * frequency * i / TONE_RATE)))
        data += bytes(int(TONE_RATE * 0.05) * 2)
    out = io.BytesIO()
    with wave.open(out, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(TONE_RATE)
        w.writeframes(bytes(data))
    return out.getvalue()


class Gesture:
    """reader() runs in a thread of its own and only marks the gesture; take() and the rest are for the main loop.
    run: command.run."""

    def __init__(self, paths, run):
        self.paths, self.run = paths, run
        self.config = read_config(paths.wifi_gesture)
        self._asked = threading.Event()
        self._thread = None
        # The thread that plays the last tone.
        self.player = None

    def start(self):
        if self.config and self._thread is None:
            self._thread = threading.Thread(target=self.reader, daemon=True)
            self._thread.start()

    def reader(self):
        counter = Counter()
        while True:
            self.read(counter)
            time.sleep(RETRY_S)

    # The events of the input device until it ends or cannot be read; clock: when each came.
    def read(self, counter, clock=time.monotonic):
        device = find_device(self.paths, self.config["device"])
        if device is None:
            return
        try:
            with open(device, "rb", buffering=0) as f:
                while True:
                    chunk = f.read(EVENT.size)
                    if len(chunk) < EVENT.size:
                        return
                    _, _, kind, code, value = EVENT.unpack(chunk)
                    if kind == EV_KEY and code == self.config["code"] and value == KEY_DOWN and counter.press(clock()):
                        self._asked.set()
        except OSError:
            return

    def take(self):
        if not self._asked.is_set():
            return False
        self._asked.clear()
        return True

    # Why the access point does not open now, None when it may: the WLAN of the mower is not connected, and an OS that
    # runs idles.
    def refusal(self, os_now):
        if not os.path.lexists(self.paths.setup_profile):
            return "no setup profile"
        # Next to a connected WLAN the access point would take the channel of that network.
        if not trial.wifi_problems(self.run):
            return "the mower has a WLAN connection"
        if (os_now or {}).get("state") != "running":
            return None
        session = gate.session_of_this_boot(self.paths) or {}
        return None if session.get("state") == "IDLE" else f"the OS is busy ({session.get('state') or 'unknown'})"

    # Opens the access point for OPEN_S: the deadline for mowbite-setup-ap, which it reads at its start.
    def open(self, now=None):
        until = int((time.time() if now is None else now) + OPEN_S)
        os.makedirs(os.path.dirname(self.paths.setup_ap_until), exist_ok=True)
        files.write_new(self.paths.setup_ap_until, f"{until}\n")
        try:
            code, _, err = self.run(["systemctl", "restart", SETUP_AP_UNIT])
        except FileNotFoundError:
            code, err = 1, "no systemctl"
        if code != 0:
            self.close()
            return f"systemctl: {err or code}"
        return None

    def close(self):
        files.remove(self.paths.setup_ap_until)

    # The deadline while the access point is open on the gesture, else None; a deadline that passed or a unit that
    # ended (wlan0 connected) closes it.
    def until(self, now=None):
        text = files.read_text(self.paths.setup_ap_until)
        if text is None:
            return None
        try:
            until = int(text.strip())
        except ValueError:
            until = 0
        if until <= (time.time() if now is None else now) or not self._unit_active():
            self.close()
            return None
        return until

    def _unit_active(self):
        try:
            return self.run(["systemctl", "is-active", "--quiet", SETUP_AP_UNIT])[0] == 0
        except FileNotFoundError:
            return False

    def tone(self, opened):
        if not self.config:
            return
        path = self.paths.tone_opened if opened else self.paths.tone_refused
        try:
            if not files.regular(path):
                os.makedirs(os.path.dirname(path), exist_ok=True)
                with open(path + ".tmp", "wb") as f:
                    f.write(tone_wav(OPENED if opened else REFUSED))
                os.replace(path + ".tmp", path)
        except OSError:
            return
        argv = ["aplay", "-q", "-D", self.config["tone_device"], path]
        self.player = threading.Thread(target=self._play, args=(argv,), daemon=True)
        self.player.start()

    def _play(self, argv):
        try:
            self.run(argv, timeout=10)
        except FileNotFoundError:
            pass
