"""The setup WLAN on a gesture of the mower's buttons, once the trial period is over: PRESSES short presses of the key
of the platform within WINDOW_S open the access point of the first start for OPEN_S, only at rest (an OS that runs
idles; without a running OS always) and while the mower has no WLAN connection; the gesture again while it is open
gives it OPEN_S from then. A tone tells whether it opened. The access point closes earlier once the mower's WLAN is
connected again, by host.wifi_set or because a known network is back.

The platform names all of it in /usr/lib/mowbite/wifi-gesture (KEY=value): device, the name of the input device in
/proc/bus/input/devices; code, the key code of a short press (counted only when it goes down); tone_device, the ALSA
device for aplay; unit, the systemd unit of the access point, which reads the deadline (unix s) the host writes to
/run/mowbite/setup-ap.until and removes it when it closes; profile, its NetworkManager profile. Without the file there
is no gesture.
"""
import array
import io
import math
import os
import struct
import sys
import threading
import time
import wave

from mowbite_host import files, gate, wifi

PRESSES = 5
WINDOW_S = 5.0
OPEN_S = 30 * 60
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
KEYS = ("device", "code", "tone_device", "unit", "profile")


# The file of the platform, None without it or with a key missing.
def read_config(path):
    config = files.read_env(path)
    if config is None or not all(config.get(key) for key in KEYS):
        return None
    try:
        config["code"] = int(config["code"])
    except ValueError:
        return None
    return config


# /dev/input/eventN of the input device with this name, None when there is none.
def find_device(paths, name):
    for block in (files.read_text(paths.input_devices) or "").split("\n\n"):
        lines = block.splitlines()
        if f'N: Name="{name}"' not in lines:
            continue
        handlers = next((line.split("=", 1)[1].split() for line in lines if line.startswith("H: Handlers=")), [])
        event = next((handler for handler in handlers if handler.startswith("event")), None)
        return os.path.join(paths.dev_input, event) if event else None
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
    samples = array.array("h")
    for frequency, seconds in notes:
        count = int(TONE_RATE * seconds)
        # A short fade at both ends keeps the speaker from clicking.
        samples.extend(int(12000 * min(1.0, i / 200, (count - i) / 200) *
                           math.sin(2 * math.pi * frequency * i / TONE_RATE)) for i in range(count))
        samples.extend([0] * int(TONE_RATE * 0.05))
    if sys.byteorder == "big":
        samples.byteswap()
    out = io.BytesIO()
    with wave.open(out, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(TONE_RATE)
        w.writeframes(samples.tobytes())
    return out.getvalue()


class Gesture:
    """reader() runs in a thread of its own and only marks the gesture; take() and the rest are for the main loop.
    run: command.run."""

    def __init__(self, paths, run):
        self.paths, self.run = paths, run
        # The file of the platform, read by start().
        self.config = None
        self._asked = threading.Event()
        self._thread = None
        # The thread that plays the last tone.
        self.player = None

    def start(self):
        self.config = read_config(self.paths.wifi_gesture)
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

    # Why the access point does not open now, None when it may: the WLAN of the mower is not connected (next to it the
    # access point would take the channel of that network), and an OS that runs idles.
    def refusal(self, os_now):
        if not os.path.lexists(self.config["profile"]):
            return "no setup profile"
        # Once open, the access point itself is a connected WLAN device.
        if self.until() is None and not wifi.problems(self.run):
            return "the mower has a WLAN connection"
        if (os_now or {}).get("state") != "running":
            return None
        session = gate.session_of_this_boot(self.paths) or {}
        return None if session.get("state") == "IDLE" else f"the OS is busy ({session.get('state') or 'unknown'})"

    # Opens the access point for OPEN_S, or gives an open one OPEN_S from now; why not, None when it did.
    def open(self):
        was_open = self.until() is not None
        os.makedirs(os.path.dirname(self.paths.setup_ap_until), exist_ok=True)
        files.write_new(self.paths.setup_ap_until, f"{int(time.time() + OPEN_S)}\n")
        if was_open:
            return None
        try:
            code, _, err = self.run(["systemctl", "--no-block", "restart", self.config["unit"]])
        except FileNotFoundError:
            code, err = 1, "no systemctl"
        if code == 0:
            return None
        files.remove(self.paths.setup_ap_until)
        return f"systemctl: {err or code}"

    # The deadline while the access point is open on the gesture, else None; the unit removes the file when it closes.
    def until(self):
        text = files.read_text(self.paths.setup_ap_until) or ""
        until = int(text.strip()) if text.strip().isdigit() else 0
        if until > time.time():
            return until
        files.remove(self.paths.setup_ap_until)
        return None

    def tone(self, opened):
        path = self.paths.tone_opened if opened else self.paths.tone_refused
        try:
            if not files.regular(path):
                os.makedirs(os.path.dirname(path), exist_ok=True)
                files.write_new(path, tone_wav(OPENED if opened else REFUSED))
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
