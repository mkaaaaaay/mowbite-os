"""Helpers of the tests of mowbite-host and mowbite-update: keys and signatures, a free port, a private mosquitto."""
import os
import socket
import subprocess
import time


def keygen(path):
    subprocess.run(["ssh-keygen", "-q", "-t", "ed25519", "-N", "", "-C", "", "-f", path], check=True)
    return path


# "ssh-ed25519 AAAA..." of the key at path, for allowed_signers.
def public(path):
    with open(path + ".pub") as f:
        return " ".join(f.read().split()[:2])


# Signs path into path.sig, replacing an older one.
def sign(path, key, namespace):
    if os.path.exists(path + ".sig"):
        os.remove(path + ".sig")
    subprocess.run(["ssh-keygen", "-q", "-Y", "sign", "-f", key, "-n", namespace, path], check=True,
                   capture_output=True, stdin=subprocess.DEVNULL, timeout=30)


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


# A mosquitto on 127.0.0.1:port with its configuration in folder, once it takes connections.
def mosquitto(folder, port):
    conf = os.path.join(folder, "mosquitto.conf")
    with open(conf, "w") as f:
        f.write(f"listener {port} 127.0.0.1\nallow_anonymous true\npersistence false\n")
    process = subprocess.Popen(["mosquitto", "-c", conf], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.time() + 5
    while time.time() < deadline:
        try:
            socket.create_connection(("127.0.0.1", port), 0.2).close()
            break
        except OSError:
            time.sleep(0.05)
    return process
