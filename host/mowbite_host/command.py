"""Programs of the system the host calls (systemctl, nmcli, the trial tool of the rootfs); tests give their own run."""
import subprocess


# (exit code, stdout, first line of stderr); 124 when it ran out of time. FileNotFoundError when it is not there.
# input: text for stdin (a password never goes into argv), else stdin is empty.
def run(argv, timeout=30, input=None):  # noqa: A002 - the name subprocess uses
    stdin = {"input": input} if input is not None else {"stdin": subprocess.DEVNULL}
    try:
        done = subprocess.run(argv, capture_output=True, text=True, errors="replace", timeout=timeout, **stdin)
    except subprocess.TimeoutExpired:
        return 124, "", f"{argv[0]} did not end within {timeout} s"
    lines = done.stderr.strip().splitlines()
    return done.returncode, done.stdout, lines[0] if lines else ""


# Fields of a line of nmcli -t: split at ':' that is not escaped, with \: and \\ undone.
def nmcli_fields(line):
    fields, current, escaped = [], [], False
    for char in line:
        if escaped:
            current.append(char)
            escaped = False
        elif char == "\\":
            escaped = True
        elif char == ":":
            fields.append("".join(current))
            current = []
        else:
            current.append(char)
    fields.append("".join(current))
    return fields
