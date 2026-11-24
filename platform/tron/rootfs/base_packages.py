#!/usr/bin/env python3
"""base-packages.txt: the packages installed in the base tar of build-base.sh, so a bundle can be checked against the
rootfs it needs without the tar. The first line names the base version and the sha256 of packages.txt it was built
from, then one package per line.
  base_packages.py write <dpkg status> <base version> <packages.txt>   the file on stdout (build-base.sh)
  base_packages.py same <dpkg status> <base-packages.txt>              exit 1 when the tar has other packages (make-image)
missing() is the check of make-bundle.sh: the runtime packages of a bundle that its base does not hold.
"""
import hashlib
import re
import sys

HEADER = re.compile(r"# base (\d+), packages\.txt sha256 ([0-9a-f]{64})$")


def installed(status_text):
    """Names of the packages dpkg lists as installed."""
    names, name = set(), None
    for line in status_text.splitlines():
        if line.startswith("Package: "):
            name = line.split(": ", 1)[1].strip()
        elif line.startswith("Status: ") and line.split()[-1] == "installed" and name:
            names.add(name)
    return names


def write(status_text, base, packages_txt):
    sha = hashlib.sha256(packages_txt).hexdigest()
    return "".join([f"# base {int(base)}, packages.txt sha256 {sha}\n"] + [f"{n}\n" for n in sorted(installed(status_text))])


def read(text):
    """(base version, sha256 of packages.txt, package names); ValueError without the header."""
    lines = text.splitlines()
    match = HEADER.match(lines[0]) if lines else None
    if not match:
        raise ValueError("base-packages.txt has no header '# base <n>, packages.txt sha256 <sha>'")
    return int(match.group(1)), match.group(2), {line.strip() for line in lines[1:] if line.strip()}


def missing(runtime, text, base):
    """The runtime packages the base does not hold, sorted; ValueError when the list is for another base version."""
    have_base, _, names = read(text)
    if have_base != int(base):
        raise ValueError(f"base-packages.txt is for base {have_base}, the bundle needs base {int(base)}")
    return sorted(set(runtime) - names)


def main(argv):
    if len(argv) == 4 and argv[0] == "write":
        with open(argv[1]) as status, open(argv[3], "rb") as packages:
            sys.stdout.write(write(status.read(), argv[2], packages.read()))
        return 0
    if len(argv) == 3 and argv[0] == "same":
        with open(argv[1]) as status, open(argv[2]) as listed:
            have, want = installed(status.read()), read(listed.read())[2]
        if have == want:
            return 0
        print(f"base tar and base-packages.txt differ: only in the tar {sorted(have - want)}, only in the list "
              f"{sorted(want - have)}", file=sys.stderr)
        return 1
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
