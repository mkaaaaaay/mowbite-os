"""What bundle.json says of a colcon installation of mowbite_planner: version from its package.xml, interface,
files_version and requires {name: oldest version} from contract.json in its python package (else in its share folder).

  planner_meta.py <install base> [--contract]
      prints them as JSON; without contract.json only version, with --contract that is refused
"""
import glob
import json
import os
import sys
import xml.etree.ElementTree as ElementTree

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from mowbite_host.compat import whole  # noqa: E402

PACKAGE = "mowbite_planner"


class Refused(Exception):
    pass


def _contract_path(base):
    root = os.path.join(base, PACKAGE)
    found = sorted(glob.glob(os.path.join(root, "lib", "python3*", "site-packages", PACKAGE, "contract.json")))
    found.append(os.path.join(root, "share", PACKAGE, "contract.json"))
    return next((path for path in found if os.path.isfile(path)), None)


def read(base, contract=False):
    xml = os.path.join(base, PACKAGE, "share", PACKAGE, "package.xml")
    try:
        version = ElementTree.parse(xml).getroot().findtext("version")
    except (OSError, ElementTree.ParseError) as error:
        raise Refused(f"no {PACKAGE} in {base}: {error}")
    if not version:
        raise Refused(f"{xml} names no version")
    meta = {"version": version.strip()}
    path = _contract_path(base)
    if path is None:
        if contract:
            raise Refused(f"{PACKAGE} in {base} has no contract.json")
        return meta
    try:
        with open(path) as f:
            terms = json.load(f)
    except (OSError, ValueError) as error:
        raise Refused(f"{path}: {error}")
    requires = terms.get("requires", {}) if isinstance(terms, dict) else None
    if (not isinstance(terms, dict) or not whole(terms.get("interface")) or not whole(terms.get("files_version")) or
            not isinstance(requires, dict) or not all(isinstance(need, str) for need in requires.values())):
        raise Refused(f"{path}: interface and files_version as whole numbers and requires as {{name: oldest "
                      f"version}} expected")
    return {**meta, "interface": terms["interface"], "files_version": terms["files_version"], "requires": requires}


def main(argv):
    if len(argv) not in (1, 2) or argv[1:] not in ([], ["--contract"]):
        print(__doc__, file=sys.stderr)
        return 2
    try:
        print(json.dumps(read(argv[0], contract=len(argv) == 2)))
    except Refused as why:
        print(f"refused: {why}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
