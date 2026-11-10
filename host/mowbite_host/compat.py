"""Whether an installation of the planner or the app goes with an OS: pure functions over their bundle.json.

The OS names api, min_app, planner_interface, planner_files_version and what it brings along (bundled {app, planner:
{revision, version, serial}}); the app names version and api {min, max}, the planner interface and files_version.
The reasons are those of UPDATE in the catalog. A value that is missing or no whole number never fits.
"""


def version_tuple(text):
    """1.4.0 -> ((1, 4, 0), 1); a pre-release (1.4.0-beta.2) sorts before its release."""
    if not isinstance(text, str):
        return None
    core, _, pre = text.lstrip("v").partition("-")
    numbers = []
    for part in core.split("."):
        if not part.isdigit():
            return None
        numbers.append(int(part))
    return tuple(numbers), 0 if pre else 1


def app_suffices(app_version, min_app):
    need = version_tuple(min_app)
    if need is None:
        return True
    have = version_tuple(app_version)
    return have is not None and have >= need


def whole(value):
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


# api.max is a hard limit: an app does not run beside an OS newer than the newest it was built for.
def app_with_os(app, os_meta):
    span = app.get("api")
    low, high = (span.get("min"), span.get("max")) if isinstance(span, dict) else (None, None)
    if not whole(low) or not whole(high) or not app_suffices(app.get("version"), os_meta.get("min_app")):
        return "APP_TOO_OLD"
    api = os_meta.get("api")
    if not whole(api):
        return "OS_TOO_OLD"
    if api > high:
        return "APP_TOO_OLD"
    return "OS_TOO_OLD" if api < low else None


# files_version: a planner with another one writes files the OS, or the planner it goes back to, does not read.
def planner_with_os(planner, os_meta):
    def same(key, os_key):
        return whole(planner.get(key)) and whole(os_meta.get(os_key)) and planner[key] == os_meta[os_key]
    if not same("interface", "planner_interface"):
        return "PLANNER_INTERFACE"
    return None if same("files_version", "planner_files_version") else "PLANNER_DATA"


CHECKS = {"app": app_with_os, "planner": planner_with_os}
# English text for the log, by reason.
EXPLAIN = {"APP_TOO_OLD": "the app is too old for the OS that is installed",
           "OS_TOO_OLD": "the OS that is installed is too old for the app",
           "PLANNER_INTERFACE": "the planner speaks another interface than the OS that is installed",
           "PLANNER_DATA": "the planner keeps its files in another version than the OS that is installed"}


# Why meta of component does not go with the OS of os_meta ({} without an OS); None when it does, and for the OS.
def reason(component, meta, os_meta):
    check = CHECKS.get(component)
    return check(meta, os_meta) if check else None


def serial(meta):
    return meta["serial"] if whole(meta.get("serial")) else 0


# What the OS of os_meta brings along of component: {revision, version, serial}, {} without it.
def bundled(os_meta, component):
    entry = os_meta.get("bundled")
    entry = entry.get(component) if isinstance(entry, dict) else None
    return entry if isinstance(entry, dict) else {}


# Why the own installation of component (meta) is not the one to use beside the OS of os_meta, None when it is:
# it has to go with that OS and be newer than the one the OS brings along.
def skip_own(component, meta, os_meta):
    why = reason(component, meta, os_meta)
    if why is None and serial(meta) <= serial(bundled(os_meta, component)):
        why = "OLD_SERIAL"
    return why
