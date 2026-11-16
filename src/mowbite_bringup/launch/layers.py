"""The parameter files of a platform in their order: stack.yaml for every mower, then the file of the platform. A value
of stack.yaml that every platform has to set stands there as PLACEHOLDER; no node starts while one is left (stack.py).
"""
import os
import re

import yaml

PLACEHOLDER = "<platform>"
CONFIG = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "config"))
STACK = os.path.join(CONFIG, "stack.yaml")


def files(platform):
    """stack.yaml and the file of this platform."""
    return [STACK, os.path.join(CONFIG, f"{platform}.yaml")]


def matches(section, node):
    """Whether a section of a parameter file holds for the node of this full name (no leading slash): its name, or
    with * for one part and ** for any number of parts of the name."""
    pattern = re.escape("/" + section.lstrip("/")).replace(r"/\*\*", r"(/\w+)*").replace(r"/\*", r"/\w+")
    return re.fullmatch(pattern, "/" + node) is not None


def flat(values, prefix=""):
    """Nested parameters under their dotted names."""
    result = {}
    for key, value in values.items():
        if isinstance(value, dict):
            result.update(flat(value, f"{prefix}{key}."))
        else:
            result[f"{prefix}{key}"] = value
    return result


def sections(path):
    """[(section, parameters)] of a parameter file in its order; ValueError when it is no parameter file."""
    with open(path) as f:
        try:
            content = yaml.safe_load(f) or {}
            return [(name, flat((section or {}).get("ros__parameters") or {})) for name, section in content.items()]
        except (yaml.YAMLError, AttributeError, TypeError) as error:
            raise ValueError(f"{path}: {error}")


def merged(loaded, node):
    """What the sections of files give this node, a later file and a later section winning."""
    result = {}
    for file_sections in loaded:
        for section, values in file_sections:
            if matches(section, node):
                result.update(values)
    return result


def parameters(paths, node):
    """What the files give this node."""
    return merged([sections(path) for path in paths], node)


def unset(paths, nodes):
    """<node>.<parameter> of every value these nodes would get as the placeholder."""
    loaded = [sections(path) for path in paths]
    return sorted(f"{node}.{key}" for node in nodes for key, value in merged(loaded, node).items()
                  if value == PLACEHOLDER)
