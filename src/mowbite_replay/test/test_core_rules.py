"""The rule that makes a core replayable: its sources read no clock, no randomness, no environment and no file, start
no thread and keep no state the journal does not see (mutable, thread_local, static variables). The sources are those
of the core libraries in the CMakeLists of their packages and the headers of those packages.

Run: python3 test_core_rules.py
"""
import os
import re
import unittest

SRC = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
CORES = {
    "mowbite_logic": "mowbite_logic_core",
    "mowbite_localization": "mowbite_localization_core",
    "mowbite_safety": "mowbite_safety_core",
    "tron_mcu": "tron_mcu_core",
    "mowbite_api": "mowbite_api_core",
    "mowbite_schedule": "mowbite_schedule_core",
}
# Not part of a core although in its library: the STOP clear mark belongs to the node, the core only gets it as
# SetStopClearMark.
NOT_CORE = {"tron_mcu/src/stop_clear_mark.cpp", "tron_mcu/include/tron_mcu/stop_clear_mark.h"}
# The parts of the logic core (FP25), in the order of their names.
LOGIC_PARTS = ["calibration", "docking", "going_home", "mowing", "recording", "release", "schedule", "setup",
               "undocking"]
RULES = [
    ("clock", r"\b(steady_clock|system_clock|high_resolution_clock|clock_gettime|gettimeofday)\b|\btime\s*\("),
    ("randomness", r"\b(random_device|rand|srand|drand48)\b\s*[({]"),
    ("environment", r"\b(getenv|secure_getenv|setenv|environ)\b"),
    ("file", r"\b(fstream|ifstream|ofstream|fopen|filesystem)\b|\bopen\s*\(|#include\s*<fstream>"),
    ("thread", r"\b(std::thread|std::async|std::mutex|pthread_\w+|thread_local)\b"),
    ("hidden state", r"\bmutable\b|^\s*static\s+(?!const\b|constexpr\b|inline\b)[\w:<>,\s*&]+?\s+\w+\s*(=|;|\{)"),
]


def core_files():
    files = []
    for package, library in CORES.items():
        with open(os.path.join(SRC, package, "CMakeLists.txt")) as f:
            cmake = f.read()
        match = re.search(rf"add_library\({library}\s+STATIC\s+([^)]*)\)", cmake)
        assert match, f"no {library} in {package}/CMakeLists.txt"
        files += [os.path.join(package, name) for name in match.group(1).split()]
        include = os.path.join(SRC, package, "include", package)
        files += [os.path.join(package, "include", package, name) for name in sorted(os.listdir(include))
                  if name.endswith(".h") and not name.endswith("_journal.h")]
    return [f for f in files if f not in NOT_CORE]


def code_lines(path):
    """The lines without comments and string literals."""
    with open(os.path.join(SRC, path)) as f:
        text = f.read()
    text = re.sub(r"/\*.*?\*/", lambda m: "\n" * m.group(0).count("\n"), text, flags=re.S)
    text = re.sub(r'"(\\.|[^"\\\n])*"', '""', text)
    return [re.sub(r"//.*", "", line) for line in text.split("\n")]


class CoreRules(unittest.TestCase):
    def test_the_cores_see_nothing_beside_the_journal(self):
        files = core_files()
        self.assertGreater(len(files), 20)
        found = []
        for path in files:
            for number, line in enumerate(code_lines(path), 1):
                # A static function or member function is no state.
                for name, pattern in RULES:
                    if re.search(pattern, line) and not (name == "hidden state" and "(" in line):
                        found.append(f"{path}:{number}: {name}: {line.strip()}")
        self.assertEqual(found, [], "\n".join(found))

    def test_the_parts_of_the_logic_do_not_include_its_core(self):
        """A part of the logic core (include/mowbite_logic/<name>_part.h and src/logic_<name>.cpp) reaches the core only
        through CoreAccess and CoreFacts, never through logic_core.h."""
        include = os.path.join(SRC, "mowbite_logic", "include", "mowbite_logic")
        parts = []
        for header in sorted(os.listdir(include)):
            if header.endswith("_part.h"):
                parts.append(os.path.join("mowbite_logic", "include", "mowbite_logic", header))
                parts.append(os.path.join("mowbite_logic", "src", "logic_" + header[:-len("_part.h")] + ".cpp"))
        names = [os.path.basename(path)[:-len("_part.h")] for path in parts[::2]]
        self.assertEqual(names, LOGIC_PARTS)
        found = []
        for path in parts:
            own = os.path.basename(path).replace("logic_", "").replace("_part.h", "").replace(".cpp", "")
            with open(os.path.join(SRC, path)) as f:
                lines = f.read().split("\n")
            for number, line in enumerate(lines, 1):
                if re.match(r'\s*#\s*include\s*[<"]mowbite_logic/logic_core\.h[>"]', line):
                    found.append(f"{path}:{number}")
                # Nor into another part: what two parts share is an edge of the core.
                other = re.match(r'\s*#\s*include\s*[<"]mowbite_logic/(\w+)_part\.h[>"]', line)
                if other and other.group(1) != own:
                    found.append(f"{path}:{number}")
        self.assertEqual(found, [], "\n".join(found))


if __name__ == "__main__":
    unittest.main()
