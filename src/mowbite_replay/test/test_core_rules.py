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


if __name__ == "__main__":
    unittest.main()
