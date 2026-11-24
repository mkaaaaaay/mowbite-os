"""Names of the emergency bits in measure.py and xcore_capture.py against the catalog of mowbite_api (where the tree
has it), the message of hw/status and the board's own names as the NX60i announced them.

Run: python3 -m unittest discover -s tools/yf/test
"""
import ast
import json
import os
import re
import sys
import unittest

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(os.path.dirname(os.path.dirname(TOOLS)), "src")
sys.path.insert(0, TOOLS)
import xcore_capture as xc  # noqa: E402


def measure_names():
    """EMERGENCY_BITS and reason_text of measure.py, taken from its source without the ROS imports."""
    with open(os.path.join(TOOLS, "measure.py")) as f:
        tree = ast.parse(f.read())
    keep = [node for node in tree.body
            if isinstance(node, ast.Assign) and [getattr(t, "id", "") for t in node.targets] == ["EMERGENCY_BITS"]
            or isinstance(node, ast.FunctionDef) and node.name == "reason_text"]
    module = {}
    exec(compile(ast.Module(body=keep, type_ignores=[]), "measure.py", "exec"), module)
    return module["EMERGENCY_BITS"], module["reason_text"]


BITS, reason_text = measure_names()


class Names(unittest.TestCase):
    def test_catalog(self):
        path = os.path.join(SRC, "mowbite_api", "api.schema.json")
        if not os.path.exists(path):
            self.skipTest("this tree has no catalog (src/mowbite_api)")
        with open(path) as f:
            catalog = {entry["bit"]: entry["name"] for entry in json.load(f)["x-emergency-bits"]}
        self.assertEqual(BITS, catalog)

    def test_message(self):
        with open(os.path.join(SRC, "mowbite_msgs", "msg", "HardwareStatus.msg")) as f:
            constants = re.findall(r"^uint32 REASON_([A-Z_]+) = (\d+)$", f.read(), re.M)
        self.assertGreaterEqual(len(constants), 15)
        for name, value in constants:
            value = int(value)
            self.assertTrue(value and not value & (value - 1), name)
            self.assertEqual(BITS.get(value.bit_length() - 1), name)

    def test_board(self):
        with open(os.path.join(SRC, "mowbite_sim", "config", "xcore_nx60i_services.json")) as f:
            enums = json.load(f)["services"]["1"]["enums"]
        board = next(e["values"] for e in enums if e["id"] == "EmergencyReason")
        self.assertEqual(xc.REASON_BITS, board)
        for name, bit in board.items():
            self.assertEqual(BITS.get(bit), name)

    def test_reason_text(self):
        self.assertEqual(reason_text(0), "0")
        self.assertEqual(reason_text(1 | 8 | 16), "LATCH|LIFT|LIFT_MULTIPLE")
        self.assertEqual(reason_text(1 << 10 | 1 << 11), "MOWER_RPM_TIMEOUT|MOWER_RPM_LIMIT")
        self.assertEqual(reason_text(1 << 18 | 1 << 19 | 1 << 20), "WHEEL_RUNAWAY|FOREIGN_HOST|DRIVER_STALL")
        self.assertEqual(reason_text(1 << 21), "UNEXPECTED_RESTART")
        self.assertEqual(reason_text(1 << 22), "INPUTS_LEARNING")
        self.assertEqual(reason_text(1 << 23), "BOARD_NOT_HEARD")
        self.assertEqual(reason_text(1 << 24), "UNKNOWN")
        self.assertEqual(reason_text(1 | 1 << 25 | 8 | 1 << 30), "LATCH|LIFT|UNKNOWN")
        every = reason_text(0xFFFFFFFF).split("|")
        self.assertEqual(every, [BITS[bit] for bit in sorted(BITS)] + ["UNKNOWN"])
        for bit in range(32):
            self.assertEqual(reason_text(1 << bit), BITS.get(bit, "UNKNOWN"), bit)

    def test_capture_names(self):
        self.assertEqual(xc.reason_names(0), "0")
        self.assertEqual(xc.reason_names(1 << 10 | 1 << 12), "MOWER_RPM_TIMEOUT|FIRMWARE_INCOMPATIBLE")
        self.assertEqual(xc.reason_names(1 << 2 | 1 << 13 | 1 << 15), "STOP|UNKNOWN")
        self.assertEqual(xc.reason_names(1 << 2 | 1 << 3, {"STOP": 2}), "STOP|UNKNOWN")


if __name__ == "__main__":
    unittest.main()
