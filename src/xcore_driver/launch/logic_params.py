"""The battery voltages of the logic, which a driver that sets them on its board takes under the same names: they stand
only in the parameters of mowbite_logic (platform.yaml of the platform). platform.py of the NX60i and
measure.launch.py hand them over.
"""
import yaml

BATTERY_VOLTAGES = ("battery_empty_voltage", "battery_full_voltage", "battery_critical_voltage")


# The battery voltages among the parameters of the logic.
def battery_voltages(logic):
    return {key: logic[key] for key in BATTERY_VOLTAGES if key in logic}


# The parameters of the logic in one file, for a start without the stack.
def logic_section(path):
    with open(path) as f:
        return ((yaml.safe_load(f) or {}).get("mowbite_logic") or {}).get("ros__parameters") or {}
