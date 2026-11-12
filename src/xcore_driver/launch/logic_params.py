"""The battery voltages of the logic, which a driver that sets them on its board takes under the same names: they stand
only in the parameter files of mowbite_logic (mowbite_bringup/config/<platform>.yaml). nx60i.launch.py and
measure.launch.py read them here.
"""
import yaml

BATTERY_VOLTAGES = ("battery_empty_voltage", "battery_full_voltage", "battery_critical_voltage")


# The battery voltages of the logic in the parameter files, the last file winning as for the node.
def battery_voltages(files):
    values = {}
    for path in files:
        with open(path) as f:
            params = ((yaml.safe_load(f) or {}).get("mowbite_logic") or {}).get("ros__parameters") or {}
        values.update({key: params[key] for key in BATTERY_VOLTAGES if key in params})
    return values
