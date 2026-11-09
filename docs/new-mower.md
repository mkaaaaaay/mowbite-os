# Adding a mower

MowBite OS reaches the hardware only through the contract `hw/*`. A new mower needs a driver for it, a fake board for
the simulation, its dimensions, a start file, a way onto the mower and a safety measurement. Logic, follower, safety
filter, map, planner, bridge and app stay as they are.

The drivers there are: `tron_mcu` (Airseekers Tron, serial MCU, GNSS and IMU as nodes of their own) and `xcore_driver`
(YardForce NX60i, OpenMower v2 board on Ethernet, passes IMU and GNSS on). Start from the closer one.

## 1. Driver

One package, its own process: a crash of another node must leave the watchdog of the driver running. Protocol code
without ROS in a library of its own with gtests; the node stays thin.

| What | Name | Type |
|---|---|---|
| publishes, latched | `hw/capabilities` | `mowbite_msgs/HardwareCapabilities` |
| publishes | `hw/status` | `mowbite_msgs/HardwareStatus` |
| publishes | `hw/wheel_odom` | `geometry_msgs/TwistStamped` |
| subscribes | `hw/cmd_vel` | `geometry_msgs/Twist` |
| service | `hw/emergency` | `mowbite_msgs/SetEmergency` |
| service | `hw/blade` | `mowbite_msgs/SetBlade` |
| service | `hw/charge` | `mowbite_msgs/SetCharge` |
| publishes | `imu/data` | `sensor_msgs/Imu` |
| publishes | `gps/absolute_pose` | `mowbite_msgs/GnssFix` |
| publishes, latched | `gps/datum` | `sensor_msgs/NavSatFix` |
| publishes | `gnss/gga` | `std_msgs/String` (for the NTRIP caster) |
| subscribes | `gnss/rtcm` | `mowbite_msgs/Rtcm` |

Duties:

- Drive: at most one frame per 100 ms to the board. Without `hw/cmd_vel` for a fixed time: three zero frames 100 ms
  apart, then none. A heartbeat alone stops nothing.
- Blade: on only with a lease (`lease_s`), off when it runs out; heights outside the limits of `hw/capabilities` are
  refused; never on while an emergency is latched.
- Emergency: reason bits numbered like the EMERGENCY event; stop button and lift stay latched until a release. The host
  latches only its own bits (`kHostLatchReasons` in `mowbite_hold/restart.h`).
- Limits until driving tests show more: 0.3 m/s and 0.3 rad/s.
- Before opening a port: no other process holds it.
- A restarted driver starts latched (`start_latched`).

## 2. Fake board

A fake board on a pty or a socket, so driver, logic and app run without the mower: see `mowbite_sim/scripts/fake_tron`
and `fake_xcore`. Then a contract test like `mowbite_sim/test/test_hw_contract.py` with your driver and your fake. It
checks capabilities, status, the frame rate, the watchdog, the blade lease, the emergency and its release.

## 3. Dimensions and settings

- `mowbite_bringup/config/geometry/<platform>.yaml`: `geometry.*` in m, base_link in the middle of the drive axle, x
  forward, y left: `platform`, `robot_width`, `robot_front`, `robot_rear`, `mower_width`, `blade_ahead`,
  `blade_offset`, `antenna_x`, `antenna_y`, `track`, `wheel_radius`, `dock_contact_x`, `ticks_per_m`, `imu_roll`,
  `imu_pitch`. `dock_contact_x` below 0: contacts at the back, the mower backs onto the station. Contacts in front are
  not built yet.
- `mowbite_bringup/config/<platform>.yaml`: what differs from `tron.yaml`, which lies underneath (battery voltages
  empty, full and critical, charge check, temperatures). Check every value you inherit.
- Driver parameters (ports, baud rates, limits) in the file of the driver; values measured on one mower in a file on
  that mower.

## 4. Start file

`mowbite_bringup/launch/<platform>.launch.py`: the driver as its own process, the rest from `stack.py` (container with
all nodes above the contract, and the planner). Copy `nx60i.launch.py`: it restarts the driver at most three times,
each time latched, and leaves `driver_exit.json`.

## 5. Onto the mower

- Docker Compose like the NX60i (`docker/mower/nx60i.compose.yaml`), or a native bundle on the mower's own rootfs like
  the Tron (`platform/tron/`).
- Map, settings and journal live in the data dir, never in a container volume.
- Every log, recording and cache gets a limit in days and MB.

## 6. Safety measurement

Before the mower drives by itself. On blocks, wheels free, blade off, someone at the stop button:

1. How long the board keeps the last setpoint without the host: `failsafe_without_host`.
2. Stop button, lift, bumper left and right: wheels and blade stop, the reason bits are right, latched until release.
3. Drive in the air: forwards, backwards, turning; left and right the right way round.
4. The blade last: briefly, in the air, heights within the limits.

Then set `safety_measurement_date` in the driver parameters. Without it the mower takes remote control only
(`NO_SAFETY_MEASUREMENT`).

## Check list

- [ ] Driver package: protocol code with gtests, build without warnings
- [ ] Fake board, contract test passes
- [ ] Geometry file and platform file, inherited values checked
- [ ] Start file, driver restarts latched
- [ ] Simulation: mowing, going home, docking; `app_interface_check`
- [ ] Way onto the mower chosen, limits for logs and journal
- [ ] Safety measurement done, date set
- [ ] First drive watched: blade off, RTK fix, at most 0.3 m/s
