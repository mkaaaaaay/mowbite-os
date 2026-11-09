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
| publishes, optional | `hw/button` | `mowbite_msgs/ButtonEvent` (reliable, never latched) |
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
  latches only its own bits (`kHostLatchReasons` in `mowbite_hold/restart.h`). A release waits a short window for the
  flags to drop; a new reason within it ends the window without releasing.
- Release at the mower (optional, G37): a driver that sees every sample of its stop button may recognise the gesture
  itself (`tron_mcu/stop_release.h`: free 2 s, held 3 to 10 s, let go) and report HELD, LONG and CANCELED on
  `hw/button`. It never releases by itself: the logic does, like `emergency.reset`, with the `latch_count` of LONG, and
  the driver refuses that with `LATCH_CHANGED` once another reason latched since. It then fills the
  `stop_release_*` times of `hw/capabilities` (the app shows them, `system.info.stop_release`). Without it they stay 0
  and the release goes by app only.
- Capabilities: `platform` is the same name as in the geometry file; set `gnss_receiver_time` only where
  `receiver_time_ms` of `gps/absolute_pose` is the UTC time of day of the receiver (Tron), not a clock of the board
  (xCore). Nothing in the code asks for a platform name.
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
  `imu_pitch`. Without `platform` or a number the start stops. `dock_contact_x` below 0: contacts at the back, the mower backs onto the station. Contacts in front are
  not built yet.
- `mowbite_bringup/config/<platform>.yaml`: what differs from `tron.yaml`, which lies underneath (battery voltages
  empty, full and critical, charge check, temperatures). Check every value you inherit.
- Driver parameters (ports, baud rates, limits) in the file of the driver; values measured on one mower in a file on
  that mower.
- The datum of the garden: one place on the mower, `MOWBITE_DATUM_LAT`, `_LONG`, `_HEIGHT` (Tron `/etc/mowbite/os.conf`,
  NX60i the environment of the compose file), handed to the GNSS node as launch arguments `datum_*` after every
  parameter file. Without a datum no pose is published.

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
   With a release at the mower: the press that latched never releases, a held press does, anything else does not.
3. Drive in the air: forwards, backwards, turning; left and right the right way round.
4. The blade last: briefly, in the air, heights within the limits.

Then set `safety_measurement_date` in the driver parameters. Without it the mower takes remote control only
(`NO_SAFETY_MEASUREMENT`).

## 7. Lights (optional)

`mowbite_light` drives a WS2812 chain on spidev after the robot state, with a profile per state in the app (`light.*`);
the emergency signal is fixed. Load it as a component of the container only when the drivers run, like
`tron.launch.py` with `hardware:=true`.

- Parameters under `mowbite_light` in `<platform>.yaml`: `output` (`none` until checked on the mower, then `spidev`;
  `file` in the simulation), `device`, `leds` (lit), `buffer_leds` (driven), `strips` (runs `first-last` that chase
  and gradient follow, backwards when last < first, not overlapping; empty: the whole chain), `frame_rate`,
  `max_brightness`, `max_current_ma`, `ma_per_channel`, `low_battery`.
- With `output: none` or an invalid value the node offers no `light.*` method and the app hides the lights; invalid
  `strips` only fall back to the whole chain.
- The service user needs the device: on a native rootfs a udev rule like
  `platform/tron/rootfs/files/usr/lib/udev/rules.d/60-mowbite-spidev.rules`, in Docker the device passed in.
- On the mower: `output: spidev` in the file of that mower, then one `light.preview`.

## Check list

- [ ] Driver package: protocol code with gtests, build without warnings
- [ ] clang-tidy without findings in the package (`.clang-tidy` of the repo)
- [ ] Fake board, contract test passes
- [ ] Geometry file and platform file, inherited values checked
- [ ] Start file, driver restarts latched
- [ ] Simulation: mowing, going home, docking; `app_interface_check`
- [ ] Way onto the mower chosen, limits for logs and journal
- [ ] Safety measurement done, date set
- [ ] Lights, if any: device rights, `output` on only after a look on the mower
- [ ] First drive watched: blade off, RTK fix, at most 0.3 m/s
