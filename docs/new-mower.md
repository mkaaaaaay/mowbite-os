# Adding a mower

MowBite OS reaches the hardware only through the contract `hw/*`. A new mower needs a driver for it, a fake board for
the simulation, a platform package (its dimensions, its values and its part of the start), a way onto the mower and a
safety measurement. Logic, follower, safety filter, map, planner, bridge, app and the start of the stack stay as they
are: no file of `mowbite_bringup` names a mower.

The drivers there are: `tron_mcu` (Airseekers Tron, serial MCU, GNSS and IMU as nodes of their own) and `xcore_driver`
(YardForce NX60i, OpenMower v2 board on Ethernet, passes IMU and GNSS on). Start from the closer one.

## 1. Driver

One package, its own process: a crash of another node must leave the watchdog of the driver running. Protocol code
without ROS in a library of its own with gtests; the node stays thin. Build the node like every component: register it
with `mowbite_component_register` instead of `rclcpp_components_register_node` (hidden symbols, several MB less RSS)
and give the base class `mowbite_component::StartupOnly(options)`: parameters are read once at the start, there are
no parameter services. A driver on a serial port takes it from `mowbite_serial` (`SerialPort`: raw 8N1, exclusive;
`ProcessesUsing`: who else holds the device, asked before the port is opened).

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
| publishes, latched, optional | `hw/inputs` | `mowbite_msgs/HardwareInputs` |
| service, optional | `hw/silence` | `mowbite_msgs/SilenceDriver` |
| service, optional | `hw/end` | `mowbite_msgs/EndDriver` |
| publishes | `imu/data` | `sensor_msgs/Imu` |
| publishes | `gps/absolute_pose` | `mowbite_msgs/GnssFix` |
| publishes, latched | `gps/datum` | `sensor_msgs/NavSatFix` |
| publishes | `gnss/gga` | `std_msgs/String` (for the NTRIP caster) |
| subscribes | `gnss/rtcm` | `mowbite_msgs/Rtcm` |
| publishes, optional | `hw/dock_marker` | `mowbite_msgs/DockMarker` |
| subscribes, optional | `hw/dock_marker_request` | `std_msgs/Empty` |

Duties:

- Drive: `hw/cmd_vel` goes on to the board as it comes, at most one frame per 100 ms. Without `hw/cmd_vel` for the
  `cmd_vel_timeout` of your driver (0.5 s at most): three zero frames 100 ms apart, then none. A heartbeat alone stops
  nothing.
- Blade: on only with a lease (`lease_s`), off when it runs out, told to the board at least three times; heights
  outside the limits of `hw/capabilities` are refused; never on while an emergency is latched. With
  `blade_height_control` the driver gives its limits and the height it cuts at for `height_mm` 0
  (`blade_height_default_mm`); the app then offers `mow_height_mm`, which the logic sends at each start of the blade
  and again at once when it changes while the blade runs: the board has to take a new height with the blade on.
  Without it the logic always sends 0.
- Blade motor fault: when your board reports a fault of the blade motor (stall, overload, over-current, ...), the
  driver switches the blade off with `BLADE_OFF_MOTOR_FAULT` and refuses it, naming the fault, until a report
  without one; no emergency.
- Emergency: reason bits numbered like the EMERGENCY event; stop button and lift stay latched until a release. The host
  latches only its own bits (`kHostLatchReasons` in `mowbite_hold/restart.h`). A release waits a short window for the
  flags to drop; a new reason within it ends the window without releasing.
- A new emergency bit goes into `EmergencyBits()` of `mowbite_api/src/catalog.cpp` with its class: symptom (in
  `ClassifiedBits`) when it only tells that a part went silent or timed out, else cause; the stop book explains a stop
  only by a bit of class cause.
- `latch_count` of `hw/status` goes up with every reason that latches and is never 0 once one did. A release that
  carries it is refused with `LATCH_CHANGED` when another reason latched since: so the setup program releases the
  emergency of an input it just checked and never one that came meanwhile. A driver that leaves it 0 gets no such
  release, the user releases in the app.
- Inputs one by one (optional): `hw/inputs` names every input with its role, the emergency it brings, whether it is
  switched, and `held` when the board keeps it switched until the release (then a release is the only way to see
  that the sensor is free). The setup program checks every line of a role with it, so a dead contact of a pair
  shows. A driver whose table of inputs is a setting (`xcore_driver`: `inputs.*`) also takes `inputs.candidates`,
  the lines a sensor can be on, and `inputs.learn`: started with it, it watches the candidates instead of the table,
  none of them stops anything, and it holds the emergency `INPUTS_LEARNING` that no release ends, so nothing moves
  until a start without it. A release is answered with `LEARNING`; what else latched meanwhile (a tilt while the
  mower is lifted) it ends by the usual checks, so the mower is never caught in learning.
- Silence and end for the setup program (optional): `hw/silence` (seconds from 0.2 to `silence_max_s` of
  `hw/capabilities`) makes the driver send nothing to the board from its answer on, keep publishing what it reads,
  refuse releases and blade requests meanwhile (`SILENT`, `kBlocked`), and at the end latch `HIGH_LEVEL` (with
  `latch_count` one up), send the blade off and three stops. `hw/end` answers and then ends the process with exit code
  3 without another byte to the board; the start counts it like a crash and starts the driver again latched. Both
  refuse with `READ_ONLY`, `CHARGING` (also on the raw contact), `BLADE` (lease, blade on, running down) and hw/silence
  with `OUT_OF_RANGE`, `BUSY` and `NO_SETUP` (no active `setup/motion` within the last second: outside a setup step
  the timeout of `hw/cmd_vel` stays in force). hw/silence with `blade` true goes silent with the blade on as well (the step `blade`
  measures that the board stops it alone); after it no lease runs on. Offer them with `silence_max_s` (a parameter
  `silence_max` well past the time the board stops without its host; 0 without) and `driver_end` in
  `hw/capabilities`. The steps `silence` and `driver_end` measure with them how the board stops without its host.
- Blade allowed: `blade_allowed` of `hw/capabilities` is true when hw/blade would take an on (`allow_blade` of the
  driver, false in read_only). The step `blade` writes `allow_blade` into the file of the setup program when it is
  false.
- Release at the mower (optional, G37): a driver that sees every sample of its stop button may recognise the gesture
  itself (`tron_mcu/stop_release.h`: free 2 s, then a short press of 0.1 to 2 s) and report HELD, LONG and CANCELED
  on `hw/button`. It never releases by itself: the logic does, like `emergency.reset`, with the `latch_count` of LONG,
  and the driver refuses that with `LATCH_CHANGED` once another reason latched since. It then fills the
  `stop_release_*` times of `hw/capabilities` (the app shows them, `system.info.stop_release`). Without it they stay 0
  and the release goes by app only. A board that keeps its stop flag until a clear frame (the Tron) shows no second
  press: the driver clears it during the emergency (`stop_clear_period`), so the board no longer locks itself then.
  Before the first such clear it writes a mark in `data_dir` and removes it on the release; a start with the mark
  latches, so a crash in that time never unlocks unseen. Measure first how the board keeps stop, bumper and lift
  (Tron: `mowbite-measure --steps T8`).
- Buttons for home, start and power (optional): the node that reads them publishes SHORT and HELD on `hw/button`
  (`ButtonEvent`, HELD once at `button_hold_s`), and `button_hold_s` of `hw/capabilities` names that time; the same
  value feeds both, from one place of the platform. `hw/button` may have several publishers, e.g. the driver for STOP
  and a node of its own for the other buttons. The logic ignores them in SETUP and while the emergency is latched,
  and while charging all but START held; a start by button passes the same checks as one from the app. Where holding POWER cuts the power
  in hardware (the Tron, after about 3 s), HELD of POWER comes just before: switching the blade off and stopping then
  is only an attempt, what is written has to be written before.
- Capabilities: `platform` is the same name as in the geometry file; set `gnss_receiver_time` only where
  `receiver_time_ms` of `gps/absolute_pose` is the UTC time of day of the receiver (Tron), not a clock of the board
  (xCore). `mowbite_clock` compares it with the wall clock: off by more than 60 s, date and sun count as unknown
  (`time/clock`, `gnss_mismatch`). Nothing in the code asks for a platform name.
- Limits until driving tests show more: 0.3 m/s and 0.3 rad/s.
- Before opening a port: no other process holds it.
- A restarted driver starts latched (`start_latched`).
- Stops: the state of the board is unknown at every start and after a port reopens, so the driver sends the stop
  frames then, latched or not. Before every frame that clears a latched sensor on the board, a stop. A board that
  reports its wheel speeds is checked after every stop like the blade: still turning `wheel_stop_time` after the stop
  (`tron_mcu`), the driver latches `MOWER_NOT_STOPPED` and stops again. When the process ends it sends the stop and the
  blade off several times; the stop only stays out on the charging contacts without a command.
- Start: until the first sample of a sensor it needs or the first version report of the board, a driver may refuse
  drive and blade without an emergency, for a fixed start time at most (`tron_mcu`: `imu_start_timeout`, 3 s for the
  version). After it the driver accepts or reports an emergency; a version that does not match is reported at once.
  Whoever starts a driver waits that long before the first request.
- Firmware: the platform file lists the firmware of each board the driver was measured with (`tron_mcu`:
  `cutter_firmware`, `chassis_firmware`); firmware outside a list blocks drive and blade (`FIRMWARE_INCOMPATIBLE`), so
  a mower on a newer vendor stand stays still until that stand is measured. The safety measurement narrows the list in
  the file of the mower to the firmware it ran with.
- Hear first: after opening its port a driver sends nothing there, no heartbeat either, until a valid frame of its
  board came. Past its telemetry timeout without one it reports `BOARD_NOT_HEARD` (not latched, the first frame ends
  it); a reopened port starts it anew. The contract test checks it with the fake board started quiet
  (`board_quiet`). A passive receiver, as the GNSS of the Tron, only waits for a valid line before it writes.
- Dock marker (optional): a process of its own that measures only while asked on `hw/dock_marker_request`, one
  image per message on `hw/dock_marker`, never an old or estimated pose; the times, states and reasons are in
  `DockMarker.msg`. What the mower drives with it and when it stops is up to the logic. The Tron's process is
  `tron_rear` (`tron_vision`, section 8). The stamp is the time the image was taken, on the clock of the host: the
  localization fuses each image at that moment and drops it when it arrives more than `marker_max_age` late.
- Sensors of its own for the app (temperatures, states of a board) go over `bridge/sensor_info` and
  `bridge/sensor_value` under ids in the words of the OS (`stereo_state`, not names of another system). The
  battery, charging, blade, drive and position values come from the logic out of `hw/status`; a driver does not send
  them again. The bridge shows every sensor with its id as `key`; with `openmower_names` (the default) it shows the
  ten sensors of the logic the app of OpenMower knows under their `om_*` ids, from the table of the catalog
  (`openmower_id` in `mowbite_api/src/catalog.cpp`). A new platform adds nothing there.

## 2. Fake board

A fake board on a pty or a socket, so driver, logic and app run without the mower: see `mowbite_sim/scripts/fake_tron`
and `fake_xcore`. In `mowbite_sim` a new mower adds: its fake in `scripts/` (and in `install(PROGRAMS ...)` of
`CMakeLists.txt`), a `test_depend` on its platform package in `package.xml`, and its case in the driver contract,
`test/contract_<platform>.py` with a class `Case` like `contract_tron.py`. The case starts your fake and your driver
with the values your platform ships, presses stop and lifts the mower at the fake and reads the drive and blade
commands the fake got; what it has to offer is listed at the top of `test_driver_contract.py`. That test runs once per
case (`MOWBITE_PLATFORM=<platform>`), a platform without a case fails `test_config_consistency.py`, and it checks
through `hw/*` alone:

- `a` capabilities latched and complete, `platform` as in the geometry file
- `b` status healthy, wheel odometry there
- `c` drive passed on and cut to `max_linear_speed`; after `cmd_vel_timeout` without `hw/cmd_vel`: three stops
  100 ms apart, then nothing
- `d` blade off when its lease runs out, told three times
- `e` blade stays on while the lease is renewed
- `f` blade requests outside lease, height or speed refused
- `fa` a blade motor fault switches the blade off and refuses it until it clears (with `blade_fault` in the case)
- `g` an emergency of the host blocks blade and drive
- `h` stop and `i` lift stay latched until the release
- `j` the host latches with its own bits only
- `k` a hold blocks blade and drive without an emergency

What only your board has (frames, configuration, restarts, firmware) goes into a test of its own like
`test_hw_contract.py` (Tron) and `test_xcore_contract.py`. `test_hw_start.py` checks the start time of a driver that
has one.

## 3. Dimensions and settings

- `geometry.yaml` of the platform package (section 4): `geometry.*` in m, base_link in the middle of the drive axle, x
  forward, y left: `platform`, `robot_width`, `robot_front`, `robot_rear`, `mower_width`, `blade_ahead`,
  `blade_offset`, `antenna_x`, `antenna_y`, `track`, `wheel_radius`, `wheel_width`, `dock_contact_x`, `ticks_per_m`,
  `imu_roll`, `imu_pitch`. Without `platform` or a number the start stops, also when the body (box or outline) does
  not cover the drive wheels from above (`wheel_radius` ahead and behind the axle, `track` / 2 ± `wheel_width` / 2
  aside); `robot.geometry.set` refuses such a body with `WHEELS_OUTSIDE`. Optional `robot_outline`: the contour from above as a
  flat list x0, y0, x1, y1, ... (at least 3 points); the planner then uses it instead of the rectangle (the Tron has
  one).
  Optional `robot_shape`: how the mower looks from above, for the app to draw it and to mark a part in a step of the
  setup program. `robot_shape.parts` lists the ids in the order they are drawn, each part has `kind` (one of
  `x-setup.shape_kinds`: body, wheel, caster, bumper, button, sensor, handle), `points` as a flat list in base_link and
  `closed` (true by default; false for a line or a single point). At most 32 parts, 200 points a part and 1000 in all,
  none farther than 1.5 m from base_link. It is shown only: the body the mower plans with stays box or outline, and
  that has to cover every part (`test_config_consistency.py`). An outline of the user that leaves a part out is told
  with `SHAPE_OUTSIDE`.
  Optional `dock_footprint`: the docking station from above in base_link of the docked mower, the same kind of list,
  for the planner to keep its lanes and turns off it; the platform's alone, without it the planner takes its own.
  `dock_contact_x` says where the charging contacts are: below 0 at the back (the mower backs onto the station and
  leaves it forwards), above 0 in front (forwards onto it, straight back off it, never faster than `docking_speed`;
  the safety filter cuts every reversing of the follower to 0.12 m/s), 0 none (no docking, `docking.start` is not
  offered). Set `contact_speed` and `dock_overshoot` to how late the board reports the contact (`platform.yaml` of the
  NX60i).
  In the same file under `mowbite_localization`: `antenna_height`, the GPS antenna above the ground (m, 0 to 1). On a
  slope the localization tilts the lever (`antenna_x`, `antenna_y`, `antenna_height`) by roll and pitch from
  `imu/data` (its orientation, or gravity when `orientation_covariance[0]` is -1), so the pose stays over the ground
  point under base_link. 0 until measured keeps the lever level.
- The dock marker, under `/**` of the platform file (one place for all nodes): `dock_marker` [x, y, yaw], the marker
  in base_link while the mower stands docked (m, rad, the direction its front faces), `marker_blind_distance` (m
  before the docked pose where it leaves the image), `marker_range` (m up to which it is found; 0: no marker, then
  no `localization/dock_pose`) and `marker_max_age` (s from the image to its arrival, measured on the mower). The
  localization then estimates base_link in the station frame (origin base_link of the docked mower, x the way in)
  from the marker alone, never mixed with the map pose. The camera has to look to the side of the contacts: every
  drive by the marker runs contacts first. In the simulation `fake_dock_marker` plays the marker with an error model
  of its own (parameters, flags file).
  Paths in the station frame (`FollowPath` frame `FRAME_DOCK`) go out on `follower/dock_cmd_vel`; the safety filter
  passes them as the dock source with the safety measurement and a recent pose from the marker with a heading and
  `dock_max_position_accuracy` at the docked place plus `dock_accuracy_per_m` per m from it (both under `/**`; the
  first is also the limit of the reference check and the gate in the map; the second follows from the error in the
  direction of the marker, measured for each platform, Tron 0.09 for 3 to 5 degrees), within
  the zone of the station in that frame (FENCE beyond `fence_margin`), at most 0.17 m/s forwards and 0.12 m/s
  backwards, with no map needed.
- With a marker the logic offers `docking.test`: docking by the marker alone, without map or RTK, from where the
  mower stands by hand on the line in front of the station (the marker at most half of `marker_cross_track` beside
  the axis of the mower, facing along the line, so that the straight way out ends on it) or at its start. The heading
  of the marker counts there only with its spread: where the line begins may lie that spread times the way to the
  marker further beside it, up to `marker_cross_track`; the gate by the marker also takes the marker beside the axis.
  Every leg runs in the station frame and reaches the wheels only as the dock source. Check on the new mower, in
  this order: the reference measurement of the marker (`dock_marker`), `marker_blind_distance` (the last part is
  blind from there), then the test with the mower watched, before any docking by the marker with RTK. Its values
  `marker_wait`, `marker_cross_track` and `marker_scale_limit` stand in `mowbite_logic` of `stack.yaml`.
- `dock_guide` in `mowbite_logic` of the platform file decides whether `docking.start` and going home use the marker:
  `none` (by RTK only, as `stack.yaml` has it for a platform without a marker), `prefer` (by
  the marker; where its source cannot measure, this try by RTK with `DOCKING_WITHOUT_MARKER`) or `require` (never
  without it). With the marker the line begins at most `marker_range` in front of the station, the logic asks for the
  marker from 3 m before that point and gives the estimate the heading of the map as a hint. Each try chooses there:
  while RTK is good the station by the marker has to lie within `dock_reference_limit` plus the accuracy of the map from
  the recorded one, at the gate also within 10 degrees of its heading (`MARKER_MISMATCH`, docking ends where the mower
  stands). Start with `require` once `docking.test` passed, and switch to `prefer` only after docking by RTK was
  measured on the mower. A camera that does not look to the side of the contacts takes `none`.
- How the mower drives, in the platform file: `mowbite_follower` speeds up at `acceleration` and brakes before corners
  and the end at `deceleration` (m/s², by default the same), turns on the spot up to `spot_max_angular` (0: as on the
  path, `max_angular`; in the station frame always `max_angular`) and slows such a turn down before its target at
  `turn_deceleration` (rad/s², 0: `heading_gain` times the error). Where the wheels slip on such turns,
  `turn_feedback` (0 to 1, 0: none) adds that share of the rate the heading stays behind the command, at most
  `turn_feedback_max` rad/s and never above the limit of the turn; not in the station frame. Wheels that slip
  unevenly move the mower while it turns: with `turn_hold_gain` (1/s, 0: off, at most 4) the follower drives against
  that along the heading, as far as the slower wheel keeps `wheel_min_speed` (m/s, the speed below which the wheels
  of the mower stick or jerk; measure it) on the `wheel_distance` the driver reports in `hw/capabilities`. With that
  distance the localization also widens the accuracy of the position by half of it times what the gyro turns less
  than the wheels, so report the real one. Without a fix younger than `turn_fix_timeout` (0.5 s, 0: no such rule) the
  follower does not turn on the spot in the map. With `max_turn_slide` (m, 0: no such limit) the path ends with
  `TURN_SLIDE` once base_link is that far from the place such a turn began at: take what the body may leave its lane
  by next to an edge. Every such turn goes out on `follower/spot_turn` (commanded, wheels,
  heading, how far base_link got) and the log sums them up per path. The calibration drive
  turns on the spot at `spin_rate` of `mowbite_calibration` (at most what the driver allows): raise it where
  the wheels stand in their dead band at that rate, the drive ends with `NOT_TURNING` then (the gyro on average
  below `turn_min_ratio` of the rate, 0.7). Its lines never go below `line_min_speed` while their end is ahead: take
  the slowest speed the wheels still turn at. `mowbite_safety` lets every moving command grow at
  most `ramp_linear` m/s² and `ramp_angular` rad/s² from the one before, whatever its source (0: no limit); less, a stop
  and an emergency go out at once. Measure how fast the wheels follow before raising any of them.
- `platform.yaml` of the platform package: the values of this mower on top of `stack.yaml` of `mowbite_bringup`, which
  holds for every mower and has the meaning of each value. `stack.yaml` marks what a platform has to set as
  `<platform>`: battery and
  charge check, docking and the contacts, how the mower drives, tilt and ramps, gyro and noise, the limits of the
  journal and the libraries it names (`journal_libraries`: a part of the names of the libraries of your packages,
  `[xcore]` on the NX60i; their build IDs stand in every run), and the devices of light and sound for a platform that
  starts them. Set each in the same section of your
  file. While one is left for a node the platform starts, the start halts with `PLATFORM_CONFIG` and its names before
  any node runs; `test_layers.py` of `mowbite_bringup` holds the same for every platform of the workspace, yours
  among them once its package is built. A part that is off in `stack.yaml` (dock marker, `dock_guide`, `turn_hold_gain`,
  `crash_record`) is
  turned on only where the mower has it. No value of another mower comes along by itself: one you take over
  unmeasured stands in your file as an assumption (the NX60i marks them "Of the Tron"), and that list is what the
  safety measurement has to settle. The battery voltages stand only
  here: a driver that sets them on its board gets them from `platform.py`, which reads them with
  `start.parameters("mowbite_logic")` and hands them over under the same names (see `platform_nx60i`).
- The way home without the planner (`docking_direct`, `direct_margin` of `stack.yaml`): a planner that
  gives no answer does not keep the mower from the station. It drives the straight line to the point
  `docking_approach_distance` m in front of the station, where no obstacle and no end of the mowing and navigation
  areas lies closer to that line than the turning circle of the body (base_link to its farthest point) plus
  `direct_margin` plus `max_position_accuracy`: on the Tron 0.90 m, more than its `docking_approach_distance`. Where
  the areas around the station are narrower, the mower waits for the planner there; for the straight way draw them
  wider or set `docking_approach_distance` larger. `direct_margin` is also how far base_link may get off the line
  before the drive ends, and has to be above 0.
- Recording areas: every ring driven forward with automatic points, without a break and without reversing, and closed
  by driving on past its start until the track crosses it flatly (at most 30 degrees), keeps the body it was recorded
  with in `properties.recorded_body` (with `by`, the id in `<data_dir>/recorded_by`), and the planner may then drive
  its line without a margin; robot.geometry.set of the body moves it on those areas. So measure the body before the
  first recording. The recorder tells reversing by `hw/wheel_odom`, so its `twist.linear.x` needs the sign (negative
  backwards) and has to come at least every 0.5 s: wheels more than 2 cm backwards, or no wheel travel, and the ring
  has no body. Backwards slower than 5 mm/s does not add up, so the speed may swing around zero while the wheels
  stand (the Tron does); a driver reports no speed it does not measure. Two steps of 5 cm backwards in a row of
  `localization/pose` also take points back.
- Driver parameters (ports, baud rates, limits) in the file of the driver; values measured on one mower in
  `<data_dir>/<platform>.yaml` on that mower, which the start reads last, for the driver and for every node. No value
  in two files.
- The clock: date, weekday and sunset count only while `time/clock` is ok (`mowbite_clock`): the kernel synchronized
  (NTP), or synchronized once since the start and no jump since. `rtc` under `mowbite_clock` (default off) lets the
  real-time clock alone count for 30 days after the last synchronization (`<data_dir>/clock.json`), when the kernel
  set the clock from it at boot (`/sys/class/rtc/rtc0/hctosys`). Turn it on only after measuring on the mower: the
  kernel writes the clock back to it, its drift over at least 3 days without network, 30 days times that drift under
  5 min. A board without a real-time clock that keeps the time of its last shutdown gives the same signs and keeps
  `rtc` off.
- The schedule: plans start the mower only where the platform file allows it (`allowed` under `mowbite_schedule`,
  default off; `schedule.get` then says so and `enabled` 1 is refused). Turn it on only after the way home, pushes,
  the incident log, the stop book and the safety measurement are done on this mower. A plan starts only from the
  station, with a safe clock, after 10 s of sound and light.
- The datum of the garden: one place on the mower, `MOWBITE_DATUM_LAT`, `_LONG`, `_HEIGHT` (Tron `/etc/mowbite/os.conf`,
  NX60i the environment of the compose file), handed to the GNSS node as launch arguments `datum_*` after every
  parameter file. Without a datum no pose is published.

## 4. Platform package

One package per mower, `platform_<name>`, with four files and no code to build. Copy `platform_nx60i` (a driver that
brings IMU and GNSS along) or `platform_tron` (nodes of its own in the container, cameras in their own processes):

- `platform.yaml` and `geometry.yaml`, the values and dimensions of section 3.
- `CMakeLists.txt` installs the three files and enters the name of the platform in the ament index
  (`ament_index_register_resource(mowbite_platform PACKAGE_NAME <name> CONTENT ${PROJECT_NAME})`); `package.xml` names
  the driver and what else the platform starts as `exec_depend`.
- `platform.py`, the part of the start only this mower has:

| Name | What |
|---|---|
| `NODES` | its other nodes, as the parameter files name them; the start checks their values and those of `DRIVER` |
| `DRIVER` | the node that holds the motors, also the label of its exit note |
| `DRIVER_RESTARTS` | how often the driver starts again after it ended (3 on both mowers, each time latched) |
| `DATA_DIR` | optional: `data_dir` when the start names none |
| `OPENMOWER_IMPORT` | optional, `True` for a mower that may have run OpenMower: its map and values come over on the first start (section 5) |
| `arguments()` | its launch arguments (ports, datum, switches), as `DeclareLaunchArgument` |
| `parts(start)` | `start.Parts(driver=, before=, after=, processes=)` |

`parts` gets `start` with `arg(name)` (a launch argument), `files` (the parameter files in their order), `setup_file`
(what the setup program wrote for this mower in `data_dir`, empty without it) and `device` (the file of this mower
kept by hand, empty without one; a driver with files of its own gets both after them, in this order), `data_dir`,
`data_file(name)`, `journal`, `logger`, `hold_file` (the hold of an update in `data_dir`, which the driver reads
itself), `data_store` (the names of the other files in `data_dir`), `parameters(node)` (what the files give a node)
and, for processes of its own,
`own_process(make, name, **limits)` and `camera_service(**limits)`. It answers with
`driver(restart)`, which builds the driver for its start number (0 the first; every later one has to come up latched,
`start_latched` of `tron_mcu` and `xcore_driver`), the components `before` and `after` those of the stack in the
container, and `processes`, its other actions. Read every argument before `driver` is built: a restart comes when the
arguments may be out of scope.

Optional `setup/setup.yaml` with `setup/images/`: the steps of the setup program the app walks the user through
(`setup.*`, node `mowbite_setup`). Without the file the methods are not offered. `catalog` counts up with every change
of a step or an image, `restart` (`self` or `host`) is needed with a step `inputs_assign` or `blade` and names how
the OS restarts for it, `steps` come in the order the app shows them:

| Field | What |
|---|---|
| `id`, `stage`, `check` | its name, one of `x-setup.stages` and one of `x-setup.checks` of the schema |
| `params` | the params of the check, within the bounds the schema names; without an entry the default there |
| `parts` | of `input`: `id`, optional `mode` (press, short), `role` and `texts`; of `inputs_assign`: `id`, `role`, `delay_ms`, optional `texts`; of `imu`: the moves in their order; of `stop_moving`: `id`, `role`, optional `texts`; none of `wheels`, `ground` and `blade` (fixed) |
| `value_texts` | of `choice`: `de` and `en` with `name` for every value |
| `confirm` | points the user ticks before the step begins (`x-setup.points`, or own ones with texts) |
| `needs` | steps ahead of it that have to be ok first |
| `skippable` | false for a step the user must not skip |
| `texts`, `note` | `de` and `en` with `name`, `help`, `fix`; needed where the check brings none; `note` adds to help |
| `image`, `accent` | files of `setup/images/`: the drawing and the part to look at, drawn over it in another colour |
| `symbol`, `mark` | a sign for a step without image, ids of `robot_shape` to mark |
| `open` | the screen the app opens; without an entry the one of the condition or check in the schema |

A step is one of: `choice` and `number` (the user answers), `confirm`, `board` (the driver reports, firmware, link,
inputs at rest, motor controllers, battery, gravity), `input` (a STOP button, the lift or the bumper by hand: every
part has to bring its emergency within `within`, a part of mode `short` must not), `imu` (tilting and turning by hand
with the right sign), `position` (the fix holds), `geometry` (the dimensions confirmed) and `done_when` (done as soon
as the OS sees corrections, a station, an area, a docking test or a trial run). None of them moves the mower. Take
`within` from what the board does: the NX60i stops 2.5 s after a lift sensor, so its list waits 3.5 s and counts 1.5 s
as short. An emergency that is on for more than 0.3 s before its input shows in `hw/status` counts as brought by
something else, and the part waits for its release: a driver reports the input with or before the emergency it brings.
A part of mode press is done once its input is off again.

The program releases the emergency such a part brought by itself, so the user does not have to after every action. It
does so only while all of this holds: the input switched and its emergency followed within `within`; since then
`hw/status` never showed a bit other than this emergency and LATCH; the driver counts (`latch_count` is not 0, the
release carries the count of the last `hw/status`); the mower is in IDLE. A try goes to `safety/release_emergency` at
the earliest 1.5 s after the input switched and then every 1.5 s, until 10 s after the step; only `hw/status` tells the
outcome. Where the input is not held it waits until the input is free. Where the board holds it (`held` in
`hw/inputs`, stop and lift of the Tron) the try itself is the probe: the board keeps a button that is still down.
Without one of the conditions the emergency stays for `emergency.reset`, and the step shows it as blocked. A driver
that counts and holds an input has to say so in `hw/inputs`, else the part waits for an input that never comes free.

With `hw/inputs` the step checks line by line: a part has to switch a line no part of its role switched before, as long
as the role has one left, and at the end every line of the roles of the step has switched once, else it fails with
`LINE_SILENT` and the lines. Two buttons on two lines need two parts; a pair of contacts on one part shows a dead one.

`inputs_assign` is for a driver whose table of inputs is a setting, on a mower that may be wired another way than the
table of the platform. `params.node` names the node of the driver, each part is one sensor with its `role` (stop,
lift, bumper_left, bumper_right) and the `delay_ms` its line gets. The step writes `inputs.learn` for that node into
`<data_dir>/<name>.setup.yaml` and restarts the OS: with `restart: self` the node ends the container in order and the
launch ends with it, for whatever runs the launch to start it again (compose on the NX60i); with `restart: host` it
asks mowbite-host (`host.restart_os`). It restarts only with the mower standing in IDLE (not driven by hand, not
calibrating), not held by an update and with no emergency but the one of learning: a restart never takes an emergency
away. With the driver learning the step looks at the lines untouched for `quiet`,
then takes one line per part, each confirmed by the user, writes the table in place of `inputs.learn`, restarts again
and holds `hw/inputs` against the table; a value the file kept by hand overrules fails it with `NOT_APPLIED` and the
key. The checks `input` are open again after it, and once it is ok the user can make the delay of a line shorter than
`delay_ms` (`delay_<part>` in `inputs[]` of the step), never longer. Make the step skippable where the table of the
platform fits a mower wired as standard. A driver that holds its board while it learns leaves it held: the xCore
comes back with HIGH_LEVEL after the last restart and waits for `emergency.reset`, which the note of the step should
tell.

A driver that learns takes these parameters under its node (names in `mowbite_api/setup.h`): `inputs.learn`, and the
table as lists with one entry per line, `inputs.gpio_lines`, `inputs.gpio_active_high`, `inputs.gpio_reasons` (stop,
lift, collision), `inputs.gpio_delays_ms` and `inputs.gpio_roles`. Started with `inputs.learn` it reports its
candidate lines in `hw/inputs` with `learning` true, `assigned` false and `active` as the level of the line, and no
line at all while it does not know the levels (the step waits, and ends with `NO_DATA` when they are lost); started
with the table, every line with `assigned`, its role, level and delay. `test_nx60i_inputs.py` runs the step against
the real start, both restarts included.

An image is an SVG of one colour without text, raster images, scripts, styles or references, at most 65536 bytes;
image and accent share one viewBox. The start checks list and images by the rules of the node (`mowbite_setup_check`)
and halts with `PLATFORM_CONFIG` and the first problem. `CMakeLists.txt` installs the folder
(`install(DIRECTORY setup DESTINATION share/${PROJECT_NAME})`). `test_setup.py` does every step of the list of each
platform by hand against the node (`test_setup_<name>`, with the case of the driver contract), but for a step
`inputs_assign`, which needs the driver and the restarts, and the steps that move the wheels.

The steps of stage `wheels` turn the wheels of the mower jacked up (kind `drive`): `wheels` (each wheel forwards,
backwards, both turns, then one wheel alone and the question which one turns), `stop_moving` (a STOP button or a side
of the bumper while the wheels turn), `no_command` (the commands stop, the watchdogs have to), `silence` (rounds of
`hw/silence`, the board has to stop by itself) and `driver_end` (`hw/end`, the driver comes back latched). Each
confirms `raised` and `hand_at_stop` and comes after the steps `input` of stop, lift and bumper, which have to be ok,
not skipped; the start of the list refuses it otherwise. While it runs the program sends `setup/motion` (the logic
holds the sub state SETUP, mowbite_safety caps the commands to 0.1 m/s and 0.4 rad/s) and drives on `setup/cmd_vel`;
the app has to call `setup.keep` at least every second. A stop or end of the driver has to bring the emergencies of
`emergency` of the step and LATCH only, the program then releases them itself as for an input, waits `hold`, and a
new command has to turn the wheels again. When a part waits for an action of the user that does not come in time
(`input`, `imu`, `stop_moving`), the step stands its wheels and asks `pressed`: yes fails it with `NOT_SEEN` (or
`NO_EMERGENCY` for `tilt_stop`), which closes the lock of section 6; no or no answer within 30 s leaves it open with
`NOT_DONE`. `stop_max` of `silence` comes from the board: the Tron stops 0.15 to 0.28 s
after its last setpoint, the xCore 1.0 s after it last heard of the host (so 1.5 s there); `emergency` is what the
driver shows after a silence (the Tron `HIGH_LEVEL`, the xCore `HIGH_LEVEL` and `TIMEOUT_HIGH_LEVEL`). `driver_end`
is offered only while the start has at least two restarts of the driver left (`DRIVER_RESTARTS`, `driver_exit.json`).
A mower that reports lift when jacked up cannot do these steps: its note has to tell how to put it up.

The step `ground` (stage `ground`, kind `drive`, `motion.raised` false) drives on the ground, its parts in one run:
`straight` goes about `distance` m and the program falls silent, the mower has to stand within `quiet_max` s and
`quiet_way` m, and the way by odometry and by `gps/absolute_pose` must not differ by more than the share `ratio`;
`turn` goes `angle` deg to the left and back, the IMU has to turn with the sign of the command and as far as the
odometry within `ratio`; `stop` drives back towards the start until the user presses STOP (at most as far as straight
went) and `bumper` at `bumper_speed` against a soft obstacle the user holds, in `bumper_direction` (where the bumper
is: the Tron forwards, the NX60i backwards), each to stand within `stop_max` s and `stop_way` m, then released as
`stop_moving` does, without a new command. A mower still moving 3 s after a stop fails with `NO_STANDSTILL`. It
confirms `area_clear` (3 m clear, the user next to it at a STOP button), needs an RTK fix (`NO_RTK`) and, while the
lock of section 6 is closed, every step of stage `wheels` passed (`SAFETY_AIR`). mowbite_safety holds it to 0.3 m/s
and 0.4 rad/s without pose and fence, like the remote control; every step that moves the wheels stops after one move
of 10 s and moves again only after a zero. Take the bounds of standing from a measurement of the platform on the
ground (the NX60i from M8); where there is none, the values of another mower and the first run as the measurement.

The step `blade` (stage `blade`, kind `blade`) turns the blade with the disc taken off (confirm `blade_removed` and
`hand_at_stop`), the wheels get no command. Its first start on a driver without `blade_allowed` writes `allow_blade`
for `node` into the file of the setup program and restarts the OS; the user starts it again after that, never the
program. A file kept by hand that sets `allow_blade: false` keeps it locked (`BLADE_NOT_ALLOWED`). Its parts in one
run: `on_off` (on at `percent` and `height` until `rpm` within `spin_up` s, then off: it stands below `still_rpm`
within `off_max` s), `lease` (one request on `lease` s that is not renewed: within the lease and `lease_extra` s),
`silent` (hw/silence with the blade: within `silent_max` s, then released as `silence`), and `stop`, `lift`, `bumper`
(the user triggers the input while the blade turns: within `stop_max` s, released by the program). After every stand
nothing turns for `hold` s. The blade turns only through the logic, in SETUP, without emergency and off the station,
and goes off when the step ends. Take the bounds from a measurement of the platform (the Tron: measured on the
mower, all within 1.0 s); where there is none, those of the board's firmware and the first run as the measurement.

The start is the same for every mower: `ros2 launch mowbite_bringup mowbite.launch.py platform:=<name>`, by default the
platform `MOWBITE_PLATFORM` names, a value of the device (the compose file of the mower sets it, the Tron in
`/usr/lib/mowbite/os.defaults`); an update does not change it. `stack.py` prepares `data_dir`, lays the parameter files
on top of each other (`stack.yaml`, `platform.yaml`, `extra_config` for a simulation or a test, then
`<data_dir>/<name>.setup.yaml`, which the setup program writes, and `<data_dir>/<name>.yaml`, kept by hand, which has
the last word), checks them for the nodes the platform can start, runs the driver in a process of its own
(restarted `DRIVER_RESTARTS` times, each end in `<data_dir>/driver_exit.json`) and beside it the container with all
nodes above the contract, the planner and the net service. A name no package entered halts with `PLATFORM_CONFIG`.
`tron.launch.py` and `nx60i.launch.py` only name their platform, for start commands that name these files.

Without a name (no `platform:=`, no `MOWBITE_PLATFORM`) the start takes the platform of `<data_dir>/platform.json`
(`data_dir:=` or `MOWBITE_DATA_DIR`). Without that file it starts the stage of the choice: only the bridge and the
setup program, no driver, no port opened, `robot_state` SETUP with sub state PLATFORM. The app lists the mowers of the
image (`setup.platforms`, texts from `ABOUT` of each `platform.py`, the drawing from above `setup/images/mower.svg`)
and chooses one (`setup.platform.set`); the program writes `platform.json`, the launch ends and whatever runs it
starts it again as that mower. Every step list begins with
the step `platform` (check `platform`), which shows the mower and lets the app choose another while the mower is idle:
every result goes back and the lock closes with a reset until the summary passed again. With `MOWBITE_PLATFORM` the
choice is fixed (`PLATFORM_FIXED`). Give a new platform an `ABOUT` with `name`, `text` and `board` (what the driver
expects at which port) in de and en.

Every node in the container runs in its own thread (`component_container_isolated`), so a node that waits stalls only
itself. A node of your own (lights, sound, camera) follows the same rule as the others: nothing that blocks in a
callback of the driving chain (pose, safety, follower, logic), no `fsync` or network there; files that must last go
through a thread of their own like `mowbite_hold::FileWriter`.

Every node in the container calls `mowbite_watch::Attach(*this)` once, at the end of its constructor (before it
starts a thread of its own); a test in `mowbite_bringup` checks it. Its executor thread then signs every 0.5 s; silent
for 10 s (S2: 2 s), or not loaded 60 s after the start (`stack.py` passes the node list), and the watch ends the
container: the way of a crash, with the node in `container_exit.json` and in UNEXPECTED_RESTART (NODE_STALLED). After
three crashes in a row the container stays down (`down` true in the note), a run of 10 min counts from 0 again. Keep
callbacks short, so the limits can come down after measuring; a sign later than a quarter of the limit is logged.
Limits are never parameters; `MOWBITE_WATCH_FACTOR` only lengthens them for sanitizer runs. A driver in its own
process stops the mower by its own timeouts and has a watch of its own: `Attach(*this, 2.0)` for its executor and a
`Watch` with the same limit for its I/O thread, signed every round (a read with a timeout, so a quiet port signs too).
A silent one ends the process (code 86) and the launch starts a latched driver, as after a crash (`tron_mcu`;
`xcore_driver` also watches its output thread, and its I/O thread without ROS signs through hooks in its config).

## 5. Onto the mower

- Docker Compose like the NX60i (`docker/mower/nx60i.compose.yaml`), or a native bundle on the mower's own rootfs like
  the Tron (`platform/tron/`).
- From OpenMower (a platform with `OPENMOWER_IMPORT`, launch argument `import_dir`, `MOWBITE_IMPORT_DIR` in the
  compose file): on the first start, while the data dir holds no data yet, `migrate.py` of `mowbite_bringup` reads
  `<import_dir>/ros/map.json` and `<import_dir>/params/mower_params.yaml` (mounted read only) before any node starts.
  The map comes over as it is if `mowbite_map_check` passes it, with its datum in `map_datum.json`; antenna, track and
  ticks go into `geometry.json` unconfirmed (the setup step `dimensions` asks for them), the NTRIP access into
  `ntrip.yaml` (0600). Nothing of the safety measurement, the blade or the charging limits comes over, and nothing of
  another broker. `migrate.json` in the data dir names what came over and what stayed behind with its reason, never a
  value; an import cut off runs again on the next start, a later start never imports. The way back:
  `docker exec <container> mowbite-migrate export /export` while the mower rests (exit 0 written, 2 not at rest,
  1 error) writes `map.json` in OpenMower's fields and `report.json`, whose `lost` lists what OpenMower cannot hold
  (other area types, inactive areas, properties, dimensions or NTRIP changed since the import).
- Map and settings live in the data dir, never in a container volume. The journal has a folder of its own outside it
  (`journal_dir`, a bind mount like the data dir); its incidents lie in `journal_dir/incidents`.
- The data dir has to stay writable for the OS: while `os-session.json` or `restart-hold.json` cannot be written
  there, nothing starts from IDLE (`STORAGE_NOT_WRITABLE`, event `STORAGE`), a job already running goes on. Put it
  on storage that lasts across power cuts and is not mounted read only on errors without a way back.
- Every log, recording and cache gets a limit in days and MB. In the platform file: `journal_max_mb`,
  `journal_max_days` for the journal and `incidents_max_mb`, `incidents_max_days`, `incidents_max_days_safety` for its
  incidents; an incident keeps a run from its start while the run takes less than a quarter
  of `incidents_max_mb`.
- The service of the OS restarts it after every end, also one with exit code 0 (the launch ends that way when the
  setup program asks for a restart of the OS), and starts only with the data partition mounted; on the Tron
  `Restart=always` and `RequiresMountsFor=/userdata` in `mowbite-os.service`.
- `mowbite-host` (`host/`, unit `mowbite-host.service` on the Tron) runs beside the OS as root, not in its bundle:
  it installs signed bundles (update.*), answers `host.restart_os` and keeps `mowbite/host/status` retained, also
  while the OS is off. It needs the broker of the OS, `python3-paho-mqtt` (1.x or 2.x), `ssh-keygen`, systemd for
  the OS service and the files of the data dir; a compose stack needs a carrier of its own (not built yet). On the
  rootfs: the seed of the library under `/usr/lib/mowbite-host` (`host/mowbite_host` and its `bundle.json`,
  `make-image.sh` writes both from the commit checked out), `mowbite_boot` under `/usr/lib/mowbite-boot` (from
  `host/boot/`, never in a bundle), the thin entry points `mowbite-host` and `mowbite-update`, the unit enabled, the
  rootfs version in `/usr/lib/mowbite/base` (raise it when a bundle needs something new of the rootfs; bundles name
  the version they need; `base-packages.txt`, written by `build-base.sh`, lists the packages of the base, and
  `make-bundle.sh` refuses a bundle whose runtime packages are not in it) and `/etc/mowbite/allowed_signers` with two keys, the main one and a reserve kept apart, for
  the namespaces `mowbite-os`, `mowbite-app`, `mowbite-planner`, `mowbite-host` and `mowbite-index`.
  `host/test/test_e2e.py` runs the
  whole way with the carrier `process` (`MOWBITE_HOST_CARRIER=process` in the environment files).
- Setup WLAN again (optional): a mower with buttons and the setup access point of the first start lets the owner open
  it after the trial period by a gesture, when the WLAN of the mower is gone (a new router). The host counts short
  presses of one key on its input device (5 within 5 s, `host/mowbite_host/gesture.py`), only at rest (an OS that runs
  idles; without a running OS always) and while the mower has no WLAN connection, writes the deadline (30 min, unix s)
  to `/run/mowbite/setup-ap.until`, restarts the unit of the access point and plays a rising tone, or a low one when
  it refuses; the gesture while it is open only moves the deadline. The unit raises the access point until the
  deadline in that file, also a later one, and removes the file when it ends (the Tron: `mowbite-setup-ap.service`,
  `/usr/lib/mowbite/setup-ap`). `setup_wifi` of `mowbite/host/status` holds `{until}` while it is open, else null; it
  closes earlier once the WLAN of the mower is connected (`host.wifi_set` or a known network back). The platform
  names all of it in `/usr/lib/mowbite/wifi-gesture` (`device`: name in `/proc/bus/input/devices`, `code`: key code
  of a short press, `tone_device`: ALSA device for `aplay`, `unit`, `profile`: the NetworkManager profile of the
  access point); without the file there is no gesture. Pick a key whose long press does not
  start anything and a code a long press never sends (the Tron: On/OK, code 25; held 3 s it sends 38 and cuts the
  power).
- The host keeps three components, each in a folder of its own with `current`, `previous` and `pending`: the OS
  (`/opt/mowbite`), the planner (`/opt/mowbite-planner`) and the app (`/opt/mowbite-app`). The OS brings a planner and
  an app along and names them in its `bundle.json` (`bundled`, `planner_interface`, `planner_files_version`, `api`,
  `api_break`, `min_app`), beside `platforms`, the mowers it is built for: the host refuses a bundle that names
  platforms without the one of this mower (`PLATFORM_MISMATCH`), which is `MOWBITE_PLATFORM` of the device, else the
  one of `<data_dir>/platform.json`, else the only platform of the running OS. An app goes with the OS when its `app.json` names `api.break` equal to
  `api_break` and an
  `api.min` not above `api` (`api.max` is shown only); an app without `api.break`, and one beside an OS without
  `api_break` (api 30 and older), needs `api` within `api.min` to `api.max` (`host/mowbite_host/compat.py`, the same
  for `app_export.py`). `kApiBreak` in `src/mowbite_api/include/mowbite_api/catalog.h` goes up only for a break, as
  the description of the schema names it (a method, field, event, reason or topic gone or renamed, a type or meaning
  changed, a parameter newly required, fewer values taken in a request, an event a client waits for no longer sent);
  all else only adds and raises `kApi`.
  An own planner or app is used only while the link `active` in its folder names it; the host alone sets it, when
  the own one goes with the installed OS and is newer than the bundled one. Whatever starts the planner or
  serves the app on your mower reads `active` first and else `<os>/planner` or `<os>/app`, and decides nothing itself.
  A planner update restarts the OS through the gate of the OS; an app update only changes files and needs no gate.
  Bundles of the planner and the app hold no machine code. On the Tron `os-setup` puts the planner `active` names
  before the one of the OS (which keeps the interfaces of both services), and tmpfiles makes the two folders.
  `mowbite-update-recover.service` runs at boot while any of the three folders has a `pending`.
- The host updates itself as a fourth component under `/opt/mowbite-host` without a gate: the switch writes `pending`
  and `current` and ends the process, the service manager starts it again (`Restart=always`). `mowbite_boot` tries
  the new one for 180 s; it confirms itself after 60 s of its main loop (and once connected, when the host before it
  was), else the next start goes back to the one before or to the seed and leaves a note for the result. After three
  starts in a boot that did not run 60 s the seed runs. Keep `Restart=always` and the watchdog on your service.
- A tool of your rootfs that stops the OS itself (the safety measurement, on the Tron `mowbite-measure`) has to keep
  the host off, which starts a stopped OS on `host.restart_os` and with every switch: stop `mowbite-host`, take
  `/run/mowbite-host.lock` (flock, held until the tool is done) and go on only when `mowbite-update unfinished`
  answers 0. While the lock is held, a host started again waits without the broker and `mowbite-update` refuses to
  install, roll back and recover. A bundle of the host does not renew such a tool: the path of the lock and that
  command stay as they are.
- A vendor kernel may crash on a plain read of one of its `/proc` entries, and a crash restarts the mower. On the
  Tron reading `/proc/bluetooth/sleep/btwrite` oopses the kernel: `mowbite-bluesleep-lock.service` (before
  `sysinit.target`) takes every mode bit off that folder and lays an empty read-only tmpfs over it. Tools and scripts
  of the rootfs read only single entries of `/proc` (a process's `cmdline`, `fd`, `stat`), never walk, grep or pack
  it; `check-image.sh` refuses a file of `files/` that does.
- The app comes from `mowbite-web` (`web/mowbite_web.py`, standard library only): port 80 from
  `mowbite-web.socket`, the service as its own user `mowbite-web` in no group, no root, no capability, nothing to
  write. GET and HEAD only, `/cgi-bin/*` is always 404 (the app reads that as "no container"); only `/app.json`,
  `/version.json` and `/cgi-bin/*` carry `Access-Control-Allow-Origin: *`, for the Android app and the page of another
  mower. Hashed parts under
  `/_next/static/` are also taken from the older installations so an open page keeps working after an update. The
  app reaches the broker over WebSocket on 9001 (`docker/mower/mosquitto.conf`). A compose stack can serve the same
  export with any static server that follows these rules. `web/test/test_web.py` runs without a mower.
- Bundles, all in `platform/tron/bundle/`: `make-bundle.sh` for the OS (serial = commit time, base, api, api_break,
  `platforms` from `PLATFORMS`, each with its package in the image, the planner's `contract.json` and what it brings
  along from the image), `make-planner-bundle.sh <repo> <commit>
  <out>` (only `mowbite_planner` and `mowbite_planner_os`, built in a throwaway container, `contract.json` required),
  `make-app-bundle.sh <zip> <out>` (the
  web export of the app release, `app.json` required), `make-host-bundle.sh <commit> <out>` (`host/mowbite_host` of
  that commit). `host/bundle/app_export.py` unpacks and checks the export for
  them and for `docker/os.Dockerfile`, which pins the app it brings along (`APP_VERSION`, `APP_SHA256`) and loads
  the zip from the release of the app, or takes it from the build context `app` when that has one;
  `--build-arg APP_VERSION=` builds an image without the app. Each is
  signed with `sign-bundle.sh` under the namespace of its component (refuses a serial not newer than the last one
  signed for it, shows its fields and the result of the test gate), then into the inbox
  `/userdata/mowbite-update/inbox` and `update.check`, or published with `publish.py` as releases of
  `mkaaaaaay/mowbite-releases` with a signed `index.json` (channel preview, stable on the owner's word).
- The host asks that index every 6 h and on `update.check` (`host/mowbite_host/net.py`) and downloads the newest
  bundle per component newer than what runs into `/userdata/mowbite-update/net`; installing stays `update.start` or
  the automatic in its window. `update.set_channel` chooses stable or preview (default stable).
  `MOWBITE_UPDATE_NET=off` in the environment files keeps a mower off the net. A new platform needs nothing more
  than `ca-certificates` and the index key in `allowed_signers` (namespace `mowbite-index`).

## 6. Safety measurement

Before the mower drives by itself. On blocks, wheels free, blade off, someone at the stop button:

1. Before any motor turns, in every run again: press and let go each stop button once and see it in `hw/status`,
   `stop` with its emergency, then released and neither left. A button `hw/status` does not show, an unplugged one for
   instance, stops nothing: no motor step until it does; one that stays or comes back after the release is stuck or
   held, no motor step either. `tools/yf/measure.py` asks for it before the first step that can turn a motor and sends
   no wheel command and no blade on before; your programme there names its stop buttons (`STOP_BUTTONS`), the steps
   that turn nothing (`STANDSTILL_STEPS`) and whether the board keeps `stop` until the release (`STOP_LATCHES`, the
   Tron's MCU): then the release follows a few seconds after the request to let go, else once `stop` is free.
2. How long the board keeps the last setpoint without the host: `failsafe_without_host`. It may hang on the order of
   the last frames: the Tron's MCU keeps it ~3.6 s, but ends it ~0.23 s after a heartbeat that comes alone after it
   (`measure.py` T9), so its driver sends one 25 ms after each setpoint (`speed_heartbeat: later`).
3. Stop button, lift, bumper left and right: wheels and blade stop, the reason bits are right, latched until release.
   With a release at the mower: the press that latched never releases, a held press does, anything else does not.
   Lift each side and the whole mower level by hand: note the tilt lift comes at, or that it does not come; with the
   blade turning, lift has to stop it like the stop button (`measure.py` T7 on the Tron).
4. Drive in the air: forwards, backwards, turning; left and right the right way round.
5. The blade last: briefly, in the air, heights within the limits.

The measurement runs the driver alone, so nothing may start the OS meanwhile: keep `mowbite-host` off for all of it
as section 5 says (stop it, hold its lock, go on only after `mowbite-update unfinished` answers 0). At the end start
the OS, give the lock back and start the host again when it ran before, as `mowbite-measure` of the Tron does
(`platform/tron/rootfs/files/usr/sbin/mowbite-measure`).

Then set `safety_measurement_date` in the driver parameters. Without it the mower takes remote control only
(`NO_SAFETY_MEASUREMENT`).

The setup program can close that lock again, whatever the date: a safety step (`input`, `imu`, `wheels`,
`stop_moving`, `no_command`, `silence`, `driver_end`, `ground`, `blade`; `safety` in `x-setup.checks`) that fails
with a reason that closes (`closes` in `x-reasons` of SETUP_STEP: a value out of bounds, wheels or a blade that start
again, a mower that does not stand, an emergency that does not come, a wrong direction or wheel, or nothing seen after the user said
the action was done) writes
`<data_dir>/safety_lock.json` before anything else. mowbite_safety, the logic and the follower read it within 0.1 s
through `mowbite_hold::SafetyMeasured` (date and lock, the one rule), so it acts at once, without a restart, against
the date of the file kept by hand as well; mowbite_safety latches `HIGH_LEVEL` once for every new failure. Releasing
that emergency does not open the lock: only passing the same step again does (skipping or `setup.reset` do not). A
file that cannot be read keeps the lock closed. Remote control and the steps of the setup program stay free. A driver
needs nothing for it; a new reader of the date has to take `SafetyMeasured` and the file in `data_dir` as well.

Instead of the date by hand the setup program can open the lock with its step `summary` (stage `safety`, kind
`confirm`, confirm `seen_myself`, after a `silence` in the list). It starts only when every safety step of the list
passed and the lock holds no cause of one (`SAFETY_STEPS`, skipped ones named apart), every step of the stage
`dimensions` passed (`LOCKED` with `needs`; values a platform took over from another mower belong there as a step the
user confirms once they are measured, as `tron_values` of the NX60i) and `time/clock` is safe (`CLOCK_UNSAFE`). It writes today's date (UTC) and the longest stand time of `silence` as `safety_measurement_date`
and `failsafe_without_host` for `node` into the file of the setup program, closes the lock with a reset meanwhile and
restarts the OS. After the restart it holds `hw/capabilities` against what it wrote: the same, the step passes and
takes the reset away (`safety.source` `program`); else `NOT_APPLIED` (also for an empty date of the file kept by
hand, which has the last word), `RESTART_FAILED` or `NO_DATA`, the file gets its old values back and the lock stays
closed until a summary passes. The driver only has to take both parameters and show them in `hw/capabilities`. A
safety step that fails later sets `summary` open again; so does a new table of `inputs_assign` (or new delays), which
also closes a lock open with a date by a reset until the checks of the inputs and a summary passed again. A failed
step whose cause did not reach the file before the OS stopped goes into it again at the next start, and
`safety_lock.json` stays as it is through a way back of the data (`data_store.py`). `setup.reset` takes date, stand time and `allow_blade` out of
the file of the setup program and, when the program had opened the lock, closes it with a reset until the next
summary passes; a date of the file kept by hand stays as it is.

## 7. Lights (optional)

`mowbite_light` drives a WS2812 chain on spidev after the robot state, with a profile per state in the app (`light.*`);
the emergency signal is fixed. Load it as a component of the container only when the drivers run, like
`platform.py` of the Tron with `hardware:=true` (`after` of its parts).

- Parameters under `mowbite_light` in `<platform>.yaml`: `output` (`none` until checked on the mower, then `spidev`;
  `file` in the simulation), `device`, `leds` (lit), `buffer_leds` (driven), `strips` (runs `first-last` that chase
  and gradient follow, backwards when last < first, not overlapping; empty: the whole chain), `frame_rate`,
  `max_brightness`, `max_current_ma`, `ma_per_channel`, `low_battery`.
- With `output: none` or an invalid value the node offers no `light.*` method and the app hides the lights; invalid
  `strips` only fall back to the whole chain.
- The service user needs the device: on a native rootfs a udev rule like
  `platform/tron/rootfs/files/usr/lib/udev/rules.d/60-mowbite-spidev.rules`, in Docker the device passed in.
- On the mower: `output: spidev` in the file of that mower, then one `light.preview`.

## 8. Cameras (optional)

A camera of the mower, shown in the app, optionally with object detection for display only. The Tron's setup is the
example: `tron_vision` (camera AI on the left and, as a second instance, on the right side camera), `tron_stereo` (live
picture), `tron_side` (only the live picture of the right side camera, with `right_node:=side`) and `tron_rear` (dock
marker with a live picture) with their start in `platform.py` of the Tron and their parameters in its `platform.yaml`.

The platform makes the pictures, the OS takes them to the app: `mowbite_camera` keeps the cameras of `hw/cameras`,
answers `camera.list`, `camera.stream` and `camera.unlock` (all show up in `rpc.methods` with the first camera),
lets two viewers in at a time and gives each an address with a token of its own. The first list is what a camera
process has to do, the second what `mowbite_camera` needs around it.

- A process of its own, never a component of the container: a crash of a vendor library takes nothing else along.
  Start it in `platform.py` with `start.own_process` only when the drivers run and a switch in
  `/etc/mowbite/os.conf` says so. It restarts at most three times and leaves `<name>_exit.json`; give it
  `oom_score_adj` 1000, a `nice` value and CPUs apart from the driver. Pass the same name to the node as `process`.
- The same node on a second camera is a second process: the same node name in a namespace of its own, absolute topic
  names in the node, the shared values under `/**/<node>` and only what differs under `/<namespace>/<node>` (Tron:
  `/right/tron_vision` with camera id, NPU core, RTSP port and a suffix for its sensor ids).
- It reports and nothing else: sensors over `bridge/sensor_info` and `bridge/sensor_value`, detections on
  `vision/detections` (`mowbite_msgs/Detections`, boxes in pixels of the camera image), each camera latched on
  `hw/cameras` (`mowbite_msgs/Camera`: id, process, size of the shown image, protocols, and while it cannot stream
  one of the reasons of the message). It subscribes to nothing of the stack but `hw/status` when it needs the state
  of the mower (best effort, depth 1); tests check its lists of publishers and subscriptions
  (`tron_vision/test/test_vision_node.cpp`, `test_stereo_node.cpp`, `test_side_node.cpp`, `test_rear_node.cpp`).
- A thread of its own beside the ROS thread signs the watch of the process after each step (`mowbite_watch`: `Attach`
  for the node, a `Watch` for the thread); a step that hangs in a vendor library then ends the process (code 86).
- Protocol `mjpeg`: JPEG on `camera/<id>/jpeg` (`sensor_msgs/CompressedImage`, best effort, depth 1), made only while
  someone subscribes, with a rate and a size limit as platform parameters. The OS subscribes while a viewer is
  connected and serves the pictures to the app. A camera that serves only viewers opens only then and closes shortly
  after the last subscriber left (`tron_side`).
- Protocol `webrtc`, for a smooth live picture: the process is an RTSP camera on the loopback address (H.264
  constrained baseline, RTP over the RTSP connection, its address `rtsp://127.0.0.1:<port>/<id>` as `source` in
  `hw/cameras`; the OS takes no other form), and the OS pulls from it only while someone watches. The connection is
  the viewer signal: PLAY starts the camera, its end stops it, and while the camera cannot stream DESCRIBE answers
  503. The parts without ROS are in `tron_vision` (`rtsp_server`, `h264_rtp`, `h264_stream`, `video_feed`, and
  `video_out` for the parameters of a camera node); a hardware encoder is loaded at run time like the other vendor
  libraries. Up to 1280 x 720 the stream stays at level 3.1, which the OS announces to the browser; a larger picture
  needs level 4.0. A camera can offer both protocols, `webrtc` first (`tron_side` and `tron_vision`: H.264 at 1280 x
  720 for the RTSP client, JPEG for a subscriber, and only JPEG if the encoder fails). When its protocols or its size
  change, it publishes its entry again: the OS tells the app (CAMERA_CHANGED) and ends the viewers of a protocol that
  is gone. A camera with detections draws
  its boxes into the frame before the encoder takes it and runs the model on a thread of its own, so the stream does
  not wait for the NPU. It offers the same frame without boxes as well, because `camera.stream` lets each viewer
  choose (`detections`): JPEG on `camera/<id>/jpeg_plain` and the RTSP path `<source>_plain` beside `<source>` on the
  same port, each made only while someone subscribes or plays, with the same limits; the OS serves that view at
  `<id>_plain`, so no camera id ends in `_plain`. One RTSP server serves several paths, one client per path, and each
  stream has its own encoder session. A frame is encoded first without the boxes, then the boxes are drawn into the
  same frame for the other stream, so there is no copy. A camera that sends JPEG, like a USB camera, is decoded scaled in the DCT into NV12 frames
  of the process (`dma_frames`, from a dma-buf heap, lines rounded up to whole macroblocks) that the encoder reads
  (`tron_rear`: 960 x 540 from 1920 x 1080, port 8558; the dock marker still goes first).
- A stream whose driver can crash the kernel does not run next to the blade by default: the loop ends it when the
  blade may turn (`BLADE_RUNNING`; every gate that needs the blade still takes `BladeMayTurn` of
  `mowbite_msgs/blade.hpp`: commanded on or any rpm reported, also while it runs down) or the state of the mower is unknown (`DRIVER_MISSING`: no `hw/status` yet or the
  last one older than 1 s by the time it came, not by its stamp), and tests the camera once per boot while the mower
  stands with the blade off (`SELF_TEST` until then). `stream_while_mowing: true` in the file of the platform lets
  the stream run next to the blade and without `hw/status`; the self-test, the pause when hot and the lock after a
  kernel crash stay. Set it only after measuring on the mower how long the blade runs without the host, or with that
  risk accepted and written next to the value (the Tron: its `platform.yaml`).
  Start such a process only with `hardware:=true`: with another host at the board `blade_on` says nothing.
- Vendor runtimes and models are loaded at run time from a directory on the mower and never go into the repo, a bundle
  or an image. Check the model when loading it (inputs, outputs, classes against the class list), else report it and
  do not stream.
- Before opening a camera: no other process holds it. A camera whose driver can crash the kernel gets a lock: a marker
  before the stream starts; found after a kernel crash, the camera stays off (`LOCKED_AFTER_CRASH`) until the lock
  file is deleted. The process that owns such a camera offers `/camera/<id>/unlock` (`std_srvs/Trigger`) for
  `camera.unlock` of the app: it deletes only the lock of that camera and answers `success` with the reason the lock
  file held (one line) as `message`, then checks whether the camera may stream and reports it again on
  `hw/cameras`; without a lock it answers `success: false` with `message` "not locked", on an error with a few words.
  The OS calls it only at rest (`IDLE` without a sub state, blade still) and waits 5 s for the answer.
- Without memory cgroups the process holds its own limits: resident memory, inferences per second, a pause when the
  SoC is hot, a deadline for frames, a few stream attempts per start. Measure them on the mower before it runs for
  long.
- The service user needs the devices: on a native rootfs groups like `video` and `render`
  (`platform/tron/rootfs/files/usr/lib/sysusers.d/mowbite.conf`) and udev rules for devices that are root only, such
  as a hardware encoder and the dma-buf heaps (`60-mowbite-stereo.rules`); in Docker the devices passed in.
- A camera that measures for the logic, like the dock marker (section 1), has its own process too and opens the
  camera only while asked. Its calibration (lens, mount, marker) has no defaults in the code: what is missing makes it
  report `NOT_SET_UP` and never measure. The mount comes from a reference run: the mower pushed straight out of the
  dock in steps against a tape; distance and side of the marker are checked against it, a wrong roll or pitch shows as
  a corner fit error well above the corner noise and as a marker height that changes with distance. A 4 cm marker
  gives its yaw only to a few degrees beyond 0.2 m, so a dock approach steers by its side offset. A USB camera gets a fixed name by a udev rule on its USB id
  (`60-mowbite-rear.rules`), never `/dev/videoN`. Pixel values for measurements go on a topic outside `hw/*`
  (`tron_rear/marker_pixels`). It may show a live picture from the same process (`tron_rear`), but the measurement
  comes first and the picture must never delay it: a request takes the camera from a viewer, a picture is made after
  the report of its frame, and a failure of the viewer's stream never reaches the reports for the logic.
- Settings of a camera that need register writes (exposure control of the Tron's bridge chip, `tron-stereo-ae`) come
  from the setup at boot, read back, before it reports the camera ready; the node never writes to i2c. A setting that
  follows the light (the gain level of the Tron) is a wish of the node in a one-line file with a level from a fixed
  list and a time; a root helper of the platform reads it as foreign input, falls back to the base level at anything
  unknown or stale, limits its rate and writes the registers under a lock that the node holds around stream start
  and stop.

Around `mowbite_camera`:

- Start it beside the camera processes that report on `hw/cameras` and only with them; a camera that only measures
  for the logic and shows no picture does not bring it along. `start.camera_service(...)`
  takes their limits (`platform.py` of the Tron). It is a process of its own too and leaves `camera_exit.json`.
- The app has to reach `mjpeg_port` 8890 (HTTP, protocol `mjpeg`), `webrtc_port` 8889 (HTTP, the WHEP exchange of
  protocol `webrtc`) and `ice_port` 8189 over UDP (its pictures). `media_api_port` 9997 and the RTSP sources stay on
  the loopback address. Other ports go under `mowbite_camera` in the file of the platform.
- `public_host` is the host in the addresses of `camera.stream`; empty, it is the IPv4 address of the default route
  at that moment. Set it in the file of the mower when the app reaches the mower under another address. The
  addresses are HTTP without TLS and hold for one connection each.
- Protocol `webrtc` runs through a stream server (MediaMTX) that `mowbite_camera` starts as a child and watches. The
  image of `docker/os.Dockerfile` brings the release pinned there, with its licence text, as `<install>/mediamtx/`;
  elsewhere `media_server` names the program. Without it `mjpeg` still works, `camera.stream` refuses `webrtc`
  and a camera with no other protocol is listed as `NOT_SET_UP`. Its files lie in a folder `mowbite_camera` in
  `runtime_dir`; by default that is `$RUNTIME_DIRECTORY` (systemd `RuntimeDirectory=`), else `/tmp`. It offers the
  addresses of the interfaces it sees, so in Docker it needs the host network.
- `position` in `hw/cameras` (`front`, `rear`, `left`, `right`; empty when unknown or not fixed) says where the camera
  looks, not where it sits; the app's switch between cameras goes by it.

## 9. Obstacle sensors (optional)

A sensor that sees obstacles on the ground ahead: a camera with object detection and depth on the Tron, later
ultrasound or LiDAR elsewhere. The platform says where things are; what they mean (tracks, confirmation, zones, the
sight of the sensors, the list and the photos for the app) is in `mowbite_obstacles` of the OS, the same for every
sensor. `obstacle_mode` show lists them without any effect on driving and notes where the mower would have stopped;
stop holds the mower (see the last points). The sensor stays an extra: the bumper and the lift stay the emergency stop.

- Name each sensor in `obstacle_sources` of `hw/capabilities` (a camera by its id of `hw/cameras`). Without a name the
  OS offers no `vision.obstacles.*` and keeps no list.
- Each sensor sends `hw/obstacles` (`mowbite_msgs/Obstacles`) at a fixed rate of at least 4 Hz, also when it finds
  nothing: `header.stamp` the time the frame was taken by the host clock (the OS takes the pose of that moment),
  `sight` false with a reason while it cannot see (dark, blinded, covered, too hot, no frames, not set up), and the
  sector it sees on the ground from its mount and lens. A place outside the sector never counts as seen free, so the
  sector has to be honest: from where the lowest ray of the image meets the ground, not from the bumper.
- Each obstacle in `base_link` on the ground: the middle of its footprint, its radius, its height (NaN unknown) and
  the covariance of x and y, which grows along the view with the distance for stereo depth. `kind` is PERSON, ANIMAL,
  OBJECT or UNKNOWN (geometry only); map the classes of the model to these in the file of the platform, an unknown
  class to OBJECT, and leave out what is no obstacle (the station, a drain cover, leaves). `cut_bottom` when the box
  touches the lower edge of the image, `position_from` and `agrees` when the position comes from the ground plane,
  from depth or both.
- `mowbite_obstacles::ground` (library without ROS) puts a box on the ground: rays of its lower edge, corners and upper
  edge from your lens model, the mount in six degrees of freedom, the ground at `-wheel_radius`; with stereo depth it
  fuses both and says whether they agree; `ViewOf` gives the sector. Measure the mount: one degree of pitch moves a
  point 2 m ahead by about 0.3 m.
- Photos only while `obstacles/photos_wanted` (latched) is true: then at most one `hw/obstacle_images` a second, the
  whole image of a frame with finds as JPEG under 100 KB, with the stamp and source of that frame; the boxes are in
  its pixels. While it is false, no image leaves the process of the sensor.
- The OS keeps the list and the photos in `<data_dir>/obstacles` (200 entries, 20 MB, 14 days by default:
  `list_entries`, `list_max_mb`, `list_days` of `mowbite_obstacles`); nothing of it goes into incidents, logs or MQTT
  topics. The zones and the confirmation are parameters of `mowbite_obstacles` as well; leave them until the sensor
  is measured on the mower.
- `obstacle_mode_max` of the driver (in `hw/capabilities`, 1 by default) is the highest mode the sensor is proven for
  on this platform; a higher setting acts as it. Raise it to 2 only after reach and delay per class were measured on
  the mower (person, animal, object, also lying and crouching) and the stops in the grass of show were counted; to 3
  only after the stops of stop were watched on the mower and the braking distance and the clearance around a thing
  were measured.
- With stop the logic pauses MOWING with the reason OBSTACLE (blade off) and holds a leg of docking towards the
  station while a confirmed thing lies in the stop zone, and goes on by itself once the zone is seen free for
  `obstacle_free_wait` (10 s; `obstacle_near_wait` 30 s after a thing within `obstacle_near_ahead` 0.5 m of the nose).
  A person or animal that stopped it holds it while it stays in the way within the slow zone (`slow_ahead`, 2 m).
  A place out of view or without sight is never free; then the user can continue past what no sensor sees. Without
  sight nothing stops. Paths of these legs carry `gate`: the follower stands without a fresh `logic/drive_gate`
  (`gate_timeout` 0.5 s) and goes at most `speed_slow` while a person or animal is in the slow zone.
- With avoid an object alone (no person, animal or thing of no kind) that held the mower for `obstacle_avoid_wait`
  (3 s) without moving more than `obstacle_avoid_still` becomes a circle: its radius, `obstacle_avoid_sigma` times its
  spread and `obstacle_avoid_clearance` (0.30 m, at least 0.1) between it and the body on the way around, with
  `obstacle_avoid_pass_clearance` (0.10 m, the spread within) for lanes that pass it straight. The planner's `~/cut_plan` (interface 3)
  cuts the rest of the area's plan where the body would touch an inner circle; the mower drives to the next piece
  within the area around the outer one with the blade on at `obstacle_avoid_speed` (0.3 m/s). The outer circles are
  temporary obstacles of every transit while mowing, the inner ones obstacles of the follower's geofence; they go
  once their place is seen free, after `obstacle_avoid_keep` (30 min) or when mowing ends. Without a cut or a way
  around the mower waits as with stop. The list entry gets the circle, the event OBSTACLE_AVOIDED tells of it.
- The Tron as an example: `tron_stereo` (`obstacles: true`) runs its stereo camera on after the self-test, the model
  on the left eye on an NPU core of its own at 5 Hz, and places each box by the ground (`mowbite_obstacles::ground`)
  and by stereo depth (search along the epipolar line, median in the lower part of the box). Its file holds the
  classes by kind (`kind_person`, `kind_animal`, `kind_ignore`), `calibration_path` (both eyes as fisheye, made from
  boards held before the camera, also low and at the edges) and `mount` (measured with a board lying flat at known
  places on the floor); without one of them, with `kind_person` empty or with a class name the model does not know
  it reports NOT_SET_UP. A size probe (`known_heights`) only ever moves objects farther away, never a person or an
  animal. A result of a frame older than 0.5 s gives no sight. Sight: dark (lower half, sooner at the highest gain),
  bright, covered (eyes unlike, or no depth while driving), frames that stand still; the limits are start values
  until measured on the mower.
- In the simulation `fake_obstacles` reports the things of a scene file (`config/obstacles_scene.example.json`) from
  the true pose of the fake board, and reads the file again when it changes; `test_obstacles.py` drives the node
  through the app interface, `test_obstacle_stop.py` the hold in the whole stack, `test_obstacle_avoid.py` driving
  around a ball.

## Check list

- [ ] Driver package: protocol code with gtests, build without warnings
- [ ] clang-tidy without findings in the package (`.clang-tidy` of the repo)
- [ ] Every node registered with `mowbite_component_register` and built on `StartupOnly(options)`
- [ ] Fake board, `test_driver_contract.py` passes with your case, the start test too where the driver has a start
      time
- [ ] Platform package: `geometry.yaml`, `platform.yaml` with inherited values checked, `platform.py`, the name in the
      ament index; `test_layers.py` green with it
- [ ] `gnss_receiver_time` only with UTC from the receiver; `rtc` of `mowbite_clock` only after the measurement
- [ ] `allowed` of `mowbite_schedule` only after the gates of the schedule
- [ ] `robot_shape` drawn from measured points, covered by box or outline
- [ ] `setup/setup.yaml` with the inputs and sensors this mower has, `within` from the board, images of one colour,
      `test_setup_<name>` passes
- [ ] A driver that learns its inputs: `inputs_assign` with `restart` in the list, a test of the step against the real
      start as `test_nx60i_inputs.py`
- [ ] `hw/silence` and `hw/end` in the driver with `silence_max`, the steps of stage `wheels` in the list with
      `stop_max` and `emergency` of this board, `test_setup_motion_<name>` passes
- [ ] `blade_allowed` in `hw/capabilities`, hw/silence with `blade`, the step `blade` with the bounds of this board
- [ ] `mowbite.launch.py platform:=<name>` starts the stack, driver restarts latched
- [ ] Every node of your own in the container attached to the watch (`test_stack.py`)
- [ ] Simulation: mowing, going home, docking; `app_interface_check`
- [ ] Way onto the mower chosen, limits for logs and journal, data dir writable (a start works)
- [ ] mowbite-host with its carrier: base version, two keys in allowed_signers, `test_e2e.py` green
- [ ] The app served (mowbite-web or the like) without root, `/cgi-bin/*` 404, the cross-origin header as above,
      broker WebSocket open
- [ ] Every stop button seen in `hw/status`, pressed and free again after the release, before the first motor turned
- [ ] Safety measurement done, date set
- [ ] Lights, if any: device rights, `output` on only after a look on the mower
- [ ] Cameras, if any: own process, publisher list tested, hw/cameras entry, vendor files only on the mower, limits
      measured; `mowbite_camera` started with them, its ports reachable from the app, each protocol watched once
- [ ] Obstacle sensors, if any: named in `obstacle_sources`, `hw/obstacles` at a fixed rate with sight and an honest
      sector, the mount measured, classes mapped in the file of the platform, photos only while wanted;
      `obstacle_mode_max` 2 only after the measurement on the mower, 3 after the stops were watched and the clearance
      measured
- [ ] First drive watched: blade off, RTK fix, at most 0.3 m/s
