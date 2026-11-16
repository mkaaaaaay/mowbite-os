# Adding a mower

MowBite OS reaches the hardware only through the contract `hw/*`. A new mower needs a driver for it, a fake board for
the simulation, its dimensions, a start file, a way onto the mower and a safety measurement. Logic, follower, safety
filter, map, planner, bridge and app stay as they are.

The drivers there are: `tron_mcu` (Airseekers Tron, serial MCU, GNSS and IMU as nodes of their own) and `xcore_driver`
(YardForce NX60i, OpenMower v2 board on Ethernet, passes IMU and GNSS on). Start from the closer one.

## 1. Driver

One package, its own process: a crash of another node must leave the watchdog of the driver running. Protocol code
without ROS in a library of its own with gtests; the node stays thin. Build the node like every component: register it
with `mowbite_component_register` instead of `rclcpp_components_register_node` (hidden symbols, several MB less RSS)
and give the base class `mowbite_component::StartupOnly(options)`: parameters are read once at the start, there are
no parameter services.

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
| publishes, optional | `hw/dock_marker` | `mowbite_msgs/DockMarker` |
| subscribes, optional | `hw/dock_marker_request` | `std_msgs/Empty` |

Duties:

- Drive: at most one frame per 100 ms to the board. Without `hw/cmd_vel` for a fixed time: three zero frames 100 ms
  apart, then none. A heartbeat alone stops nothing.
- Blade: on only with a lease (`lease_s`), off when it runs out; heights outside the limits of `hw/capabilities` are
  refused; never on while an emergency is latched. With `blade_height_control` the driver gives its limits and the
  height it cuts at for `height_mm` 0 (`blade_height_default_mm`); the app then offers `mow_height_mm`, which the logic
  sends at each start of the blade. Without it the logic always sends 0.
- Emergency: reason bits numbered like the EMERGENCY event; stop button and lift stay latched until a release. The host
  latches only its own bits (`kHostLatchReasons` in `mowbite_hold/restart.h`). A release waits a short window for the
  flags to drop; a new reason within it ends the window without releasing.
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
- Capabilities: `platform` is the same name as in the geometry file; set `gnss_receiver_time` only where
  `receiver_time_ms` of `gps/absolute_pose` is the UTC time of day of the receiver (Tron), not a clock of the board
  (xCore). `mowbite_clock` compares it with the wall clock: off by more than 60 s, date and sun count as unknown
  (`time/clock`, `gnss_mismatch`). Nothing in the code asks for a platform name.
- Limits until driving tests show more: 0.3 m/s and 0.3 rad/s.
- Before opening a port: no other process holds it.
- A restarted driver starts latched (`start_latched`).
- Start: until the first sample of a sensor it needs, a driver may refuse drive and blade without an emergency, for a
  fixed start time at most (`tron_mcu`: `imu_start_timeout`). After it the driver accepts or reports an emergency.
  Whoever starts a driver waits that long before the first request.
- Dock marker (optional): a process of its own that measures only while asked on `hw/dock_marker_request`, one
  image per message on `hw/dock_marker`, never an old or estimated pose; the times, states and reasons are in
  `DockMarker.msg`. What the mower drives with it and when it stops is up to the logic. The Tron's process is
  `tron_rear` (`tron_vision`, section 8). The stamp is the time the image was taken, on the clock of the host: the
  localization fuses each image at that moment and drops it when it arrives more than `marker_max_age` late.

## 2. Fake board

A fake board on a pty or a socket, so driver, logic and app run without the mower: see `mowbite_sim/scripts/fake_tron`
and `fake_xcore`. Then a contract test like `mowbite_sim/test/test_hw_contract.py` with your driver and your fake. It
checks capabilities, status, the frame rate, the watchdog, the blade lease, the emergency and its release.
`test_hw_start.py` checks the start time of a driver that has one.

## 3. Dimensions and settings

- `mowbite_bringup/config/geometry/<platform>.yaml`: `geometry.*` in m, base_link in the middle of the drive axle, x
  forward, y left: `platform`, `robot_width`, `robot_front`, `robot_rear`, `mower_width`, `blade_ahead`,
  `blade_offset`, `antenna_x`, `antenna_y`, `track`, `wheel_radius`, `wheel_width`, `dock_contact_x`, `ticks_per_m`,
  `imu_roll`, `imu_pitch`. Without `platform` or a number the start stops, also when the body (box or outline) does
  not cover the drive wheels from above (`wheel_radius` ahead and behind the axle, `track` / 2 ± `wheel_width` / 2
  aside); `robot.geometry.set` refuses such a body with `WHEELS_OUTSIDE`. Optional `robot_outline`: the contour from above as a
  flat list x0, y0, x1, y1, ... (at least 3 points); the planner then uses it instead of the rectangle (`tron.yaml`).
  Optional `dock_footprint`: the docking station from above in base_link of the docked mower, the same kind of list,
  for the planner to keep its lanes and turns off it; the platform's alone, without it the planner takes its own.
  `dock_contact_x` says where the charging contacts are: below 0 at the back (the mower backs onto the station and
  leaves it forwards), above 0 in front (forwards onto it, straight back off it, never faster than `docking_speed`;
  the safety filter cuts every reversing of the follower to 0.12 m/s), 0 none (no docking, `docking.start` is not
  offered). Set `contact_speed` and `dock_overshoot` to how late the board reports the contact (`nx60i.yaml`).
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
  `marker_wait`, `marker_cross_track` and `marker_scale_limit` stand in `mowbite_logic` of the platform file.
- `dock_guide` in `mowbite_logic` of the platform file decides whether `docking.start` and going home use the marker:
  `none` (by RTK only; `tron.yaml` underneath has `require`, so a platform without a marker sets `none`), `prefer` (by
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
  `turn_feedback_max` rad/s and never above the limit of the turn; not in the station frame. `mowbite_safety` lets every moving command grow at
  most `ramp_linear` m/s² and `ramp_angular` rad/s² from the one before, whatever its source (0: no limit); less, a stop
  and an emergency go out at once. Measure how fast the wheels follow before raising any of them.
- `mowbite_bringup/config/<platform>.yaml`: what differs from `tron.yaml`, which lies underneath (battery voltages
  empty, full and critical, charge check, temperatures). Check every value you inherit. The battery voltages stand only
  here: a driver that sets them on its board gets them in the start file from `battery_voltages` of
  `xcore_driver/launch/logic_params.py`, under the same names (see `nx60i.launch.py`).
- The way home without the planner (`docking_direct`, `direct_margin`, inherited from `tron.yaml`): a planner that
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
- Driver parameters (ports, baud rates, limits) in the file of the driver; values measured on one mower in a file on
  that mower. No value in two files.
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

## 4. Start file

`mowbite_bringup/launch/<platform>.launch.py`: the driver as its own process, the rest from `stack.py` (container with
all nodes above the contract, and the planner). Copy `nx60i.launch.py`: it restarts the driver at most three times,
each time latched, and leaves `driver_exit.json`.

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
process is outside this watch and stops the mower by its own timeouts.

## 5. Onto the mower

- Docker Compose like the NX60i (`docker/mower/nx60i.compose.yaml`), or a native bundle on the mower's own rootfs like
  the Tron (`platform/tron/`).
- Map and settings live in the data dir, never in a container volume. The journal has a folder of its own outside it
  (`journal_dir`, a bind mount like the data dir); its incidents lie in `journal_dir/incidents`.
- The data dir has to stay writable for the OS: while `os-session.json` or `restart-hold.json` cannot be written
  there, nothing starts from IDLE (`STORAGE_NOT_WRITABLE`, event `STORAGE`), a job already running goes on. Put it
  on storage that lasts across power cuts and is not mounted read only on errors without a way back.
- Every log, recording and cache gets a limit in days and MB. In the platform file: `journal_max_mb`,
  `journal_max_days` for the journal and `incidents_max_mb`, `incidents_max_days`, `incidents_max_days_safety` for its
  incidents (`tron.yaml`, `nx60i.yaml`); an incident keeps a run from its start while the run takes less than a quarter
  of `incidents_max_mb`.
- `mowbite-host` (`host/`, unit `mowbite-host.service` on the Tron) runs beside the OS as root, not in its bundle:
  it installs signed bundles (update.*), answers `host.restart_os` and keeps `mowbite/host/status` retained, also
  while the OS is off. It needs the broker of the OS, `python3-paho-mqtt` (1.x or 2.x), `ssh-keygen`, systemd for
  the OS service and the files of the data dir; a compose stack needs a carrier of its own (not built yet). On the
  rootfs: the seed of the library under `/usr/lib/mowbite-host` (`host/mowbite_host` and its `bundle.json`,
  `make-image.sh` writes both from the commit checked out), `mowbite_boot` under `/usr/lib/mowbite-boot` (from
  `host/boot/`, never in a bundle), the thin entry points `mowbite-host` and `mowbite-update`, the unit enabled, the
  rootfs version in `/usr/lib/mowbite/base` (raise it when a bundle needs something new of the rootfs; bundles name
  the version they need) and `/etc/mowbite/allowed_signers` with two keys, the main one and a reserve kept apart, for
  the namespaces `mowbite-os`, `mowbite-app`, `mowbite-planner`, `mowbite-host` and `mowbite-index`.
  `host/test/test_e2e.py` runs the
  whole way with the carrier `process` (`MOWBITE_HOST_CARRIER=process` in the environment files).
- The host keeps three components, each in a folder of its own with `current`, `previous` and `pending`: the OS
  (`/opt/mowbite`), the planner (`/opt/mowbite-planner`) and the app (`/opt/mowbite-app`). The OS brings a planner and
  an app along and names them in its `bundle.json` (`bundled`, `planner_interface`, `planner_files_version`, `api`,
  `api_break`, `min_app`). An app goes with the OS when its `app.json` names `api.break` equal to `api_break` and an
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
- The app comes from `mowbite-web` (`web/mowbite_web.py`, standard library only): port 80 from
  `mowbite-web.socket`, the service as its own user `mowbite-web` in no group, no root, no capability, nothing to
  write. GET and HEAD only, `/cgi-bin/*` is always 404 (the app reads that as "no container"); only `/app.json`,
  `/version.json` and `/cgi-bin/*` carry `Access-Control-Allow-Origin: *`, for the Android app and the page of another
  mower. Hashed parts under
  `/_next/static/` are also taken from the older installations so an open page keeps working after an update. The
  app reaches the broker over WebSocket on 9001 (`docker/mower/mosquitto.conf`). A compose stack can serve the same
  export with any static server that follows these rules. `web/test/test_web.py` runs without a mower.
- Bundles, all in `platform/tron/bundle/`: `make-bundle.sh` for the OS (serial = commit time, base, api, api_break,
  the planner's `contract.json` and what it brings along from the image), `make-planner-bundle.sh <repo> <commit>
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
2. How long the board keeps the last setpoint without the host: `failsafe_without_host`.
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

## 8. Cameras (optional)

A camera of the mower, shown in the app, optionally with object detection for display only. The Tron's setup is the
example: `tron_vision` (camera AI on the left and, as a second instance, on the right side camera), `tron_stereo` (live
picture), `tron_side` (only the live picture of the right side camera, with `right_node:=side`) and `tron_rear` (dock
marker with a live picture) with their start in `tron.launch.py` and their parameters in `config/tron.yaml`.

The platform makes the pictures, the OS takes them to the app: `mowbite_camera` keeps the cameras of `hw/cameras`,
answers `camera.list` and `camera.stream` (both show up in `rpc.methods` with the first camera), lets two viewers in
at a time and gives each an address with a token of its own. The first list is what a camera process has to do, the
second what `mowbite_camera` needs around it.

- A process of its own, never a component of the container: a crash of a vendor library takes nothing else along.
  Start it from the start file with `stack.own_process` only when the drivers run and a switch in
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
  not wait for the NPU. A camera that sends JPEG, like a USB camera, is decoded scaled in the DCT into NV12 frames
  of the process (`dma_frames`, from a dma-buf heap, lines rounded up to whole macroblocks) that the encoder reads
  (`tron_rear`: 960 x 540 from 1920 x 1080, port 8558; the dock marker still goes first).
- A stream whose driver can crash the kernel does not run next to the blade by default: the loop ends it when the
  blade starts (`BLADE_RUNNING`) or the state of the mower is unknown (`DRIVER_MISSING`: no `hw/status` yet or the
  last one older than 1 s by the time it came, not by its stamp), and tests the camera once per boot while the mower
  stands with the blade off (`SELF_TEST` until then). `stream_while_mowing: true` in the file of the platform lets
  the stream run next to the blade and without `hw/status`; the self-test, the pause when hot and the lock after a
  kernel crash stay. Set it only after measuring on the mower how long the blade runs without the host, or with that
  risk accepted and written next to the value (the Tron: `config/tron.yaml`).
  Start such a process only with `hardware:=true`: with another host at the board `blade_on` says nothing.
- Vendor runtimes and models are loaded at run time from a directory on the mower and never go into the repo, a bundle
  or an image. Check the model when loading it (inputs, outputs, classes against the class list), else report it and
  do not stream.
- Before opening a camera: no other process holds it. A camera whose driver can crash the kernel gets a lock: a marker
  before the stream starts; found after a kernel crash, the camera stays off until the lock file is deleted.
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
  from the setup at boot, read back, before it reports the camera ready; the node never writes to i2c.

Around `mowbite_camera`:

- Start it beside the camera processes that report on `hw/cameras` and only with them; a camera that only measures
  for the logic and shows no picture does not bring it along. `stack.camera_service(files, data_dir, logger, ...)`
  takes their limits (`tron.launch.py`). It is a process of its own too and leaves `camera_exit.json`.
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

## Check list

- [ ] Driver package: protocol code with gtests, build without warnings
- [ ] clang-tidy without findings in the package (`.clang-tidy` of the repo)
- [ ] Every node registered with `mowbite_component_register` and built on `StartupOnly(options)`
- [ ] Fake board, contract test passes, the start test too where the driver has a start time
- [ ] Geometry file and platform file, inherited values checked
- [ ] `gnss_receiver_time` only with UTC from the receiver; `rtc` of `mowbite_clock` only after the measurement
- [ ] `allowed` of `mowbite_schedule` only after the gates of the schedule
- [ ] Start file, driver restarts latched
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
- [ ] First drive watched: blade off, RTK fix, at most 0.3 m/s
