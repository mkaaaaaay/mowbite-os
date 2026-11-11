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
  empty, full and critical, charge check, temperatures). Check every value you inherit. The battery voltages stand only
  here: a driver that sets them on its board gets them in the start file from `stack.battery_voltages`, under the
  same names (see `nx60i.launch.py`).
- Driver parameters (ports, baud rates, limits) in the file of the driver; values measured on one mower in a file on
  that mower. No value in two files.
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
- Map, settings and journal live in the data dir, never in a container volume.
- The data dir has to stay writable for the OS: while `os-session.json` or `restart-hold.json` cannot be written
  there, nothing starts from IDLE (`STORAGE_NOT_WRITABLE`, event `STORAGE`), a job already running goes on. Put it
  on storage that lasts across power cuts and is not mounted read only on errors without a way back.
- Every log, recording and cache gets a limit in days and MB.
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
  `min_app`). An own planner or app is used only while the link `active` in its folder names it; the host alone sets
  it, when the own one goes with the installed OS and is newer than the bundled one. Whatever starts the planner or
  serves the app on your mower reads `active` first and else `<os>/planner` or `<os>/app`, and decides nothing itself.
  A planner update restarts the OS through the gate of the OS; an app update only changes files and needs no gate.
  Bundles of the planner and the app hold no machine code. On the Tron `os-setup` puts the planner `active` names
  before the one of the OS (which keeps `mowbite_planner_interfaces`), and tmpfiles makes the two folders.
  `mowbite-update-recover.service` runs at boot while any of the three folders has a `pending`.
- The host updates itself as a fourth component under `/opt/mowbite-host` without a gate: the switch writes `pending`
  and `current` and ends the process, the service manager starts it again (`Restart=always`). `mowbite_boot` tries
  the new one for 180 s; it confirms itself after 60 s of its main loop (and once connected, when the host before it
  was), else the next start goes back to the one before or to the seed and leaves a note for the result. After three
  starts in a boot that did not run 60 s the seed runs. Keep `Restart=always` and the watchdog on your service.
- The app comes from `mowbite-web` (`web/mowbite_web.py`, standard library only): port 80 from
  `mowbite-web.socket`, the service as its own user `mowbite-web` in no group, no root, no capability, nothing to
  write. GET and HEAD only, `/cgi-bin/*` is always 404 (the app reads that as "no container"); only `/app.json`,
  `/version.json` and `/cgi-bin/*` carry `Access-Control-Allow-Origin: *`, for the Android app and the page of another
  mower. Hashed parts under
  `/_next/static/` are also taken from the older installations so an open page keeps working after an update. The
  app reaches the broker over WebSocket on 9001 (`docker/mower/mosquitto.conf`). A compose stack can serve the same
  export with any static server that follows these rules. `web/test/test_web.py` runs without a mower.
- Bundles, all in `platform/tron/bundle/`: `make-bundle.sh` for the OS (serial = commit time, base, api, the planner's
  `contract.json` and what it brings along from the image), `make-planner-bundle.sh <repo> <commit> <out>` (only
  `mowbite_planner`, built in a throwaway container, `contract.json` required), `make-app-bundle.sh <zip> <out>` (the
  web export of the app release, `app.json` required), `make-host-bundle.sh <commit> <out>` (`host/mowbite_host` of
  that commit). `host/bundle/app_export.py` unpacks and checks the export for
  them and for `docker/os.Dockerfile` (build context `app`, `APP_VERSION`, `APP_SHA256`; none without them). Each is
  signed with `sign-bundle.sh` under the namespace of its component (refuses a serial not newer than the last one
  signed for it, shows its fields and the result of the test gate), then into the inbox
  `/userdata/mowbite-update/inbox` and `update.check`.

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

## 8. Cameras (optional)

A camera of the mower, shown in the app, optionally with object detection for display only. The Tron's setup is the
example: `tron_vision` (camera AI) and `tron_stereo` (live picture) with their start in `tron.launch.py` and their
parameters in `config/tron.yaml`.

The platform makes the pictures, the OS takes them to the app: `mowbite_camera` keeps the cameras of `hw/cameras`,
answers `camera.list` and `camera.stream` (both show up in `rpc.methods` with the first camera), lets two viewers in
at a time and gives each an address with a token of its own. The first list is what a camera process has to do, the
second what `mowbite_camera` needs around it.

- A process of its own, never a component of the container: a crash of a vendor library takes nothing else along.
  Start it from the start file with `stack.own_process` only when the drivers run and a switch in
  `/etc/mowbite/os.conf` says so. It restarts at most three times and leaves `<name>_exit.json`; give it
  `oom_score_adj` 1000, a `nice` value and CPUs apart from the driver. Pass the same name to the node as `process`.
- It reports and nothing else: sensors over `bridge/sensor_info` and `bridge/sensor_value`, detections on
  `vision/detections` (`mowbite_msgs/Detections`, boxes in pixels of the camera image), each camera latched on
  `hw/cameras` (`mowbite_msgs/Camera`: id, process, size of the shown image, protocols, and while it cannot stream
  one of the reasons of the message). It subscribes to nothing of the stack but `hw/status` when it needs the state
  of the mower (best effort, depth 1); tests check its lists of publishers and subscriptions
  (`tron_vision/test/test_vision_node.cpp`, `test_stereo_node.cpp`).
- A thread of its own beside the ROS thread signs the watch of the process after each step (`mowbite_watch`: `Attach`
  for the node, a `Watch` for the thread); a step that hangs in a vendor library then ends the process (code 86).
- Protocol `mjpeg`: JPEG on `camera/<id>/jpeg` (`sensor_msgs/CompressedImage`, best effort, depth 1), made only while
  someone subscribes, with a rate and a size limit as platform parameters. The OS subscribes while a viewer is
  connected and serves the pictures to the app.
- Protocol `webrtc`, for a smooth live picture: the process is an RTSP camera on the loopback address (H.264
  constrained baseline, RTP over the RTSP connection, its address `rtsp://127.0.0.1:<port>/<id>` as `source` in
  `hw/cameras`; the OS takes no other form), and the OS pulls from it only while someone watches. The connection is
  the viewer signal: PLAY starts the camera, its end stops it, and while the camera cannot stream DESCRIBE answers
  503. The parts without ROS are in `tron_vision` (`rtsp_server`, `h264_rtp`, `stereo_loop`); a hardware encoder is
  loaded at run time like the other vendor libraries.
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
- Settings of a camera that need register writes (exposure control of the Tron's bridge chip, `tron-stereo-ae`) come
  from the setup at boot, read back, before it reports the camera ready; the node never writes to i2c.

Around `mowbite_camera`:

- Start it beside the camera processes and only with them: `stack.camera_service(files, data_dir, logger, ...)` with
  their limits (`tron.launch.py`). It is a process of its own too and leaves `camera_exit.json`.
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
- `position` in `hw/cameras` (`front`, `rear`, `left`, `right`; empty when unknown or not fixed) is what the app's
  switch between cameras goes by.

## Check list

- [ ] Driver package: protocol code with gtests, build without warnings
- [ ] clang-tidy without findings in the package (`.clang-tidy` of the repo)
- [ ] Fake board, contract test passes
- [ ] Geometry file and platform file, inherited values checked
- [ ] Start file, driver restarts latched
- [ ] Every node of your own in the container attached to the watch (`test_stack.py`)
- [ ] Simulation: mowing, going home, docking; `app_interface_check`
- [ ] Way onto the mower chosen, limits for logs and journal, data dir writable (a start works)
- [ ] mowbite-host with its carrier: base version, two keys in allowed_signers, `test_e2e.py` green
- [ ] The app served (mowbite-web or the like) without root, `/cgi-bin/*` 404, the cross-origin header as above,
      broker WebSocket open
- [ ] Safety measurement done, date set
- [ ] Lights, if any: device rights, `output` on only after a look on the mower
- [ ] Cameras, if any: own process, publisher list tested, hw/cameras entry, vendor files only on the mower, limits
      measured; `mowbite_camera` started with them, its ports reachable from the app, each protocol watched once
- [ ] First drive watched: blade off, RTK fix, at most 0.3 m/s
