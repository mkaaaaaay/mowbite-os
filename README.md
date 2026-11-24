# MowBite OS

<img src="docs/logo.svg" alt="MowBite logo" width="96" align="right">

Software that runs a robot mower. It sits on the mower's own computer, finds its place on the lawn with RTK GPS, mows
the areas you recorded, drives home, docks and charges. No cloud and no account: the mower works on its own in your
home network.

MowBite OS is built on [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/). To the outside it talks over MQTT and JSON-RPC,
and the [MowBite app](https://github.com/mkaaaaaay/mowbite) runs on it. The mower lists itself what it can do
(`rpc.methods`), so an app only shows what your mower really has.

MowBite OS is a community project and not affiliated with Airseekers or YardForce.

**Still in development.** One mower model runs it today, and things still change. Read
[Supported mowers](#supported-mowers) and [Safety](#safety) before you put it on a mower.

Questions, ideas and help: the [MowBite Discord](https://discord.gg/VpPFRx9Ce4) has channels for MowBite OS.

![Mowing, the setup, updates and obstacles in the MowBite app](docs/screens.webp)

- [What it does](#what-it-does)
- [Supported mowers](#supported-mowers)
- [Safety](#safety)
- [How it works](#how-it-works)
- [Install and update](#install-and-update)
- [Try it on a PC](#try-it-on-a-pc)
- [Build and test](#build-and-test)
- [What is where](#what-is-where)
- [Add a mower](#add-a-mower)
- [App interface](#app-interface)
- [License](#license)

## What it does

- **Map**: drive the mower along the edges once and it records the areas and the docking station. The map is kept on
  the mower and can be changed from the app.
- **Mowing**: the [MowBite Planner](https://github.com/mkaaaaaay/mowbite-planner) plans loops along the edge and
  straight lanes for the whole body of the mower, and a path follower of its own drives them. Pause, continue, skip a
  path or an area, mow several areas in one job. Where the mower can set its cutting height, you choose it in the app,
  also per area.
- **Going home**: when the job is done, the battery gets low, it rains or the sun sets, the mower drives to its
  station, docks and checks that it really charges. A try that fails is repeated. The Tron finds the last metres by a
  marker at the station that its rear camera sees.
- **Schedule**: plans with days, start, end and areas. It can skip rain, cold and a bad forecast, wait for the battery
  and stay home in the dark. Off until you allow it for your mower.
- **Position**: RTK GNSS, gyro and wheel odometry in a filter of its own. The corrections come from an NTRIP caster
  you choose (SAPOS for example). A calibration drive on the lawn measures wheel ticks, track width, antenna offset and
  how the IMU is mounted.
- **Setup in the app**: step by step. Choose your mower, check stop button, lift, bumper and IMU by hand, then the
  wheels with the mower jacked up, a short drive on the ground and the blade with its disc taken off. These steps are
  the safety measurement of your mower (see [Safety](#safety)).
- **Cameras and obstacles**: the live picture of the mower's cameras in the app. A camera with object detection can
  show obstacles, stop for them or drive around objects. How far a mower may go there depends on what was measured on
  it.
- **Updates**: signed bundles from GitHub releases, installed from the app or at night, only while the mower rests in
  its station. If the new version doesn't come up healthy, the one before comes back.
- **Activity**: event history, the track of every job, mowing statistics, maintenance reminders by blade hours, sensor
  values of the last 48 hours, push messages through [ntfy](https://ntfy.sh).
- **Diagnosis**: the OS keeps a journal of what its core parts got and decided, and around an emergency or a crash an
  incident you can download. A journal replays bit for bit on a PC.
- **Lights and sounds** where the mower has them: a light profile per state, announcements, your own MP3s.

## Supported mowers

| Mower | Board | Runs as | Status |
| --- | --- | --- | --- |
| Airseekers Tron | its MCU on a serial port, GNSS and IMU read directly | native on a root file system of its own, updated with signed bundles | Runs on the mower: recording, mowing, driving home and docking work while someone watches. Mowing by schedule is still being tested, the camera only shows obstacles so far. |
| YardForce NX60i with an xCore board | xCore on Ethernet | Docker Compose | In progress: driver, platform package and simulation are there. The safety measurement on the mower is still open, so it is not ready for use. |

Every mower is reached through one neutral contract (`hw/*`), nothing above the driver knows a mower by name. That
keeps a new mower a matter of a driver, a platform package and a measurement, see [Add a mower](#add-a-mower).

## Safety

A robot mower has a turning blade and drives by itself. It can hurt people and animals. MowBite OS comes without any
warranty, and you are responsible for the mower you run it on.

Several layers, each working without the ones above it:

1. **The board of the mower** with its own firmware: stop button, lift and bumper. What a board does by itself, for
   example how long it keeps driving without its host, is measured on each mower model and never assumed.
2. **The driver**, a process of its own with its own watchdogs. Without a new drive command for 0.5 s it stops the
   wheels. The blade only runs on a lease that has to be renewed all the time, and never during an emergency. A driver
   that was restarted comes up locked. A crash of anything else leaves it running.
3. **The safety filter** (`mowbite_safety`), the only writer of drive commands to the driver. It stops the mower when
   it tilts too far, when it leaves the areas of the map while driving by itself, and when the IMU falls silent.
4. **The logic** asks for the blade only while it mows a lane and is not paused. Every pause, by you, by a lost
   position or by an emergency, lets the blade stop.
5. **The plan**: the planner checks the whole body of the mower along a path, not just its middle.

On top of that:

- **Emergency**: stop button, lift and bumper latch an emergency that stays until you release it.
- **Watchdogs for the software itself**: every node has to report regularly, and one that hangs ends the stack. If
  the OS ends that way while the mower is working, nothing drives or mows after the restart until you release it. After
  three crashes in a row the stack stays down.
- **Nothing by itself before the safety measurement**: a mower without it only takes remote control. The blade is
  locked until its own step passed. The measurement belongs to one mower, not to a model: it is done on yours, with the
  setup steps in the app, before it drives by itself. A safety step that fails later closes the lock again.
- **Careful defaults**: a new mower is limited to 0.3 m/s and 0.3 rad/s until driving tests show more. The schedule is
  off until you allow it. Stopping for obstacles a camera sees is only released for a mower after it was measured
  there.
- **Updates** of the OS switch only while the mower stands in its station without an emergency.

What you should know:

- The MQTT broker on the mower has no login. Whoever is in your network can command the mower. Keep it in a network
  you trust and never open its ports to the internet.
- A camera that sees obstacles is an extra. Bumper, lift and stop button stay the emergency stop.
- The first drive of a mower: blade off, RTK fix, slow, someone at the stop button.

## How it works

```
  phone or browser
  +-------------+
  | MowBite app |
  +------+------+
         | MQTT (WebSocket), JSON-RPC
 --------+------------------------------------------------ on the mower ---
  +------+------+     +--------------+
  |   broker    +-----+ mowbite-host |  updates, runs beside the OS
  +------+------+     +--------------+
         |
  +------+------+
  |   bridge    |  MQTT topics and RPC of the app interface
  +------+------+
         |
  logic, map, schedule, setup, planner, localization, statistics, cameras, ...
         |
  +------+--------+
  | safety filter |  every drive command passes here
  +------+--------+
         | hw/* contract
  +------+------+
  |   driver    |  one per mower, a process of its own
  +------+------+
         |
  board of the mower: motors, blade, buttons, sensors
```

- The **driver** is the only part that knows the board. It offers the same topics and services on every mower
  (`hw/status`, `hw/cmd_vel`, `hw/blade`, `hw/emergency`, ...).
- All other nodes run together in one container process, each in its own thread, and are the same for every mower. The
  planner, the network service and the cameras run as processes of their own.
- A **platform package** (`platform_<name>`) holds what one mower needs: dimensions, values, its part of the start and
  the steps of the setup program.
- **mowbite-host** runs beside the OS and installs updates, also while the OS is off.
- Map and settings live in one data folder on the mower. Every log, recording and cache has a limit in days and MB.

The mower itself connects to: your NTRIP caster, a time server, [Open-Meteo](https://open-meteo.com) for the forecast
of the schedule, the ntfy server if you set up push messages, and GitHub for updates (`MOWBITE_UPDATE_NET=off` turns
that off).

## Install and update

How a mower gets MowBite OS depends on the mower.

**Airseekers Tron.** The [MowBite OS installer](https://github.com/mkaaaaaay/mowbite-installer) sets it up from a
Windows or Linux PC over USB. It backs up the whole flash of the mower first, then writes a root file system of its own
(Ubuntu 24.04, ROS 2 Jazzy) in place of the vendor's software, and the app takes over from there. The same program
writes the backup back. The vendor parts the system needs come from the backup of your own mower, never from a
download. [platform/tron/rootfs](platform/tron/rootfs) builds the base system the installer writes: `build-base.sh`
makes the base, `make-image.sh` the image, `check-image.sh` checks it. The OS itself comes as a bundle.

**YardForce NX60i.** A Docker Compose stack: [docker/mower/nx60i.compose.yaml](docker/mower/nx60i.compose.yaml) with
`mosquitto.conf` and a `.env` made from `env.example`, and an image built from
[docker/os.Dockerfile](docker/os.Dockerfile). Whatever ran the mower before is stopped first, two programs never talk
to the board at once. Not ready for use yet, see [Supported mowers](#supported-mowers).

**Updates** (today on the Tron):

- Four parts are updated on their own: the OS, the planner, the app and mowbite-host. Each comes as a signed bundle,
  and the mower only installs what holds for the keys in its root file system.
- Bundles are published as releases of [mowbite-releases](https://github.com/mkaaaaaay/mowbite-releases). The mower
  looks there every 6 hours and when you ask in the app. Without internet a bundle can be put into the inbox folder on
  the mower.
- Two channels: `stable` (the default) and `preview`. So far every release is a preview.
- Installing is your tap in the app, or automatic at night if you switch that on (off by default, 02:00 to 05:00).
- The OS and the planner switch only while the mower idles on its charging contacts without an emergency. After the
  switch the new OS has to come up healthy, else the one before comes back. You can also go back by hand.
- Over SSH the same works with `mowbite-update` (`install`, `rollback`, `status`).

A mower on Docker Compose has no updater for the OS yet, its image is updated by hand.

## Try it on a PC

The whole OS runs against a simulated Tron (board, GNSS and IMU on byte level), with a broker and the app. You need
Docker with buildx and a clone of the planner beside this repo:

```bash
git clone https://github.com/mkaaaaaay/mowbite-planner.git ../mowbite-planner
docker buildx build --load --build-context planer=../mowbite-planner -f docker/dev.Dockerfile -t mowbite-os-dev:jazzy .
docker compose -f docker/sim/compose.yaml up -d --build
```

Then open `http://localhost:48082`. MQTT is on port 41883, its WebSocket on 49001. `OMTRON_APP_IMAGE` chooses another
image of the app. Sensors of the simulated board are set through a file, for example the lift:

```bash
docker exec omtron-mowbite_os-1 sh -c 'echo "{\"lift\": 1}" > /tmp/mowbite_sim_flags.json'
```

Stop it with `docker compose -f docker/sim/compose.yaml down`.

## Build and test

Everything builds and runs in the image `mowbite-os-dev:jazzy` from above (ROS 2 Jazzy, the planner at the commit
`docker/dev.Dockerfile` pins).

Unit tests of all packages, with compiler warnings as errors:

```bash
docker run --rm -v "$PWD/src:/ws/src:ro" mowbite-os-dev:jazzy bash -c '
  . /opt/ros/jazzy/setup.sh && . /opt/mowbite_planner/setup.sh && cd /ws &&
  colcon build --packages-skip mowbite_sim --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_COMPILE_WARNING_AS_ERROR=ON &&
  colcon test --packages-skip mowbite_sim --return-code-on-test-failure; colcon test-result --verbose'
```

Tests of the whole stack in the simulation are launch tests in `src/mowbite_sim/test`, one file per topic (mowing,
docking, going home, the driver contract of each mower, the setup program, ...). Each brings its own broker. In the
simulation image, for example:

```bash
docker run --rm local/mowbite-os-sim:dev launch_test /ws/src/mowbite_sim/test/test_mowing.py
```

The updater and the web server are plain Python. Their tests need `mosquitto`, `python3-paho-mqtt` and `ssh-keygen`:

```bash
for t in host/test/test_*.py web/test/test_web.py platform/tron/bundle/test_update.py; do python3 -I "$t"; done
```

`src/mowbite_sim/test/app_interface_check.py` checks a running OS against the app interface. The workflow
[check.yaml](.github/workflows/check.yaml) runs the unit tests on arm64 and amd64, the app interface check against the
simulation and the replay of its journal.

How the code is written: protocol and control code without ROS in libraries with unit tests, thin ROS nodes around
them, C++ formatted with `.clang-format`, no findings of `.clang-tidy`.

## What is where

| Folder | What |
| --- | --- |
| `src/` | the ROS 2 packages, see below |
| `platform/tron/` | what the Tron needs besides its packages: its root file system (`rootfs/`) and the scripts that make, sign and publish bundles (`bundle/`) |
| `host/` | mowbite-host, the updater beside the OS, with its tests |
| `web/` | mowbite-web, a small web server for the app on the mower |
| `docker/` | Dockerfiles for building and testing, the simulation and the mower image, and the compose files |
| `tools/yf/` | the program of the safety measurement at the mower (`measure.py`) and its tests |
| `docs/` | [new-mower.md](docs/new-mower.md), how to add a mower, and the pictures of this page |

The packages in `src/`:

| Package | What |
| --- | --- |
| **Driving and mowing** | |
| `mowbite_logic` | states, actions and sensors: mowing, docking, undocking, recording |
| `mowbite_follower` | path follower |
| `mowbite_safety` | safety filter between every source of drive commands and the driver |
| `mowbite_localization` | position from gyro, wheel speed and RTK GNSS |
| `mowbite_map` | areas and docking stations, stored on the mower |
| `mowbite_calibration` | calibration drive |
| `mowbite_obstacles` | obstacles the sensors of a mower see: tracks, zones, the list with photos |
| `mowbite_schedule`, `mowbite_time` | the schedule, the sun and whether the clock can be trusted |
| `mowbite_setup` | setup program |
| `mowbite_stats` | statistics, maintenance, incidents |
| **To the outside** | |
| `mowbite_bridge` | the MQTT topics and JSON-RPC of the app, event history, mowing track |
| `mowbite_api` | the catalog of the app interface and its JSON Schema |
| `mowbite_ntrip` | NTRIP client for the RTK corrections |
| `mowbite_net` | weather for the schedule, push messages |
| `mowbite_camera` | the cameras for the app, MJPEG and WebRTC |
| `mowbite_light`, `mowbite_sound` | lights and sounds |
| **Mowers** | |
| `platform_tron`, `tron_mcu`, `tron_gnss`, `tron_imu`, `tron_vision` | Airseekers Tron: platform package, driver of its MCU, GNSS, IMU, cameras |
| `platform_nx60i`, `xcore_driver` | YardForce NX60i: platform package, driver for its xCore board |
| **Shared parts** | |
| `mowbite_bringup` | the start of the stack and the parameters every mower shares (`stack.yaml`) |
| `mowbite_msgs` | messages and services |
| `mowbite_sim` | simulated boards and the tests of the whole stack |
| `mowbite_journal`, `mowbite_replay` | the journal and its replay |
| `mowbite_watch`, `mowbite_hold`, `mowbite_component` | watchdog of a process, holds and locks, how nodes are built and started |
| `mowbite_serial`, `mowbite_gnss`, `mowbite_outbox` | serial port, GNSS helpers and queues for drivers |

## Add a mower

[docs/new-mower.md](docs/new-mower.md) is the guide, with a check list at its end. In short, a new mower needs:

1. a **driver** for the `hw/*` contract, in a process of its own
2. a **fake board** for the simulation, so the driver contract test runs against it
3. a **platform package** with its dimensions, values, start and setup steps
4. a way **onto the mower**: Docker Compose or a root file system of its own
5. a **safety measurement** on the mower

Logic, follower, safety filter, map, planner, bridge and app stay as they are. Lights, cameras and obstacle sensors are
optional. Start from the closer one of the two drivers there are.

## App interface

The interface an app talks to is described in [src/mowbite_api/api.schema.json](src/mowbite_api/api.schema.json): the
MQTT topics, every RPC method, events, reasons with their texts, settings and the setup program. It is made from the
code, and `system.api` gives it at run time. It carries two numbers: `x-api` goes up with every change, `x-api-break`
only when something is taken away or changes its meaning. A client works with every OS of its break number whose api
is at least the one it needs.

## License

Copyright 2026 mkay

[PolyForm Strict 1.0.0](https://polyformproject.org/licenses/strict/1.0.0), see [LICENSE](LICENSE). You may use MowBite
OS for any noncommercial purpose. You may not pass it on or change it, with one exception: you may change it to
propose that change as a pull request to this repository. With a contribution you give the licensor the right to use
it under any terms, commercial ones too. The exact words are at the top of [LICENSE](LICENSE). Commercial use needs a
separate license from the licensor, for contact see the profile of [mkaaaaaay](https://github.com/mkaaaaaay). MowBite OS
comes without any warranty.

- `src/mowbite_sound/third_party/minimp3` is [minimp3](https://github.com/lieff/minimp3) under CC0 1.0.
- Images and bundles also bring programs of their own under their own licenses: the MowBite Planner and the MowBite
  app (PolyForm Noncommercial 1.0.0), MediaMTX (MIT) for the camera streams and, on arm64, Rockchip MPP (Apache-2.0 or
  MIT).
- Models and runtimes of a mower's vendor are never part of this repository, of a published image or of a bundle.
  They stay on the mower.
