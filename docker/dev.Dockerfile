# Build and test environment for MowBite OS on the PC (amd64).
FROM ros:jazzy-ros-base
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      libpaho-mqttpp-dev libpaho-mqtt-dev nlohmann-json3-dev libasound2-dev libturbojpeg0-dev libcurl4-openssl-dev \
      mosquitto mosquitto-clients python3-paho-mqtt \
      git python3-shapely python3-jsonschema clang-tidy \
 && rm -rf /var/lib/apt/lists/*

# The MowBite Planner (own process of the stack) at a fixed commit of its branch os-pin, taken from a local clone:
#   docker buildx build --load --build-context planer=/home/mkay/tron-projekt/planer -f docker/dev.Dockerfile ...
# Source it after ROS: . /opt/ros/jazzy/setup.bash && . /opt/mowbite_planner/setup.bash
ARG PLANNER_COMMIT=88b6928ba269a5e2a1359fba3ab6909ffb2ef1b3
RUN --mount=type=bind,from=planer,target=/src/planer \
    git init -q /tmp/planner && cd /tmp/planner \
 && git -c safe.directory='*' fetch -q --depth 1 /src/planer "$PLANNER_COMMIT" \
 && git checkout -q FETCH_HEAD \
 && test "$(git rev-parse HEAD)" = "$PLANNER_COMMIT" \
 && . /opt/ros/jazzy/setup.sh \
 && colcon --log-base /tmp/planner_log build \
      --base-paths ros2/mowbite_planner_interfaces ros2/mowbite_planner \
        os/mowbite_planner_os_interfaces os/mowbite_planner_os \
      --build-base /tmp/planner_build --install-base /opt/mowbite_planner \
 && . /opt/mowbite_planner/setup.sh \
 && for p in mowbite_planner_interfaces mowbite_planner mowbite_planner_os_interfaces mowbite_planner_os; do \
      ros2 pkg prefix $p > /dev/null || exit 1; done \
 && rm -rf /tmp/planner /tmp/planner_build /tmp/planner_log
# DDS on the loopback only, without shared memory (fastdds_loopback.xml of mowbite_bringup); the launch halts without it.
COPY src/mowbite_bringup/config/fastdds_loopback.xml /etc/mowbite/fastdds_loopback.xml
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT FASTRTPS_DEFAULT_PROFILES_FILE=/etc/mowbite/fastdds_loopback.xml
