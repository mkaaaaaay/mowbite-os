# Build and test environment for MowBite OS on the PC (amd64).
FROM ros:jazzy-ros-base
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      libpaho-mqttpp-dev libpaho-mqtt-dev nlohmann-json3-dev \
      mosquitto mosquitto-clients python3-paho-mqtt \
      git python3-shapely \
 && rm -rf /var/lib/apt/lists/*

# The MowBite Planner (own process of the stack) at a fixed commit, plan-transit on top of main 026e17a.
# Source it after ROS: . /opt/ros/jazzy/setup.bash && . /opt/mowbite_planner/setup.bash
ARG PLANNER_COMMIT=8ff8f7315563eafff81625c07193837be5f5cfe3
RUN git init -q /tmp/planner && cd /tmp/planner \
 && git fetch -q --depth 1 https://github.com/mkaaaaaay/mowbite-planner.git "$PLANNER_COMMIT" \
 && git checkout -q FETCH_HEAD \
 && test "$(git rev-parse HEAD)" = "$PLANNER_COMMIT" \
 && . /opt/ros/jazzy/setup.sh \
 && colcon --log-base /tmp/planner_log build \
      --base-paths ros2/mowbite_planner_interfaces ros2/mowbite_planner \
      --build-base /tmp/planner_build --install-base /opt/mowbite_planner \
 && . /opt/mowbite_planner/setup.sh \
 && ros2 pkg prefix mowbite_planner_interfaces > /dev/null && ros2 pkg prefix mowbite_planner > /dev/null \
 && rm -rf /tmp/planner /tmp/planner_build /tmp/planner_log
