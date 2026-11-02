# MowBite OS for the mower, one image for every platform (launch and configuration choose the platform).
# Built from a clean tree that has src/ at its top, e.g. for arm64 on the PC (qemu):
#   docker buildx build --platform linux/arm64 --load --build-context planer=/home/mkay/tron-projekt/planer \
#     --build-arg SOURCE_COMMIT=<commit> -f docker/os.Dockerfile -t local/mowbite-os:arm64-<commit> <tree>
# No simulation, no tests, no models in it.
ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base AS build
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      libpaho-mqttpp-dev libpaho-mqtt-dev nlohmann-json3-dev libssl-dev libeigen3-dev git \
 && rm -rf /var/lib/apt/lists/*

# Compilers at once: BUILD_WORKERS packages with BUILD_JOBS jobs each.
ARG BUILD_WORKERS=2
ARG BUILD_JOBS=2

# The MowBite Planner at a fixed commit of its branch tron, only its two ROS 2 packages, the source removed after.
ARG PLANNER_COMMIT=50934541edc1bc7731d47a015edbb29f57a9a219
RUN --mount=type=bind,from=planer,target=/src/planer \
    git init -q /tmp/planner && cd /tmp/planner \
 && git -c safe.directory='*' fetch -q --depth 1 /src/planer "$PLANNER_COMMIT" \
 && git checkout -q FETCH_HEAD \
 && test "$(git rev-parse HEAD)" = "$PLANNER_COMMIT" \
 && . /opt/ros/$ROS_DISTRO/setup.sh \
 && MAKEFLAGS=-j$BUILD_JOBS colcon --log-base /tmp/planner_log build \
      --base-paths ros2/mowbite_planner_interfaces ros2/mowbite_planner \
      --build-base /tmp/planner_build --install-base /opt/mowbite_planner \
      --parallel-workers $BUILD_WORKERS --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
 && rm -rf /tmp/planner /tmp/planner_build /tmp/planner_log

# Every package but the simulation; PACKAGES narrows it (colcon selection, e.g. "--packages-up-to xcore_driver").
ARG PACKAGES="--packages-skip mowbite_sim"
COPY src /ws/src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && . /opt/mowbite_planner/setup.sh && cd /ws \
 && MAKEFLAGS=-j$BUILD_JOBS colcon build $PACKAGES --parallel-workers $BUILD_WORKERS \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
 && rm -rf /ws/build /ws/log

# Runtime packages of all installed packages: ROS keys by name, other keys mapped here, an unknown key stops the build.
RUN rosdep keys --from-paths /ws/install /opt/mowbite_planner --ignore-src -t exec | sort -u > /tmp/keys \
 && for k in $(cat /tmp/keys); do \
      if [ -f /opt/ros/$ROS_DISTRO/share/$k/package.xml ]; then echo "ros-$ROS_DISTRO-$(echo $k | tr _ -)"; \
      else case $k in \
        libpaho-mqttpp-dev) echo libpaho-mqttpp3-1 ;; \
        libssl-dev) echo libssl3t64 ;; \
        python3-shapely|python3-paho-mqtt) echo $k ;; \
        nlohmann-json-dev|eigen) ;; \
        *) echo "unknown runtime key $k" >&2; exit 1 ;; \
      esac; fi; \
    done > /opt/runtime-packages.txt \
 && cat /opt/runtime-packages.txt

FROM ros:${ROS_DISTRO}-ros-core
COPY --from=build /opt/runtime-packages.txt /opt/runtime-packages.txt
RUN apt-get update \
 && apt-get install -y --no-install-recommends $(cat /opt/runtime-packages.txt) \
 && rm -rf /var/lib/apt/lists/*
COPY --from=build /opt/mowbite_planner /opt/mowbite_planner
COPY --from=build /ws/install /ws/install
# Every library a binary of the image links must be there.
RUN . /ws/install/setup.sh \
 && find /ws/install /opt/mowbite_planner -type f | while read -r f; do \
      if [ "$(head -c 4 "$f" | tail -c 3)" = "ELF" ]; then ldd "$f" | sed -n "s|^\(.*not found\)|$f: \1|p"; fi; \
    done > /tmp/missing \
 && if [ -s /tmp/missing ]; then cat /tmp/missing; exit 1; fi \
 && rm /tmp/missing
ARG SOURCE_COMMIT=unknown
LABEL org.opencontainers.image.title="mowbite-os" org.opencontainers.image.revision="$SOURCE_COMMIT"
# ros2 launch catches only SIGINT; as PID 1 it ignores SIGTERM and docker stop would kill it without blade off.
STOPSIGNAL SIGINT
ENTRYPOINT ["/bin/bash", "-c", "source /ws/install/setup.bash && exec \"$@\"", "--"]
