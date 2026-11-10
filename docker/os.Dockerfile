# MowBite OS for the mower, one image for every platform (launch and configuration choose the platform).
# Built from a clean tree that has src/ and tools/ at its top, e.g. for arm64 on the PC (qemu):
#   docker buildx build --platform linux/arm64 --load --build-context planer=/home/mkay/tron-projekt/planer \
#     --build-arg SOURCE_COMMIT=<commit> --build-arg MIN_APP=$(cat <tree>/docker/min_app) \
#     -f docker/os.Dockerfile -t local/mowbite-os:arm64-<commit> <tree>
# With the app: --build-context app=<folder with MowBite-web.zip> --build-arg APP_VERSION=<v> --build-arg APP_SHA256=<h>.
# No simulation, no tests, no models in it.
ARG ROS_DISTRO=jazzy

# Rockchip MPP (Apache-2.0 or MIT) for the hardware H.264 encoder of the Tron, arm64 only: cross-built on the build
# machine from the release tarball, so it costs little under qemu. tron_vision loads it at run time (encoder_library)
# and its licence texts go into the bundle with it. Empty on amd64.
FROM --platform=$BUILDPLATFORM ros:${ROS_DISTRO}-ros-base AS mpp
ARG TARGETARCH
ARG MPP_VERSION=1.1.0
ARG MPP_SHA256=7bb0040f364468ad9abbb71a04553ebc6a5814584e083c2fc9a553783e9656df
RUN mkdir -p /opt/mpp /opt/mpp-inc \
 && if [ "$TARGETARCH" = arm64 ]; then \
      apt-get update \
      && apt-get install -y --no-install-recommends gcc-aarch64-linux-gnu g++-aarch64-linux-gnu \
      && curl -fsSL -o /tmp/mpp.tar.gz "https://github.com/rockchip-linux/mpp/archive/refs/tags/$MPP_VERSION.tar.gz" \
      && echo "$MPP_SHA256  /tmp/mpp.tar.gz" | sha256sum -c - \
      && tar -xzf /tmp/mpp.tar.gz -C /tmp \
      && cmake -S /tmp/mpp-$MPP_VERSION -B /tmp/mpp-build -DCMAKE_BUILD_TYPE=Release -DBUILD_TEST=OFF \
           -DTOOLCHAIN=aarch64-linux-gnu- \
           -DCMAKE_TOOLCHAIN_FILE=/tmp/mpp-$MPP_VERSION/build/linux/aarch64/arm.linux.cross.cmake > /dev/null \
      && cmake --build /tmp/mpp-build -j4 --target rockchip_mpp > /dev/null \
      && mkdir -p /opt/mpp/lib /opt/mpp/share/licenses \
      && cp -L /tmp/mpp-build/mpp/librockchip_mpp.so.1 /opt/mpp/lib/ \
      && aarch64-linux-gnu-strip /opt/mpp/lib/librockchip_mpp.so.1 \
      && cp /tmp/mpp-$MPP_VERSION/LICENSES/Apache-2.0 /tmp/mpp-$MPP_VERSION/LICENSES/MIT /opt/mpp/share/licenses/ \
      && cp /tmp/mpp-$MPP_VERSION/inc/*.h /opt/mpp-inc/; \
    fi

# The folder with MowBite-web.zip, replaced by --build-context app=<folder>; empty without one.
FROM scratch AS app

FROM ros:${ROS_DISTRO}-ros-base AS build
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      libpaho-mqttpp-dev libpaho-mqtt-dev nlohmann-json3-dev libssl-dev libeigen3-dev zlib1g-dev libasound2-dev \
      libturbojpeg0-dev git \
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
 && commit_time=$(git show -s --format=%ct HEAD) \
 && . /opt/ros/$ROS_DISTRO/setup.sh \
 && MAKEFLAGS=-j$BUILD_JOBS colcon --log-base /tmp/planner_log build \
      --base-paths ros2/mowbite_planner_interfaces ros2/mowbite_planner \
      --build-base /tmp/planner_build --install-base /opt/mowbite_planner \
      --parallel-workers $BUILD_WORKERS --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
 && printf '{"commit": "%s", "serial": %s}\n' "$PLANNER_COMMIT" "$commit_time" > /opt/mowbite_planner/source.json \
 && rm -rf /tmp/planner /tmp/planner_build /tmp/planner_log

# Every package but the simulation; PACKAGES narrows it (colcon selection, e.g. "--packages-up-to xcore_driver").
ARG PACKAGES="--packages-skip mowbite_sim"
# Headers of the media library, only for the check of tron_vision against its declarations; empty on amd64.
COPY --from=mpp /opt/mpp-inc /opt/mpp-inc
ENV TRON_VISION_MPP_INCLUDE=/opt/mpp-inc
COPY src /ws/src
# The FILES_VERSION of the pinned planner has to be the one data_schema.json of mowbite_bridge expects.
RUN . /opt/ros/$ROS_DISTRO/setup.sh && . /opt/mowbite_planner/setup.sh && cd /ws \
 && MAKEFLAGS=-j$BUILD_JOBS colcon build $PACKAGES --parallel-workers $BUILD_WORKERS \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
 && rm -rf /ws/build /ws/log \
 && . /ws/install/setup.sh \
 && if [ -d /ws/install/mowbite_bringup ]; then \
      python3 /ws/install/mowbite_bringup/share/mowbite_bringup/launch/data_store.py --check-planner; fi

# Runtime packages of all installed packages: ROS keys by name, other keys mapped here, an unknown key stops the build.
# Each package directory on its own: rosdep skips the install bases, colcon marks them with COLCON_IGNORE.
RUN rosdep keys --from-paths /ws/install/*/ /opt/mowbite_planner/*/ --ignore-src -t exec | sort -u > /tmp/keys \
 && grep -qx rclcpp /tmp/keys \
 && for k in $(cat /tmp/keys); do \
      if [ -f /opt/ros/$ROS_DISTRO/share/$k/package.xml ]; then echo "ros-$ROS_DISTRO-$(echo $k | tr _ -)"; \
      else case $k in \
        libpaho-mqttpp-dev) echo libpaho-mqttpp3-1 ;; \
        libssl-dev) echo libssl3t64 ;; \
        libasound2-dev) echo libasound2t64 ;; \
        zlib) echo zlib1g ;; \
        libzstd-dev) echo libzstd1 ;; \
        python3-shapely|python3-paho-mqtt|python3-yaml|libturbojpeg) echo $k ;; \
        nlohmann-json-dev|eigen) ;; \
        *) echo "unknown runtime key $k" >&2; exit 1 ;; \
      esac; fi; \
    done > /opt/runtime-packages.txt \
 && cat /opt/runtime-packages.txt

# The app it brings along: MowBite-web.zip of the build context app, of APP_VERSION with APP_SHA256, unpacked and
# checked by host/bundle/app_export.py against docker/min_app and the api of this build; without APP_VERSION none.
ARG APP_VERSION=
ARG APP_SHA256=
RUN --mount=type=bind,from=app,target=/src/app --mount=type=bind,source=host,target=/src/host \
    --mount=type=bind,source=docker/min_app,target=/tmp/min_app \
    if [ -z "$APP_VERSION" ]; then mkdir /opt/mowbite_app; \
    else python3 -I -B /src/host/bundle/app_export.py export /src/app/MowBite-web.zip /opt/mowbite_app \
           --sha256 "${APP_SHA256:?APP_VERSION needs APP_SHA256}" --version "$APP_VERSION" \
           --min-app "$(cat /tmp/min_app)" --api-schema /ws/install/mowbite_api/share/mowbite_api/api.schema.json; \
    fi

FROM ros:${ROS_DISTRO}-ros-core
COPY --from=build /opt/runtime-packages.txt /opt/runtime-packages.txt
RUN apt-get update \
 && apt-get install -y --no-install-recommends $(cat /opt/runtime-packages.txt) \
 && rm -rf /var/lib/apt/lists/*
COPY --from=build /opt/mowbite_planner /opt/mowbite_planner
COPY --from=build /ws/install /ws/install
# Outside the packages, but in the install tree that the bundle takes along.
COPY --from=mpp /opt/mpp /ws/install/mpp
# The safety measurement runs on the image that drives (the driver's normal caps, no fast runs of M8).
COPY tools/yf/measure.py tools/yf/xcore_capture.py /ws/tools/
# For docker exec, which skips the entrypoint: docker exec -it <container> measure --steps ...
RUN printf '#!/bin/bash\nsource /ws/install/setup.bash\nexec python3 /ws/tools/measure.py "$@"\n' > /usr/local/bin/measure \
 && chmod 755 /usr/local/bin/measure
# DDS on the loopback only, without shared memory (fastdds_loopback.xml of mowbite_bringup); the launch halts without it.
COPY src/mowbite_bringup/config/fastdds_loopback.xml /etc/mowbite/fastdds_loopback.xml
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT FASTRTPS_DEFAULT_PROFILES_FILE=/etc/mowbite/fastdds_loopback.xml
# Every library a binary of the image links must be there.
RUN . /ws/install/setup.sh \
 && find /ws/install /opt/mowbite_planner -type f | while read -r f; do \
      if [ "$(head -c 4 "$f" | tail -c 3)" = "ELF" ]; then ldd "$f" | sed -n "s|^\(.*not found\)|$f: \1|p"; fi; \
    done > /tmp/missing \
 && if [ -s /tmp/missing ]; then cat /tmp/missing; exit 1; fi \
 && rm /tmp/missing
# No machine code, so after the library check: a new app leaves the layers above cached.
COPY --from=build /opt/mowbite_app /opt/mowbite_app
ARG SOURCE_COMMIT=unknown
# The oldest MowBite app that handles this OS (label mowbite.min_app for the updater): the value of docker/min_app,
# X.Y.Z or X.Y.Z-beta.N. No image without it; checked here, so that the layers above stay cached.
ARG MIN_APP
RUN --mount=type=bind,source=docker/min_app,target=/tmp/min_app \
    if ! printf '%s' "$MIN_APP" | grep -Eqx '[0-9]+\.[0-9]+\.[0-9]+(-beta\.[0-9]+)?' \
       || [ "$MIN_APP" != "$(cat /tmp/min_app)" ]; then \
      echo "MIN_APP '$MIN_APP' has to be the value of docker/min_app, X.Y.Z or X.Y.Z-beta.N" >&2; exit 1; \
    fi
LABEL org.opencontainers.image.title="mowbite-os" org.opencontainers.image.revision="$SOURCE_COMMIT" \
      mowbite.min_app="$MIN_APP"
# Who wrote data_schema.json in data_dir.
ENV MOWBITE_OS_REVISION=$SOURCE_COMMIT
# ros2 launch catches only SIGINT; as PID 1 it ignores SIGTERM and docker stop would kill it without blade off.
STOPSIGNAL SIGINT
# Healthy while the broker of the OS (MOWBITE_MQTT_HOST, MOWBITE_MQTT_PORT) passes on robot_state/json of the logic;
# after a halt of the launch over data_dir unhealthy, with its MOWBITE_OS_HALT line in Health.Log.
HEALTHCHECK --interval=30s --timeout=10s --start-period=120s --start-interval=5s --retries=3 \
  CMD ["/ws/install/mowbite_bridge/lib/mowbite_bridge/mowbite_health"]
ENTRYPOINT ["/bin/bash", "-c", "source /ws/install/setup.bash && exec \"$@\"", "--"]
