# MowBite OS for the mower, one image for every platform (launch and configuration choose the platform).
# Built from a clean tree that has src/ and tools/ at its top, e.g. for arm64 on the PC (qemu):
#   docker buildx build --platform linux/arm64 --load --build-context planer=/home/mkay/tron-projekt/planer \
#     --build-arg SOURCE_COMMIT=<commit> --build-arg MIN_APP=$(cat <tree>/docker/min_app) \
#     -f docker/os.Dockerfile -t local/mowbite-os:arm64-<commit> <tree>
# The app comes from its release; --build-context app=<folder with MowBite-web.zip> takes that zip instead,
# --build-arg APP_VERSION= builds without the app.
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

# MediaMTX (MIT), the stream server mowbite_camera starts for WebRTC (media_server): the release binary of the target
# architecture, unchanged, with its licence text. check.yaml reads the version and the sums here for the unit tests.
FROM --platform=$BUILDPLATFORM ros:${ROS_DISTRO}-ros-base AS mediamtx
ARG TARGETARCH
ARG MEDIAMTX_VERSION=1.21.3
ARG MEDIAMTX_SHA256_AMD64=0a2ea7eba96023631efa1bea7804823a9145b043b6646a8a05798d4ac24ed635
ARG MEDIAMTX_SHA256_ARM64=27f038237cb5f77a724f6afc58eca2798f2bc4edf9989895631a80eeb41a4fe2
RUN case "$TARGETARCH" in \
      amd64) sum=$MEDIAMTX_SHA256_AMD64 ;; \
      arm64) sum=$MEDIAMTX_SHA256_ARM64 ;; \
      *) echo "no MediaMTX pinned for $TARGETARCH" >&2; exit 1 ;; \
    esac \
 && release=https://github.com/bluenviron/mediamtx/releases/download/v$MEDIAMTX_VERSION \
 && curl -fsSL -o /tmp/mediamtx.tar.gz "$release/mediamtx_v${MEDIAMTX_VERSION}_linux_$TARGETARCH.tar.gz" \
 && echo "$sum  /tmp/mediamtx.tar.gz" | sha256sum -c - \
 && mkdir -p /opt/mediamtx \
 && tar -xzf /tmp/mediamtx.tar.gz -C /opt/mediamtx mediamtx LICENSE \
 && rm /tmp/mediamtx.tar.gz

# The folder with MowBite-web.zip, replaced by --build-context app=<folder>; empty without one.
FROM scratch AS app

# The app the image brings along, pinned by APP_VERSION and APP_SHA256: MowBite-web.zip of the build context app or,
# when that has none, of the release v$APP_VERSION of the app. host/bundle/app_export.py unpacks it and checks it
# against the sum, docker/min_app and the api of this tree. A stage of its own on the build machine, so a wrong zip
# stops the build at its start. An empty APP_VERSION builds the image without the app.
FROM --platform=$BUILDPLATFORM ros:${ROS_DISTRO}-ros-base AS app_export
ARG APP_VERSION=1.5.0-preview.1
ARG APP_SHA256=1fc88502c9cc4fa5dcd977dcd63cc759a34c8adcf6e00ce3d345435287ffa6fa
RUN --mount=type=bind,from=app,target=/src/app --mount=type=bind,source=host,target=/src/host \
    --mount=type=bind,source=docker/min_app,target=/tmp/min_app \
    --mount=type=bind,source=src/mowbite_api/api.schema.json,target=/tmp/api.schema.json \
    if [ -z "$APP_VERSION" ]; then mkdir /opt/mowbite_app; \
    else \
      zip=/src/app/MowBite-web.zip \
      && if [ ! -f "$zip" ]; then \
           zip=/tmp/MowBite-web.zip \
           && curl -fsSL -o "$zip" \
                "https://github.com/mkaaaaaay/mowbite/releases/download/v$APP_VERSION/MowBite-web.zip"; \
         fi \
      && python3 -I -B /src/host/bundle/app_export.py export "$zip" /opt/mowbite_app \
           --sha256 "${APP_SHA256:?APP_VERSION needs APP_SHA256}" --version "$APP_VERSION" \
           --min-app "$(cat /tmp/min_app)" --api-schema /tmp/api.schema.json; \
    fi

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

FROM ros:${ROS_DISTRO}-ros-core
COPY --from=build /opt/runtime-packages.txt /opt/runtime-packages.txt
RUN apt-get update \
 && apt-get install -y --no-install-recommends $(cat /opt/runtime-packages.txt) \
 && rm -rf /var/lib/apt/lists/*
COPY --from=build /opt/mowbite_planner /opt/mowbite_planner
COPY --from=build /ws/install /ws/install
# Outside the packages, but in the install tree that the bundle takes along.
COPY --from=mpp /opt/mpp /ws/install/mpp
COPY --from=mediamtx /opt/mediamtx /ws/install/mediamtx
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
COPY --from=app_export /opt/mowbite_app /opt/mowbite_app
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
