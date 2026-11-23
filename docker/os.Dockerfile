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
ARG APP_VERSION=1.5.0-preview.16
ARG APP_SHA256=75c892a5c4e8390d1d2f95fc49151188d3576ee1b0e7f08da6d7fc81f45d65fe
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

# The compiler for an arm64 image built on an amd64 machine: the programs of Ubuntu's cross GCC and binutils with the
# libraries they load, under /cross. They belong to the build machine, so inside the emulated build stage the kernel
# runs them directly. Only programs come from here: where the cross compiler looks for its headers, start files and
# libraries, links lead to those of the image, the ones the emulated compiler uses. Likewise the Python interpreter
# with its standard library under /cross/opt/cross-python, for the rosidl code generators of the image. Empty otherwise.
FROM --platform=$BUILDPLATFORM ros:${ROS_DISTRO}-ros-base AS cross
ARG BUILDARCH
ARG TARGETARCH
RUN mkdir /cross \
 && if [ "$BUILDARCH" = amd64 ] && [ "$TARGETARCH" = arm64 ]; then \
      apt-get update \
      && apt-get install -y --no-install-recommends g++-aarch64-linux-gnu \
      && v=$(aarch64-linux-gnu-gcc -dumpversion) \
      && py=$(readlink -f /usr/bin/python3) \
      && mkdir -p /cross/usr/bin /cross/usr/lib/x86_64-linux-gnu /cross/usr/lib64 /cross/usr/libexec \
           /cross/usr/aarch64-linux-gnu/include/c++ /cross/usr/include/c++/$v /cross/opt/cross-python/bin \
           /cross/opt/cross-python/lib \
      && cp -a /usr/bin/aarch64-linux-gnu-* /cross/usr/bin/ \
      && cp -a /usr/libexec/gcc-cross /cross/usr/libexec/ \
      && cp -a /usr/aarch64-linux-gnu/bin /cross/usr/aarch64-linux-gnu/ \
      && cp -L /lib64/ld-linux-x86-64.so.2 /cross/usr/lib64/ \
      && cp "$py" /cross/opt/cross-python/bin/ \
      && ln -s "${py##*/}" /cross/opt/cross-python/bin/python3 \
      && cp -a /usr/lib/"${py##*/}" /cross/opt/cross-python/lib/ \
      && rm -rf /cross/opt/cross-python/lib/"${py##*/}"/config-* \
      && for f in /usr/bin/aarch64-linux-gnu-* $(find /usr/libexec/gcc-cross -type f) "$py" \
             /usr/lib/"${py##*/}"/lib-dynload/*.so; do \
           ldd "$f" 2> /dev/null | awk '$3 ~ "^/" { print $3 }'; \
         done | sort -u | xargs cp -L -t /cross/usr/lib/x86_64-linux-gnu/ \
      && ln -s gcc /cross/usr/lib/gcc-cross \
      && ln -s /usr/include/c++/$v /cross/usr/aarch64-linux-gnu/include/c++/$v \
      && ln -s ../../aarch64-linux-gnu/c++/$v /cross/usr/include/c++/$v/aarch64-linux-gnu; \
    fi

FROM ros:${ROS_DISTRO}-ros-base AS build
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      libpaho-mqttpp-dev libpaho-mqtt-dev nlohmann-json3-dev libssl-dev libeigen3-dev zlib1g-dev libasound2-dev \
      libturbojpeg0-dev libcurl4-openssl-dev git ccache \
 && rm -rf /var/lib/apt/lists/*

# No Rust bindings: rosidl generates for every package in this index group, and nothing here uses Rust.
RUN rm -f /opt/ros/$ROS_DISTRO/share/ament_index/resource_index/rosidl_generator_packages/rosidl_generator_rs

# On an amd64 build machine the stage cross takes over, and the build stops if a version differs from the emulated
# one: its compiler and binutils replace those of the image under the same names, and its Python runs the rosidl
# code generators: docker/rosidl_cross.py takes the place of each generator script (the generators and typesupports
# of the ament index) and of the module rosidl_adapter, and runs the original, renamed to *.native.
RUN --mount=type=bind,from=cross,source=/cross,target=/cross \
    --mount=type=bind,source=docker/rosidl_cross.py,target=/tmp/rosidl_cross.py \
    if [ -d /cross/usr ]; then \
      emulated="$(c++ --version | head -n 1), $(ld --version | head -n 1)" \
      && cp -a /cross/usr/. /usr/ \
      && if [ ! -e /lib64 ]; then ln -s usr/lib64 /lib64; fi \
      && direct="$(c++ --version | head -n 1), $(ld --version | head -n 1)" \
      && echo "emulated: $emulated" && echo "direct:   $direct" \
      && if [ "$emulated" != "$direct" ]; then echo "the compiler of the stage cross differs" >&2; exit 1; fi \
      && cp -a /cross/opt/cross-python /opt/ \
      && v='import sys; print(tuple(sys.version_info))' \
      && emulated=$(python3 -c "$v") && direct=$(/opt/cross-python/bin/python3 -I -S -c "$v") \
      && echo "python emulated: $emulated, direct: $direct" \
      && if [ "$emulated" != "$direct" ]; then echo "the Python of the stage cross differs" >&2; exit 1; fi \
      && index=/opt/ros/$ROS_DISTRO/share/ament_index/resource_index lib=/opt/ros/$ROS_DISTRO/lib \
      && for f in $(find $index/rosidl_generator_packages -mindepth 1 -printf "$lib/%f/%f\n" \
                    && find $index/rosidl_typesupport_c $index/rosidl_typesupport_cpp -maxdepth 1 \
                         -printf "$lib/%f/%f\n") \
           $lib/python3*/site-packages/rosidl_adapter/__main__.py; do \
           cp -a "$f" "$f.native" && cat /tmp/rosidl_cross.py > "$f" && echo "direct: $f" || exit 1; \
         done; \
    fi

# Fixed hash seed: the generators collect includes in sets, so their output and the compiler cache stay stable.
ENV PYTHONHASHSEED=0

# Compilers at once: BUILD_WORKERS packages with BUILD_JOBS jobs each.
ARG BUILD_WORKERS=2
ARG BUILD_JOBS=2

# The MowBite Planner at a fixed commit of its branch os-pin: its two ROS 2 packages and the two of MowBite OS
# (os/: the node and its service) with its licence text, the source removed after.
ARG PLANNER_COMMIT=5d89e65eb5a2169922bc07b2cc4f85b0ee04badf
RUN --mount=type=bind,from=planer,target=/src/planer \
    git init -q /tmp/planner && cd /tmp/planner \
 && git -c safe.directory='*' fetch -q --depth 1 /src/planer "$PLANNER_COMMIT" \
 && git checkout -q FETCH_HEAD \
 && test "$(git rev-parse HEAD)" = "$PLANNER_COMMIT" \
 && commit_time=$(git show -s --format=%ct HEAD) \
 && . /opt/ros/$ROS_DISTRO/setup.sh \
 && MAKEFLAGS=-j$BUILD_JOBS colcon --log-base /tmp/planner_log build \
      --base-paths ros2/mowbite_planner_interfaces ros2/mowbite_planner \
        os/mowbite_planner_os_interfaces os/mowbite_planner_os \
      --build-base /tmp/planner_build --install-base /opt/mowbite_planner \
      --parallel-workers $BUILD_WORKERS --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
 && printf '{"commit": "%s", "serial": %s}\n' "$PLANNER_COMMIT" "$commit_time" > /opt/mowbite_planner/source.json \
 && cp LICENSE /opt/mowbite_planner/LICENSE \
 && rm -rf /tmp/planner /tmp/planner_build /tmp/planner_log

# Every package but the simulation; PACKAGES narrows it (colcon selection, e.g. "--packages-up-to xcore_driver").
ARG PACKAGES="--packages-skip mowbite_sim"
# Headers of the media library, only for the check of tron_vision against its declarations; empty on amd64.
COPY --from=mpp /opt/mpp-inc /opt/mpp-inc
ENV TRON_VISION_MPP_INCLUDE=/opt/mpp-inc
COPY src /ws/src
# The FILES_VERSION of the pinned planner has to be the one data_schema.json of mowbite_bridge expects. The compiler
# cache stays on the builder between builds: under qemu a build that changes little then compiles little.
RUN --mount=type=cache,target=/ccache \
    . /opt/ros/$ROS_DISTRO/setup.sh && . /opt/mowbite_planner/setup.sh && cd /ws \
 && export CCACHE_DIR=/ccache CCACHE_COMPILERCHECK=content \
 && MAKEFLAGS=-j$BUILD_JOBS colcon build $PACKAGES --parallel-workers $BUILD_WORKERS \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
 && ccache --show-stats \
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
        libcurl-dev) echo libcurl4t64 ;; \
        zlib) echo zlib1g ;; \
        libzstd-dev) echo libzstd1 ;; \
        python3-shapely|python3-numpy|python3-paho-mqtt|python3-yaml|libturbojpeg) echo $k ;; \
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
# Outside the packages, but in the install tree that the bundle takes along; before it, so that an update of the
# packages leaves these layers as they are.
COPY --from=mpp /opt/mpp /ws/install/mpp
COPY --from=mediamtx /opt/mediamtx /ws/install/mediamtx
COPY --from=build /opt/mowbite_planner /opt/mowbite_planner
COPY --from=build /ws/install /ws/install
# The safety measurement runs on the image that drives (the driver's normal caps, no fast runs of M8).
COPY tools/yf/measure.py tools/yf/xcore_capture.py /ws/tools/
# For docker exec, which skips the entrypoint: docker exec -it <container> measure --steps ...
RUN printf '#!/bin/bash\nsource /ws/install/setup.bash\nexec python3 /ws/tools/measure.py "$@"\n' > /usr/local/bin/measure \
 && chmod 755 /usr/local/bin/measure
# The way back to OpenMower (mowbite_bringup migrate.py): docker exec <container> mowbite-migrate export <folder>
RUN printf '#!/bin/bash\nsource /ws/install/setup.bash\nexec python3 /ws/install/mowbite_bringup/share/mowbite_bringup/launch/migrate.py "$@"\n' \
      > /usr/local/bin/mowbite-migrate \
 && chmod 755 /usr/local/bin/mowbite-migrate
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
# The stack of a mower that runs the OS with Docker, for its updater: docker run --rm --entrypoint cat <image> <file>.
COPY docker/mower/nx60i.compose.yaml docker/mower/mosquitto.conf docker/mower/env.example /usr/share/mowbite-os/mower/
ARG SOURCE_COMMIT=unknown
# The oldest MowBite app that handles this OS (label mowbite.min_app for the updater): the value of docker/min_app,
# X.Y.Z or X.Y.Z-beta.N. No image without it; checked here, so that the layers above stay cached.
ARG MIN_APP
RUN --mount=type=bind,source=docker/min_app,target=/tmp/min_app \
    if ! printf '%s' "$MIN_APP" | grep -Eqx '[0-9]+\.[0-9]+\.[0-9]+(-beta\.[0-9]+)?' \
       || [ "$MIN_APP" != "$(cat /tmp/min_app)" ]; then \
      echo "MIN_APP '$MIN_APP' has to be the value of docker/min_app, X.Y.Z or X.Y.Z-beta.N" >&2; exit 1; \
    fi
# The firmware major version the xCore driver expects (label mowbite.xcore.firmware.major, read by the updater before
# it stops OpenMower): expected_major_version of src/xcore_driver/config/nx60i.yaml, checked here.
ARG XCORE_FIRMWARE_MAJOR
RUN --mount=type=bind,source=src/xcore_driver/config/nx60i.yaml,target=/tmp/nx60i.yaml \
    expected=$(sed -n 's/^ *expected_major_version: *\([0-9][0-9]*\) *$/\1/p' /tmp/nx60i.yaml) \
    && if [ -z "$expected" ] || [ "$XCORE_FIRMWARE_MAJOR" != "$expected" ]; then \
      echo "XCORE_FIRMWARE_MAJOR '$XCORE_FIRMWARE_MAJOR' has to be expected_major_version of nx60i.yaml" >&2; exit 1; \
    fi
LABEL org.opencontainers.image.title="mowbite-os" org.opencontainers.image.revision="$SOURCE_COMMIT" \
      mowbite.min_app="$MIN_APP" mowbite.xcore.firmware.major="$XCORE_FIRMWARE_MAJOR"
# Who wrote data_schema.json in data_dir; the version of system.info.
ENV MOWBITE_OS_REVISION=$SOURCE_COMMIT
# system.info.update_path: the image is updated as a whole by hand.
ENV MOWBITE_UPDATE_PATH=compose
# ros2 launch catches only SIGINT; as PID 1 it ignores SIGTERM and docker stop would kill it without blade off.
STOPSIGNAL SIGINT
# Healthy while the broker of the OS (MOWBITE_MQTT_HOST, MOWBITE_MQTT_PORT) passes on robot_state/json of the logic;
# after a halt of the launch over data_dir unhealthy, with its MOWBITE_OS_HALT line in Health.Log.
HEALTHCHECK --interval=30s --timeout=10s --start-period=120s --start-interval=5s --retries=3 \
  CMD ["/ws/install/mowbite_bridge/lib/mowbite_bridge/mowbite_health"]
ENTRYPOINT ["/bin/bash", "-c", "source /ws/install/setup.bash && exec \"$@\"", "--"]
