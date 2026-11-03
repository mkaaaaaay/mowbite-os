# Image for the safety measurement of a mower with an xCore board: xcore_driver with its minimal launch, measure.py
# and xcore_capture.py; no logic, bridge, planner or simulation. Built from a clean tree with src/ and tools/ at its
# top, e.g. for arm64 on the PC (qemu):
#   docker buildx build --platform linux/arm64 --load --build-arg SOURCE_COMMIT=<commit> \
#     -f docker/measure.Dockerfile -t local/mowbite-measure:<commit>-arm64 <tree>
ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base AS build
ARG BUILD_JOBS=2
COPY src /ws/src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && cd /ws \
 && MAKEFLAGS=-j$BUILD_JOBS colcon build --packages-up-to xcore_driver \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
 && rm -rf /ws/build /ws/log

FROM ros:${ROS_DISTRO}-ros-core
COPY --from=build /ws/install /ws/install
COPY tools/yf/measure.py tools/yf/xcore_capture.py /ws/tools/
# Every library a binary of the image links must be there.
RUN . /ws/install/setup.sh \
 && find /ws/install -type f | while read -r f; do \
      if [ "$(head -c 4 "$f" | tail -c 3)" = "ELF" ]; then ldd "$f" | sed -n "s|^\(.*not found\)|$f: \1|p"; fi; \
    done > /tmp/missing \
 && if [ -s /tmp/missing ]; then cat /tmp/missing; exit 1; fi \
 && rm /tmp/missing
ARG SOURCE_COMMIT=unknown
LABEL org.opencontainers.image.title="mowbite-measure" org.opencontainers.image.revision="$SOURCE_COMMIT"
STOPSIGNAL SIGINT
ENTRYPOINT ["/bin/bash", "-c", "source /ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "launch", "xcore_driver", "measure.launch.py"]
