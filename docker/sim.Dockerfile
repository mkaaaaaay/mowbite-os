# MowBite OS built for the simulation on the PC (amd64), see sim/compose.yaml.
FROM mowbite-os-dev:jazzy
COPY src /ws/src
# DDS on the loopback only, without shared memory (fastdds_loopback.xml of mowbite_bringup); the launch halts without it.
COPY src/mowbite_bringup/config/fastdds_loopback.xml /etc/mowbite/fastdds_loopback.xml
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT FASTRTPS_DEFAULT_PROFILES_FILE=/etc/mowbite/fastdds_loopback.xml
RUN . /opt/ros/jazzy/setup.sh && . /opt/mowbite_planner/setup.sh && cd /ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf /ws/build /ws/log
# ros2 launch catches only SIGINT; as PID 1 it ignores SIGTERM and docker stop would kill it without blade off.
STOPSIGNAL SIGINT
ENTRYPOINT ["/bin/bash", "-c", "source /ws/install/setup.bash && exec \"$@\"", "--"]
