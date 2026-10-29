# MowBite OS built for the simulation on the PC (amd64), see sim/compose.yaml.
FROM mowbite-os-dev:jazzy
COPY src /ws/src
RUN . /opt/ros/jazzy/setup.sh && . /opt/mowbite_planner/setup.sh && cd /ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf /ws/build /ws/log
# ros2 launch catches only SIGINT; as PID 1 it ignores SIGTERM and docker stop would kill it without blade off.
STOPSIGNAL SIGINT
ENTRYPOINT ["/bin/bash", "-c", "source /ws/install/setup.bash && exec \"$@\"", "--"]
