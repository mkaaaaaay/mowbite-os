# MowBite OS built for the simulation on the PC (amd64), see sim/compose.yaml.
FROM mowbite-os-dev:jazzy
COPY src /ws/src
RUN . /opt/ros/jazzy/setup.sh && . /opt/mowbite_planner/setup.sh && cd /ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf /ws/build /ws/log
ENTRYPOINT ["/bin/bash", "-c", "source /ws/install/setup.bash && exec \"$@\"", "--"]
