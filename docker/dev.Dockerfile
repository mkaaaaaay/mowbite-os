# Build and test environment for MowBite OS on the PC (amd64).
FROM ros:jazzy-ros-base
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      libpaho-mqttpp-dev libpaho-mqtt-dev nlohmann-json3-dev \
      mosquitto mosquitto-clients python3-paho-mqtt \
 && rm -rf /var/lib/apt/lists/*
