#!/bin/bash
set -e

# Sourced zuerst die ROS 1 Umgebung
source "/opt/ros/noetic/setup.bash"

# Sourced danach die ROS 2 Umgebung
source "/opt/ros/foxy/setup.bash"

echo "ROS 1 & 2 environments sourced. Starting dynamic_bridge..."

# Führt den Befehl aus, der an den Container übergeben wird (aus docker-compose.yaml)
exec "$@"
