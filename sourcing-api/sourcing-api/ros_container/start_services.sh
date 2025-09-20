#!/bin/bash


# -- Strikte Fehlerbehandlung --
# Bricht das Skript ab, wenn ein Befehl fehlschlägt
set -e

# Wait for the database to be ready
echo "Waiting for database..."
while ! nc -z localhost 5432; do
  sleep 1
done
echo "Database is ready."
cd /home/sourcingapi/ros2_ws/
# --- START DER SERVICES ---
echo "Starting services..."
# echo "sourcingapi" | sudo -S chown -R sourcingapi:sourcingapi /opt/my_services_package 
# echo "sourcingapi" | sudo -S  rm -r /home/sourcingapi/ros2_ws/src/api/src/ros_api_package

# echo "sourcingapi" | sudo -S  mv -f /opt/my_services_package /home/sourcingapi/ros2_ws/src/api/src/ros_api_package
# echo "sourcingapi" | sudo -S chown -R sourcingapi:sourcingapi /home/sourcingapi/ros2_ws/src/api/src/ros_api_package

# Source the ROS setup script, build the workspace, and source the workspace
source /opt/ros/humble/setup.bash && source /home/sourcingapi/ros2_ws/ros_api_package/install/setup.bash

# echo "--- Building ROS2 workspace ---"
# mkdir -p log
# chmod -R 777 log
# echo "--- Sourcing ROS2 workspace ---"


# Start rosbridge_server in the background
# ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
sleep 2
# Starte rosbridge_server mit korrekten Parametern
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  default_call_service_timeout:=5.0 \
  call_services_in_new_thread:=true \
  send_action_goals_in_new_thread:=true &
ros2 launch foxglove_bridge foxglove_bridge_launch.xml  \
  default_call_service_timeout:=5.0 \
  call_services_in_new_thread:=true \
  send_action_goals_in_new_thread:=true &
# Warte auf erfolgreichen Start
sleep 2
ros2 daemon status
ros2 node list
# Install my_services_package into the poetry environment
# poetry run pip install /opt/my_services_package
cd /home/sourcingapi/ros2_ws/api/src

ls my_services_package &
# Start debugpy
poetry run python3 -m debugpy --listen 0.0.0.0:5678 --wait-for-client /home/sourcingapi/ros2_ws/src/api/src/debug.py &
# Start the FastAPI application
source /opt/ros/humble/setup.bash
poetry run python3 -m uvicorn main:app --host 0.0.0.0 --port 8000 --app-dir /home/sourcingapi/ros2_ws/src/api/src --reload --reload-dir /home/sourcingapi/ros2_ws/src/api/src &


# Wait for all background processes to finish
wait
