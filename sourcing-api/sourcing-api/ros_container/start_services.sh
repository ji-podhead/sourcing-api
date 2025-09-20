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
# Source the ROS setup script
source /opt/ros/humble/setup.bash
# source /home/sourcingapi/ros2_ws/install/setup.bash

# --- START DER SERVICES ---
echo "Starting services..."

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
cd api
# Install my_services_package into the poetry environment
poetry run pip install /opt/my_services_package

# Start debugpy
poetry run python3 -m debugpy --listen 0.0.0.0:5678 --wait-for-client /home/sourcingapi/ros2_ws/src/api/src/debug.py &
# Start the FastAPI application
source /opt/ros/humble/setup.bash
poetry run PYTHONPATH=$PYTHONPATH:/opt/my_services_package/install/my_services_package/lib/python3.10/site-packages python3 -m uvicorn main:app --host 0.0.0.0 --port 8000 --app-dir /home/sourcingapi/ros2_ws/src/api/src --reload --reload-dir /home/sourcingapi/ros2_ws/src/api/src &

# Start the gige_camera_node in the background
# poetry run python3 /home/sourcingapi/services/gige_camera_node.py worker1 &

# Wait for all background processes to finish
wait
