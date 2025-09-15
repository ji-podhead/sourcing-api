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

# --- START DER SERVICES ---
echo "Starting services..."

# Start rosbridge_server in the background
# ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
sleep 10

# Starte rosbridge_server mit korrekten Parametern
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  default_call_service_timeout:=5.0 \
  call_services_in_new_thread:=true \
  send_action_goals_in_new_thread:=true &

# Warte auf erfolgreichen Start
sleep 5

# Zeige Status
ros2 daemon status
ros2 node list
cd api
# Start debugpy
poetry run python3 -m debugpy --listen 0.0.0.0:5678 --wait-for-client /home/sourcingapi/ros2_ws/src/api/src/debug.py &

# Start the FastAPI application
poetry run python3 -m uvicorn main:app --host 0.0.0.0 --port 8000 --app-dir /home/sourcingapi/ros2_ws/src/api/src --reload --reload-dir /home/sourcingapi/ros2_ws/src/api/src
