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
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

# Add a delay to allow rosbridge_server to initialize
sleep 1
cd api
# Start debugpy
poetry run python3 -m debugpy --listen 0.0.0.0:5678 --wait-for-client /home/sourcingapi/ros2_ws/src/api/src/debug.py &

# Start the FastAPI application
poetry run python3 -m uvicorn main:app --host 0.0.0.0 --port 8000 --app-dir /home/sourcingapi/ros2_ws/src/api/src
