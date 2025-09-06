#!/bin/bash

# Wait for the database to be ready
echo "Waiting for database..."
while ! nc -z localhost 5432; do
  sleep 1
done
echo "Database is ready."

# Source the ROS setup script
source /opt/ros/humble/setup.bash

# 1. Install rosbridge_server (if not already installed)
sudo apt-get update && sudo apt-get install -y ros-humble-rosbridge-server

# 2. Start rosbridge_server in the background
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
pip install rosros roslibpy
# Add a delay to allow rosbridge_server to initialize
sleep 10

# Start the FastAPI application
# filepath: ros_container/start_services.sh
python3 -m uvicorn main:app --host 0.0.0.0 --port 8000 --reload --app-dir /home/sourcingapi/ros2_ws/src/api/src
