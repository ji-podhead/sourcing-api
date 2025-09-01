#!/bin/bash

# Source the ROS setup script
source /opt/ros/humble/setup.bash
# The following line was removed as it caused an error and is not needed for a Humble environment:
# source /opt/ros/noetic/setup.bash
# Source Humble setup again to ensure it's correctly applied
source /opt/ros/humble/setup.bash
# Launch Xenics camera directly using python3
# Ensure the working directory is set correctly for config file access
# The script itself uses Path.cwd() which should resolve to container root '/'

# Start rosbridge_server in the background using ros2 launch
# Corrected launch file name

#ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

# Add a delay to allow xenics_camera node to initialize
sleep 5

ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
# Start the FastAPI application
# Change directory to where main.py is located before running uvicorn
cd /home/developer/api/src && python3 -m uvicorn main:app --host 0.0.0.0 --port 8000 --reload --reload-dir /home/developer/api/src 

# Wait for background processes to finish (optional, depending on desired container behavior)
# wait
