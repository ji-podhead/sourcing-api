#!/bin/bash

source /opt/ros/humble/setup.bash


ros2 launch foxglove_bridge foxglove_bridge_launch.xml  \
  default_call_service_timeout:=5.0 \
  call_services_in_new_thread:=true \
  send_action_goals_in_new_thread:=true &

sleep 1

poetry run python3 /home/sourcingapi/ros2_ws/api/calibrate.py
