#!/bin/bash

# Update and install necessary packages
apt-get update
apt-get install -y python3-pip

# Install Python dependencies
pip install -r requirements.txt

# Build the ROS workspace
source /opt/ros/humble/setup.bash
colcon build
