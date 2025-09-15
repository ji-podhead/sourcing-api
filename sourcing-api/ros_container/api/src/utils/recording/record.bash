#!/usr/bin/bash

# The path of the program
PROGRAM=$0

# This is where the data is stored to.
DATA_PATH=$1

# This is the name of the recording.
RECORDING_NAME=$2

# This is where the Ouster configuration is stored
OUSTER_CONFIG=config/ouster.yaml

# Path of the permanent storage
RECORDINGS=/media/recordings


# Simply print the usage.
function usage() {
    echo "Usage":
    echo "  $PROGRAM <DATA PATH> <RECORDING NAME>"
    echo ""
    echo "Paramters:"
    echo "  <DATA PATH>: path where the data is stored"
    echo "  <RECORDING NAME>: set a name for the current recording"
}


# Print an error message, the usage and exit the program
function error_and_usage() {
    print_error_message "$1"
    echo ""
    usage

    exit 1
}


function error() {
    print_error_message "$1"

    exit 1
}


function print_error_message() {
    echo "🔴 Error: $1"
}


function print_success_message() {
    echo "🟢 $1"
}


# Make sure that the data path is provided
if [[ -z $DATA_PATH ]]; then
    error_and_usage "Data path is not set!"
fi


# Make sure that the recording name is available
if [[ -z $RECORDING_NAME ]]; then
    error_and_usage "Recording name is not set!"
fi


# Make sure that there is a camera available
CAMERA_ADDRESS=$(python3 scripts/discover_imaging_source_camera.py --mode single)
if [[ $? -ne 0 ]]; then
    error "Unable to detect a camera! Reason: $CAMERA_ADDRESS"
else
    print_success_message "Camera is available ($CAMERA_ADDRESS)"
fi


# Make sure that the camera is online
./scripts/is_online.bash $CAMERA_ADDRESS "Imaging Source Camera"
if [[ $? -ne 0 ]]; then
    error "Camera is not online. Try to add a route (see README)."
else
    print_success_message "Camera is online"
fi


# Make sure the is a LiDAR available
LIDAR_ADDRESS=$(python3 scripts/ouster_address.py $OUSTER_CONFIG)
if [[ $? -ne 0 ]]; then
    error "Unable to find lidar configuration Reason: $LIDAR_ADDRESS"
else
    print_success_message "LiDAR configuration is available ($LIDAR_ADDRESS)"
fi


# Make sure that the LiDAR is online
echo "Pinging LiDAR at address: $LIDAR_ADDRESS"
./scripts/is_online.bash $LIDAR_ADDRESS Ouster &>2 /dev/null
if [[ $? -ne 0 ]]; then
    error "LiDAR is not online. Try to add a route (see README)"
else
    print_success_message "LiDAR is online"
fi


# Make sure that the LiDAR is online
./scripts/is_online.bash $LIDAR_ADDRESS Ouster > /dev/null
if [[ $? -ne 0 ]]; then
    error "LiDAR is not online. Try to add a route (see README)"
else
    print_success_message "LiDAR is online"
fi


# Create target directory
TARGET=$RECORDINGS/$DATA_PATH
mkdir -p $TARGET

# Create a (more or less) unique name
FILENAME=$(python3 scripts/create_rosbag_name.py $TARGET $RECORDING_NAME)
print_success_message "Saving bagfile as: $FILENAME"

# Record bagfile and check it afterwards
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch launch/record.py destination:=$FILENAME && python3 scripts/checkbag.py $FILENAME config/bag_checker_config.yaml

#rsync -rah --progress $TEMPORARY $TARGET --delete
