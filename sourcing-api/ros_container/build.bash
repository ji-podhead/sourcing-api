#!/usr/bin/bash

SOURCE=src
OUSTER=$SOURCE/ouster
ARAVIS2=$SOURCE/camera_aravis2

source /opt/ros/humble/setup.bash
colcon build --paths $OUSTER/* $ARAVIS2/*
