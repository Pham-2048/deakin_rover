#!/bin/bash

sudo ip link set can1 down
sudo ip link set can1 up type can bitrate 1000000

source install/setup.bash

ros2 launch rover_bringup bringup.launch.xml
