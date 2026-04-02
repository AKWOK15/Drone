#!/bin/bash

set -e

mavlink-routerd -c ~/drone/config/mavlink-router/main.conf

cd oak_d_vins_cpp
./feature_tracker
cd ../

cd VINS-Fusion/vins_estimator
./vins_fusion oak_d.yaml
cd ../

cd mavlink-udp-proxy
./mavlink_udp
cd ../

cd ros2_ws
source install/setup.bash
ros2 launch drone_launch drone_launch.py
