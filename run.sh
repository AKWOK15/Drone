#!/bin/bash

set -e

function killSubproc(){
    kill -- -$$   # kills the entire process group
}

trap killSubproc INT TERM EXIT

#sudo chmod g+r /dev/ttyAMA0
#mavlink-routerd -c ~/Drone/config/mavlink-router/main.conf &

#cd oak_d_vins_cpp
#./feature_tracker &
#cd ../

# cd VINS-Fusion/vins_estimator
# ./vins_fusion oak_d.yaml &
# cd ../../

# cd mavlink-udp-proxy
# ./mavlink_udp &
# cd ../

cd ros2_ws
source install/setup.bash
ros2 launch drone_launch drone_launch.py &

wait
