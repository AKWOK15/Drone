#!/bin/bash
read -p "Control node to run: " control_node
timestamp="$(date +'%m-%d_%H:%M')"
echo "timestamp: ${timestamp}"
set -e


# Find first avaiable trial number
for i in $(seq 1 50);
do
	# & mean files descriptor instead of file name
	# > /dev/null stdout goes to /dev/null
	# 2>&1 stderr follows stout
	if ! compgen -G "./data/${control_node}_${i}*" > /dev/null 2>&1; then
		potential_directory="${control_node}_${i}*"
		echo "Found directory for next avaiable trial number: $potential_directory"
		break
	fi
done

if [[ ! -v potential_directory ]]; then
	echo "Could not get a directory to store data"
	exit 1
fi
final_directory="${HOME}/Drone/data/${potential_directory:0:-1}_${timestamp}"
echo "final_directory: ${final_directory}"
mkdir -p ${final_directory}

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
ros2 launch drone_launch drone_launch.py control_node_param:="${control_node}" data_subfolder_param:="${final_directory}" & 

wait
