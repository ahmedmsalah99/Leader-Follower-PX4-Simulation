#!/bin/bash


# Function to display usage
usage() {
    echo "Usage: $0 [-n NUM_DRONES] [-h]"
    echo "  -n NUM_DRONES  Number of drones to start (default: 1)"
    echo "  -h             Run in headless mode (no Gazebo GUI)"
    exit 1
}

# Default values
NUM_DRONES=1
HEADLESS=0


# Parse command-line arguments
while getopts ":n:h" opt; do
    case $opt in
        n) NUM_DRONES="$OPTARG" ;;
        h) HEADLESS=1 ;;
        *) usage ;;
    esac
done
# very important as it can cause issues with sensors
pkill -x px4 || true
sleep 1

# Change to PX4-Autopilot directory
# cd ../PX4-Autopilot
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="${SCRIPT_DIR}/../PX4-Autopilot"
build_path=${src_path}/build/px4_sitl_default
gz sim empty.sdf &
sleep 1
for ((i = 1; i <= NUM_DRONES; i++)); do
    # Calculate the model pose (spread out drones in a line)
    POSE_X=$((i - 1))
    POSE_Y=0
    working_dir="$build_path/instance_$i"
    [ ! -d "$working_dir" ] && mkdir -p "$working_dir"

    pushd "$working_dir" &>/dev/null
	echo "starting instance $n in $(pwd)"
	# $build_path/bin/px4 -i $n -d "$build_path/etc" >out.log 2>err.log &
	
    export PX4_GZ_WORLD="home/stark/stuff/Projects/LeaderFollower/SITL/PX4-Autopilot/Tools/simulation/gz/worlds/default"
    export PX4_GZ_MODEL_PATH="home/stark/stuff/Projects/LeaderFollower/SITL/PX4-Autopilot/Tools/simulation/gz/models/x500_vision"
    echo "Starting drone $i at position ($POSE_X, $POSE_Y)..."
    if [ "$HEADLESS" -eq 1 ]; then
        # konsole -e bash -c "cd ../PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="$POSE_X,$POSE_Y" PX4_SIM_MODEL=gz_x500_vision HEADLESS=1 $build_path/bin/px4 -i ${i} -d "$build_path/etc" >out.log 2>err.log" &
        PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="$POSE_X,$POSE_Y" PX4_SIM_MODEL=gz_x500_vision HEADLESS=1 $build_path/bin/px4 -i ${i} -d "$build_path/etc" >out.log 2>err.log &
    else
        # konsole -e bash -c "cd ../PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="$POSE_X,$POSE_Y" PX4_SIM_MODEL=gz_x500_vision $build_path/bin/px4 -i ${i} -d "$build_path/etc" >out.log 2>err.log" &
        PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="$POSE_X,$POSE_Y" PX4_SIM_MODEL=gz_x500_vision HEADLESS=1 $build_path/bin/px4 -i ${i} -d "$build_path/etc" >out.log 2>err.log &
    fi
    popd &>/dev/null
    if [ "$i" -eq 1 ]; then
        sleep 5
    fi

done




# Start first drone
# konsole -e bash -c "cd ../PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 HEADLESS=1  ./build/px4_sitl_default/bin/px4 -i 1" &
# sleep 5
# konsole  -e bash -c "cd ../PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1" PX4_SIM_MODEL=gz_x500 HEADLESS=1  ./build/px4_sitl_default/bin/px4 -i 2" &

# konsole  -e bash -c "cd ../PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1" PX4_SIM_MODEL=gz_x500 HEADLESS=1  ./build/px4_sitl_default/bin/px4 -i 3"

