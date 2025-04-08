
# Function to display usage
usage() {
    echo "Usage: $0 [-n NUM_DRONES]"
    echo "  -n NUM_DRONES  Number of drones to start (default: 1)"
    exit 1
}

# Default values
NUM_DRONES=1



# Parse command-line arguments
while getopts ":n:h" opt; do
    case $opt in
        n) NUM_DRONES="$OPTARG" ;;
        *) usage ;;
    esac
done

pkill -9 -f "swarm_manager_node"


source ../../ros2_ws/install/setup.sh

ros2 run swarm_manager swarm_manager_node &
ros2 launch drones_intracom_py swarm_launch.py num_drones:=${NUM_DRONES} &
sleep 5
ros2 launch drone_controller launch.py num_drones:=${NUM_DRONES}