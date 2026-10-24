#!/bin/bash


# Function to display usage
usage() {
    echo "Usage: $0 [-n NUM_DRONES] [-w WORLD] [-h]"
    echo "  -n NUM_DRONES  Number of drones to start (default: 1)"
    echo "  -w WORLD       Gazebo world to spawn in, a file name from"
    echo "                 \$PX4_PATH/Tools/simulation/gz/worlds without .sdf (default: default)"
    echo "  -h             Run in headless mode (no Gazebo GUI)"
    exit 1
}

# Default values
NUM_DRONES=1
WORLD=default
HEADLESS=0


# Parse command-line arguments
while getopts ":n:w:h" opt; do
    case $opt in
        n) NUM_DRONES="$OPTARG" ;;
        w) WORLD="$OPTARG" ;;
        h) HEADLESS=1 ;;
        *) usage ;;
    esac
done
# very important as it can cause issues
pkill -9 -x px4 || true
pkill -9 -f "gz sim"

# Verify PX4 directory
if [ ! -d "$PX4_PATH" ]; then
    echo "Error: PX4 directory does not exist: $PX4_PATH"
    exit 1
fi

# Convert to an absolute path
PX4_PATH="$(cd "$PX4_PATH" && pwd)"

# Verify the world exists
WORLDS_DIR="$PX4_PATH/Tools/simulation/gz/worlds"
if [ ! -f "$WORLDS_DIR/$WORLD.sdf" ]; then
    echo "Error: world '$WORLD' not found in $WORLDS_DIR"
    echo "Available worlds: $(ls "$WORLDS_DIR" 2>/dev/null | grep '\.sdf$' | sed 's/\.sdf$//' | tr '\n' ' ')"
    exit 1
fi

# Change to PX4-Autopilot directory
cd $PX4_PATH

echo "Headless mode: $HEADLESS"
echo "PX4 path: $PX4_PATH"
echo "World: $WORLD"

for ((i = 1; i <= NUM_DRONES; i++)); do
    # Calculate the model pose (spread out drones in a line)
    POSE_X=$((i - 1))
    POSE_Y=0

    echo "Starting drone $i at position ($POSE_X, $POSE_Y)..."
    konsole --hold -e bash -c '
    cd "$1" || { echo "cd failed: $1"; exec bash; }
    export PX4_SYS_AUTOSTART=4001
    export GZ_TRANSPORT_LOCALHOST_ONLY=1
    export IGN_TRANSPORT_DISABLE_MULTICAST=1
    export IGN_IP=127.0.0.1
    export GZ_IP=127.0.0.1
    export PX4_GZ_MODEL_POSE="$2,$3"
    export PX4_SIM_MODEL=gz_x500_vision
    export PX4_GZ_WORLD="$6"
    if [ "$4" = "1" ]; then export HEADLESS=1; else unset HEADLESS; fi
    exec ./build/px4_sitl_default/bin/px4 -i "$5"
' bash "$PX4_PATH" "$POSE_X" "$POSE_Y" "$HEADLESS" "$i" "$WORLD" &

    # if [ "$HEADLESS" -eq 1 ]; then
    #     konsole -e bash -c "cd $PX4_PATH && PX4_SYS_AUTOSTART=4001 GZ_TRANSPORT_LOCALHOST_ONLY=1 IGN_TRANSPORT_DISABLE_MULTICAST=1 IGN_IP=127.0.0.1 GZ_IP=127.0.0.1 PX4_GZ_MODEL_POSE="$POSE_X,$POSE_Y" PX4_SIM_MODEL=gz_x500_vision HEADLESS=1 ./build/px4_sitl_default/bin/px4 -i ${i}" &
    # else
    #     konsole -e bash -c "cd $PX4_PATH && PX4_SYS_AUTOSTART=4001 GZ_TRANSPORT_LOCALHOST_ONLY=1 IGN_TRANSPORT_DISABLE_MULTICAST=1 IGN_IP=127.0.0.1 GZ_IP=127.0.0.1 PX4_GZ_MODEL_POSE="$POSE_X,$POSE_Y" PX4_SIM_MODEL=gz_x500_vision ./build/px4_sitl_default/bin/px4 -i ${i}" &
    # fi
    if [ "$i" -eq 1 ]; then
        until GZ_IP=127.0.0.1 gz topic -l 2>/dev/null | grep -q "/world/$WORLD/clock"; do sleep 1; done
        sleep 2
    else
        sleep 3
    fi

done




# Start first drone
# konsole -e bash -c "cd ../PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 HEADLESS=1  ./build/px4_sitl_default/bin/px4 -i 1" &
# sleep 5
# konsole  -e bash -c "cd ../PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1" PX4_SIM_MODEL=gz_x500 HEADLESS=1  ./build/px4_sitl_default/bin/px4 -i 2" &

# konsole  -e bash -c "cd ../PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1" PX4_SIM_MODEL=gz_x500 HEADLESS=1  ./build/px4_sitl_default/bin/px4 -i 3"