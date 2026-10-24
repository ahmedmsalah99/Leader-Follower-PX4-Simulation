#!/bin/bash

# Check if number of drones argument is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <number_of_drones>"
    exit 1
fi

NUM_DRONES=$1
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "${SCRIPT_DIR}/../../ros2_ws/install/setup.sh"

# Function to send command to a specific drone
send_command_to_drone() {
    local drone_num=$1
    local command=$2
    ros2 topic pub -1 /drone_${drone_num}/command/${command} std_msgs/msg/Bool "data: true" > /dev/null
}

# Function to send command to all drones in parallel
send_command_to_all() {
    local command=$1
    for i in $(seq 1 $NUM_DRONES); do
        send_command_to_drone $i "$command" &
    done
    wait
}

# Upload a mission plan to the current leader and start it
run_leader_mission() {
    local plan=$1
    local leader_id
    leader_id=$(timeout 5 ros2 topic echo --once --field leader_id /global/swarm_status 2>/dev/null | head -n 1)
    if [ -z "$leader_id" ] || [ "$leader_id" -eq 0 ]; then
        echo "Error: no leader elected yet"
        return
    fi
    echo "Leader is drone $((leader_id - 1)) (system id $leader_id)"
    if [ -n "$plan" ]; then
        python3 "${SCRIPT_DIR}/mission_upload.py" "$leader_id" "$plan"
    else
        python3 "${SCRIPT_DIR}/mission_upload.py" "$leader_id"
    fi
}

# Main command loop
while true; do
    echo -e "\nEnter command (takeoff/land/return) [drone_number], mission [plan_file], or exit"
    echo "Examples:"
    echo "  takeoff     - arms all drones and takes off"
    echo "  land 2      - lands drone 2"
    echo "  return 1    - returns drone 1 to launch"
    echo "  mission     - flies SITL/missions/simple_mission.plan with the leader"
    echo "  mission my.plan - flies my.plan with the leader"
    echo -n "> "

    read command arg

    case $command in
        "exit")
            echo "Exiting..."
            exit 0
            ;;
        "takeoff"|"land"|"return")
            if [ -n "$arg" ]; then
                if [ "$arg" -gt 0 ] 2>/dev/null && [ "$arg" -le "$NUM_DRONES" ]; then
                    echo "Sending $command to drone $arg"
                    send_command_to_drone $arg "$command"
                else
                    echo "Error: Drone number must be between 1 and $NUM_DRONES"
                fi
            else
                echo "Sending $command to all drones"
                send_command_to_all "$command"
            fi
            ;;
        "mission")
            run_leader_mission "$arg"
            ;;
        *)
            echo "Invalid command. Use takeoff, land, return, mission, or exit"
            ;;
    esac
done
