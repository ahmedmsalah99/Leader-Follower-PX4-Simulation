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
    local plan="${1/#\~/$HOME}"  # read doesn't expand ~
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

print_help() {
    echo ""
    echo "Commands (leave out the drone number to command ALL $NUM_DRONES drones):"
    echo "  takeoff [N]          arm and take off"
    echo "  land [N]             land"
    echo "  return [N]           return to launch"
    echo "  mission [plan_file]  fly a QGroundControl .plan with the current leader"
    echo "                       (default: SITL/missions/simple_mission.plan)"
    echo "  help                 show this help"
    echo "  exit                 quit"
    echo "Examples:"
    echo "  takeoff              all drones take off"
    echo "  takeoff 2            only drone 2 takes off"
    echo "  land                 all drones land"
    echo "  mission              the leader flies simple_mission.plan, the others follow"
}

print_help

# Main command loop
while true; do
    echo -n "> "

    read command arg || exit 0

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
        "help")
            print_help
            ;;
        "")
            ;;
        *)
            echo "Invalid command '$command'"
            print_help
            ;;
    esac
done
