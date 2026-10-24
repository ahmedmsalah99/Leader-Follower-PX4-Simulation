#!/bin/bash

# Check if number of drones argument is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <number_of_drones>"
    exit 1
fi

NUM_DRONES=$1

# Function to send command to a specific drone
send_command_to_drone() {
    local drone_num=$1
    local command=$2
    local value=$3
    ros2 topic pub -1 /drone_${drone_num}/command/${command} std_msgs/msg/Bool "data: ${value}"
}

# Function to send command to all drones
send_command_to_all() {
    local command=$1
    local value=$2
    for i in $(seq 1 $NUM_DRONES); do
        send_command_to_drone $i "$command" "$value"
    done
}

# Main command loop
while true; do
    echo -e "\nEnter command (arm/launch/full_launch/return) [drone_number]"
    echo "Examples:"
    echo "  arm 2     - arms drone 2"
    echo "  launch    - launches all drones"
    echo "  full_launch - launches all drones and arms them"
    echo "  return 1  - returns drone 1"
    echo "  exit      - exits the program"
    echo -n "> "
    
    read input
    
    # Split input into command and drone number
    command=$(echo $input | cut -d' ' -f1)
    drone_num=$(echo $input | cut -d' ' -f2)
    
    case $command in
        "exit")
            echo "Exiting..."
            exit 0
            ;;
        "arm"|"launch"|"full_launch"|"return")
            value="true"
            if [ "$drone_num" != "$command" ]; then
                if [ "$drone_num" -gt 0 ] && [ "$drone_num" -le "$NUM_DRONES" ]; then
                    echo "Sending $command to drone $drone_num"
                    send_command_to_drone $drone_num "$command" "$value"
                else
                    echo "Error: Drone number must be between 1 and $NUM_DRONES"
                fi
            else
                echo "Sending $command to all drones"
                send_command_to_all "$command" "$value"
            fi
            ;;
        *)
            echo "Invalid command. Use arm, launch, return, or exit"
            ;;
    esac
done


