#!/bin/bash

# Function to display usage
usage() {
    echo "Usage: $0 [-n NUM_DRONES] [-a ALTITUDE] [-t TIMEOUT]"
    echo "  -n NUM_DRONES  Number of drones to hover (default: 1)"
    echo "  -a ALTITUDE    Hover altitude above home in meters (default: 2.5)"
    echo "  -t TIMEOUT     Give up after this many seconds (default: 60)"
    exit 1
}

# Default values
NUM_DRONES=1
ALTITUDE=2.5
TIMEOUT=60

# Parse command-line arguments
while getopts ":n:a:t:h" opt; do
    case $opt in
        n) NUM_DRONES="$OPTARG" ;;
        a) ALTITUDE="$OPTARG" ;;
        t) TIMEOUT="$OPTARG" ;;
        *) usage ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "${SCRIPT_DIR}/../../ros2_ws/install/setup.sh"

python3 "${SCRIPT_DIR}/hover.py" -n "${NUM_DRONES}" -a "${ALTITUDE}" -t "${TIMEOUT}"
