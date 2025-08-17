#!/bin/bash

# PX4 Drone Flight Troubleshooting Script
# This script addresses common issues with the single drone flight system

echo "============================================================="
echo "PX4 Single Drone Flight - Troubleshooting & Fix Script"
echo "============================================================="

# Function to check if a process is running
check_process() {
    if pgrep -f "$1" > /dev/null; then
        echo "✓ $1 is running"
        return 0
    else
        echo "✗ $1 is not running"
        return 1
    fi
}

# Function to kill existing processes
cleanup_processes() {
    echo "Cleaning up existing processes..."
    pkill -f "px4"
    pkill -f "gazebo"
    pkill -f "MicroXRCE"
    sleep 2
    echo "✓ Cleanup complete"
}

# Function to wait for PX4 stabilization
wait_for_px4_stabilization() {
    echo "Waiting for PX4 velocity estimation to stabilize..."
    echo "This addresses the 'vertical velocity unstable' error..."
    
    local counter=0
    local max_wait=60  # 60 seconds
    
    while [ $counter -lt $max_wait ]; do
        if check_process "px4" > /dev/null; then
            printf "."
            sleep 1
            counter=$((counter + 1))
        else
            echo ""
            echo "ERROR: PX4 process not found!"
            return 1
        fi
    done
    
    echo ""
    echo "✓ PX4 stabilization period complete"
    return 0
}

# Main troubleshooting flow
main() {
    echo "1. Checking current system state..."
    
    # Check if anything is already running
    local px4_running=$(pgrep -f "px4" | wc -l)
    local gazebo_running=$(pgrep -f "gazebo" | wc -l)
    local micro_running=$(pgrep -f "MicroXRCE" | wc -l)
    
    echo "Current process status:"
    echo "  PX4 processes: $px4_running"
    echo "  Gazebo processes: $gazebo_running"
    echo "  MicroXRCE processes: $micro_running"
    
    if [ $px4_running -gt 0 ] || [ $gazebo_running -gt 0 ] || [ $micro_running -gt 0 ]; then
        echo ""
        echo "Existing processes detected. Cleaning up first..."
        cleanup_processes
    fi
    
    echo ""
    echo "2. Starting fresh simulation environment..."
    
    # Start MicroXRCE-DDS Agent
    echo "Starting MicroXRCE-DDS Agent..."
    gnome-terminal --tab --title="MicroXRCE Agent" -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash" &
    sleep 3
    
    # Start PX4 SITL
    echo "Starting PX4 SITL with Gazebo..."
    gnome-terminal --tab --title="PX4 SITL" -- bash -c "cd ~/PX4-Autopilot && make px4_sitl gazebo-classic_iris; exec bash" &
    
    echo "Waiting for simulation to boot up..."
    sleep 10
    
    # Wait for processes to be running
    local retry_count=0
    while [ $retry_count -lt 10 ]; do
        if check_process "px4" && check_process "MicroXRCE"; then
            break
        fi
        echo "Waiting for processes to start..."
        sleep 2
        retry_count=$((retry_count + 1))
    done
    
    if ! check_process "px4" || ! check_process "MicroXRCE"; then
        echo "ERROR: Failed to start required processes!"
        echo "Please check:"
        echo "  - PX4-Autopilot is installed in ~/PX4-Autopilot"
        echo "  - MicroXRCEAgent is installed and in PATH"
        exit 1
    fi
    
    echo ""
    echo "3. Allowing PX4 to stabilize (fixes velocity estimation errors)..."
    wait_for_px4_stabilization
    
    echo ""
    echo "4. System ready for flight control!"
    echo ""
    echo "============================================================="
    echo "FIXES APPLIED:"
    echo "✓ Increased startup delay to allow velocity estimation stabilization"
    echo "✓ Modified state machine to prevent rapid state transitions"
    echo "✓ Added proper timing controls for arming sequence"
    echo "✓ Reduced command frequency to avoid overwhelming PX4"
    echo "============================================================="
    echo ""
    echo "You can now run the flight control with:"
    echo "  cd /home/krish/flying_single_drone_ws"
    echo "  source install/setup.bash"
    echo "  ros2 run single_drone_flight single_drone_control"
    echo ""
    echo "Or use the launch file (recommended):"
    echo "  ros2 launch single_drone_flight single_drone_flight.launch.py"
    echo ""
    echo "The system will now:"
    echo "  1. Wait 5 seconds after startup before attempting to arm"
    echo "  2. Send commands at a reasonable frequency (not spam)"
    echo "  3. Handle velocity estimation errors gracefully"
    echo "  4. Provide proper state transition logic"
}

# Run the main function
main "$@"
