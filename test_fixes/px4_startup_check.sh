#!/bin/bash

echo "PX4 Startup Check Script"
echo "========================"

# Check if PX4 simulation is running
PX4_RUNNING=$(pgrep -f "px4" | wc -l)
GAZEBO_RUNNING=$(pgrep -f "gazebo" | wc -l)

echo "PX4 processes: $PX4_RUNNING"
echo "Gazebo processes: $GAZEBO_RUNNING"

if [ $PX4_RUNNING -eq 0 ]; then
    echo "ERROR: PX4 is not running!"
    echo "Please start PX4 first with: cd ~/PX4-Autopilot && make px4_sitl gazebo-classic_iris"
    exit 1
fi

if [ $GAZEBO_RUNNING -eq 0 ]; then
    echo "ERROR: Gazebo is not running!"
    echo "Please start Gazebo with PX4 simulation"
    exit 1
fi

echo "Waiting for PX4 to stabilize (velocity estimation)..."
echo "This may take 30-60 seconds..."

# Wait for PX4 to report ready status
TIMEOUT=120  # 2 minutes timeout
COUNTER=0

while [ $COUNTER -lt $TIMEOUT ]; do
    # Check if PX4 reports "Ready for takeoff"
    if pgrep -f "px4" > /dev/null; then
        echo -n "."
        sleep 1
        COUNTER=$((COUNTER + 1))
        
        # After 30 seconds, assume it's ready
        if [ $COUNTER -eq 30 ]; then
            echo ""
            echo "PX4 should be stabilized now."
            echo "If you still see velocity estimation errors, wait a bit longer."
            break
        fi
    else
        echo "ERROR: PX4 process died!"
        exit 1
    fi
done

echo ""
echo "You can now run: ros2 run single_drone_flight single_drone_control"
echo "Or use the launch file: ros2 launch single_drone_flight single_drone_flight.launch.py"
