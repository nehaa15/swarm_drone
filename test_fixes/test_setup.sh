#!/bin/bash

# Simple test script for flying_single_drone_ws

echo "=" * 60
echo "Flying Single Drone Workspace - Quick Test"
echo "=" * 60

# Check if workspace was built correctly
echo "Checking workspace setup..."

if [ -d "/home/krish/flying_single_drone_ws/install" ]; then
    echo "✓ Workspace built successfully"
else
    echo "✗ Workspace build failed"
    exit 1
fi

# Source the workspace
source /home/krish/flying_single_drone_ws/install/setup.bash

echo "✓ Workspace sourced"

# Check if our nodes are available
echo "Checking available nodes..."

if ros2 pkg list | grep -q "single_drone_flight"; then
    echo "✓ single_drone_flight package found"
else
    echo "✗ single_drone_flight package not found"
    exit 1
fi

echo "Available executables:"
ros2 pkg executables single_drone_flight

echo
echo "=" * 60
echo "Setup complete! You can now run:"
echo "1. ros2 launch single_drone_flight single_drone_flight.launch.py"
echo "   OR"
echo "2. ros2 run single_drone_flight simulation_launcher"
echo "   then: ros2 run single_drone_flight single_drone_control"
echo "=" * 60
