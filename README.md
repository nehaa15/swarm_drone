# Single Drone Flight Workspace

This workspace contains a simplified ROS2 package for autonomously flying a single drone to 5 meters altitude using PX4 offboard control.

## Overview

The `single_drone_flight` package demonstrates basic autonomous flight capabilities:
- Automatic arming sequence
- Autonomous takeoff
- Climbing to target altitude (5 meters)
- Position holding at target altitude

The package provides both **Python** and **C++** implementations of the flight controller, allowing you to choose based on your preference or requirements.

## Prerequisites

Before running this package, ensure you have:

1. **ROS2 Humble** installed
2. **PX4 Autopilot** installed in `~/PX4-Autopilot`
3. **MicroXRCE-DDS Agent** installed and available in PATH
4. Required Python dependencies:
   ```bash
   pip3 install --user -U empy pyros-genmsg setuptools
   pip3 install kconfiglib
   pip install --user jsonschema
   pip install --user jinja2
   ```

## Installation

1. Clone or create this workspace:
   ```bash
   cd /home/krish/flying_single_drone_ws
   ```

2. Source ROS2:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. Build the workspace:
   ```bash
   colcon build
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

### Method 1: Using Launch Files (Recommended)

#### Python Controller (Default)
Run the complete system with Python controller:
```bash
ros2 launch single_drone_flight single_drone_flight.launch.py
```

#### C++ Controller
Run the complete system with C++ controller:
```bash
ros2 launch single_drone_flight single_drone_flight_cpp.launch.py
```

Both launch files will:
1. Start the simulation components (MicroXRCE-DDS Agent and PX4 SITL)
2. Wait 30 seconds for startup and stabilization
3. Launch the respective flight control implementation

### Method 2: Manual Step-by-Step

#### For Python Controller

1. **Start Simulation Components:**
   ```bash
   ros2 run single_drone_flight simulation_launcher
   ```

2. **Wait for PX4 to boot** (look for "Ready for takeoff!" message in PX4 terminal)

3. **Start Python Flight Control:**
   ```bash
   ros2 run single_drone_flight single_drone_control
   ```

#### For C++ Controller

1. **Start Simulation Components:**
   ```bash
   ros2 run single_drone_flight simulation_launcher
   ```

2. **Wait for PX4 to boot** (look for "Ready for takeoff!" message in PX4 terminal)

3. **Start C++ Flight Control:**
   ```bash
   ros2 run single_drone_flight single_drone_control_cpp
   ```

## Package Contents

- **`single_drone_control.py`**: Python implementation of autonomous flight control node
- **`single_drone_control_cpp`**: C++ implementation of autonomous flight control node
- **`simulation_launcher.py`**: Launches PX4 SITL and MicroXRCE-DDS Agent
- **`single_drone_flight.launch.py`**: Coordinated launch file for Python controller
- **`single_drone_flight_cpp.launch.py`**: Coordinated launch file for C++ controller

## Flight Behavior

The drone will automatically execute this sequence:

1. **IDLE**: Wait for flight checks to pass
2. **ARMING**: Send arm commands until vehicle is armed
3. **TAKEOFF**: Send takeoff command and wait for takeoff state
4. **CLIMBING**: Wait for loiter state, then switch to offboard mode
5. **REACHING_TARGET**: Use position control to reach exact 5m altitude
6. **HOVERING**: Maintain position at target altitude

## Safety Features

- Continuous monitoring of flight checks
- Automatic return to IDLE state if safety conditions fail
- Failsafe detection and handling
- Arming state monitoring

## Coordinate Frames

The system uses PX4's NED (North-East-Down) coordinate frame:
- Positive X: North
- Positive Y: East
- Positive Z: Down (so -5.0 means 5 meters above ground)

## Monitoring

The flight control node provides detailed logging of:
- State transitions
- Vehicle status changes
- Current altitude
- Safety condition checks

## Troubleshooting

### Common Issues and Solutions

1. **"Vertical velocity unstable" and "velocity estimate error"**
   - **Cause**: PX4 needs time to stabilize its velocity estimation after startup
   - **Solution**: Wait 30-60 seconds after PX4 starts before running flight control
   - **Quick Fix**: Use the provided fix script: `./fix_drone_issues.sh`

2. **Rapid state transitions between IDLE and ARMING**
   - **Cause**: Improved safety checks in the flight control logic
   - **Solution**: The updated code now includes proper timing delays and less frequent command sending

3. **Simulation doesn't start**: Check that PX4-Autopilot is installed in `~/PX4-Autopilot`

4. **MicroXRCE-DDS Agent not found**: Make sure it's installed and in PATH

5. **Drone doesn't arm**: 
   - Check PX4 terminal for error messages
   - Ensure simulation has been running for at least 30 seconds
   - Look for "Ready for takeoff!" message in PX4 terminal

6. **Flight checks fail**: Ensure simulation is fully loaded before starting flight control

### Quick Fix Script

If you're experiencing the velocity estimation errors, run:
```bash
./fix_drone_issues.sh
```

This script will:
- Clean up any existing processes
- Start fresh simulation components
- Wait for PX4 to stabilize
- Provide guidance for next steps

### Manual Step-by-Step Troubleshooting

1. **Kill any existing processes:**
   ```bash
   pkill -f px4
   pkill -f gazebo
   pkill -f MicroXRCE
   ```

2. **Start MicroXRCE-DDS Agent:**
   ```bash
   MicroXRCEAgent udp4 -p 8888
   ```

3. **Start PX4 SITL (in another terminal):**
   ```bash
   cd ~/PX4-Autopilot
   make px4_sitl gazebo-classic_iris
   ```

4. **Wait for "Ready for takeoff!" message in PX4 terminal**

5. **Wait additional 30 seconds for velocity estimation to stabilize**

6. **Run flight control:**
   ```bash
   cd /home/krish/flying_single_drone_ws
   source install/setup.bash
   
   # For Python controller:
   ros2 run single_drone_flight single_drone_control
   
   # OR for C++ controller:
   ros2 run single_drone_flight single_drone_control_cpp
   ```

## Customization

### Python Controller
To modify the target altitude, edit the `target_altitude` variable in `single_drone_control.py`:
```python
self.target_altitude = -5.0  # Change this value (negative for up in NED frame)
```

### C++ Controller
To modify the target altitude, edit the `target_altitude_` variable in `SingleDroneControl.cpp`:
```cpp
target_altitude_ = -5.0f;  // Change this value (negative for up in NED frame)
```

## Based On

This package is simplified from the excellent ROS2_PX4_Offboard_Example by ARK Electronics:
https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example
