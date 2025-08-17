#!/usr/bin/env python3

"""
Simulation Launcher for Single Drone Flight

This node launches the necessary simulation components:
- MicroXRCE-DDS Agent for PX4 communication
- PX4 SITL simulation with Gazebo

Author: Auto-generated for single drone flight demo
"""

import subprocess
import time
import rclpy
from rclpy.node import Node


class SimulationLauncher(Node):
    """ROS2 Node to launch simulation components"""
    
    def __init__(self):
        super().__init__('simulation_launcher')
        self.get_logger().info("Simulation Launcher Node Initialized")
        self.launch_simulation()
    
    def launch_simulation(self):
        """Launch the simulation components in separate terminal windows"""
        
        self.get_logger().info("Starting simulation components...")
        
        # List of commands to run in separate terminal tabs
        commands = [
            # Run the Micro XRCE-DDS Agent
            "MicroXRCEAgent udp4 -p 8888",
            
            # Run the PX4 SITL simulation with Gazebo
            "cd ~/PX4-Autopilot && make px4_sitl gazebo-classic_iris",
        ]
        
        # Launch each command in a new terminal tab
        for i, command in enumerate(commands):
            try:
                if i == 0:
                    self.get_logger().info("Starting MicroXRCE-DDS Agent...")
                elif i == 1:
                    self.get_logger().info("Starting PX4 SITL simulation...")
                
                # Launch command in new gnome-terminal tab
                subprocess.run([
                    "gnome-terminal", 
                    "--tab", 
                    "--title", f"Simulation Component {i+1}",
                    "--", 
                    "bash", 
                    "-c", 
                    command + "; exec bash"
                ])
                
                # Short pause between launching components
                time.sleep(2)
                
            except Exception as e:
                self.get_logger().error(f"Failed to launch command {i+1}: {e}")
        
        self.get_logger().info("All simulation components started!")
        self.get_logger().info("Wait for PX4 to boot up completely before running the flight control.")
        self.get_logger().info("You should see 'INFO [commander] Ready for takeoff!' in the PX4 terminal.")


def main(args=None):
    rclpy.init(args=args)
    
    print("=" * 60)
    print("Single Drone Flight - Simulation Launcher")
    print("=" * 60)
    print("This will launch:")
    print("1. MicroXRCE-DDS Agent (for ROS2-PX4 communication)")
    print("2. PX4 SITL with Gazebo simulation")
    print("=" * 60)
    print("Make sure you have:")
    print("- PX4-Autopilot installed in ~/PX4-Autopilot")
    print("- MicroXRCEAgent installed and in PATH")
    print("=" * 60)
    
    launcher = SimulationLauncher()
    
    try:
        # Keep the node running for a bit, then exit
        time.sleep(5)
        print("\nSimulation launcher completed.")
        print("The simulation components are now running in separate terminals.")
        print("Run the flight control with: ros2 run single_drone_flight single_drone_control")
    except KeyboardInterrupt:
        print("\nLauncher interrupted...")
    finally:
        launcher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()