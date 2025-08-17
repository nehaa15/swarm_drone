#!/usr/bin/env python3

"""
Launch file for Single Drone Flight

This launch file starts both the simulation and the flight control.
Use this for a complete autonomous flight demonstration.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        # Launch simulation components first
        Node(
            package='single_drone_flight',
            namespace='single_drone_flight',
            executable='modified_simulation_launcher.py',
            name='simulation_launcher',
            output='screen'
        ),
        
        # Wait 30 seconds for simulation to start up and stabilize, then launch flight control
        TimerAction(
            period=30.0,
            actions=[
                Node(
                    package='single_drone_flight',
                    namespace='single_drone_flight',
                    executable='modified_single_drone_control.py',
                    name='single_drone_control',
                    output='screen'
                )
            ]
        )
    ])
