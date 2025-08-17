#!/usr/bin/env python3

"""
Single Drone Flight Control

This node autonomously flies a drone to 5 meters altitude using PX4 offboard control.
Based on the ROS2_PX4_Offboard_Example but simplified for autonomous flight to a target altitude.

Author: Auto-generated for single drone flight demo
"""

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
import time


class SingleDroneControl(Node):
    """
    ROS2 Node for controlling a single drone to fly to 5 meters altitude autonomously.
    
    State Machine:
    IDLE -> ARMING -> TAKEOFF -> CLIMBING -> REACHED_TARGET -> HOVERING
    """

    def __init__(self):
        super().__init__('single_drone_control')
        
        # QoS profile for PX4 communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile)
            
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile)

        # Create publishers
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10)

        # Create timers
        # Main control loop timer (50Hz for smooth control)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.control_loop_callback)
        
        # State machine timer (10Hz for state transitions)
        state_timer_period = 0.1  # seconds
        self.state_timer = self.create_timer(state_timer_period, self.state_machine_callback)

        # Initialize state variables
        self.current_state = "IDLE"
        self.last_state = "IDLE"
        self.target_altitude = -5.0  # NED frame: negative Z is up
        self.altitude_tolerance = 0.2  # meters
        self.climb_velocity = 1.0  # m/s
        
        # Timing variables for state transitions
        self.arming_start_time = 0
        self.arming_timeout = 10.0  # seconds to wait for arming
        self.startup_delay = 5.0  # seconds to wait before starting arming process
        
        # Vehicle status variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_DISARMED
        self.failsafe = False
        self.flight_check = False
        self.offboard_mode = False
        
        # Position and attitude variables
        self.current_altitude = 0.0  # NED frame
        self.current_position = Vector3()
        self.takeoff_position = Vector3()
        self.yaw = 0.0
        
        # Control variables
        self.counter = 0
        self.start_time = time.time()
        
        self.get_logger().info("Single Drone Control Node Initialized")
        self.get_logger().info(f"Target altitude: {-self.target_altitude} meters above ground")

    def vehicle_status_callback(self, msg):
        """Callback for vehicle status updates"""
        if msg.nav_state != self.nav_state:
            self.get_logger().info(f"Navigation State: {msg.nav_state}")
        
        if msg.arming_state != self.arm_state:
            self.get_logger().info(f"Arming State: {msg.arming_state}")

        if msg.failsafe != self.failsafe:
            self.get_logger().info(f"Failsafe: {msg.failsafe}")
            
        if msg.pre_flight_checks_pass != self.flight_check:
            self.get_logger().info(f"Flight Check: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flight_check = msg.pre_flight_checks_pass

    def attitude_callback(self, msg):
        """Callback for vehicle attitude updates"""
        # Extract yaw from quaternion
        orientation_q = msg.q
        self.yaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                              1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))

    def local_position_callback(self, msg):
        """Callback for vehicle local position updates"""
        self.current_position.x = msg.x
        self.current_position.y = msg.y
        self.current_position.z = msg.z
        self.current_altitude = msg.z  # NED frame: negative is up

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """Publish a vehicle command"""
        msg = VehicleCommand()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher_.publish(msg)

    def arm(self):
        """Send arm command"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    def takeoff(self, altitude=5.0):
        """Send takeoff command"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=altitude)
        self.get_logger().info(f"Takeoff command sent - altitude: {altitude}m")

    def set_offboard_mode(self):
        """Switch to offboard mode"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.offboard_mode = True
        self.get_logger().info("Offboard mode command sent")

    def state_machine_callback(self):
        """State machine for autonomous flight"""
        
        if self.current_state != self.last_state:
            self.get_logger().info(f"State transition: {self.last_state} -> {self.current_state}")
            self.last_state = self.current_state

        if self.current_state == "IDLE":
            # Wait for flight checks to pass and some startup time
            current_time = time.time()
            if self.flight_check and (current_time - self.start_time) > self.startup_delay:
                self.current_state = "ARMING"
                self.arming_start_time = current_time
                self.get_logger().info("Flight checks passed and startup delay complete, starting arming sequence")

        elif self.current_state == "ARMING":
            current_time = time.time()
            
            # Check for arming timeout
            if (current_time - self.arming_start_time) > self.arming_timeout:
                self.current_state = "IDLE"
                self.get_logger().warn("Arming timeout, returning to IDLE state")
                return
                
            # Send arm command and wait for armed state
            if not self.flight_check:
                self.current_state = "IDLE"
                self.get_logger().warn("Flight check failed during arming")
            elif self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.counter > 10:
                self.current_state = "TAKEOFF"
                # Store takeoff position for reference
                self.takeoff_position = self.current_position
                self.get_logger().info("Armed successfully, initiating takeoff")
            else:
                # Only send arm command every few cycles to avoid spamming
                if self.counter % 10 == 0:  # Send every second (10 * 0.1s)
                    self.arm()

        elif self.current_state == "TAKEOFF":
            # Send takeoff command and wait for takeoff state
            if not self.flight_check:
                self.current_state = "IDLE"
                self.get_logger().warn("Flight check failed during takeoff")
            elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                self.current_state = "CLIMBING"
                self.get_logger().info("Takeoff initiated, starting climb to target altitude")
            else:
                # Send commands less frequently to avoid overwhelming the system
                if self.counter % 10 == 0:  # Send every second
                    self.arm()  # Keep sending arm command
                    self.takeoff(5.0)  # Send takeoff command

        elif self.current_state == "CLIMBING":
            # Wait for loiter state, then switch to offboard
            if not self.flight_check or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe:
                self.current_state = "IDLE"
                self.get_logger().warn("Safety condition failed during climb")
            elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                self.current_state = "REACHING_TARGET"
                self.set_offboard_mode()
                self.get_logger().info("Switched to offboard mode, climbing to target altitude")
            else:
                # Send arm command less frequently
                if self.counter % 10 == 0:
                    self.arm()  # Keep sending arm command

        elif self.current_state == "REACHING_TARGET":
            # Use offboard control to reach exact target altitude
            if not self.flight_check or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe:
                self.current_state = "IDLE"
                self.get_logger().warn("Safety condition failed while reaching target")
            elif abs(self.current_altitude - self.target_altitude) < self.altitude_tolerance:
                self.current_state = "HOVERING"
                self.get_logger().info(f"Target altitude reached! Current: {-self.current_altitude:.2f}m")
            else:
                # Continue climbing in offboard mode
                pass

        elif self.current_state == "HOVERING":
            # Maintain position at target altitude
            if not self.flight_check or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe:
                self.current_state = "IDLE"
                self.get_logger().warn("Safety condition failed while hovering")
            else:
                # Log status every 50 seconds
                if self.counter % 500 == 0:  # 500 * 0.1s = 50s
                    self.get_logger().info(f"Hovering at {-self.current_altitude:.2f}m altitude")

        # Only apply safety check if we're not already in IDLE and not in critical states
        if (self.arm_state != VehicleStatus.ARMING_STATE_ARMED and 
            self.current_state not in ["IDLE", "ARMING"] and
            self.counter > 20):  # Give some time for initial state transitions
            self.current_state = "IDLE"
            self.get_logger().warn("Vehicle disarmed, returning to IDLE state")

        self.counter += 1

    def control_loop_callback(self):
        """Main control loop for offboard commands"""
        if self.offboard_mode and (self.current_state == "REACHING_TARGET" or self.current_state == "HOVERING"):
            # Publish offboard control mode
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = True
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = False
            self.publisher_offboard_mode.publish(offboard_msg)

            # Publish trajectory setpoint
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            
            if self.current_state == "REACHING_TARGET":
                # Position control to reach target altitude
                trajectory_msg.position[0] = self.takeoff_position.x  # Hold X position
                trajectory_msg.position[1] = self.takeoff_position.y  # Hold Y position
                trajectory_msg.position[2] = self.target_altitude      # Target altitude
                trajectory_msg.yaw = self.yaw                          # Hold current yaw
                
            elif self.current_state == "HOVERING":
                # Hold position at target altitude
                trajectory_msg.position[0] = self.takeoff_position.x
                trajectory_msg.position[1] = self.takeoff_position.y
                trajectory_msg.position[2] = self.target_altitude
                trajectory_msg.yaw = self.yaw

            # Set unused fields to NaN
            trajectory_msg.velocity[0] = float('nan')
            trajectory_msg.velocity[1] = float('nan')
            trajectory_msg.velocity[2] = float('nan')
            trajectory_msg.acceleration[0] = float('nan')
            trajectory_msg.acceleration[1] = float('nan')
            trajectory_msg.acceleration[2] = float('nan')
            trajectory_msg.yawspeed = float('nan')

            self.publisher_trajectory.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)
    
    print("=" * 60)
    print("Single Drone Flight Control")
    print("=" * 60)
    print("This node will autonomously:")
    print("1. Arm the drone")
    print("2. Takeoff")
    print("3. Climb to 5 meters altitude")
    print("4. Hover at target altitude")
    print("=" * 60)
    
    single_drone_control = SingleDroneControl()
    
    try:
        rclpy.spin(single_drone_control)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        single_drone_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()