#!/usr/bin/env python3
# SPDX-License-Identifier: MIT

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Vector3
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleStatus,
    VehicleAttitude,
    VehicleCommand,
    VehicleLocalPosition,
)


class DroneSquareControl(Node):
    """
    Square-flight mission using PX4 Offboard.
    """

    def __init__(self):
        super().__init__("drone_square_control")

        # --- QoS tuned for PX4 <-> ROS 2 bridge topics ---
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- Subs ---
        self.status_sub = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, qos_profile
        )
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude", self.attitude_callback, qos_profile
        )
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.local_position_callback, qos_profile
        )

        # --- Pubs ---
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        # --- Timers ---
        self.timer_hz = 50.0
        self.timer = self.create_timer(1.0 / self.timer_hz, self.control_loop_callback)  # 50 Hz setpoints
        self.state_timer = self.create_timer(0.1, self.state_machine_callback)  # 10 Hz FSM

        # --- Mission / state ---
        self.current_state = "IDLE"
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_DISARMED
        self.failsafe = False
        self.flight_check = False
        self.offboard_mode = False

        # --- Params / tuning ---
        self.target_altitude = -3.0  # NED z (negative is up)
        self.altitude_tolerance = 0.3
        self.position_tolerance = 1.5
        self.hold_time_at_wp = 5.0
        self.waypoint_timeout = 30.0
        self.waypoint_max_xy_speed = 1.2
        self.approach_switch_dist = 1.2

        self.arming_timeout = 10.0
        self.startup_delay = 5.0

        # --- Pose / attitude ---
        self.current_altitude = 0.0
        self.current_position = Vector3()
        self.takeoff_position = Vector3()
        self.yaw = 0.0

        # --- Counters / helpers ---
        self.counter = 0
        self.start_time = time.time()
        self.arming_start_time = 0.0

        # --- Landing helpers ---
        self.landing_initiated = False
        self.landing_cmd_sent = False  # ensure we only send LAND once

        # --- Waypoints ---
        self.square_points = []
        self.current_waypoint = 0
        self.waypoint_start_time = 0.0

        # --- For averaging takeoff origin ---
        self.position_samples = []

        # --- Offboard warmup management ---
        self.offboard_warmup_started = False
        self.offboard_warmup_start_time = 0.0
        self.offboard_warmup_duration = 3.0  # seconds of pre-stream before switching

        # --- Climb velocity used after Offboard ON until we reach target_altitude ---
        self.climbing_velocity = -1.0  # NED z velocity (negative = up)

        self.get_logger().info("Drone Square Control Node Initialized")

    # ========================= PX4 / ROS Callbacks =========================

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flight_check = msg.pre_flight_checks_pass

    def attitude_callback(self, msg: VehicleAttitude):
        # VehicleAttitude.q layout: [w, x, y, z]
        w, x, y, z = msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        self.yaw = float(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))

    def local_position_callback(self, msg: VehicleLocalPosition):
        self.current_position.x = msg.x  # North
        self.current_position.y = msg.y  # East
               # Down (negative up)
        self.current_position.z = msg.z
        self.current_altitude = msg.z

    # ========================= PX4 Commands =========================

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # PX4 expects µs
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param7 = float(param7)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher_.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def takeoff(self, altitude=5.0):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=altitude)

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, param7=0.0)

    def set_offboard_mode_cmd(self):
        # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1, PX4 custom main mode OFFBOARD = 6
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.offboard_mode = True

    def set_land_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0)
        self.get_logger().info("Landing mode requested")

    # ========================= Mission Helpers =========================

    def create_square_waypoints(self, side=4.0):
        half = side / 2.0
        cx, cy = self.takeoff_position.x, self.takeoff_position.y
        self.square_points = [
            (cx + half, cy + half),
            (cx - half, cy + half),
            (cx - half, cy - half),
            (cx + half, cy - half),
            (cx + 0.0, cy + 0.0),    # back to center
        ]
        self.get_logger().info(f"Square waypoints created: {len(self.square_points)} points")
        for i, (x, y) in enumerate(self.square_points):
            self.get_logger().info(f"WP{i}: ({x:.1f}, {y:.1f})")

    def begin_offboard_warmup(self):
        if not self.offboard_warmup_started:
            self.offboard_warmup_started = True
            self.offboard_warmup_start_time = time.time()
            self.get_logger().info("Starting Offboard warmup stream (position setpoints)")

    def offboard_warmup_complete(self):
        return self.offboard_warmup_started and (time.time() - self.offboard_warmup_start_time) >= self.offboard_warmup_duration

    # ========================= State Machine =========================

    def state_machine_callback(self):
        # When landing, let PX4 take over entirely; do not send setpoints from here
        if self.landing_initiated:
            if self.arm_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.current_state = "LANDED"
            return

        # Safety: if disarmed unexpectedly, go back to IDLE
        if (
            self.arm_state != VehicleStatus.ARMING_STATE_ARMED
            and self.current_state not in ["IDLE", "ARMING", "LANDING", "LANDED"]
            and self.counter > 20
        ):
            self.current_state = "IDLE"

        if self.current_state == "IDLE":
            now = time.time()
            if self.flight_check and (now - self.start_time) > self.startup_delay:
                self.current_state = "ARMING"
                self.arming_start_time = now
                self.get_logger().info("State -> ARMING")

        elif self.current_state == "ARMING":
            now = time.time()
            if (now - self.arming_start_time) > self.arming_timeout:
                self.get_logger().warn("Arming timeout -> IDLE")
                self.current_state = "IDLE"
                return

            if not self.flight_check:
                self.get_logger().warn("Preflight checks failed -> IDLE")
                self.current_state = "IDLE"
            elif self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.counter > 10:
                self.current_state = "TAKEOFF"
                self.get_logger().info("Armed -> TAKEOFF")
            else:
                if self.counter % 10 == 0:
                    self.arm()

        elif self.current_state == "TAKEOFF":
            if not self.flight_check:
                self.current_state = "IDLE"
                self.get_logger().warn("Preflight failed during TAKEOFF -> IDLE")
            elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                self.current_state = "CLIMBING"
                # Average takeoff position
                self.position_samples.append((self.current_position.x, self.current_position.y))
                if len(self.position_samples) >= 10:
                    avg_x = sum(x for x, _ in self.position_samples) / len(self.position_samples)
                    avg_y = sum(y for _, y in self.position_samples) / len(self.position_samples)
                    self.takeoff_position.x = avg_x
                    self.takeoff_position.y = avg_y
                    self.get_logger().info(f"Averaged takeoff position: ({avg_x:.2f}, {avg_y:.2f})")
                    self.position_samples = []
            else:
                if self.counter % 10 == 0:
                    self.arm()
                    self.takeoff(5.0)

        elif self.current_state == "CLIMBING":
            if not self.flight_check or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe:
                self.get_logger().warn("Safety gate triggered in CLIMBING -> IDLE")
                self.current_state = "IDLE"
            elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                # Reached loiter after takeoff height: warm up Offboard streaming
                self.begin_offboard_warmup()
                self.current_state = "REACHING_TARGET"
                self.get_logger().info("Reached loiter -> REACHING_TARGET (Offboard warmup)")
            else:
                if self.counter % 10 == 0:
                    self.arm()

        elif self.current_state == "REACHING_TARGET":
            # Safety
            if not self.flight_check or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe:
                self.get_logger().warn("Safety gate in REACHING_TARGET -> IDLE")
                self.current_state = "IDLE"
                return

            # Request Offboard once warmup is complete
            if self.offboard_warmup_complete() and not self.offboard_mode:
                self.set_offboard_mode_cmd()
                self.get_logger().info("Requested Offboard mode")

            # When at target altitude AND Offboard is ON -> start mission
            if abs(self.current_altitude - self.target_altitude) < self.altitude_tolerance and self.offboard_mode:
                self.current_state = "SQUARE_FLIGHT"
                self.create_square_waypoints(side=4.0)
                self.current_waypoint = 0
                self.waypoint_start_time = time.time()
                self.get_logger().info("Altitude reached & Offboard ON -> SQUARE_FLIGHT")

        elif self.current_state == "SQUARE_FLIGHT":
            if not self.flight_check or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe:
                self.current_state = "IDLE"
                self.get_logger().warn("Safety condition failed during square flight -> IDLE")
                return

            if self.current_waypoint < len(self.square_points):
                target_x, target_y = self.square_points[self.current_waypoint]
                dx = target_x - self.current_position.x
                dy = target_y - self.current_position.y
                dist = float(math.hypot(dx, dy))

                if self.counter % 20 == 0:
                    self.get_logger().info(
                        f"SQUARE - WP{self.current_waypoint}/{len(self.square_points)-1}: "
                        f"Target=({target_x:.1f},{target_y:.1f}), "
                        f"Current=({self.current_position.x:.1f},{self.current_position.y:.1f}), "
                        f"Dist:{dist:.2f}m Alt:{-self.current_altitude:.1f}m"
                    )

                # Timeout protection
                if (time.time() - self.waypoint_start_time) > self.waypoint_timeout:
                    self.get_logger().warn(f"Timeout at WP{self.current_waypoint} (Dist:{dist:.2f}m), advancing")
                    self.current_waypoint += 1
                    self.waypoint_start_time = time.time()
                    return

                # Arrival check
                if dist < self.position_tolerance:
                    hold_elapsed = time.time() - self.waypoint_start_time
                    if hold_elapsed > self.hold_time_at_wp:
                        self.get_logger().info(f"Waypoint {self.current_waypoint} complete -> next")
                        self.current_waypoint += 1
                        self.waypoint_start_time = time.time()
                    elif self.counter % 10 == 0:
                        remaining = self.hold_time_at_wp - hold_elapsed
                        self.get_logger().info(f"Holding at WP{self.current_waypoint} for {remaining:.1f}s more")
                else:
                    # Reset hold timer if we drift out
                    self.waypoint_start_time = time.time()
            else:
                self.get_logger().info("Square flight complete, initiating landing")
                self.current_state = "LANDING"
                self.offboard_mode = False  # stop using offboard
                self.landing_cmd_sent = False  # ensure we send LAND once

        elif self.current_state == "LANDING":
            if not self.landing_cmd_sent:
                # ✅ Directly command PX4 to land, no hover setpoint
                self.set_land_mode()
                self.landing_cmd_sent = True
                self.landing_initiated = True
                self.get_logger().info("LAND command sent to PX4")
            # ✅ Stop state machine activities for landing
            return

        elif self.current_state == "LANDED":
            if self.counter % 100 == 0:
                self.get_logger().info("Mission complete - drone has landed")

        self.counter += 1

    # ========================= Control Loop (50 Hz) =========================

    def control_loop_callback(self):
        """
        Continuously publishes OffboardControlMode + TrajectorySetpoint
        when appropriate. We *do not* publish any setpoints during LANDING.
        """
        # Do not publish in landing or landed
        if self.current_state in ["LANDING", "LANDED"]:
            return

        # Decide whether to publish offboard messages:
        publish = False
        if self.current_state in ["REACHING_TARGET", "SQUARE_FLIGHT"]:
            publish = True
        if self.offboard_warmup_started and not self.offboard_mode:
            publish = True  # stream during warmup

        if not publish:
            return

        now_us = int(Clock().now().nanoseconds / 1000)

        # Build OffboardControlMode message
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = now_us

        traj = TrajectorySetpoint()
        traj.timestamp = now_us

        # Defaults to NaN when a dimension is not controlled
        traj.position = [float("nan"), float("nan"), float("nan")]
        traj.velocity = [float("nan"), float("nan"), float("nan")]
        traj.acceleration = [float("nan"), float("nan"), float("nan")]
        traj.yaw = float("nan")
        traj.yawspeed = float("nan")

        # Warmup: stream POSITION setpoints to current x,y and target z
        if self.offboard_warmup_started and (self.current_state in ["REACHING_TARGET", "CLIMBING", "TAKEOFF"]):
            offboard_msg.position = True
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = False

            traj.position[0] = self.current_position.x
            traj.position[1] = self.current_position.y
            traj.position[2] = self.target_altitude
            traj.yaw = 0.0

        # ✅ Guard: After Offboard is ON, descend with velocity until near target,
        # then switch to position hold (no lingering downward velocity)
        if self.current_state == "REACHING_TARGET" and self.offboard_mode:
            offboard_msg.position = False
            offboard_msg.velocity = True
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = False

            if (self.current_altitude - self.target_altitude) > 0.2:
                # Keep descending
                traj.velocity[0] = 0.0
                traj.velocity[1] = 0.0
                traj.velocity[2] = self.climbing_velocity  # negative -> up in NED, so this should be negative to go UP; here we descend toward a less-negative z
            else:
                # Switch to position hold near target
                offboard_msg.velocity = False
                offboard_msg.position = True
                traj.position[0] = self.current_position.x
                traj.position[1] = self.current_position.y
                traj.position[2] = self.target_altitude
            traj.yaw = 0.0

        elif self.current_state == "SQUARE_FLIGHT" and self.current_waypoint < len(self.square_points):
            target_x, target_y = self.square_points[self.current_waypoint]
            dx = target_x - self.current_position.x
            dy = target_y - self.current_position.y
            dist = float(math.hypot(dx, dy))

            if dist > self.approach_switch_dist:
                # Velocity mode (cruise)
                offboard_msg.position = False
                offboard_msg.velocity = True
                offboard_msg.acceleration = False
                offboard_msg.attitude = False
                offboard_msg.body_rate = False

                ux = dx / (dist + 1e-6)
                uy = dy / (dist + 1e-6)

                traj.velocity[0] = ux * self.waypoint_max_xy_speed
                traj.velocity[1] = uy * self.waypoint_max_xy_speed
                traj.velocity[2] = 0.0
                traj.yaw = 0.0

            else:
                # Position mode (precise approach/hold)
                offboard_msg.position = True
                offboard_msg.velocity = False
                offboard_msg.acceleration = False
                offboard_msg.attitude = False
                offboard_msg.body_rate = False

                if dist > self.position_tolerance * 2.0:
                    smooth = 1.0
                elif dist > self.position_tolerance:
                    smooth = 0.7
                else:
                    smooth = 0.3

                smoothed_x = self.current_position.x + smooth * dx
                smoothed_y = self.current_position.y + smooth * dy

                traj.position[0] = smoothed_x
                traj.position[1] = smoothed_y
                traj.position[2] = self.target_altitude
                traj.yaw = 0.0

        # If no control bit set, default to position @ target altitude
        if not (offboard_msg.position or offboard_msg.velocity or offboard_msg.acceleration or offboard_msg.attitude or offboard_msg.body_rate):
            offboard_msg.position = True
            traj.position[0] = self.current_position.x
            traj.position[1] = self.current_position.y
            traj.position[2] = self.target_altitude
            traj.yaw = 0.0

        # Publish
        self.publisher_offboard_mode.publish(offboard_msg)
        self.publisher_trajectory.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = DroneSquareControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
