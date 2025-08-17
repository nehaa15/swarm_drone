/**
 * @file SingleDroneControl.hpp
 * @brief ROS 2 Node for autonomous single drone flight control using PX4 offboard control.
 * @details This node autonomously flies a drone to 5 meters altitude and maintains hover.
 * State Machine: IDLE -> ARMING -> TAKEOFF -> CLIMBING -> REACHING_TARGET -> HOVERING
 * @author Auto-generated for single drone flight demo
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <chrono>
#include <string>
#include <memory>

using namespace std::chrono;
using namespace std::chrono_literals;

/**
 * @enum FlightState
 * @brief Enumeration of flight states for the state machine
 */
enum class FlightState {
    IDLE,
    ARMING,
    TAKEOFF,
    CLIMBING,
    REACHING_TARGET,
    HOVERING
};

/**
 * @class SingleDroneControl
 * @brief ROS 2 Node for autonomous single drone flight control.
 */
class SingleDroneControl : public rclcpp::Node {
    using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
    using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
    using VehicleStatus = px4_msgs::msg::VehicleStatus;
    using VehicleAttitude = px4_msgs::msg::VehicleAttitude;
    using VehicleCommand = px4_msgs::msg::VehicleCommand;
    using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;
    using Vector3 = geometry_msgs::msg::Vector3;

public:
    /**
     * @brief Constructor for the SingleDroneControl class.
     */
    SingleDroneControl();

private:
    /**
     * @brief Callback function for vehicle status updates.
     * @param msg The received vehicle status message.
     */
    void vehicle_status_callback(const VehicleStatus::SharedPtr msg);

    /**
     * @brief Callback function for vehicle attitude updates.
     * @param msg The received vehicle attitude message.
     */
    void attitude_callback(const VehicleAttitude::SharedPtr msg);

    /**
     * @brief Callback function for vehicle local position updates.
     * @param msg The received vehicle local position message.
     */
    void local_position_callback(const VehicleLocalPosition::SharedPtr msg);

    /**
     * @brief Publish a vehicle command.
     * @param command The command to publish.
     * @param param1 Command parameter 1.
     * @param param2 Command parameter 2.
     * @param param7 Command parameter 7.
     */
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param7 = 0.0);

    /**
     * @brief Send arm command to the vehicle.
     */
    void arm();

    /**
     * @brief Send takeoff command to the vehicle.
     * @param altitude Target takeoff altitude in meters.
     */
    void takeoff(float altitude = 5.0);

    /**
     * @brief Switch the vehicle to offboard mode.
     */
    void set_offboard_mode();

    /**
     * @brief State machine callback for autonomous flight control.
     */
    void state_machine_callback();

    /**
     * @brief Main control loop callback for offboard commands.
     */
    void control_loop_callback();

    /**
     * @brief Convert flight state enum to string for logging.
     * @param state The flight state to convert.
     * @return String representation of the flight state.
     */
    std::string flight_state_to_string(FlightState state);

private:
    // Publishers
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // Subscribers
    rclcpp::Subscription<VehicleStatus>::SharedPtr status_subscriber_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_subscriber_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscriber_;

    // Timers
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;

    // State variables
    FlightState current_state_;
    FlightState last_state_;
    float target_altitude_;  // NED frame: negative Z is up
    float altitude_tolerance_;  // meters
    float climb_velocity_;  // m/s

    // Timing variables
    steady_clock::time_point arming_start_time_;
    steady_clock::time_point start_time_;
    double arming_timeout_;  // seconds
    double startup_delay_;  // seconds

    // Vehicle status variables
    uint8_t nav_state_;
    uint8_t arm_state_;
    bool failsafe_;
    bool flight_check_;
    bool offboard_mode_;

    // Position and attitude variables
    float current_altitude_;  // NED frame
    Vector3 current_position_;
    Vector3 takeoff_position_;
    float yaw_;

    // Control variables
    uint32_t counter_;
};
