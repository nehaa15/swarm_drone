/**
 * @file SingleDroneControl.cpp
 * @brief Implementation of SingleDroneControl class for autonomous single drone flight.
 * @author Auto-generated for single drone flight demo
 */

#include "SingleDroneControl.hpp"
#include <cmath>
#include <limits>

/**
 * @brief Constructor for the SingleDroneControl class.
 */
SingleDroneControl::SingleDroneControl() : Node("single_drone_control") {
    // QoS profile for PX4 communication
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    // Create publishers
    offboard_mode_publisher_ = this->create_publisher<OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos);
    trajectory_publisher_ = this->create_publisher<TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", qos);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    // Create subscribers
    status_subscriber_ = this->create_subscription<VehicleStatus>(
        "/fmu/out/vehicle_status", qos,
        [this](const VehicleStatus::SharedPtr msg) {
            this->vehicle_status_callback(msg);
        });

    attitude_subscriber_ = this->create_subscription<VehicleAttitude>(
        "/fmu/out/vehicle_attitude", qos,
        [this](const VehicleAttitude::SharedPtr msg) {
            this->attitude_callback(msg);
        });

    local_position_subscriber_ = this->create_subscription<VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos,
        [this](const VehicleLocalPosition::SharedPtr msg) {
            this->local_position_callback(msg);
        });

    // Create timers
    // Main control loop timer (50Hz for smooth control)
    control_timer_ = this->create_wall_timer(
        20ms, [this]() { this->control_loop_callback(); });

    // State machine timer (10Hz for state transitions)
    state_timer_ = this->create_wall_timer(
        100ms, [this]() { this->state_machine_callback(); });

    // Initialize state variables
    current_state_ = FlightState::IDLE;
    last_state_ = FlightState::IDLE;
    target_altitude_ = -5.0f;  // NED frame: negative Z is up
    altitude_tolerance_ = 0.2f;  // meters
    climb_velocity_ = 1.0f;  // m/s

    // Timing variables
    arming_timeout_ = 10.0;  // seconds
    startup_delay_ = 5.0;  // seconds
    start_time_ = steady_clock::now();

    // Vehicle status variables
    nav_state_ = VehicleStatus::NAVIGATION_STATE_MAX;
    arm_state_ = VehicleStatus::ARMING_STATE_DISARMED;
    failsafe_ = false;
    flight_check_ = false;
    offboard_mode_ = false;

    // Position and attitude variables
    current_altitude_ = 0.0f;
    current_position_ = Vector3();
    takeoff_position_ = Vector3();
    yaw_ = 0.0f;

    // Control variables
    counter_ = 0;

    RCLCPP_INFO(this->get_logger(), "Single Drone Control Node Initialized");
    RCLCPP_INFO(this->get_logger(), "Target altitude: %.1f meters above ground", -target_altitude_);
}

/**
 * @brief Callback function for vehicle status updates.
 */
void SingleDroneControl::vehicle_status_callback(const VehicleStatus::SharedPtr msg) {
    if (msg->nav_state != nav_state_) {
        RCLCPP_INFO(this->get_logger(), "Navigation State: %d", msg->nav_state);
    }

    if (msg->arming_state != arm_state_) {
        RCLCPP_INFO(this->get_logger(), "Arming State: %d", msg->arming_state);
    }

    if (msg->failsafe != failsafe_) {
        RCLCPP_INFO(this->get_logger(), "Failsafe: %s", msg->failsafe ? "true" : "false");
    }

    if (msg->pre_flight_checks_pass != flight_check_) {
        RCLCPP_INFO(this->get_logger(), "Flight Check: %s", msg->pre_flight_checks_pass ? "true" : "false");
    }

    nav_state_ = msg->nav_state;
    arm_state_ = msg->arming_state;
    failsafe_ = msg->failsafe;
    flight_check_ = msg->pre_flight_checks_pass;
}

/**
 * @brief Callback function for vehicle attitude updates.
 */
void SingleDroneControl::attitude_callback(const VehicleAttitude::SharedPtr msg) {
    // Extract yaw from quaternion (q = [w, x, y, z])
    auto q = msg->q;
    yaw_ = -std::atan2(2.0f * (q[3] * q[0] + q[1] * q[2]),
                       1.0f - 2.0f * (q[0] * q[0] + q[1] * q[1]));
}

/**
 * @brief Callback function for vehicle local position updates.
 */
void SingleDroneControl::local_position_callback(const VehicleLocalPosition::SharedPtr msg) {
    current_position_.x = msg->x;
    current_position_.y = msg->y;
    current_position_.z = msg->z;
    current_altitude_ = msg->z;  // NED frame: negative is up
}

/**
 * @brief Publish a vehicle command.
 */
void SingleDroneControl::publish_vehicle_command(uint16_t command, float param1, float param2, float param7) {
    VehicleCommand msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param7 = param7;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_publisher_->publish(msg);
}

/**
 * @brief Send arm command to the vehicle.
 */
void SingleDroneControl::arm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

/**
 * @brief Send takeoff command to the vehicle.
 */
void SingleDroneControl::takeoff(float altitude) {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 1.0f, 0.0f, altitude);
    RCLCPP_INFO(this->get_logger(), "Takeoff command sent - altitude: %.1fm", altitude);
}

/**
 * @brief Switch the vehicle to offboard mode.
 */
void SingleDroneControl::set_offboard_mode() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    offboard_mode_ = true;
    RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
}

/**
 * @brief Convert flight state enum to string for logging.
 */
std::string SingleDroneControl::flight_state_to_string(FlightState state) {
    switch (state) {
        case FlightState::IDLE: return "IDLE";
        case FlightState::ARMING: return "ARMING";
        case FlightState::TAKEOFF: return "TAKEOFF";
        case FlightState::CLIMBING: return "CLIMBING";
        case FlightState::REACHING_TARGET: return "REACHING_TARGET";
        case FlightState::HOVERING: return "HOVERING";
        default: return "UNKNOWN";
    }
}

/**
 * @brief State machine callback for autonomous flight control.
 */
void SingleDroneControl::state_machine_callback() {
    if (current_state_ != last_state_) {
        RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s",
                    flight_state_to_string(last_state_).c_str(),
                    flight_state_to_string(current_state_).c_str());
        last_state_ = current_state_;
    }

    auto current_time = steady_clock::now();

    switch (current_state_) {
        case FlightState::IDLE: {
            // Wait for flight checks to pass and some startup time
            auto elapsed = duration_cast<seconds>(current_time - start_time_).count();
            if (flight_check_ && elapsed > startup_delay_) {
                current_state_ = FlightState::ARMING;
                arming_start_time_ = current_time;
                RCLCPP_INFO(this->get_logger(), "Flight checks passed and startup delay complete, starting arming sequence");
            }
            break;
        }

        case FlightState::ARMING: {
            // Check for arming timeout
            auto elapsed = duration_cast<seconds>(current_time - arming_start_time_).count();
            if (elapsed > arming_timeout_) {
                current_state_ = FlightState::IDLE;
                RCLCPP_WARN(this->get_logger(), "Arming timeout, returning to IDLE state");
                return;
            }

            // Send arm command and wait for armed state
            if (!flight_check_) {
                current_state_ = FlightState::IDLE;
                RCLCPP_WARN(this->get_logger(), "Flight check failed during arming");
            } else if (arm_state_ == VehicleStatus::ARMING_STATE_ARMED && counter_ > 10) {
                current_state_ = FlightState::TAKEOFF;
                // Store takeoff position for reference
                takeoff_position_ = current_position_;
                RCLCPP_INFO(this->get_logger(), "Armed successfully, initiating takeoff");
            } else {
                // Only send arm command every few cycles to avoid spamming
                if (counter_ % 10 == 0) {  // Send every second (10 * 0.1s)
                    arm();
                }
            }
            break;
        }

        case FlightState::TAKEOFF: {
            // Send takeoff command and wait for takeoff state
            if (!flight_check_) {
                current_state_ = FlightState::IDLE;
                RCLCPP_WARN(this->get_logger(), "Flight check failed during takeoff");
            } else if (nav_state_ == VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF) {
                current_state_ = FlightState::CLIMBING;
                RCLCPP_INFO(this->get_logger(), "Takeoff initiated, starting climb to target altitude");
            } else {
                // Send commands less frequently to avoid overwhelming the system
                if (counter_ % 10 == 0) {  // Send every second
                    arm();  // Keep sending arm command
                    takeoff(5.0f);  // Send takeoff command
                }
            }
            break;
        }

        case FlightState::CLIMBING: {
            // Wait for loiter state, then switch to offboard
            if (!flight_check_ || arm_state_ != VehicleStatus::ARMING_STATE_ARMED || failsafe_) {
                current_state_ = FlightState::IDLE;
                RCLCPP_WARN(this->get_logger(), "Safety condition failed during climb");
            } else if (nav_state_ == VehicleStatus::NAVIGATION_STATE_AUTO_LOITER) {
                current_state_ = FlightState::REACHING_TARGET;
                set_offboard_mode();
                RCLCPP_INFO(this->get_logger(), "Switched to offboard mode, climbing to target altitude");
            } else {
                // Send arm command less frequently
                if (counter_ % 10 == 0) {
                    arm();  // Keep sending arm command
                }
            }
            break;
        }

        case FlightState::REACHING_TARGET: {
            // Use offboard control to reach exact target altitude
            if (!flight_check_ || arm_state_ != VehicleStatus::ARMING_STATE_ARMED || failsafe_) {
                current_state_ = FlightState::IDLE;
                RCLCPP_WARN(this->get_logger(), "Safety condition failed while reaching target");
            } else if (std::abs(current_altitude_ - target_altitude_) < altitude_tolerance_) {
                current_state_ = FlightState::HOVERING;
                RCLCPP_INFO(this->get_logger(), "Target altitude reached! Current: %.2fm", -current_altitude_);
            }
            // Continue climbing in offboard mode
            break;
        }

        case FlightState::HOVERING: {
            // Maintain position at target altitude
            if (!flight_check_ || arm_state_ != VehicleStatus::ARMING_STATE_ARMED || failsafe_) {
                current_state_ = FlightState::IDLE;
                RCLCPP_WARN(this->get_logger(), "Safety condition failed while hovering");
            } else {
                // Log status every 50 seconds
                if (counter_ % 500 == 0) {  // 500 * 0.1s = 50s
                    RCLCPP_INFO(this->get_logger(), "Hovering at %.2fm altitude", -current_altitude_);
                }
            }
            break;
        }
    }

    // Only apply safety check if we're not already in IDLE and not in critical states
    if (arm_state_ != VehicleStatus::ARMING_STATE_ARMED &&
        current_state_ != FlightState::IDLE &&
        current_state_ != FlightState::ARMING &&
        counter_ > 20) {  // Give some time for initial state transitions
        current_state_ = FlightState::IDLE;
        RCLCPP_WARN(this->get_logger(), "Vehicle disarmed, returning to IDLE state");
    }

    counter_++;
}

/**
 * @brief Main control loop callback for offboard commands.
 */
void SingleDroneControl::control_loop_callback() {
    if (offboard_mode_ && (current_state_ == FlightState::REACHING_TARGET || current_state_ == FlightState::HOVERING)) {
        // Publish offboard control mode
        OffboardControlMode offboard_msg{};
        offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_msg.position = true;
        offboard_msg.velocity = false;
        offboard_msg.acceleration = false;
        offboard_msg.attitude = false;
        offboard_msg.body_rate = false;
        offboard_mode_publisher_->publish(offboard_msg);

        // Publish trajectory setpoint
        TrajectorySetpoint trajectory_msg{};
        trajectory_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        if (current_state_ == FlightState::REACHING_TARGET) {
            // Position control to reach target altitude
            trajectory_msg.position[0] = takeoff_position_.x;  // Hold X position
            trajectory_msg.position[1] = takeoff_position_.y;  // Hold Y position
            trajectory_msg.position[2] = target_altitude_;     // Target altitude
            trajectory_msg.yaw = yaw_;                         // Hold current yaw
        } else if (current_state_ == FlightState::HOVERING) {
            // Hold position at target altitude
            trajectory_msg.position[0] = takeoff_position_.x;
            trajectory_msg.position[1] = takeoff_position_.y;
            trajectory_msg.position[2] = target_altitude_;
            trajectory_msg.yaw = yaw_;
        }

        // Set unused fields to NaN
        trajectory_msg.velocity[0] = std::numeric_limits<float>::quiet_NaN();
        trajectory_msg.velocity[1] = std::numeric_limits<float>::quiet_NaN();
        trajectory_msg.velocity[2] = std::numeric_limits<float>::quiet_NaN();
        trajectory_msg.acceleration[0] = std::numeric_limits<float>::quiet_NaN();
        trajectory_msg.acceleration[1] = std::numeric_limits<float>::quiet_NaN();
        trajectory_msg.acceleration[2] = std::numeric_limits<float>::quiet_NaN();
        trajectory_msg.yawspeed = std::numeric_limits<float>::quiet_NaN();

        trajectory_publisher_->publish(trajectory_msg);
    }
}

/**
 * @brief Main function to start the SingleDroneControl node.
 */
int main(int argc, char *argv[]) {
    std::cout << "============================================================" << std::endl;
    std::cout << "Single Drone Flight Control" << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cout << "This node will autonomously:" << std::endl;
    std::cout << "1. Arm the drone" << std::endl;
    std::cout << "2. Takeoff" << std::endl;
    std::cout << "3. Climb to 5 meters altitude" << std::endl;
    std::cout << "4. Hover at target altitude" << std::endl;
    std::cout << "============================================================" << std::endl;

    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<SingleDroneControl>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}
