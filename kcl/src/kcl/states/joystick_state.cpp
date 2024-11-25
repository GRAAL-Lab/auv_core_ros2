#include "states/joystick_state.hpp"

// Constructor
JoystickState::JoystickState(fsm::FSM* fsm)
    : BaseAUVState(fsm, "JOYSTICK") {}

// onEntry: Initialize joystick state
fsm::retval JoystickState::OnEntry() noexcept {
    // Ensure control data is valid
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("JoystickState"), "Control data is null!");
        return fsm::fail;
    }

    RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Entering JOYSTICK state");

    // Reset pose goal to zero
    ctrlData->poseGoal.setZero();

    
    CalibrateJoystick();


    return fsm::ok;
}

// execute: Process joystick input
fsm::retval JoystickState::Execute() noexcept {
    // Ensure control data is valid
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("JoystickState"), "Control data is null!");
        return fsm::fail;
    }

    MapJoystickToVelocity(ctrlData->joystickAxes,  &ctrlData->joystickVelocityDesired);

    // Update desired velocities from joystick inputs
    ctrlData->velocityDesired(0) = ctrlData->joystickVelocityDesired.linear.x;
    ctrlData->velocityDesired(1) = ctrlData->joystickVelocityDesired.linear.y;
    ctrlData->velocityDesired(2) = ctrlData->joystickVelocityDesired.linear.z;
    ctrlData->velocityDesired(3) = ctrlData->joystickVelocityDesired.angular.x;
    ctrlData->velocityDesired(4) = ctrlData->joystickVelocityDesired.angular.y;
    ctrlData->velocityDesired(5) = ctrlData->joystickVelocityDesired.angular.z;

    // Log the desired velocities for debugging (optional)
    // RCLCPP_INFO(rclcpp::get_logger("JoystickState"),
    //             "Joystick velocities: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
    //             ctrlData->velocityDesired(0), ctrlData->velocityDesired(1),
    //             ctrlData->velocityDesired(2), ctrlData->velocityDesired(3),
    //             ctrlData->velocityDesired(4), ctrlData->velocityDesired(5));

    return fsm::ok;
}

// onExit: Cleanup joystick state
fsm::retval JoystickState::OnExit() noexcept {
    RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Exiting JOYSTICK state");

    // No specific cleanup is needed in this implementation
    return fsm::ok;
}


// Calibrate joystick inputs
void JoystickState::CalibrateJoystick() {
    RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Starting joystick calibration...");

    // List of calibration steps
    std::vector<std::string> calibrationSteps = {
        "Move forward axis to maximum forward",
        "Move forward axis to maximum backward",
        "Yaw maximum clockwise",
        "Yaw maximum counter-clockwise",
        "Up maximum forward",
        "Down maximum backward",
        "Right maximum",
        "Left maximum",
        "Roll maximum clockwise",
        "Roll maximum counter-clockwise",
        "Pitch bottom clockwise",
        "Pitch bottom counter-clockwise"
    };

    for (const auto& step : calibrationSteps) {
        RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "%s", step.c_str());
        RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Press and hold the joystick input, then press Enter...");

        // Wait for user to press Enter
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        // Capture joystick input
        bool inputCaptured = false;

        // Check joystick axes
        for (size_t i = 0; i < ctrlData->joystickAxes.size(); ++i) {
            if (fabs(ctrlData->joystickAxes[i]) > 0.1) { // Assuming threshold to detect active input
                joystickCalibration[step] = i;
                joystickMaxValues[step] = ctrlData->joystickAxes[i];
                RCLCPP_INFO(rclcpp::get_logger("JoystickState"),
                            "Mapped '%s' to axis %zu with value %.2f",
                            step.c_str(), i, ctrlData->joystickAxes[i]);
                inputCaptured = true;
                break;
            }
        }

        if (!inputCaptured) {
            RCLCPP_WARN(rclcpp::get_logger("JoystickState"),
                        "No input detected for '%s'. Try again.", step.c_str());
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Joystick calibration completed.");
}


void JoystickState::MapJoystickToVelocity(const std::vector<float>& axes, geometry_msgs::msg::Twist* velocity_desired) {
    if (axes.size() < 6) return; // Ensure there are enough axes

    // Linear velocities based on joystick input
    double raw_linear_x = 1.0 * axes[0]; // Left/right
    double raw_linear_y = 1.0 * axes[1]; // Forward/backward
    double raw_linear_z = ((axes[3] + 1) / 2) * 1.0 - ((axes[4] + 1) / 2) * 1.0; // Up/down

    // Angular velocities based on joystick input
    double raw_angular_z = 0.5 * axes[2]; // Yaw (turn left/right)
    double raw_angular_y = 0.5 * axes[5]; // Pitch (tilt forward/backward)

    // Calculate magnitudes and scale
    double magnitude_linear = sqrt(raw_linear_x * raw_linear_x + raw_linear_y * raw_linear_y + raw_linear_z * raw_linear_z);
    double magnitude_angular = sqrt(raw_angular_z * raw_angular_z + raw_angular_y * raw_angular_y);

    // Normalize and scale linear velocities
    if (magnitude_linear > 0) {
        double max_linear_speed = 1.5; // Adjust as needed
        velocity_desired->linear.x = (raw_linear_x / magnitude_linear) * max_linear_speed;
        velocity_desired->linear.y = (raw_linear_y / magnitude_linear) * max_linear_speed;
        velocity_desired->linear.z = (raw_linear_z / magnitude_linear) * max_linear_speed;
    } else {
        velocity_desired->linear.x = 0;
        velocity_desired->linear.y = 0;
        velocity_desired->linear.z = 0;
    }

    // Normalize and scale angular velocities
    if (magnitude_angular > 0) {
        double max_angular_speed = 2.8; // Adjust as needed
        velocity_desired->angular.z = (raw_angular_z / magnitude_angular) * max_angular_speed;
        velocity_desired->angular.y = (raw_angular_y / magnitude_angular) * max_angular_speed;
    } else {
        velocity_desired->angular.z = 0;
        velocity_desired->angular.y = 0;
    }
}
