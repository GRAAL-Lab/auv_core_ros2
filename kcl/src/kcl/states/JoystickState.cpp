#include "states/JoystickState.hpp"
#include <iostream>

// Constructor
JoystickState::JoystickState(fsm::FSM* fsm)
    : BaseAUVState(fsm, "JOYSTICK") {}

// OnEntry: Initialize joystick state
fsm::retval JoystickState::OnEntry() noexcept {
    // Check if control data is valid
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("JoystickState"), "Control data is null!");
        return fsm::fail;
    }

    RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Entering JOYSTICK state");

    // Reset pose goal to zero
    ctrlData->pose_goal_.setZero();

    return fsm::ok;
}

// Execute: Process joystick input
fsm::retval JoystickState::Execute() noexcept {
    // Check if control data is valid
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("JoystickState"), "Control data is null!");
        return fsm::fail;
    }

    // Update desired velocities from joystick inputs
    ctrlData->velocity_desired_(0) = ctrlData->joystick_velocity_desired_.linear.x;
    ctrlData->velocity_desired_(1) = ctrlData->joystick_velocity_desired_.linear.y;
    ctrlData->velocity_desired_(2) = ctrlData->joystick_velocity_desired_.linear.z;
    ctrlData->velocity_desired_(3) = ctrlData->joystick_velocity_desired_.angular.x;
    ctrlData->velocity_desired_(4) = ctrlData->joystick_velocity_desired_.angular.y;
    ctrlData->velocity_desired_(5) = ctrlData->joystick_velocity_desired_.angular.z;

    // Log the desired velocities for debugging
    RCLCPP_INFO(rclcpp::get_logger("JoystickState"),
                "Joystick velocities: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                ctrlData->velocity_desired_(0), ctrlData->velocity_desired_(1),
                ctrlData->velocity_desired_(2), ctrlData->velocity_desired_(3),
                ctrlData->velocity_desired_(4), ctrlData->velocity_desired_(5));

    return fsm::ok;
}

// OnExit: Cleanup joystick state
fsm::retval JoystickState::OnExit() noexcept {
    RCLCPP_INFO(rclcpp::get_logger("JoystickState"), "Exiting JOYSTICK state");

    // No specific cleanup is needed in this implementation
    return fsm::ok;
}
