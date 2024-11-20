#include "states/joystick_state.hpp"
#include <iostream>

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

    return fsm::ok;
}

// execute: Process joystick input
fsm::retval JoystickState::Execute() noexcept {
    // Ensure control data is valid
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("JoystickState"), "Control data is null!");
        return fsm::fail;
    }

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
