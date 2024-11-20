#include "states/IdleState.hpp"
#include <iostream>

// Constructor
IdleState::IdleState(fsm::FSM* fsm)
    : BaseAUVState(fsm, "IDLE") {}

// onEntry: Reset all control data
fsm::retval IdleState::OnEntry() noexcept {
    // Ensure control data is valid
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("IdleState"), "Control data is null!");
        return fsm::fail;
    }

    RCLCPP_INFO(rclcpp::get_logger("IdleState"), "Entering IDLE state");

    // Reset desired velocities to zero
    ctrlData->velocityDesired.setZero();

    // Reset joystick velocities to zero
    ctrlData->joystickVelocityDesired.linear.x = 0.0;
    ctrlData->joystickVelocityDesired.linear.y = 0.0;
    ctrlData->joystickVelocityDesired.linear.z = 0.0;
    ctrlData->joystickVelocityDesired.angular.x = 0.0;
    ctrlData->joystickVelocityDesired.angular.y = 0.0;
    ctrlData->joystickVelocityDesired.angular.z = 0.0;

    // Reset the goal pose to zero
    ctrlData->poseGoal.setZero();

    return fsm::ok;
}

// execute: Perform no active operations
fsm::retval IdleState::Execute() noexcept {
    // Ensure control data is valid
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("IdleState"), "Control data is null!");
        return fsm::fail;
    }

    // The idle state does not perform any operations
    return fsm::ok;
}

// onExit: Log the state exit
fsm::retval IdleState::OnExit() noexcept {
    RCLCPP_INFO(rclcpp::get_logger("IdleState"), "Exiting IDLE state");

    // No specific cleanup is needed for the idle state
    return fsm::ok;
}
