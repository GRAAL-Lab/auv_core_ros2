#include "states/IdleState.hpp"
#include <iostream>

// Constructor
IdleState::IdleState(fsm::FSM* fsm)
    : BaseAUVState(fsm, "IDLE") {}

// OnEntry: Reset all control data
fsm::retval IdleState::OnEntry() noexcept {
    // Ensure control data is valid
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("IdleState"), "Control data is null!");
        return fsm::fail;
    }

    RCLCPP_INFO(rclcpp::get_logger("IdleState"), "Entering IDLE state");

    // Reset desired velocities to zero
    ctrlData->velocity_desired_.setZero();

    // Reset joystick velocities to zero
    ctrlData->joystick_velocity_desired_.linear.x = 0.0;
    ctrlData->joystick_velocity_desired_.linear.y = 0.0;
    ctrlData->joystick_velocity_desired_.linear.z = 0.0;
    ctrlData->joystick_velocity_desired_.angular.x = 0.0;
    ctrlData->joystick_velocity_desired_.angular.y = 0.0;
    ctrlData->joystick_velocity_desired_.angular.z = 0.0;

    // Reset the goal pose to zero
    ctrlData->pose_goal_.setZero();

    return fsm::ok;
}

// Execute: Perform no active operations
fsm::retval IdleState::Execute() noexcept {
    // Ensure control data is valid
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("IdleState"), "Control data is null!");
        return fsm::fail;
    }

    // The idle state does not perform any operations
    return fsm::ok;
}

// OnExit: Log the state exit
fsm::retval IdleState::OnExit() noexcept {
    RCLCPP_INFO(rclcpp::get_logger("IdleState"), "Exiting IDLE state");

    // No specific cleanup needed for the idle state
    return fsm::ok;
}
