#include "states/JoystickState.hpp"
#include <iostream>

// Constructor
JoystickState::JoystickState(fsm::FSM* fsm) : BaseAUVState(fsm, "JOYSTICK") {}

// OnEntry
fsm::retval JoystickState::OnEntry() {
    std::cout << "Entering JOYSTICK state" << std::endl;
    ctrlData->pose_goal_.setZero();
    return fsm::ok;
}

// Execute
fsm::retval JoystickState::Execute() {
    // Process joystick input for control
    ctrlData->velocity_desired_(0) = ctrlData->joystick_velocity_desired_.linear.x;
    ctrlData->velocity_desired_(1) = ctrlData->joystick_velocity_desired_.linear.y;
    ctrlData->velocity_desired_(2) = ctrlData->joystick_velocity_desired_.linear.z;
    ctrlData->velocity_desired_(3) = ctrlData->joystick_velocity_desired_.angular.x;
    ctrlData->velocity_desired_(4) = ctrlData->joystick_velocity_desired_.angular.y;
    ctrlData->velocity_desired_(5) = ctrlData->joystick_velocity_desired_.angular.z;
    return fsm::ok;
}

// OnExit
fsm::retval JoystickState::OnExit() {
    std::cout << "Exiting JOYSTICK state" << std::endl;
    return fsm::ok;
}
