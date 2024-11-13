#include "states/IdleState.hpp"
#include <iostream>

// Constructor
IdleState::IdleState(fsm::FSM* fsm) : BaseAUVState(fsm, "IDLE") {}

// OnEntry
fsm::retval IdleState::OnEntry() {
    std::cout << "Entering IDLE state" << std::endl;
    // Reset everything to zero
    ctrlData->velocity_desired_.setZero();
    ctrlData->joystick_velocity_desired_.linear.x = 0;
    ctrlData->joystick_velocity_desired_.linear.y = 0;
    ctrlData->joystick_velocity_desired_.linear.z = 0;
    ctrlData->joystick_velocity_desired_.angular.x = 0;
    ctrlData->joystick_velocity_desired_.angular.y = 0;
    ctrlData->joystick_velocity_desired_.angular.z = 0;
    ctrlData->pose_goal_.setZero();
    return fsm::ok;
}

// Execute
fsm::retval IdleState::Execute() {
    return fsm::ok;
}

// OnExit
fsm::retval IdleState::OnExit() {
    std::cout << "Exiting IDLE state" << std::endl;
    return fsm::ok;
}
