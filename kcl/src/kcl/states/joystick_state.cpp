#include "states/joystick_state.hpp"
#include <cmath>
#include <cstdio>

double JoystickState::map(double x, double in_min, double in_max,double out_min, double out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min)+ out_min;
}

// ---------------------------------------------------------
JoystickState::JoystickState(fsm::FSM* fsm)
    : BaseAUVState(fsm, "JOYSTICK")
{}

// ---------------------------------------------------------
fsm::retval JoystickState::OnEntry() noexcept{
    if (!ctrlData->joystickMsg) {
        RCLCPP_ERROR(rclcpp::get_logger("JoystickState"), "Joystick not connected or node not running.");
        return fsm::fail;
    }

    RCLCPP_INFO(rclcpp::get_logger("JoystickState"),"Entering JOYSTICK state");

    ctrlData->poseGoal.setZero();
    ctrlData->velocityDesired.setZero();
    return fsm::ok;
}

fsm::retval JoystickState::Execute() noexcept
{
    if (!ctrlData->joystickMsg) {
        RCLCPP_ERROR(rclcpp::get_logger("JoystickState"),"Joystick not connected or node not running.");
        return fsm::fail;
    }


    // --------------------------------------------------
    // SURGE (forward/back)  axis 1
    // --------------------------------------------------
    ctrlData->velocityDesired[0] =map(ctrlData->joystickMsg->axes[AXIS_SURGE], -1.0, 1.0,ctrlData->minVelocity[0],ctrlData->maxVelocity[0]);
    ctrlData->velocityDesired[0] =std::clamp(ctrlData->velocityDesired[0],ctrlData->minVelocity[0],ctrlData->maxVelocity[0]);

    // --------------------------------------------------
    // SWAY (left/right) axis 0
    // --------------------------------------------------
    ctrlData->velocityDesired[1] =map(ctrlData->joystickMsg->axes[AXIS_SWAY], -1.0, 1.0,ctrlData->minVelocity[1],ctrlData->maxVelocity[1]);
    ctrlData->velocityDesired[1] =std::clamp(ctrlData->velocityDesired[1],ctrlData->minVelocity[1],ctrlData->maxVelocity[1]);


    // --------------------------------------------------
    // HEAVE (up/down) axis 4
    // --------------------------------------------------
    ctrlData->velocityDesired[2] =map(ctrlData->joystickMsg->axes[AXIS_HEAVE], -1.0, 1.0,ctrlData->minVelocity[2],ctrlData->maxVelocity[2]);
    ctrlData->velocityDesired[2] =std::clamp(ctrlData->velocityDesired[2],ctrlData->minVelocity[2],ctrlData->maxVelocity[2]);

    // --------------------------------------------------
    // ROLL
    // --------------------------------------------------
    double rt_norm = (1.0 - ctrlData->joystickMsg->axes[AXIS_ROLL_RIGHT]) * 0.5;
    double lt_norm = (1.0 - ctrlData->joystickMsg->axes[AXIS_ROLL_LEFT])  * 0.5;
    double roll_norm = rt_norm - lt_norm;   // -1 .. +1
    ctrlData->velocityDesired[3] =map(roll_norm, -1.0, 1.0, ctrlData->minVelocity[3], ctrlData->maxVelocity[3]);
    ctrlData->velocityDesired[3] =std::clamp(ctrlData->velocityDesired[3],ctrlData->minVelocity[3],ctrlData->maxVelocity[3]);
    
    // --------------------------------------------------
    // PITCH 
    // --------------------------------------------------
    // ctrlData->joystickMsg->axes[AXIS_PITCH] = (1.0 - ctrlData->joystickMsg->axes[AXIS_PITCH]) * 0.5;
    // ctrlData->velocityDesired[4] = map(ctrlData->joystickMsg->axes[AXIS_PITCH], -1.0, 1.0, ctrlData->minVelocity[4], ctrlData->maxVelocity[4]);
    // ctrlData->velocityDesired[4] = std::clamp(ctrlData->velocityDesired[4], ctrlData->minVelocity[4], ctrlData->maxVelocity[4]);

    // --------------------------------------------------
    // YAW (heading) axis 3
    // --------------------------------------------------
    ctrlData->velocityDesired[5] =map(ctrlData->joystickMsg->axes[AXIS_YAW], -1.0, 1.0,ctrlData->minVelocity[5],ctrlData->maxVelocity[5]);
    ctrlData->velocityDesired[5] =std::clamp(ctrlData->velocityDesired[5],ctrlData->minVelocity[5],ctrlData->maxVelocity[5]);

    return fsm::ok;
}


// ---------------------------------------------------------
fsm::retval JoystickState::OnExit() noexcept
{
    RCLCPP_INFO(rclcpp::get_logger("JoystickState"),"Exiting JOYSTICK state");

    ctrlData->velocityDesired.setZero();
    return fsm::ok;
}
