#include "states/HoldState.hpp"
#include <iostream>

// Constructor
HoldState::HoldState(fsm::FSM* fsm) : BaseAUVState(fsm, "HOLD") {}

// OnEntry
fsm::retval HoldState::OnEntry() {
    std::cout << "Entering HOLD state" << std::endl;
    ctrlData->pose_goal_ = ctrlData->pose_actual_; // Set the goal pose to the current pose

    // Initialize PID controllers
    ctb::PIDGains gainsX = {ctrlData->gainsX_(0), ctrlData->gainsX_(1), ctrlData->gainsX_(2), ctrlData->gainsX_(3), ctrlData->gainsX_(4), ctrlData->gainsX_(5)};
    ctb::PIDGains gainsY = {ctrlData->gainsY_(0), ctrlData->gainsY_(1), ctrlData->gainsY_(2), ctrlData->gainsY_(3), ctrlData->gainsY_(4), ctrlData->gainsY_(5)};
    ctb::PIDGains gainsZ = {ctrlData->gainsZ_(0), ctrlData->gainsZ_(1), ctrlData->gainsZ_(2), ctrlData->gainsZ_(3), ctrlData->gainsZ_(4), ctrlData->gainsZ_(5)};
    ctb::PIDGains gainsRoll = {ctrlData->gainsRoll_(0), ctrlData->gainsRoll_(1), ctrlData->gainsRoll_(2), ctrlData->gainsRoll_(3), ctrlData->gainsRoll_(4), ctrlData->gainsRoll_(5)};
    ctb::PIDGains gainsPitch = {ctrlData->gainsPitch_(0), ctrlData->gainsPitch_(1), ctrlData->gainsPitch_(2), ctrlData->gainsPitch_(3), ctrlData->gainsPitch_(4), ctrlData->gainsPitch_(5)};
    ctb::PIDGains gainsYaw = {ctrlData->gainsYaw_(0), ctrlData->gainsYaw_(1), ctrlData->gainsYaw_(2), ctrlData->gainsYaw_(3), ctrlData->gainsYaw_(4), ctrlData->gainsYaw_(5)};


    pidX.Initialize(gainsX, 0.0, ctrlData->maxVelocity_[0]); // Initialize the PID controller for longitudinal position
    pidY.Initialize(gainsY, 0.0, ctrlData->maxVelocity_[1]); // Initialize the PID controller for lateral position
    pidZ.Initialize(gainsZ, 0.0, ctrlData->maxVelocity_[2]); // Initialize the PID controller for depth position
    pidRoll.Initialize(gainsRoll, 0.0, ctrlData->maxVelocity_[3]); // Initialize the PID controller for roll
    pidPitch.Initialize(gainsPitch, 0.0, ctrlData->maxVelocity_[4]); // Initialize the PID controller for pitch
    pidYaw.Initialize(gainsYaw, 0.0, ctrlData->maxVelocity_[5]); // Initialize the PID controller for yaw

    return fsm::ok;
}

// Execute
fsm::retval HoldState::Execute() {
    // Compute velocity desired using proportional control for all components

    // Compute errors and desired velocities for x, y, pitch, and yaw
    positionXError = ctrlData->pose_goal_(0) - ctrlData->pose_actual_(0);
    positionYError = ctrlData->pose_goal_(1) - ctrlData->pose_actual_(1);
    positionZError = ctrlData->pose_goal_(2) - ctrlData->pose_actual_(2);
    rollError = ctrlData->pose_goal_(3) - ctrlData->pose_actual_(3);
    yawError = ctrlData->pose_goal_(5) - ctrlData->pose_actual_(5);
    pitchError = ctrlData->pose_goal_(4) - ctrlData->pose_actual_(4);
    ctb::NormalizeAngle(rollError);
    ctb::NormalizeAngle(yawError);
    ctb::NormalizeAngle(pitchError);

    // std::cout << "Errors: " << positionXError << " " << positionYError << " " << positionZError << " " << rollError << " " << pitchError << " " << yawError << std::endl;

    // Set desired velocities
    ctrlData->velocity_desired_(0) = -pidX.Compute(0, positionXError);
    ctrlData->velocity_desired_(1) = -pidY.Compute(0, positionYError);
    ctrlData->velocity_desired_(2) = -pidZ.Compute(0, positionZError);
    ctrlData->velocity_desired_(3) = -pidRoll.Compute(0, rollError);
    ctrlData->velocity_desired_(4) = -pidPitch.Compute(0, pitchError);
    ctrlData->velocity_desired_(5) = -pidYaw.Compute(0, yawError);

    return fsm::ok;
}

fsm::retval HoldState::OnExit() {
    std::cout << "Exiting HOLD state" << std::endl;
    return fsm::ok;
}
