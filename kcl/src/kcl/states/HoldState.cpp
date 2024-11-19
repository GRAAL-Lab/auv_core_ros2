#include "states/HoldState.hpp"
#include <iostream> // For debugging if needed

// Constructor
HoldState::HoldState(fsm::FSM* fsm)
    : BaseAUVState(fsm, "HOLD") {
    // Constructor logic (if needed)
}

// OnEntry: Initialize the hold state
fsm::retval HoldState::OnEntry() noexcept {
    // Check if control data is valid
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("HoldState"), "Control data is null!");
        return fsm::fail;
    }

    RCLCPP_INFO(rclcpp::get_logger("HoldState"), "Entering HOLD state");

    // Set the goal pose to the current pose
    ctrlData->pose_goal_ = ctrlData->pose_actual_;

    // Initialize PID gains for each axis
    ctb::PIDGains gainsX = {ctrlData->gainsX_(0), ctrlData->gainsX_(1), ctrlData->gainsX_(2), ctrlData->gainsX_(3), ctrlData->gainsX_(4), ctrlData->gainsX_(5)};
    ctb::PIDGains gainsY = {ctrlData->gainsY_(0), ctrlData->gainsY_(1), ctrlData->gainsY_(2), ctrlData->gainsY_(3), ctrlData->gainsY_(4), ctrlData->gainsY_(5)};
    ctb::PIDGains gainsZ = {ctrlData->gainsZ_(0), ctrlData->gainsZ_(1), ctrlData->gainsZ_(2), ctrlData->gainsZ_(3), ctrlData->gainsZ_(4), ctrlData->gainsZ_(5)};
    ctb::PIDGains gainsRoll = {ctrlData->gainsRoll_(0), ctrlData->gainsRoll_(1), ctrlData->gainsRoll_(2), ctrlData->gainsRoll_(3), ctrlData->gainsRoll_(4), ctrlData->gainsRoll_(5)};
    ctb::PIDGains gainsPitch = {ctrlData->gainsPitch_(0), ctrlData->gainsPitch_(1), ctrlData->gainsPitch_(2), ctrlData->gainsPitch_(3), ctrlData->gainsPitch_(4), ctrlData->gainsPitch_(5)};
    ctb::PIDGains gainsYaw = {ctrlData->gainsYaw_(0), ctrlData->gainsYaw_(1), ctrlData->gainsYaw_(2), ctrlData->gainsYaw_(3), ctrlData->gainsYaw_(4), ctrlData->gainsYaw_(5)};

    // Initialize PID controllers with the specified gains
    pidX.Initialize(gainsX, 0.0, ctrlData->maxVelocity_[0]);
    pidY.Initialize(gainsY, 0.0, ctrlData->maxVelocity_[1]);
    pidZ.Initialize(gainsZ, 0.0, ctrlData->maxVelocity_[2]);
    pidRoll.Initialize(gainsRoll, 0.0, ctrlData->maxVelocity_[3]);
    pidPitch.Initialize(gainsPitch, 0.0, ctrlData->maxVelocity_[4]);
    pidYaw.Initialize(gainsYaw, 0.0, ctrlData->maxVelocity_[5]);

    return fsm::ok;
}

// Execute: Perform hold logic
fsm::retval HoldState::Execute() noexcept {
    // Check if control data is valid
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("HoldState"), "Control data is null!");
        return fsm::fail;
    }

    // Calculate position and orientation errors
    positionXError = ctrlData->pose_goal_(0) - ctrlData->pose_actual_(0);
    positionYError = ctrlData->pose_goal_(1) - ctrlData->pose_actual_(1);
    positionZError = ctrlData->pose_goal_(2) - ctrlData->pose_actual_(2);
    rollError = ctrlData->pose_goal_(3) - ctrlData->pose_actual_(3);
    pitchError = ctrlData->pose_goal_(4) - ctrlData->pose_actual_(4);
    yawError = ctrlData->pose_goal_(5) - ctrlData->pose_actual_(5);

    // Normalize angular errors
    ctb::NormalizeAngle(rollError);
    ctb::NormalizeAngle(pitchError);
    ctb::NormalizeAngle(yawError);

    // Compute corrective velocities using PID controllers
    ctrlData->velocity_desired_(0) = -pidX.Compute(0, positionXError);
    ctrlData->velocity_desired_(1) = -pidY.Compute(0, positionYError);
    ctrlData->velocity_desired_(2) = -pidZ.Compute(0, positionZError);
    ctrlData->velocity_desired_(3) = -pidRoll.Compute(0, rollError);
    ctrlData->velocity_desired_(4) = -pidPitch.Compute(0, pitchError);
    ctrlData->velocity_desired_(5) = -pidYaw.Compute(0, yawError);

    // Log computed errors and velocities (for debugging)
    RCLCPP_INFO(rclcpp::get_logger("HoldState"),
                "Errors: X=%.2f, Y=%.2f, Z=%.2f, Roll=%.2f, Pitch=%.2f, Yaw=%.2f",
                positionXError, positionYError, positionZError, rollError, pitchError, yawError);

    RCLCPP_INFO(rclcpp::get_logger("HoldState"),
                "Computed Velocities: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                ctrlData->velocity_desired_(0), ctrlData->velocity_desired_(1),
                ctrlData->velocity_desired_(2), ctrlData->velocity_desired_(3),
                ctrlData->velocity_desired_(4), ctrlData->velocity_desired_(5));

    return fsm::ok;
}

// OnExit: Cleanup logic
fsm::retval HoldState::OnExit() noexcept {
    RCLCPP_INFO(rclcpp::get_logger("HoldState"), "Exiting HOLD state");

    // No specific cleanup is needed in this implementation
    return fsm::ok;
}
