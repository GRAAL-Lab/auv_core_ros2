#include "states/HoldState.hpp"
#include <iostream> // For debugging, if needed

// Constructor
HoldState::HoldState(fsm::FSM* fsm)
    : BaseAUVState(fsm, "HOLD") {
    // Constructor logic (if needed)
}

// onEntry: Initialize the hold state
fsm::retval HoldState::OnEntry() noexcept {
    // Ensure control data is valid
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("HoldState"), "Control data is null!");
        return fsm::fail;
    }

    RCLCPP_INFO(rclcpp::get_logger("HoldState"), "Entering HOLD state");

    // Set the goal pose to the current pose
    ctrlData->poseGoal = ctrlData->poseActual;


    // Initialize PID gains here
    gainsX_ = {ctrlData->gainsX(0), ctrlData->gainsX(1), ctrlData->gainsX(2),
               ctrlData->gainsX(3), ctrlData->gainsX(4), ctrlData->gainsX(5)};
    gainsY_ = {ctrlData->gainsY(0), ctrlData->gainsY(1), ctrlData->gainsY(2),
               ctrlData->gainsY(3), ctrlData->gainsY(4), ctrlData->gainsY(5)};
    gainsZ_ = {ctrlData->gainsZ(0), ctrlData->gainsZ(1), ctrlData->gainsZ(2),
               ctrlData->gainsZ(3), ctrlData->gainsZ(4), ctrlData->gainsZ(5)};
    gainsRoll_ = {ctrlData->gainsRoll(0), ctrlData->gainsRoll(1), ctrlData->gainsRoll(2),
                  ctrlData->gainsRoll(3), ctrlData->gainsRoll(4), ctrlData->gainsRoll(5)};
    gainsPitch_ = {ctrlData->gainsPitch(0), ctrlData->gainsPitch(1), ctrlData->gainsPitch(2),
                   ctrlData->gainsPitch(3), ctrlData->gainsPitch(4), ctrlData->gainsPitch(5)};
    gainsYaw_ = {ctrlData->gainsYaw(0), ctrlData->gainsYaw(1), ctrlData->gainsYaw(2),
                 ctrlData->gainsYaw(3), ctrlData->gainsYaw(4), ctrlData->gainsYaw(5)};



    // Initialize PID controllers with the specified gains
    pidX_.Initialize(gainsX_, 0.0, ctrlData->maxVelocity(0));
    pidY_.Initialize(gainsY_, 0.0, ctrlData->maxVelocity(1));
    pidZ_.Initialize(gainsZ_, 0.0, ctrlData->maxVelocity(2));
    pidRoll_.Initialize(gainsRoll_, 0.0, ctrlData->maxVelocity(3));
    pidPitch_.Initialize(gainsPitch_, 0.0, ctrlData->maxVelocity(4));
    pidYaw_.Initialize(gainsYaw_, 0.0, ctrlData->maxVelocity(5));

    return fsm::ok;
}

// execute: Perform hold logic
fsm::retval HoldState::Execute() noexcept {
    // Ensure control data is valid
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("HoldState"), "Control data is null!");
        return fsm::fail;
    }

    // Calculate position and orientation errors
    positionXError_ = ctrlData->poseGoal(0) - ctrlData->poseActual(0);
    positionYError_ = ctrlData->poseGoal(1) - ctrlData->poseActual(1);
    positionZError_ = ctrlData->poseGoal(2) - ctrlData->poseActual(2);
    rollError_ = ctrlData->poseGoal(3) - ctrlData->poseActual(3);
    yawError_ = ctrlData->poseGoal(5) - ctrlData->poseActual(5);
    pitchError_ = ctrlData->poseGoal(4) - ctrlData->poseActual(4);
    ctb::NormalizeAngle(rollError_);
    ctb::NormalizeAngle(yawError_);
    ctb::NormalizeAngle(pitchError_);

    // Compute corrective velocities using PID controllers
    ctrlData->velocityDesired(0) = -pidX_.Compute(0, positionXError_);
    ctrlData->velocityDesired(1) = -pidY_.Compute(0, positionYError_);
    ctrlData->velocityDesired(2) = -pidZ_.Compute(0, positionZError_);
    ctrlData->velocityDesired(3) = -pidRoll_.Compute(0, rollError_);
    ctrlData->velocityDesired(4) = -pidPitch_.Compute(0, pitchError_);
    ctrlData->velocityDesired(5) = -pidYaw_.Compute(0, yawError_);

    // Log computed errors and velocities (for debugging, if needed)
    // RCLCPP_INFO(rclcpp::get_logger("HoldState"),
    //             "Errors: X=%.2f, Y=%.2f, Z=%.2f, Roll=%.2f, Pitch=%.2f, Yaw=%.2f",
    //             positionXError_, positionYError_, positionZError_, rollError_, pitchError_, yawError_);
    // RCLCPP_INFO(rclcpp::get_logger("HoldState"),
    //             "Computed Velocities: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
    //             ctrlData->velocityDesired(0), ctrlData->velocityDesired(1),
    //             ctrlData->velocityDesired(2), ctrlData->velocityDesired(3),
    //             ctrlData->velocityDesired(4), ctrlData->velocityDesired(5));

    return fsm::ok;
}

// onExit: Cleanup logic
fsm::retval HoldState::OnExit() noexcept {
    RCLCPP_INFO(rclcpp::get_logger("HoldState"), "Exiting HOLD state");

    // No specific cleanup is needed in this implementation
    return fsm::ok;
}
