#include "states/hold_state.hpp"

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
    ctb::PIDGains gainsX = {ctrlData->gainsX(0), ctrlData->gainsX(1), ctrlData->gainsX(2), ctrlData->gainsX(3), ctrlData->gainsX(4), ctrlData->gainsX(5)};
    ctb::PIDGains gainsY = {ctrlData->gainsY(0), ctrlData->gainsY(1), ctrlData->gainsY(2), ctrlData->gainsY(3), ctrlData->gainsY(4), ctrlData->gainsY(5)};
    ctb::PIDGains gainsZ = {ctrlData->gainsZ(0), ctrlData->gainsZ(1), ctrlData->gainsZ(2), ctrlData->gainsZ(3), ctrlData->gainsZ(4), ctrlData->gainsZ(5)};
    ctb::PIDGains gainsRoll = {ctrlData->gainsRoll(0), ctrlData->gainsRoll(1), ctrlData->gainsRoll(2), ctrlData->gainsRoll(3), ctrlData->gainsRoll(4), ctrlData->gainsRoll(5)};
    ctb::PIDGains gainsPitch = {ctrlData->gainsPitch(0), ctrlData->gainsPitch(1), ctrlData->gainsPitch(2), ctrlData->gainsPitch(3), ctrlData->gainsPitch(4), ctrlData->gainsPitch(5)};
    ctb::PIDGains gainsYaw = {ctrlData->gainsYaw(0), ctrlData->gainsYaw(1), ctrlData->gainsYaw(2), ctrlData->gainsYaw(3), ctrlData->gainsYaw(4), ctrlData->gainsYaw(5)};


    // Initialize PID controllers with the specified gains
    pidX_.Initialize(gainsX, ctrlData->dt, (ctrlData->maxVelocity(0) > std::abs(ctrlData->minVelocity(0))) ? ctrlData->maxVelocity(0) : std::abs(ctrlData->minVelocity(0))); // Initialize the PID controller for longitudinal position
    pidY_.Initialize(gainsY, ctrlData->dt, (ctrlData->maxVelocity(1) > std::abs(ctrlData->minVelocity(1))) ? ctrlData->maxVelocity(1) : std::abs(ctrlData->minVelocity(1))); // Initialize the PID controller for lateral position
    pidZ_.Initialize(gainsZ, ctrlData->dt, (ctrlData->maxVelocity(2) > std::abs(ctrlData->minVelocity(2))) ? ctrlData->maxVelocity(2) : std::abs(ctrlData->minVelocity(2))); // Initialize the PID controller for depth position
    pidRoll_.Initialize(gainsRoll, ctrlData->dt, (ctrlData->maxVelocity(3) > std::abs(ctrlData->minVelocity(3))) ? ctrlData->maxVelocity(3) : std::abs(ctrlData->minVelocity(3))); // Initialize the PID controller for roll
    pidPitch_.Initialize(gainsPitch, ctrlData->dt, (ctrlData->maxVelocity(4) > std::abs(ctrlData->minVelocity(4))) ? ctrlData->maxVelocity(4) : std::abs(ctrlData->minVelocity(4))); // Initialize the PID controller for pitch
    pidYaw_.Initialize(gainsYaw, ctrlData->dt, (ctrlData->maxVelocity(5) > std::abs(ctrlData->minVelocity(5))) ? ctrlData->maxVelocity(5) : std::abs(ctrlData->minVelocity(5))); // Initialize the PID controller for yaw

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
    // Compute desired body angular velocities using PID controllers
    Eigen::Vector3d wDesired = Eigen::Vector3d::Zero();
    wDesired[0] = -pidRoll_.Compute(0, rollError_);   // Roll rate
    wDesired[1] = -pidPitch_.Compute(0, pitchError_); // Pitch rate
    wDesired[2] = -pidYaw_.Compute(0, yawError_);     // Yaw rate
    // Convert body angular velocities to Euler angle rates
    Eigen::Vector3d eulerRatesDesired = ConvertAngularVelocitiesToEulerRates(ctrlData->poseActual(3), ctrlData->poseActual(4), wDesired);
    // Set the desired Euler angle rates
    ctrlData->velocityDesired(3) = eulerRatesDesired[0]; // Desired roll rate
    ctrlData->velocityDesired(4) = eulerRatesDesired[1]; // Desired pitch rate
    ctrlData->velocityDesired(5) = eulerRatesDesired[2]; // Desired yaw rate


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
