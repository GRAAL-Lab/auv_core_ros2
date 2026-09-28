#include "states/waypoint_navigation_state.hpp"

#include <algorithm>
#include <cmath>

// Constructor
WaypointNavigationState::WaypointNavigationState(fsm::FSM* fsm)
    : BaseAUVState(fsm, States::WAYPOINT_NAVIGATION) {
}

// OnEntry: Initialize waypoint navigation
fsm::retval WaypointNavigationState::OnEntry() noexcept {
    if (!ctrlData || !ctrlData->poseGoal.allFinite() ||
        !ctrlData->poseActual.allFinite()) {
        RCLCPP_ERROR(rclcpp::get_logger("WaypointNavigationState"), "Control data or pose is invalid!");
        return fsm::fail;
    }

    alignmentPosition_ = ctrlData->poseActual.head<3>();
    phase_ = Phase::Align;
    ctrlData->velocityDesired.setZero();

    // Initialize PID gains from the shared control data
    ctb::PIDGains gainsX = {ctrlData->gainsX(0), ctrlData->gainsX(1), ctrlData->gainsX(2), ctrlData->gainsX(3), ctrlData->gainsX(4), ctrlData->gainsX(5)};
    ctb::PIDGains gainsY = {ctrlData->gainsY(0), ctrlData->gainsY(1), ctrlData->gainsY(2), ctrlData->gainsY(3), ctrlData->gainsY(4), ctrlData->gainsY(5)};
    ctb::PIDGains gainsZ = {ctrlData->gainsZ(0), ctrlData->gainsZ(1), ctrlData->gainsZ(2), ctrlData->gainsZ(3), ctrlData->gainsZ(4), ctrlData->gainsZ(5)};
    ctb::PIDGains gainsRoll = {ctrlData->gainsRoll(0), ctrlData->gainsRoll(1), ctrlData->gainsRoll(2), ctrlData->gainsRoll(3), ctrlData->gainsRoll(4), ctrlData->gainsRoll(5)};
    ctb::PIDGains gainsPitch = {ctrlData->gainsPitch(0), ctrlData->gainsPitch(1), ctrlData->gainsPitch(2), ctrlData->gainsPitch(3), ctrlData->gainsPitch(4), ctrlData->gainsPitch(5)};
    ctb::PIDGains gainsYaw = {ctrlData->gainsYaw(0), ctrlData->gainsYaw(1), ctrlData->gainsYaw(2), ctrlData->gainsYaw(3), ctrlData->gainsYaw(4), ctrlData->gainsYaw(5)};

    // Initialize PID controllers with the configured velocity limits
    pidX_.Initialize(gainsX, ctrlData->dt, std::max(ctrlData->maxVelocity(0), std::abs(ctrlData->minVelocity(0))));
    pidY_.Initialize(gainsY, ctrlData->dt, std::max(ctrlData->maxVelocity(1), std::abs(ctrlData->minVelocity(1))));
    pidZ_.Initialize(gainsZ, ctrlData->dt, std::max(ctrlData->maxVelocity(2), std::abs(ctrlData->minVelocity(2))));
    pidRoll_.Initialize(gainsRoll, ctrlData->dt, std::max(ctrlData->maxVelocity(3), std::abs(ctrlData->minVelocity(3))));
    pidPitch_.Initialize(gainsPitch, ctrlData->dt, std::max(ctrlData->maxVelocity(4), std::abs(ctrlData->minVelocity(4))));
    pidYaw_.Initialize(gainsYaw, ctrlData->dt, std::max(ctrlData->maxVelocity(5), std::abs(ctrlData->minVelocity(5))));
    ResetPIDControllers();
    RCLCPP_INFO(rclcpp::get_logger("WaypointNavigationState"),
                "Aligning heading before navigating to waypoint");
    return fsm::ok;
}

// Execute: Align the heading and navigate to the goal pose
fsm::retval WaypointNavigationState::Execute() noexcept {
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("WaypointNavigationState"), "Control data is null!");
        return fsm::fail;
    }
    if (!ctrlData->poseActual.allFinite() || !ctrlData->poseGoal.allFinite()) {
        ctrlData->velocityDesired.setZero();
        return fsm::fail;
    }

    Eigen::Vector3d offset = ctrlData->poseGoal.head<3>() - ctrlData->poseActual.head<3>();
    double horizontalDistance = offset.head<2>().norm();
    double heading = ctrlData->poseGoal(5);
    if (horizontalDistance > POSITION_TOLERANCE) {
        heading = std::atan2(offset.y(), offset.x());
    }
    double headingError = ctb::AngleDifference(heading, ctrlData->poseActual(5));

    if (phase_ != Phase::HoldGoal && offset.norm() <= POSITION_TOLERANCE) {
        phase_ = Phase::HoldGoal;
        ResetPIDControllers();
        RCLCPP_INFO(rclcpp::get_logger("WaypointNavigationState"),
                    "Waypoint reached; maintaining the requested final pose");
    } else if (phase_ == Phase::Navigate && std::abs(headingError) > REALIGN_TOLERANCE) {
        phase_ = Phase::Align;
        alignmentPosition_ = ctrlData->poseActual.head<3>();
        ResetPIDControllers();
    }

    if (phase_ == Phase::Align && std::abs(headingError) <= HEADING_TOLERANCE) {
        phase_ = Phase::Navigate;
        ResetPIDControllers();
    }

    // Alignment holds the entry position; navigation tracks the waypoint in body axes.
    Eigen::Vector3d positionTarget = ctrlData->poseGoal.head<3>();
    if (phase_ == Phase::Align) {
        positionTarget = alignmentPosition_;
    }

    // Calculate position and orientation errors in the World frame
    positionXError_ = positionTarget(0) - ctrlData->poseActual(0);
    positionYError_ = positionTarget(1) - ctrlData->poseActual(1);
    positionZError_ = positionTarget(2) - ctrlData->poseActual(2);
    rollError_ = ctb::AngleDifference(ctrlData->poseGoal(3), ctrlData->poseActual(3));
    pitchError_ = ctb::AngleDifference(ctrlData->poseGoal(4), ctrlData->poseActual(4));
    yawError_ = headingError;
    if (phase_ == Phase::HoldGoal) {
        yawError_ = ctb::AngleDifference(ctrlData->poseGoal(5), ctrlData->poseActual(5));
    }

    // Convert linear position errors to body frame
    rml::EulerRPY rpy(ctrlData->poseActual(3), ctrlData->poseActual(4), ctrlData->poseActual(5));
    Eigen::Matrix3d R = rpy.ToRotationMatrix().matrix();
    Eigen::Vector3d errorWorld(positionXError_, positionYError_, positionZError_);
    Eigen::Vector3d errorBody = R.transpose() * errorWorld;

    // Compute corrective velocities using PID controllers on body-frame errors
    ctrlData->velocityDesired(0) = -pidX_.Compute(0, errorBody(0));
    ctrlData->velocityDesired(1) = -pidY_.Compute(0, errorBody(1));
    ctrlData->velocityDesired(2) = -pidZ_.Compute(0, errorBody(2));

    // Compute desired body angular velocities
    Eigen::Vector3d wDesired = Eigen::Vector3d::Zero();
    wDesired[0] = -pidRoll_.Compute(0, rollError_);
    wDesired[1] = -pidPitch_.Compute(0, pitchError_);
    wDesired[2] = -pidYaw_.Compute(0, yawError_);

    ctrlData->velocityDesired(3) = wDesired[0];
    ctrlData->velocityDesired(4) = wDesired[1];
    ctrlData->velocityDesired(5) = wDesired[2];
    return fsm::ok;
}

// OnExit: Stop motion and reset the controllers
fsm::retval WaypointNavigationState::OnExit() noexcept {
    RCLCPP_INFO(rclcpp::get_logger("WaypointNavigationState"), "Exiting WAYPOINT_NAVIGATION state");
    if (ctrlData) {
        ctrlData->velocityDesired.setZero();
    }
    ResetPIDControllers();
    return fsm::ok;
}

void WaypointNavigationState::ResetPIDControllers() noexcept {
    pidX_.Reset();
    pidY_.Reset();
    pidZ_.Reset();
    pidRoll_.Reset();
    pidPitch_.Reset();
    pidYaw_.Reset();
}
