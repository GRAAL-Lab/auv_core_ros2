#include "states/return_home_state.hpp"

#include <algorithm>
#include <cmath>

ReturnHomeState::ReturnHomeState(fsm::FSM* fsm)
    : BaseAUVState(fsm, States::RETURN_HOME) {
}

fsm::retval ReturnHomeState::OnEntry() noexcept {
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("ReturnHomeState"), "Control data is null!");
        return fsm::fail;
    }

    RCLCPP_INFO(rclcpp::get_logger("ReturnHomeState"), "Entering RETURN_HOME state");

    ctrlData->poseGoal.setZero();

    ctb::PIDGains gainsX = {ctrlData->gainsX(0), ctrlData->gainsX(1), ctrlData->gainsX(2), ctrlData->gainsX(3), ctrlData->gainsX(4), ctrlData->gainsX(5)};
    ctb::PIDGains gainsY = {ctrlData->gainsY(0), ctrlData->gainsY(1), ctrlData->gainsY(2), ctrlData->gainsY(3), ctrlData->gainsY(4), ctrlData->gainsY(5)};
    ctb::PIDGains gainsZ = {ctrlData->gainsZ(0), ctrlData->gainsZ(1), ctrlData->gainsZ(2), ctrlData->gainsZ(3), ctrlData->gainsZ(4), ctrlData->gainsZ(5)};
    ctb::PIDGains gainsRoll = {ctrlData->gainsRoll(0), ctrlData->gainsRoll(1), ctrlData->gainsRoll(2), ctrlData->gainsRoll(3), ctrlData->gainsRoll(4), ctrlData->gainsRoll(5)};
    ctb::PIDGains gainsPitch = {ctrlData->gainsPitch(0), ctrlData->gainsPitch(1), ctrlData->gainsPitch(2), ctrlData->gainsPitch(3), ctrlData->gainsPitch(4), ctrlData->gainsPitch(5)};
    ctb::PIDGains gainsYaw = {ctrlData->gainsYaw(0), ctrlData->gainsYaw(1), ctrlData->gainsYaw(2), ctrlData->gainsYaw(3), ctrlData->gainsYaw(4), ctrlData->gainsYaw(5)};

    pidX_.Initialize(gainsX, ctrlData->dt, std::max(ctrlData->maxVelocity(0), std::abs(ctrlData->minVelocity(0))));
    pidY_.Initialize(gainsY, ctrlData->dt, std::max(ctrlData->maxVelocity(1), std::abs(ctrlData->minVelocity(1))));
    pidZ_.Initialize(gainsZ, ctrlData->dt, std::max(ctrlData->maxVelocity(2), std::abs(ctrlData->minVelocity(2))));
    pidRoll_.Initialize(gainsRoll, ctrlData->dt, std::max(ctrlData->maxVelocity(3), std::abs(ctrlData->minVelocity(3))));
    pidPitch_.Initialize(gainsPitch, ctrlData->dt, std::max(ctrlData->maxVelocity(4), std::abs(ctrlData->minVelocity(4))));
    pidYaw_.Initialize(gainsYaw, ctrlData->dt, std::max(ctrlData->maxVelocity(5), std::abs(ctrlData->minVelocity(5))));

    return fsm::ok;
}

fsm::retval ReturnHomeState::Execute() noexcept {
    if (!ctrlData) {
        RCLCPP_ERROR(rclcpp::get_logger("ReturnHomeState"), "Control data is null!");
        return fsm::fail;
    }

    positionXError_ = ctrlData->poseGoal(0) - ctrlData->poseActual(0);
    positionYError_ = ctrlData->poseGoal(1) - ctrlData->poseActual(1);
    positionZError_ = ctrlData->poseGoal(2) - ctrlData->poseActual(2);
    rollError_ = ctb::AngleDifference(ctrlData->poseGoal(3), ctrlData->poseActual(3));
    pitchError_ = ctb::AngleDifference(ctrlData->poseGoal(4), ctrlData->poseActual(4));
    yawError_ = ctb::AngleDifference(ctrlData->poseGoal(5), ctrlData->poseActual(5));

    rml::EulerRPY rpy(ctrlData->poseActual(3), ctrlData->poseActual(4), ctrlData->poseActual(5));
    Eigen::Matrix3d R = rpy.ToRotationMatrix().matrix();

    Eigen::Vector3d errorWorld(positionXError_, positionYError_, positionZError_);
    Eigen::Vector3d errorBody = R.transpose() * errorWorld;

    ctrlData->velocityDesired(0) = -pidX_.Compute(0, errorBody(0));
    ctrlData->velocityDesired(1) = -pidY_.Compute(0, errorBody(1));
    ctrlData->velocityDesired(2) = -pidZ_.Compute(0, errorBody(2));

    ctrlData->velocityDesired(3) = -pidRoll_.Compute(0, rollError_);
    ctrlData->velocityDesired(4) = -pidPitch_.Compute(0, pitchError_);
    ctrlData->velocityDesired(5) = -pidYaw_.Compute(0, yawError_);

    return fsm::ok;
}

fsm::retval ReturnHomeState::OnExit() noexcept {
    RCLCPP_INFO(rclcpp::get_logger("ReturnHomeState"), "Exiting RETURN_HOME state");
    return fsm::ok;
}
