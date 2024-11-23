#include "states/trajectory_following_state.hpp"

// Constructor
TrajectoryFollowingState::TrajectoryFollowingState(fsm::FSM* fsm)
    : BaseAUVState(fsm, States::TRAJECTORY_FOLLOWING), tTotal_(0.0), tCurrentStart_(0.0) {}

// OnEntry
fsm::retval TrajectoryFollowingState::OnEntry() {
    tTotal_ = ctrlData->tpGoalTime;                    // Total duration of the trajectory in seconds
    tCurrentStart_ = ctrlData->timeActual.seconds();  // Capture the start time of the trajectory
    poseInitial_ = ctrlData->poseActual;              // Store the initial pose
    poseGoal_ = ctrlData->poseGoal;                   // Store the target pose
    return fsm::ok;
}

// Execute
fsm::retval TrajectoryFollowingState::Execute() {
    double tCurrent = ctrlData->timeActual.seconds() - tCurrentStart_; // Current elapsed time

    if (tCurrent >= tTotal_) {
        // Stop movement if the total time is reached
        ctrlData->velocityDesired.setZero();

        if ((ctrlData->poseActual - poseGoal_).norm() < 0.5) {
            // If the goal is reached within a tolerance
            std::cout << "Time Elapsed, Goal Reached!" << std::endl;
            fsm_->SetNextState(States::HOLD);  // Transition to the HOLD state
        } else {
            // If the goal is not reached
            std::cout << "Time Elapsed, Goal Not Reached!" << std::endl;
            fsm_->SetNextState(States::HOLD);
        }
        return fsm::ok;
    }

    // Compute the next trajectory point
    ctrlData->velocityDesired = FindNextTrajectoryPoint(poseInitial_, poseGoal_, tTotal_, tCurrent);

    return fsm::ok;
}

// OnExit
fsm::retval TrajectoryFollowingState::OnExit() {
    return fsm::ok;
}

// FindNextTrajectoryPoint
Eigen::Matrix<double, 6, 1> TrajectoryFollowingState::FindNextTrajectoryPoint(
    const Eigen::Matrix<double, 6, 1>& poseInitial_,
    const Eigen::Matrix<double, 6, 1>& poseGoal_,
    double tTotal_,
    double tCurrent_) const {

    Eigen::Matrix<double, 6, 1> velocityDesired;
    double tRatio = tCurrent_ / tTotal_; // Normalized time ratio

    for (int i = 0; i < 6; i++) {
        double deltaPose = poseGoal_[i] - poseInitial_[i]; // Difference between goal and initial pose
        velocityDesired[i] = (30 * std::pow(tRatio, 2) - 60 * std::pow(tRatio, 3) + 30 * std::pow(tRatio, 4)) 
                             * deltaPose / tTotal_;
    }

    return velocityDesired;
}
