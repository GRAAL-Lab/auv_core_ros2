#include "states/trajectory_following_state.hpp"

// Constructor
TrajectoryFollowingState::TrajectoryFollowingState(fsm::FSM* fsm)
    : BaseAUVState(fsm, States::TRAJECTORY_FOLLOWING), tTotal_(0.0), tCurrentStart_(0.0) {}

// OnEntry
fsm::retval TrajectoryFollowingState::OnEntry() {
    tTotal_ = ctrlData->tpGoalTime;                   // Total duration of the trajectory
    tCurrentStart_ = ctrlData->timeActual.seconds();  // Start time of the trajectory
    poseInitial_ = ctrlData->poseActual;              // Initial pose
    poseGoal_ = ctrlData->poseGoal;                   // Target pose
    return fsm::ok;
}

// Execute
fsm::retval TrajectoryFollowingState::Execute() {
    double tCurrent = ctrlData->timeActual.seconds() - tCurrentStart_; // Elapsed time

    if (tCurrent >= tTotal_) {
        // Trajectory time elapsed, stop the vehicle
        ctrlData->velocityDesired.setZero();

        // Check goal proximity
        if ((ctrlData->poseActual - poseGoal_).norm() < 0.5) {
            std::cout << "Time Elapsed, Goal Reached!" << std::endl;
            fsm_->SetNextState(States::HOLD);  // Transition to HOLD
        } else {
            std::cout << "Time Elapsed, Goal Not Reached!" << std::endl;
            fsm_->SetNextState(States::HOLD);
        }
        return fsm::ok;
    }

    // Compute the next trajectory point velocities in World frame + Euler angle rates
    Eigen::Matrix<double, 6, 1> velWorldEulerRates = FindNextTrajectoryPoint(poseInitial_, poseGoal_, tTotal_, tCurrent);

    // Separate linear and Euler angle rates
    Eigen::Vector3d vWorld = velWorldEulerRates.head<3>();    // Linear velocities in World frame
    Eigen::Vector3d eulerRates = velWorldEulerRates.tail<3>(); // Euler angle rates (roll_dot, pitch_dot, yaw_dot)

    // Get current orientation
    double roll  = ctrlData->poseActual(3);
    double pitch = ctrlData->poseActual(4);
    double yaw   = ctrlData->poseActual(5);

    rml::EulerRPY rpy(roll, pitch, yaw);

    // Convert linear velocity to body frame
    Eigen::Matrix3d R = rpy.ToRotationMatrix().matrix();
    Eigen::Vector3d vBody = R.transpose() * vWorld;

    // Convert Euler angle rates to body angular velocities
    Eigen::Vector3d wBody = rpy.Omega(eulerRates);

    // Set desired velocities in body frame
    ctrlData->velocityDesired(0) = vBody.x();
    ctrlData->velocityDesired(1) = vBody.y();
    ctrlData->velocityDesired(2) = vBody.z();
    ctrlData->velocityDesired(3) = wBody.x();
    ctrlData->velocityDesired(4) = wBody.y();
    ctrlData->velocityDesired(5) = wBody.z();

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

    // Polynomial-based velocity profile for each DOF
    for (int i = 0; i < 6; i++) {
        double deltaPose = poseGoal_[i] - poseInitial_[i];
        // Using a polynomial velocity profile
        velocityDesired[i] = (30 * std::pow(tRatio, 2)
                              - 60 * std::pow(tRatio, 3)
                              + 30 * std::pow(tRatio, 4)) * deltaPose / tTotal_;
    }

    // Note: The last three components here are Euler angle rates (roll_dot, pitch_dot, yaw_dot).
    // We'll convert them into body-frame angular velocities in Execute().
    return velocityDesired;
}
