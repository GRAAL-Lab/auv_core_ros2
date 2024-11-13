#include "states/TrajectoryPlanningState.hpp"
#include <iostream>
#include <Eigen/Dense>


// Constructor
TrajectoryPlanningState::TrajectoryPlanningState(fsm::FSM* fsm)
    : BaseAUVState(fsm, States::TRAJECTORY_FOLLOWING), T_total(0), T_current_start(0) {}

// OnEntry
fsm::retval TrajectoryPlanningState::OnEntry() {
    T_total = ctrlData->TP_goal_time_; // Total duration of the trajectory in seconds
    T_current_start = ctrlData->time_actual_.seconds(); // Capture start time for the trajectory
    pose_initial = ctrlData->pose_actual_;
    pose_goal = ctrlData->pose_goal_;
    return fsm::ok;
}

// Execute
fsm::retval TrajectoryPlanningState::Execute() {
    double T_current = ctrlData->time_actual_.seconds() - T_current_start;
    if (T_current >= T_total) {
        ctrlData->velocity_desired_.setZero();
        if ((ctrlData->pose_actual_ - pose_goal).norm() < 0.5) {
            std::cout << "Time Elapsed, Goal Reached!" << std::endl;
            fsm_->SetNextState(States::HOLD); // Transition to IDLE state when goal is reached
        } else {
            std::cout << "Time Elapsed, Goal Not Reached!" << std::endl;
            fsm_->SetNextState(States::HOLD);
        }
        return fsm::ok;
    }

    ctrlData->velocity_desired_ = FindNextTrajectoryPoint(pose_initial, pose_goal, T_total, T_current);


    return fsm::ok;
}

// OnExit
fsm::retval TrajectoryPlanningState::OnExit() {
    return fsm::ok;
}

Eigen::Matrix<double, 6, 1> TrajectoryPlanningState::FindNextTrajectoryPoint(
    Eigen::Matrix<double, 6, 1> pose_initial,
    Eigen::Matrix<double, 6, 1> pose_goal,
    double T_total,
    double T_current) {

    Eigen::Matrix<double, 6, 1> velocity_desired;
    double tr = T_current / T_total;

    for (int i = 0; i < 6; i++) {
        double temp = pose_goal[i] - pose_initial[i];
        velocity_desired[i] = (30 * std::pow(tr, 2) - 60 * std::pow(tr, 3) + 30 * std::pow(tr, 4)) * temp / T_total;
    }
    return velocity_desired;
}