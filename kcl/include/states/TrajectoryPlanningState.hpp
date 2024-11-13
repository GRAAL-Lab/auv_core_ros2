#pragma once

#include "states/BaseAUVState.hpp"
#include <Eigen/Dense>

class TrajectoryPlanningState : public BaseAUVState {
private:
    Eigen::Matrix<double, 6, 1> pose_initial;
    Eigen::Matrix<double, 6, 1> pose_goal;
    double T_total;
    double T_current_start;

public:
    explicit TrajectoryPlanningState(fsm::FSM* fsm);
    fsm::retval OnEntry() override;
    fsm::retval Execute() override;
    fsm::retval OnExit() override;
    Eigen::Matrix<double, 6, 1> FindNextTrajectoryPoint(Eigen::Matrix<double, 6, 1> pose_initial, Eigen::Matrix<double, 6, 1> pose_goal, double T_total, double T_current);
};
