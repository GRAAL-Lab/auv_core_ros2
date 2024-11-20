#pragma once

#include "states/base_auv_state.hpp"

class TrajectoryPlanningState : public BaseAUVState {
private:
    Eigen::Matrix<double, 6, 1> poseInitial_;
    Eigen::Matrix<double, 6, 1> poseGoal_;
    double tTotal_;
    double tCurrentStart_;

public:
    explicit TrajectoryPlanningState(fsm::FSM* fsm);
    fsm::retval OnEntry() override;
    fsm::retval Execute() override;
    fsm::retval OnExit() override;
    Eigen::Matrix<double, 6, 1> FindNextTrajectoryPoint(
        const Eigen::Matrix<double, 6, 1>& poseInitial_,
        const Eigen::Matrix<double, 6, 1>& poseGoal_,
        double tTotal_,
        double tCurrent_) const;
};
