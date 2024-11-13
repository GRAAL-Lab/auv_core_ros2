#pragma once

#include "states/BaseAUVState.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>
#include <filesystem>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "sisl_toolbox/sisl_toolbox.hpp"
#include <fstream>
#include <sstream>
#include <ikcl/ikcl.h>
#include "ctrl_toolbox/ctrl_toolbox.hpp"
#include "ctrl_toolbox/pid/DigitalPID.h"
#include "kcl/data_structs.hpp"
#include <chrono>

class PathPlanningState : public BaseAUVState {
private:
    std::shared_ptr<sisl::Path> path;
    std::vector<Eigen::Vector3d> sampledPoints;
    bool isCurveSet_ = false;
    bool isVehicleOnPathDirection = false;

    // PID controllers
    ctb::DigitalPID pidX;
    ctb::DigitalPID pidY;
    ctb::DigitalPID pidZ;
    ctb::DigitalPID pidRoll;
    ctb::DigitalPID pidPitch;
    ctb::DigitalPID pidYaw;

    ctb::DigitalPID pidDelta;





public:
    PathPlanningState(fsm::FSM* fsm);
    fsm::retval OnEntry() override;
    fsm::retval Execute() override;
    fsm::retval OnExit() override;
    std::chrono::time_point<std::chrono::system_clock> last_update_time;
    bool updateHeadingPitch(
        const Eigen::Vector3d& currentPos,
        const Eigen::Vector3d& goalPos,
        const Eigen::Vector3d& closestPos,
        double Delta,
        double epsilon,
        double& desiredHeading,
        double& desiredPitch,
        double& crossTrackError,
        double& verticalTrackError);
    double currentAbscissa_ = 0.0;
    double crossTrackError = 0.0; // Initialize crossTrackError to zero
    double verticalTrackError = 0.0;
    double beta_hat_c = 0.0; // Estimated crab angle
    double theta_hat_c = 0.0; // Estimated heading angle
    double gamma_crosstrack = 0.125;
    double gamma_verticaltrack = 0.125;
    double deltaMin = 0.2;
    // double deltaMax = sqrt(pow(ctrlData->maxVelocity_(0),2) + pow(ctrlData->maxVelocity_(1),2));
    double deltaMax = 2.0; // Set deltaMax to 1.0 for now
    double delta = deltaMax; // Look-ahead distance
    double epsilon = 0.0; // Epsilon for activating ALOS
    Eigen::Vector3d goalPosDot;
    double tangentsDifferenceNorm = 0.0;

    double positionXError = 0.0;
    double positionYError = 0.0;
    double positionZError = 0.0;
    double rollError = 0.0;
    double yawError = 0.0;
    double pitchError = 0.0;


    double closestPointAbscissa = 0.0;
};
