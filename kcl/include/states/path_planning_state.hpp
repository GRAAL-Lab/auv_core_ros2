#pragma once

#include "states/base_auv_state.hpp"
#include <sisl_toolbox/sisl_toolbox.hpp>
#include <Eigen/Dense>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <filesystem>
#include <limits>
#include <auv_core_helper/path_modes.hpp>

/// The `PathPlanningState` class manages the autonomous path planning behavior of the AUV.
/// It supports various path types (e.g., serpentine, helical) and uses PID controllers for guidance.
class PathPlanningState : public BaseAUVState {
private:
    // Path-related variables
    std::shared_ptr<sisl::Path> path; ///< SISL-generated path.
    std::vector<Eigen::Vector3d> sampledPoints; ///< Sampled points from the path.
    bool isCurveSet_ = false; ///< Whether the curve/path is set.
    bool isVehicleOnPathDirection_ = false; ///< Whether the vehicle is aligned with the path direction.
    double currentAbscissa_ = 0.0; ///< Current abscissa (progress along the path).
    double closestPointAbscissa_ = 0.0; ///< Abscissa of the closest point on the path.

    // Path planning parameters
    double deltaMin_ = 0.2; ///< Minimum look-ahead distance.
    double deltaMax_ = 2.0; ///< Maximum look-ahead distance.
    double delta_ = deltaMax_; ///< Current look-ahead distance.
    double epsilon = 0.0; ///< Threshold for activating adaptive look-ahead.

    // Error metrics
    double crossTrackError_ = 0.0; ///< Cross-track error.
    double verticalTrackError_ = 0.0; ///< Vertical track error.
    double positionXError_ = 0.0; ///< Error in the X position.
    double positionYError_ = 0.0; ///< Error in the Y position.
    double positionZError_ = 0.0; ///< Error in the Z position.
    double rollError_ = 0.0; ///< Error in the roll orientation.
    double yawError_ = 0.0; ///< Error in the yaw orientation.
    double pitchError_ = 0.0; ///< Error in the pitch orientation.

    // PID controllers
    ctb::DigitalPID pidX_, pidY_, pidZ_; ///< Position PIDs.
    ctb::DigitalPID pidRoll_, pidPitch_, pidYaw_; ///< Orientation PIDs.
    ctb::DigitalPID pidDelta_; ///< Look-ahead distance PID.

    // ALOS parameters
    double betaHat_c = 0.0; ///< Estimated crab angle.
    double thetaHat_c = 0.0; ///< Estimated pitch angle.
    double gamma_crosstrack = 0.125; ///< Crosstrack error gain.
    double gamma_verticaltrack = 0.125; ///< Vertical track error gain.

    // Time tracking
    std::chrono::time_point<std::chrono::system_clock> last_update_time;

    // Constants
    static constexpr double MAX_PITCH = 0.610865; ///< Maximum pitch angle (±35 degrees) in radians.
    static constexpr double MIN_PITCH = -0.610865; ///< Minimum pitch angle (±35 degrees) in radians.

    // Helper functions

    /// Updates the heading and pitch of the vehicle using cross-track and vertical-track errors.
    /// @param currentPos The current position of the vehicle.
    /// @param goalPos The desired goal position.
    /// @param closestPos The closest position on the path to the vehicle.
    /// @param Delta The look-ahead distance.
    /// @param epsilon The threshold for error activation.
    /// @param DesiredHeading Output: The updated heading angle (yaw).
    /// @param DesiredPitch Output: The updated pitch angle.
    /// @param crossTrackError Output: The cross-track error.
    /// @param verticalTrackError Output: The vertical-track error.
    /// @return True if the update succeeded, false otherwise.
    bool updateHeadingPitch(
        const Eigen::Vector3d& currentPos,
        const Eigen::Vector3d& goalPos,
        const Eigen::Vector3d& closestPos,
        double Delta,
        double epsilon,
        double& DesiredHeading,
        double& DesiredPitch,
        double& crossTrackError_,
        double& verticalTrackError_);

public:
    /// Constructor
    explicit PathPlanningState(fsm::FSM* fsm);

    /// Called when entering the path planning state.
    fsm::retval OnEntry() noexcept override;

    /// Called repeatedly during the path planning state.
    fsm::retval Execute() noexcept override;

    /// Called when exiting the path planning state.
    fsm::retval OnExit() noexcept override;
};
