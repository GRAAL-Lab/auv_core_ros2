#pragma once

#include "states/base_auv_state.hpp"
#include "ctrl_toolbox/pid/DigitalPID.h"

/// Aligns the AUV heading before navigating to the shared goal pose using PID control.
class WaypointNavigationState : public BaseAUVState {
private:
    // --------------------------
    // PID Controllers
    // --------------------------
    ctb::DigitalPID pidX_;     ///< PID controller for X-axis position.
    ctb::DigitalPID pidY_;     ///< PID controller for Y-axis position.
    ctb::DigitalPID pidZ_;     ///< PID controller for Z-axis position.
    ctb::DigitalPID pidRoll_;  ///< PID controller for roll angle.
    ctb::DigitalPID pidPitch_; ///< PID controller for pitch angle.
    ctb::DigitalPID pidYaw_;   ///< PID controller for yaw angle.

    // --------------------------
    // Error Tracking
    // --------------------------
    double positionXError_ = 0.0; ///< Position error in the X direction.
    double positionYError_ = 0.0; ///< Position error in the Y direction.
    double positionZError_ = 0.0; ///< Position error in the Z direction.
    double rollError_ = 0.0;      ///< Orientation error in roll.
    double pitchError_ = 0.0;     ///< Orientation error in pitch.
    double yawError_ = 0.0;       ///< Orientation error in yaw.

    // --------------------------
    // Navigation Parameters
    // --------------------------
    enum class Phase { Align, Navigate, HoldGoal };
    Phase phase_ = Phase::Align;
    Eigen::Vector3d alignmentPosition_ = Eigen::Vector3d::Zero(); ///< Position held during heading alignment.
    static constexpr double HEADING_TOLERANCE = 0.10; ///< Heading alignment tolerance in radians.
    static constexpr double REALIGN_TOLERANCE = 0.35; ///< Heading error requiring realignment in radians.
    static constexpr double POSITION_TOLERANCE = 0.15; ///< Waypoint arrival tolerance in metres.

    void ResetPIDControllers() noexcept;

public:
    /// Constructor
    explicit WaypointNavigationState(fsm::FSM* fsm);

    /// Initializes the PID controllers and records the position held during alignment.
    fsm::retval OnEntry() noexcept override;

    /// Aligns the heading, navigates to the waypoint, and maintains the final pose.
    fsm::retval Execute() noexcept override;

    /// Stops desired motion and resets the PID controllers.
    fsm::retval OnExit() noexcept override;
};
