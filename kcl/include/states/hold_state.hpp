#pragma once

#include "states/base_auv_state.hpp"
#include "ctrl_toolbox/pid/DigitalPID.h" // Include PID controller library
#include <iostream>


/// The `HoldState` class represents a state in which the AUV holds its current position and orientation.
/// It uses PID controllers to correct for deviations in position (X, Y, Z) and orientation (roll, pitch, yaw).
class HoldState : public BaseAUVState {
private:
    // --------------------------
    // PID Controllers
    // --------------------------
    ctb::DigitalPID pidX_;     ///< PID controller for X-axis position.
    ctb::DigitalPID pidY_;     ///< PID controller for Y-axis position.
    ctb::DigitalPID pidZ_;     ///< PID controller for Z-axis position.
    ctb::DigitalPID pidRoll_;  ///< PID controller for roll angle.
    ctb::DigitalPID pidYaw_;   ///< PID controller for yaw angle.
    ctb::DigitalPID pidPitch_; ///< PID controller for pitch angle.

    // Initialize PID controllers
    ctb::PIDGains gainsX_;
    ctb::PIDGains gainsY_;
    ctb::PIDGains gainsZ_;
    ctb::PIDGains gainsRoll_;
    ctb::PIDGains gainsPitch_;
    ctb::PIDGains gainsYaw_;

    // --------------------------
    // Error Tracking
    // --------------------------
    double positionXError_ = 0.0; ///< Position error in the X direction.
    double positionYError_ = 0.0; ///< Position error in the Y direction.
    double positionZError_ = 0.0; ///< Position error in the Z direction.
    double rollError_ = 0.0;      ///< Orientation error in roll.
    double yawError_ = 0.0;       ///< Orientation error in yaw.
    double pitchError_ = 0.0;     ///< Orientation error in pitch.

public:
    /// Constructor for the `HoldState` class.
    /// @param fsm Pointer to the FSM controlling this state.
    explicit HoldState(fsm::FSM* fsm);

    /// Called when the AUV enters the hold state.
    /// Initializes the PID controllers and sets the desired hold position and orientation.
    /// @return Return value indicating the result of entering the state.
    fsm::retval OnEntry() noexcept override;

    /// Called repeatedly while the AUV is in the hold state.
    /// Updates PID controllers based on current pose and computes corrective velocities.
    /// @return Return value indicating the result of execution.
    fsm::retval Execute() noexcept override;

    /// Called when the AUV exits the hold state.
    /// Resets the PID controllers and any temporary state variables.
    /// @return Return value indicating the result of exiting the state.
    fsm::retval OnExit() noexcept override;
};
