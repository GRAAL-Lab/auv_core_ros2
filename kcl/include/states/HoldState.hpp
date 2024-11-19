#pragma once

#include "states/BaseAUVState.hpp"
#include "ctrl_toolbox/pid/DigitalPID.h" // Include PID controller library

/// The `HoldState` class represents a state in which the AUV holds its current position and orientation.
/// It uses PID controllers to correct for deviations in position (X, Y, Z) and orientation (roll, pitch, yaw).
class HoldState : public BaseAUVState {
private:
    // --------------------------
    // PID Controllers
    // --------------------------
    ctb::DigitalPID pidX;     ///< PID controller for X-axis position.
    ctb::DigitalPID pidY;     ///< PID controller for Y-axis position.
    ctb::DigitalPID pidZ;     ///< PID controller for Z-axis position.
    ctb::DigitalPID pidRoll;  ///< PID controller for roll angle.
    ctb::DigitalPID pidYaw;   ///< PID controller for yaw angle.
    ctb::DigitalPID pidPitch; ///< PID controller for pitch angle.

    // --------------------------
    // Error Tracking
    // --------------------------
    double positionXError = 0.0; ///< Position error in the X direction.
    double positionYError = 0.0; ///< Position error in the Y direction.
    double positionZError = 0.0; ///< Position error in the Z direction.
    double rollError = 0.0;      ///< Orientation error in roll.
    double yawError = 0.0;       ///< Orientation error in yaw.
    double pitchError = 0.0;     ///< Orientation error in pitch.

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
