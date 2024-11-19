#pragma once

#include "states/BaseAUVState.hpp"

/// The `JoystickState` class represents a state where the AUV is controlled via joystick input.
/// In this state, the desired velocities are directly set from joystick commands.
class JoystickState : public BaseAUVState {
public:
    /// Constructor for the `JoystickState` class.
    /// @param fsm Pointer to the FSM controlling this state.
    explicit JoystickState(fsm::FSM* fsm);

    /// Called when the AUV enters the joystick control state.
    /// Initializes the state and resets any necessary data.
    fsm::retval OnEntry() noexcept override;

    /// Called repeatedly while the AUV is in the joystick control state.
    /// Processes joystick input to update desired velocities.
    fsm::retval Execute() noexcept override;

    /// Called when the AUV exits the joystick control state.
    /// Can be used to reset joystick-related state or perform cleanup.
    fsm::retval OnExit() noexcept override;
};
