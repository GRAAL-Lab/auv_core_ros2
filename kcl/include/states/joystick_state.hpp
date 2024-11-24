#pragma once

#include "states/base_auv_state.hpp"

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


    /// Maps joystick axes to desired velocity components.
    /// @param axes Vector of joystick axes values.
    /// @param velocity_desired Pointer to the desired velocity message to update.
    void MapJoystickToVelocity(const std::vector<float>& axes, geometry_msgs::msg::Twist* velocity_desired);

};

