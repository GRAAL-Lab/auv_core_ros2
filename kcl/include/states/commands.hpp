#ifndef AUV_STATES_COMMANDS_HPP
#define AUV_STATES_COMMANDS_HPP

#include <string>

/// Namespace containing state identifiers for the AUV FSM.
namespace States {
    constexpr char IDLE[] = "IDLE"; ///< AUV is in an idle state.
    constexpr char HOLD[] = "HOLD"; ///< AUV is holding its current position.
    constexpr char JOYSTICK[] = "JOYSTICK"; ///< AUV is being controlled via joystick.
    constexpr char TRAJECTORY_FOLLOWING[] = "TRAJECTORY_FOLLOWING"; ///< AUV is following a predefined trajectory.
    constexpr char PATH_FOLLOWING[] = "PATH_FOLLOWING"; ///< AUV is following a planned path.
}

#endif // AUV_STATES_COMMANDS_HPP
