#pragma once

#include "states/base_auv_state.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <algorithm>  // std::clamp

class JoystickState : public BaseAUVState
{
public:
    explicit JoystickState(fsm::FSM* fsm);

    fsm::retval OnEntry() noexcept override;
    fsm::retval Execute() noexcept override;
    fsm::retval OnExit() noexcept override;

    // Generic range-mapping function
    static double map(double x, double in_min, double in_max,double out_min, double out_max);

    // Axis definitions should be moved to a config file later
    static constexpr int AXIS_SURGE  = 1;
    static constexpr int AXIS_SWAY   = 0;
    static constexpr int AXIS_HEAVE  = 4;
    static constexpr int AXIS_YAW    = 3;

    static constexpr int AXIS_ROLL_LEFT  = 2;
    static constexpr int AXIS_ROLL_RIGHT = 5;
};
