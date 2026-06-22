#pragma once

#include "states/base_auv_state.hpp"
#include "ctrl_toolbox/pid/DigitalPID.h"

/// The `ReturnHomeState` drives the AUV back to the home pose [0, 0, 0, 0, 0, 0].
/// It uses the same PID pose regulation approach as HOLD, but with a fixed goal.
class ReturnHomeState : public BaseAUVState {
private:
    ctb::DigitalPID pidX_;
    ctb::DigitalPID pidY_;
    ctb::DigitalPID pidZ_;
    ctb::DigitalPID pidRoll_;
    ctb::DigitalPID pidPitch_;
    ctb::DigitalPID pidYaw_;

    double positionXError_ = 0.0;
    double positionYError_ = 0.0;
    double positionZError_ = 0.0;
    double rollError_ = 0.0;
    double pitchError_ = 0.0;
    double yawError_ = 0.0;

public:
    explicit ReturnHomeState(fsm::FSM* fsm);

    fsm::retval OnEntry() noexcept override;
    fsm::retval Execute() noexcept override;
    fsm::retval OnExit() noexcept override;
};
