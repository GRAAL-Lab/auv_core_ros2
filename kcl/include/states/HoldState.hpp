#pragma once

#include "states/BaseAUVState.hpp"


class HoldState : public BaseAUVState {
private:
    ctb::DigitalPID pidX;
    ctb::DigitalPID pidY;
    ctb::DigitalPID pidZ;
    ctb::DigitalPID pidRoll;
    ctb::DigitalPID pidYaw;
    ctb::DigitalPID pidPitch;


public:
    HoldState(fsm::FSM* fsm);
    fsm::retval OnEntry() override;
    fsm::retval Execute() override;
    fsm::retval OnExit() override;

    double positionXError = 0.0;
    double positionYError = 0.0;
    double positionZError = 0.0;
    double rollError = 0.0;
    double yawError = 0.0;
    double pitchError = 0.0;
};
