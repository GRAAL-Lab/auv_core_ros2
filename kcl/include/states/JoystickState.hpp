#pragma once

#include "states/BaseAUVState.hpp"

class JoystickState : public BaseAUVState {
public:
    JoystickState(fsm::FSM* fsm);
    fsm::retval OnEntry() override;
    fsm::retval Execute() override;
    fsm::retval OnExit() override;
};
