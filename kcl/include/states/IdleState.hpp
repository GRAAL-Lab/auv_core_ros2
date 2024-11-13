#pragma once

#include "states/BaseAUVState.hpp"

class IdleState : public BaseAUVState {
public:
    IdleState(fsm::FSM* fsm);
    fsm::retval OnEntry() override;
    fsm::retval Execute() override;
    fsm::retval OnExit() override;
};
