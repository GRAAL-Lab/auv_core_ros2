#include "states/BaseAUVState.hpp"
#include "kcl/data_structs.hpp"
#include "states/commands.hpp"


// Constructor
BaseAUVState::BaseAUVState(fsm::FSM* fsm, const std::string& name) 
    : fsm_(fsm), stateName_(name) {}

// Virtual Destructor
BaseAUVState::~BaseAUVState() {}
