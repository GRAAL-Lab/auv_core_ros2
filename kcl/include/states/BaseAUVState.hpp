#pragma once

#include <fsm/fsm.h>
#include <string>
#include <memory>
#include <iostream>


#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "kcl/data_structs.hpp"
#include "auv_core_helper/srv/control_command.hpp"
#include "auv_core_helper/msg/pose_stamped.hpp"
#include "states/commands.hpp"
#include "ctrl_toolbox/pid/DigitalPID.h"
#include "ctrl_toolbox/HelperFunctions.h"
class BaseAUVState : public fsm::BaseState {
protected:
    fsm::FSM* fsm_;
    std::string stateName_;

public:
    std::shared_ptr<auv::ControlData> ctrlData;
    BaseAUVState(fsm::FSM* fsm, const std::string& name);
    virtual ~BaseAUVState();
    virtual fsm::retval OnEntry() = 0;
    virtual fsm::retval Execute() = 0;
    virtual fsm::retval OnExit() = 0;
};
