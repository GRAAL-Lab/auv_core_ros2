#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <fsm/fsm.h>
#include "kcl/data_structs.hpp"
#include "auv_core_helper/srv/control_command.hpp"
#include "auv_core_helper/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

// Include state headers
#include "states/BaseAUVState.hpp"
#include "states/IdleState.hpp"
#include "states/HoldState.hpp"
#include "states/JoystickState.hpp"
#include "states/TrajectoryPlanningState.hpp"
#include "states/PathPlanningState.hpp"
#include "states/commands.hpp"
#include "nav_msgs/msg/path.hpp"
#include "auv_msgs_ros2/topicnames.hpp"
#include "auv_core_helper/helper_lib.hpp"




using namespace std::chrono_literals;
using namespace std::placeholders;

class KCL : public rclcpp::Node {
public:
    KCL(const std::string& config_name);
    void executeFSM();
private:
    fsm::FSM fsm_;
    std::unique_ptr<IdleState> idleState_;
    std::unique_ptr<HoldState> holdState_;
    std::unique_ptr<JoystickState> joystickState_;
    std::unique_ptr<TrajectoryPlanningState> trajectoryPlanningState_;
    std::unique_ptr<PathPlanningState> pathPlanningState_;

    rclcpp::Client<auv_core_helper::srv::ControlCommand>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;

    rclcpp::Publisher<auv_core_helper::msg::PoseStamped>::SharedPtr pose_goal_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_desired_publisher_;
    rclcpp::TimerBase::SharedPtr fsm_timer_;
    rclcpp::Subscription<auv_core_helper::msg::PoseStamped>::SharedPtr pose_actual_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_actual_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr acceleration_actual_subscriber_;

    rclcpp::Service<auv_core_helper::srv::ControlCommand>::SharedPtr control_command_service_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    std::shared_ptr<auv::ControlData> ctrlData_; //shared pointer to control data struct

    void setupTransitions();
    void JoyStickCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void pose_actual_callback(const auv_core_helper::msg::PoseStamped::SharedPtr msg);
    void velocity_actual_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void acceleration_actual_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void handleControlCommand(const std::shared_ptr<auv_core_helper::srv::ControlCommand::Request> request, std::shared_ptr<auv_core_helper::srv::ControlCommand::Response> response);
};
