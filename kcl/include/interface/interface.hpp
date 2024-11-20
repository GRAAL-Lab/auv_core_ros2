#pragma once

#include <rclcpp/rclcpp.hpp>
#include <auv_core_helper/srv/control_command.hpp>
#include <string>
#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <auv_msgs_ros2/topicnames.hpp>
#include <std_msgs/msg/string.hpp>
#include <states/commands.hpp>
#include <geometry_msgs/msg/vector3.hpp>


class InterfaceNode : public rclcpp::Node {
public:
    InterfaceNode();

private:
    // Private member variables
    std::string currentState_;
    std::mutex mutex_;
    std::condition_variable conditionVariable_;
    bool isStateInitialized_;

    // Default path parameters
    double serpentineAngle_{0.0};
    bool serpentineDirection_{false};
    double serpentineOffset_{4.0};
    std::vector<geometry_msgs::msg::Vector3> serpentinePolygonVertices_{
        geometry_msgs::msg::Vector3(),
        geometry_msgs::msg::Vector3(),
        geometry_msgs::msg::Vector3(),
        geometry_msgs::msg::Vector3()
    };

    double diveDepth_{5.0};
    double curvature_{3.0};
    double diveNumPoints_{1000};
    double diveLength_{4.0};

    geometry_msgs::msg::Vector3 helixStartPosition_{};
    geometry_msgs::msg::Vector3 helixAxisPosition_{};
    geometry_msgs::msg::Vector3 helixAxisDirection_{};
    double helixFrequency_{1.0};
    int helixNumQuadrants_{20};
    bool helixCounterClockwise_{false};

    int pathChoice_;  // Determines if path is 2D or 3D

    // ROS communication members
    rclcpp::Client<auv_core_helper::srv::ControlCommand>::SharedPtr controlCommandClient_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stateSubscription_;

    // Private member functions
    void ProcessStateAndSendRequest(const std::string& state);
    std::string GetStateByNumber(int stateNumber) const;
    void SendServiceRequest(const std::string& state, const std::vector<double>& coordinates, double timeToReach);
    void StateCallback(const std_msgs::msg::String::SharedPtr msg);
    void StartUserInputThread();

    void Gather2DSerpentinePathParameters();
    void Gather3DSerpentinePathParameters();

    void Gather3DHelixPathParameters();
    void SendPathRequest();
    void GatherPathDetails();

    void HandleCancelRequest();
};