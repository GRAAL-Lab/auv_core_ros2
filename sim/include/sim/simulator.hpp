#pragma once

// ROS 2 headers
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

// AUV-specific headers
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "underwater_vehicle_model.hpp"
#include "auv_core_helper/helper_lib.hpp"
#include "auv_msgs_ros2/topicnames.hpp"

// Graal library
#include "rml/EulerRPY.h"

// Standard headers
#include <chrono>
#include <cmath>

class Simulator : public rclcpp::Node {
public:
    Simulator();

private:
    // Callback functions
    void ForcesDesiredCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void Simulate();
    void KclStateCallback(const std_msgs::msg::String::SharedPtr msg);

    // Utility functions
    void CheckAndSetZero(Eigen::Matrix<double, 6, 1>& vec);
    Eigen::Matrix3d RotationMatrix(double roll, double pitch, double yaw);

    // Dynamics model
    std::unique_ptr<mvm::UnderwaterVehicleModel> dynamicsModel_; 

    // State vectors
    Eigen::Matrix<double, 6, 1> poseActual_;
    Eigen::Matrix<double, 6, 1> velocityActual_;
    Eigen::Matrix<double, 6, 1> accelerationActual_;
    Eigen::VectorXd forcesDesired_;

    // Parameters
    double currentYVelocity_, currentZVelocity_; // Time step
    Eigen::Matrix<double, 6, 1> currentVelocity_; // Current velocity in the world frame
    double dt_;

    std::string kclCurrentState_; // Default state

    // ROS 2 communication
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr poseActualPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocityActualPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr accelerationActualPublisher_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr forcesDesiredSubscription_;
    rclcpp::TimerBase::SharedPtr simulationTimer_;
    rclcpp::Time simulationTime_;  // Simulation time tracker
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr kclStateSubscription_;
};
