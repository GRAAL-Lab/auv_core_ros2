#pragma once

// ROS 2 headers
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

// AUV-specific headers
#include "auv_core_helper/msg/pose_stamped.hpp"
#include "6DOF_model.hpp"
#include "auv_core_helper/helper_lib.hpp"
#include "auv_msgs_ros2/topicnames.hpp"

// Graal library
#include "rml/EulerRPY.h"
#include <cmath>



// Dynamic model

using namespace std::chrono_literals;
using namespace std::placeholders;

class simulator : public rclcpp::Node {
public:
    simulator(const std::string& config_name);

private:
    void forces_desired_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void simulate();
    void KCL_state_callback(const std_msgs::msg::String::SharedPtr msg);
    void check_and_set_zero(Eigen::Matrix<double, 6, 1>& vec);
    Eigen::Matrix3d rotationMatrix(double roll, double pitch, double yaw);
    double m_;
    Eigen::Vector3d CG_;
    Eigen::Matrix3d I_;
    Eigen::Matrix<double, 6, 1> M_a_diag_;
    Eigen::Matrix<double, 6, 1> D_diag_;
    double B_;
    Eigen::Vector3d CB_;
    Eigen::Vector3d G_;
    Eigen::MatrixXd thruster_positions_;
    Eigen::MatrixXd thruster_orientations_;
    Eigen::Matrix<double, 6, 1> pose_actual_;
    Eigen::Matrix<double, 6, 1> velocity_actual_;
    Eigen::Matrix<double, 6, 1> acceleration_actual_;
    Eigen::VectorXd forces_desired_;
    Eigen::Matrix<double, 6, 6> M;
    Eigen::MatrixXd ThrustersWrenchMatrix;
    std::string KCL_current_state_; // Default state
    rclcpp::Publisher<auv_core_helper::msg::PoseStamped>::SharedPtr pose_actual_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_actual_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr acceleration_actual_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr forces_desired_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time simulation_time_;  // Simulation time tracker
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr KCL_state_subscription_;
};
