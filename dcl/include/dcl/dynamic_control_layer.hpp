#pragma once

// ROS 2 headers
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

// AUV-specific headers
#include "auv_core_helper/msg/pose_stamped.hpp"
#include "auv_core_helper/helper_lib.hpp"
#include "auv_msgs_ros2/topicnames.hpp"

// Dynamic model
#include "6DOF_model.hpp"





using namespace std::placeholders;
class DCL : public rclcpp::Node {
public:
    DCL(const std::string& config_name);
private:
    void pose_desired_callback(const auv_core_helper::msg::PoseStamped::SharedPtr msg);
    void velocity_desired_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void acceleration_desired_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void pose_actual_callback(const auv_core_helper::msg::PoseStamped::SharedPtr msg);
    void velocity_actual_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void KCL_state_callback(const std_msgs::msg::String::SharedPtr msg);
    void force_compute_callback();

    // Add Eigen data types to your header if needed, ensure Eigen is included properly
    double m_; // Vehicle mass
    Eigen::Vector3d CG_; // Center of Gravity
    Eigen::Matrix3d I_; // Inertia tensor
    Eigen::Matrix<double, 6, 1> M_a_diag_; // Added mass
    Eigen::Matrix<double, 6, 1> D_diag_; // Damping coefficients
    double B_; // Buoyancy
    Eigen::Vector3d CB_; // Center of Buoyancy
    Eigen::Vector3d G_; // Gravity vector
    Eigen::MatrixXd thruster_positions_;
    Eigen::MatrixXd thruster_orientations_;
    Eigen::VectorXd thruster_upper_limits;
    Eigen::VectorXd thruster_lower_limits;
    Eigen::VectorXd thruster_allocation_weights;
    Eigen::Matrix<double, 6, 1> pose_desired_;
    Eigen::Matrix<double, 6, 1> velocity_desired_;
    Eigen::Matrix<double, 6, 1> acceleration_desired_;

    Eigen::Matrix<double, 6, 1> pose_actual_;
    Eigen::Matrix<double, 6, 1> velocity_actual_;
    Eigen::Matrix<double, 6, 1> acceleration_actual_;

    Eigen::Matrix<double, 6, 6> M;
    Eigen::MatrixXd ThrustersWrenchMatrix;
    std::string KCL_current_state_; // Default state

    rclcpp::Subscription<auv_core_helper::msg::PoseStamped>::SharedPtr pose_desired_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_desired_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr acceleration_desired_subscriber_;

    rclcpp::Subscription<auv_core_helper::msg::PoseStamped>::SharedPtr pose_actual_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_actual_subscriber_;

    rclcpp::TimerBase::SharedPtr force_compute_timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forces_desired_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr KCL_state_subscription_;
};
