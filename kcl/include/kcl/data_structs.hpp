#ifndef AUV_CTRL_DATA_STRUCTS_HPP
#define AUV_CTRL_DATA_STRUCTS_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Dense>
#include <vector>

namespace auv {

/// The `ControlData` structure holds all control-related data for the AUV.
/// It is designed to manage state information, control goals, limits, and parameters
/// for path planning, velocity control, and state estimation.
struct ControlData {
    // ------------------------------
    // State Information
    // ------------------------------
    Eigen::Matrix<double, 6, 1> pose_actual_; ///< Current AUV pose (x, y, z, roll, pitch, yaw).
    rclcpp::Time time_actual_; ///< Timestamp for the current pose.
    Eigen::Matrix<double, 6, 1> velocity_actual_; ///< Current linear and angular velocities.
    Eigen::Matrix<double, 6, 1> acceleration_actual_; ///< Current linear and angular accelerations.

    // ------------------------------
    // Desired State
    // ------------------------------
    Eigen::Matrix<double, 6, 1> velocity_desired_; ///< Desired linear and angular velocities.
    geometry_msgs::msg::Twist joystick_velocity_desired_; ///< Joystick-derived velocity commands.
    Eigen::Matrix<double, 6, 1> pose_goal_ = Eigen::Matrix<double, 6, 1>::Zero(); ///< Desired pose goal.
    double TP_goal_time_ = 0.0; ///< Time to reach the trajectory planning goal.

    // ------------------------------
    // Path Planning Parameters
    // ------------------------------
    int path_planning_2d_3d = 0; ///< Path planning mode: 2D or 3D.

    // 2D Serpentine Path Parameters
    double serpentine_angle_ = 0.0; ///< Serpentine angle for 2D path planning.
    bool serpentine_direction_ = true; ///< Serpentine direction: true = forward, false = backward.
    double serpentine_offset_ = 0.0; ///< Offset for the serpentine path.
    std::vector<Eigen::Vector3d> serpentine_polygon_vertices_; ///< Polygon vertices for serpentine path planning.

    // 3D Serpentine Path Parameters
    double dive_depth_ = 0.0; ///< Dive depth for 3D serpentine path planning.
    double curvature_ = 0.0; ///< Path curvature.
    double dip_num_points_ = 0.0; ///< Number of points in the serpentine "dip".
    double dive_length_ = 0.0; ///< Length of the dive.

    // 3D Helix Path Parameters
    Eigen::Vector3d helix_startPos = Eigen::Vector3d::Zero(); ///< Start position on the helix.
    Eigen::Vector3d helix_axisPos = Eigen::Vector3d::Zero(); ///< Point on the helix axis.
    Eigen::Vector3d helix_axisDir = Eigen::Vector3d::Zero(); ///< Direction of the helix axis.
    double helix_frequency = 0.0; ///< Length along the helix axis for one revolution.
    int helix_numQuadrants = 0; ///< Number of quadrants in the helix.
    bool helix_counterClockwise = true; ///< Revolution direction: true = counter-clockwise.

    // ------------------------------
    // Planned Path
    // ------------------------------
    nav_msgs::msg::Path planned_path_; ///< The planned path for the AUV.

    // ------------------------------
    // Control Gains
    // ------------------------------
    Eigen::VectorXd gainsX_ = Eigen::VectorXd(6); ///< Control gains for the X-axis.
    Eigen::VectorXd gainsY_ = Eigen::VectorXd(6); ///< Control gains for the Y-axis.
    Eigen::VectorXd gainsZ_ = Eigen::VectorXd(6); ///< Control gains for the Z-axis.
    Eigen::VectorXd gainsRoll_ = Eigen::VectorXd(6); ///< Control gains for roll.
    Eigen::VectorXd gainsPitch_ = Eigen::VectorXd(6); ///< Control gains for pitch.
    Eigen::VectorXd gainsYaw_ = Eigen::VectorXd(6); ///< Control gains for yaw.

    // ------------------------------
    // Velocity Limits
    // ------------------------------
    Eigen::VectorXd maxVelocity_ = Eigen::VectorXd(6); ///< Maximum allowed velocities (linear and angular).
    Eigen::VectorXd minVelocity_ = Eigen::VectorXd(6); ///< Minimum allowed velocities (linear and angular).

    // ------------------------------
    // Constructor
    // ------------------------------
    ControlData() {
        // Initialize Eigen matrices and vectors with default values
        pose_actual_.setZero();
        velocity_actual_.setZero();
        acceleration_actual_.setZero();
        velocity_desired_.setZero();
        maxVelocity_.setConstant(1.0); // Example default maximum velocities
        minVelocity_.setConstant(0.0); // Example default minimum velocities
    }
};

} // namespace auv

#endif // AUV_CTRL_DATA_STRUCTS_HPP
