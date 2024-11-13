#ifndef AUV_CTRL_DATA_STRUCTS_HPP
#define AUV_CTRL_DATA_STRUCTS_HPP

#include <rclcpp/rclcpp.hpp>
#include "ctrl_toolbox/ctrl_toolbox.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

namespace auv {

struct ControlData {
    Eigen::Matrix<double, 6, 1> pose_actual_;
    rclcpp::Time time_actual_;
    Eigen::Matrix<double, 6, 1> velocity_actual_;
    Eigen::Matrix<double, 6, 1> acceleration_actual_;

    Eigen::Matrix<double, 6, 1> velocity_desired_;

    geometry_msgs::msg::Twist joystick_velocity_desired_;

    Eigen::Matrix<double, 6, 1> pose_goal_ = Eigen::Matrix<double, 6, 1>::Zero();
    double TP_goal_time_;


    int path_planning_2d_3d;

    //For Path following 2d serpentine
    double serpentine_angle_;
    bool serpentine_direction_; //1= FORWARD, 0= BACKWARD
    double serpentine_offset_;
    std::vector<Eigen::Vector3d> serpentine_polygon_vertices_;

    //For Path following 3d serpentine
    double dive_depth_;
    double curvature_;
    double dip_num_points_;
    double dive_length_;

    //For Path following 3d helix
    Eigen::Vector3d helix_startPos; //Start position on the helix.
    Eigen::Vector3d helix_axisPos; //Point on the helix axis.
    Eigen::Vector3d helix_axisDir; //Direction of the helix axis.
    double helix_frequency; //The length along the helix axis for one period of revolution.
    int helix_numQuadrants; //Number of quadrants in the helix.
    bool helix_counterClockwise; //Flag for direction of revolution: = 0 : clockwise, = 1 : counter clockwise.

    nav_msgs::msg::Path planned_path_;

    // Change to Eigen::VectorXd with size 6
    Eigen::VectorXd gainsX_ = Eigen::VectorXd(6);
    Eigen::VectorXd gainsY_ = Eigen::VectorXd(6);
    Eigen::VectorXd gainsZ_ = Eigen::VectorXd(6);
    Eigen::VectorXd gainsRoll_ = Eigen::VectorXd(6);
    Eigen::VectorXd gainsPitch_ = Eigen::VectorXd(6);
    Eigen::VectorXd gainsYaw_ = Eigen::VectorXd(6);

    // Max and Min linear & angular velocities
    Eigen::VectorXd maxVelocity_ = Eigen::VectorXd(6);
    Eigen::VectorXd minVelocity_ = Eigen::VectorXd(6);

};

}

#endif //  AUV_CTRL_DATA_STRUCTS_HPP
