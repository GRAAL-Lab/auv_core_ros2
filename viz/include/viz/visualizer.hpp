#pragma once

// ROS 2 headers
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

// Utility headers
#include <atomic>
#include <Eigen/Geometry>
#include <fstream>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

// AUV-specific headers
#include "auv_core_helper/msg/pose_stamped.hpp"
#include "auv_msgs_ros2/topicnames.hpp"
#include "nav_msgs/msg/path.hpp"



class Visualizer : public rclcpp::Node {
public:
    explicit Visualizer(const std::string& configName);

private:
    // Callback functions
    void PoseCallback(const auv_core_helper::msg::PoseStamped::SharedPtr msg);
    void PoseGoalCallback(const auv_core_helper::msg::PoseStamped::SharedPtr msg);
    void PathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    // Utility functions
    void PublishPose(double x, double y, double z, double roll, double pitch, double yaw);
    void PublishLightSource(double x, double y, double z);
    void PublishArrowArray(double spacing, double length, int gridSize);

    // ROS 2 communication
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPublisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pathPublisher_;
    rclcpp::Subscription<auv_core_helper::msg::PoseStamped>::SharedPtr poseSubscription_;
    rclcpp::Subscription<auv_core_helper::msg::PoseStamped>::SharedPtr poseGoalSubscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSubscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

    // Parameters
    std::string meshFile_;           // Path to the mesh file
    std::string configName_;         // Configuration name to be used across the class
    std::atomic<bool> firstPoseReceived_ = false; // Indicates if the first pose was received
};
