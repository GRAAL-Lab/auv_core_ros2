#pragma once

#include <rclcpp/rclcpp.hpp>
#include "auv_core_helper/srv/control_command.hpp"
#include <string>
#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include "auv_msgs_ros2/topicnames.hpp"
#include "std_msgs/msg/string.hpp"
#include "states/commands.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std::placeholders;

class InterfaceNode : public rclcpp::Node {
public:
    InterfaceNode();

private:
    std::string KCL_current_state_;
    void gather_state_details_and_send(const std::string& state);
    std::string get_state_by_number(int number);
    void send_service_request(const std::string& state, const std::vector<double>& coords, double time_to_reach);
    void KCL_state_callback(const std_msgs::msg::String::SharedPtr msg);
    void user_input_thread();

    void gather_2d_serpentine_path_params();
    void gather_3d_serpentine_path_params();

    void gather_3d_helix_path_params();
    void send_path_request();

    void handle_cancel();

    rclcpp::Client<auv_core_helper::srv::ControlCommand>::SharedPtr client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr KCL_state_subscription_;

    std::mutex mtx_;
    std::condition_variable cv_;
    bool state_initialized_;

    // Default path parameters
    double serpentine_angle_ = 0.0;
    bool serpentine_direction_ = false;
    double serpentine_offset_ = 4.0;
    std::vector<geometry_msgs::msg::Vector3> serpentine_polygon_vertices_ = {
        geometry_msgs::msg::Vector3(),
        geometry_msgs::msg::Vector3(),
        geometry_msgs::msg::Vector3(),
        geometry_msgs::msg::Vector3()
    };

    double dive_depth_ = 5.0;
    double curvature = 3.0;
    double dip_num_points = 1000;
    double dive_length = 4.0;

    geometry_msgs::msg::Vector3 helix_start_pos_ = geometry_msgs::msg::Vector3();
    geometry_msgs::msg::Vector3 helix_axis_pos_ = geometry_msgs::msg::Vector3();
    geometry_msgs::msg::Vector3 helix_axis_dir_ = geometry_msgs::msg::Vector3();
    double helix_frequency_ = 1;
    int helix_num_quadrants_ = 20;
    bool helix_counter_clockwise_ = false;

    int path_choice;  // Flag to determine if path is 2D or 3D
};
