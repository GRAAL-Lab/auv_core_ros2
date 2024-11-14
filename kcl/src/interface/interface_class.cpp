#include "interface/interface_class.hpp"

InterfaceNode::InterfaceNode() : Node("interface_node"), state_initialized_(false), path_choice(0) {
    client_ = this->create_client<auv_core_helper::srv::ControlCommand>(auv_core_helper::topicnames::control_cmd_service);
    KCL_state_subscription_ = this->create_subscription<std_msgs::msg::String>(auv_core_helper::topicnames::kcl_state, 1, std::bind(&InterfaceNode::KCL_state_callback, this, _1));
    std::thread(&InterfaceNode::user_input_thread, this).detach();

    // Initialize default path parameters
    serpentine_polygon_vertices_[0].x = 0;
    serpentine_polygon_vertices_[0].y = 0;
    serpentine_polygon_vertices_[0].z = 0;

    serpentine_polygon_vertices_[1].x = 50;
    serpentine_polygon_vertices_[1].y = 0;
    serpentine_polygon_vertices_[1].z = 0;

    serpentine_polygon_vertices_[2].x = 50;
    serpentine_polygon_vertices_[2].y = 20;
    serpentine_polygon_vertices_[2].z = 0;

    serpentine_polygon_vertices_[3].x = 0;
    serpentine_polygon_vertices_[3].y = 20;
    serpentine_polygon_vertices_[3].z = 0;

    helix_start_pos_.x = 0;
    helix_start_pos_.y = 0;
    helix_start_pos_.z = 0;

    helix_axis_pos_.x = 0;
    helix_axis_pos_.y = 10;
    helix_axis_pos_.z = 0;

    helix_axis_dir_.x = 0;
    helix_axis_dir_.y = 0;
    helix_axis_dir_.z = -1;
}

void InterfaceNode::user_input_thread() {
    {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [this] { return state_initialized_; });
    }

    while (rclcpp::ok()) {
        int input;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            system("clear");  // Clear the terminal
            std::cout << "Current state: " << KCL_current_state_ << std::endl;
            std::cout << "PLEASE CHOOSE NEXT STATE:" << std::endl;
            std::cout << "1. IDLE" << std::endl;
            std::cout << "2. HOLD" << std::endl;
            std::cout << "3. JoyStick Control" << std::endl;
            std::cout << "4. Trajectory Following" << std::endl;
            std::cout << "5. Path Following" << std::endl;
            std::cout << "Enter choice: ";
        }

        std::cin >> input;
        std::string state = get_state_by_number(input);
        if (!state.empty()) {
            gather_state_details_and_send(state);
        } else {
            std::cin.clear(); // Clear error state
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discard bad input
            std::cout << "Invalid input. Please try again." << std::endl;
        }

        // Short delay to avoid excessive terminal clearing
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void InterfaceNode::gather_state_details_and_send(const std::string& state) {
    std::vector<double> coords;
    double time_to_reach = 0;

    if (state == States::TRAJECTORY_FOLLOWING) {
        coords.resize(6);  // x, y, z, roll, pitch, yaw
        std::cout << "Enter coordinates (x, y, z, roll, pitch, yaw) or 'c' to cancel: ";
        if (!(std::cin >> coords[0] >> coords[1] >> coords[2] >> coords[3] >> coords[4] >> coords[5])) {
            if (std::cin.fail()) {
                handle_cancel();
                return;
            }
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid coordinates input. Please try again." << std::endl;
            return;
        }

        std::cout << "Enter desired time to reach the goal (seconds) or 'c' to cancel: ";
        if (!(std::cin >> time_to_reach)) {
            handle_cancel();
            return;
        }
    } else if (state == States::PATH_FOLLOWING) {
        std::cout << "Do you want to follow a 2D or 3D path? (1 for 2D Serpentine, 2 for 3D Serpentine, 3 for 3D Helix, or 'c' to cancel): ";
        if (!(std::cin >> path_choice) || (path_choice < 1 || path_choice > 3)) {
            handle_cancel();
            return;
        }

        std::cout << "Default path parameters:" << std::endl;
        if (path_choice == 1) {
            std::cout << "2D Serpentine Path Parameters:" << std::endl;
            std::cout << "Angle: " << serpentine_angle_ << std::endl;
            std::cout << "Direction: " << (serpentine_direction_ ? "Forward" : "Backward") << std::endl;
            std::cout << "Offset: " << serpentine_offset_ << std::endl;
            std::cout << "Polygon Vertices:" << std::endl;
            for (const auto& vertex : serpentine_polygon_vertices_) {
                std::cout << "(" << vertex.x << ", " << vertex.y << ", " << vertex.z << ")" << std::endl;
            }
        } else if (path_choice == 2) {
            std::cout << "3D Serpentine Path Parameters:" << std::endl;
            std::cout << "Angle: " << serpentine_angle_ << std::endl;
            std::cout << "Direction: " << (serpentine_direction_ ? "Forward" : "Backward") << std::endl;
            std::cout << "Offset: " << serpentine_offset_ << std::endl;
            std::cout << "Polygon Vertices:" << std::endl;
            for (const auto& vertex : serpentine_polygon_vertices_) {
                std::cout << "(" << vertex.x << ", " << vertex.y << ", " << vertex.z << ")" << std::endl;
            }
            std::cout << "Dive Depth: " << dive_depth_ << std::endl;
            std::cout << "Curvature: " << curvature << std::endl;
            std::cout << "Number of Points for Dip: " << dip_num_points << std::endl;
            std::cout << "Dive Length: " << dive_length << std::endl;
        } else if (path_choice == 3) {
            std::cout << "3D Helix Path Parameters:" << std::endl;
            std::cout << "Start Position: (" << helix_start_pos_.x << ", " << helix_start_pos_.y << ", " << helix_start_pos_.z << ")" << std::endl;
            std::cout << "Axis Position: (" << helix_axis_pos_.x << ", " << helix_axis_pos_.y << ", " << helix_axis_pos_.z << ")" << std::endl;
            std::cout << "Axis Direction: (" << helix_axis_dir_.x << ", " << helix_axis_dir_.y << ", " << helix_axis_dir_.z << ")" << std::endl;
            std::cout << "Frequency: " << helix_frequency_ << std::endl;
            std::cout << "Number of Quadrants: " << helix_num_quadrants_ << std::endl;
            std::cout << "Direction: " << (helix_counter_clockwise_ ? "Counter Clockwise" : "Clockwise") << std::endl;
        }

        std::cout << "Do you want to use the default path parameters? (y/n or 'c' to cancel): ";
        char default_choice;
        if (!(std::cin >> default_choice) || (default_choice != 'y' && default_choice != 'n' && default_choice != 'c')) {
            handle_cancel();
            return;
        }

        if (default_choice == 'c' || default_choice == 'C') {
            std::cout << "Operation canceled. Returning to main menu." << std::endl;
            return;
        }

        if (default_choice == 'n') {
            if (path_choice == 1) {
                gather_2d_serpentine_path_params();
            } else if (path_choice == 2) {
                gather_3d_serpentine_path_params();
            } else if (path_choice == 3) {
                gather_3d_helix_path_params();
            }
        }

        send_path_request();
        return;
    }

    send_service_request(state, coords, time_to_reach);
}

void InterfaceNode::gather_2d_serpentine_path_params() {
    std::cout << "Enter serpentine angle: ";
    if (!(std::cin >> serpentine_angle_)) {
        handle_cancel();
        return;
    }

    std::cout << "Enter serpentine direction (1 for forward, 0 for backward): ";
    if (!(std::cin >> serpentine_direction_)) {
        handle_cancel();
        return;
    }

    std::cout << "Enter serpentine offset: ";
    if (!(std::cin >> serpentine_offset_)) {
        handle_cancel();
        return;
    }

    std::cout << "Enter polygon vertices (x y z) for 4 vertices or 'c' to cancel: ";
    for (auto &vertex : serpentine_polygon_vertices_) {
        if (!(std::cin >> vertex.x >> vertex.y >> vertex.z)) {
            handle_cancel();
            return;
        }
    }
}

void InterfaceNode::gather_3d_helix_path_params() {
    std::cout << "Enter helix start position (x y z): ";
    if (!(std::cin >> helix_start_pos_.x >> helix_start_pos_.y >> helix_start_pos_.z)) {
        handle_cancel();
        return;
    }

    std::cout << "Enter helix axis position (x y z): ";
    if (!(std::cin >> helix_axis_pos_.x >> helix_axis_pos_.y >> helix_axis_pos_.z)) {
        handle_cancel();
        return;
    }

    std::cout << "Enter helix axis direction (x y z): ";
    if (!(std::cin >> helix_axis_dir_.x >> helix_axis_dir_.y >> helix_axis_dir_.z)) {
        handle_cancel();
        return;
    }

    std::cout << "Enter helix frequency: ";
    if (!(std::cin >> helix_frequency_)) {
        handle_cancel();
        return;
    }

    std::cout << "Enter number of quadrants: ";
    if (!(std::cin >> helix_num_quadrants_)) {
        handle_cancel();
        return;
    }

    std::cout << "Enter helix direction (1 for counter clockwise, 0 for clockwise): ";
    if (!(std::cin >> helix_counter_clockwise_)) {
        handle_cancel();
        return;
    }
}

void InterfaceNode::gather_3d_serpentine_path_params() {
    std::cout << "Enter serpentine angle: ";
    if (!(std::cin >> serpentine_angle_)) {
        handle_cancel();
        return;
    }

    std::cout << "Enter serpentine direction (1 for forward, 0 for backward): ";
    if (!(std::cin >> serpentine_direction_)) {
        handle_cancel();
        return;
    }

    std::cout << "Enter serpentine offset: ";
    if (!(std::cin >> serpentine_offset_)) {
        handle_cancel();
        return;
    }

    std::cout << "Enter polygon vertices (x y z) for 4 vertices or 'c' to cancel: ";
    for (auto &vertex : serpentine_polygon_vertices_) {
        if (!(std::cin >> vertex.x >> vertex.y >> vertex.z)) {
            handle_cancel();
            return;
        }
    }

    std::cout << "Enter dive depth: ";
    if (!(std::cin >> dive_depth_)) {
        handle_cancel();
        return;
    }

    std::cout << "Enter curvature: ";
    if (!(std::cin >> curvature)) {
        handle_cancel();
        return;
    }

    std::cout << "Enter number of points for dip: ";
    if (!(std::cin >> dip_num_points)) {
        handle_cancel();
        return;
    }

    std::cout << "Enter dive length: ";
    if (!(std::cin >> dive_length)) {
        handle_cancel();
        return;
    }
}

void InterfaceNode::send_path_request() {
    auto request = std::make_shared<auv_core_helper::srv::ControlCommand::Request>();
    request->state = States::PATH_FOLLOWING;
    request->path_planning_2d_3d = path_choice;

    if (path_choice == 1) { // 2D Serpentine path
        request->serpentine_angle = serpentine_angle_;
        request->serpentine_direction = serpentine_direction_;
        request->serpentine_offset = serpentine_offset_;
        request->serpentine_polygon_vertices = serpentine_polygon_vertices_;
    } else if (path_choice == 2) { // 3D Serpentine path
        request->serpentine_angle = serpentine_angle_;
        request->serpentine_direction = serpentine_direction_;
        request->serpentine_offset = serpentine_offset_;
        request->serpentine_polygon_vertices = serpentine_polygon_vertices_;
        request->dive_depth = dive_depth_;
        request->curvature = curvature;
        request->dip_num_points = dip_num_points;
        request->dive_length = dive_length;
    } else if (path_choice == 3) { // 3D Helix path
        request->helix_start_pos = helix_start_pos_;
        request->helix_axis_pos = helix_axis_pos_;
        request->helix_axis_dir = helix_axis_dir_;
        request->helix_frequency = helix_frequency_;
        request->helix_num_quadrants = helix_num_quadrants_;
        request->helix_counter_clockwise = helix_counter_clockwise_;
    }

    auto result_future = client_->async_send_request(request,
                                                     [](rclcpp::Client<auv_core_helper::srv::ControlCommand>::SharedFuture future) {
                                                         try {
                                                             auto response = future.get();
                                                             if (!response->success) {
                                                                 std::cout << "Failed to process request" << std::endl;
                                                             }
                                                         } catch (const std::exception& e) {
                                                             std::cout << "Service call failed: " << e.what() << std::endl;
                                                         }
                                                     });
}

std::string InterfaceNode::get_state_by_number(int number) {
    switch (number) {
    case 1: return States::IDLE;
    case 2: return States::HOLD;
    case 3: return States::JOYSTICK;
    case 4: return States::TRAJECTORY_FOLLOWING;
    case 5: return States::PATH_FOLLOWING;
    default: return "";
    }
}

void InterfaceNode::send_service_request(const std::string& state, const std::vector<double>& coords, double time_to_reach) {
    auto request = std::make_shared<auv_core_helper::srv::ControlCommand::Request>();
    request->state = state;
    if (!coords.empty() && state == States::TRAJECTORY_FOLLOWING) {
        request->x = coords[0];
        request->y = coords[1];
        request->z = coords[2];
        request->roll = coords[3];
        request->pitch = coords[4];
        request->yaw = coords[5];
        request->time_to_reach = time_to_reach;
    }

    auto result_future = client_->async_send_request(request,
                                                     [](rclcpp::Client<auv_core_helper::srv::ControlCommand>::SharedFuture future) {
                                                         try {
                                                             auto response = future.get();
                                                             if (!response->success) {
                                                                 std::cout << "Failed to process request" << std::endl;
                                                             }
                                                         } catch (const std::exception& e) {
                                                             std::cout << "Service call failed: " << e.what() << std::endl;
                                                         }
                                                     });
}

void InterfaceNode::KCL_state_callback(const std_msgs::msg::String::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(mtx_);
        KCL_current_state_ = msg->data;
        state_initialized_ = true;
    }
    cv_.notify_one();
}

void InterfaceNode::handle_cancel() {
    if (std::cin.fail()) {
        std::cin.clear();
        std::string cancel;
        std::cin >> cancel;
        if (cancel == "c" || cancel == "C") {
            std::cout << "Operation canceled. Returning to main menu." << std::endl;
            return;  // Exit the function if the user cancels the operation
        }
    }
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Invalid input. Please try again." << std::endl;
}
