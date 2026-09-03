#include "interface/interface.hpp"

InterfaceNode::InterfaceNode()
    : Node("interface_node"), isStateInitialized_(false), pathChoice_(0) {
    controlCommandClient_ = this->create_client<auv_core_helper::srv::ControlCommand>(
        auv_core_helper::topicnames::control_cmd_service);

    stateSubscription_ = this->create_subscription<std_msgs::msg::String>(
        auv_core_helper::topicnames::kcl_state, 1,
        std::bind(&InterfaceNode::StateCallback, this, std::placeholders::_1)
);

    std::thread(&InterfaceNode::StartUserInputThread, this).detach();

    // Initialize default path parameters
    serpentinePolygonVertices_[0].x = 0;
    serpentinePolygonVertices_[0].y = 0;
    serpentinePolygonVertices_[0].z = 0;

    serpentinePolygonVertices_[1].x = 18;
    serpentinePolygonVertices_[1].y = 0;
    serpentinePolygonVertices_[1].z = 0;

    serpentinePolygonVertices_[2].x = 18;
    serpentinePolygonVertices_[2].y = 40;
    serpentinePolygonVertices_[2].z = 0;

    serpentinePolygonVertices_[3].x = 0;
    serpentinePolygonVertices_[3].y = 40;
    serpentinePolygonVertices_[3].z = 0;

    helixStartPosition_.x = 0;
    helixStartPosition_.y = 0;
    helixStartPosition_.z = 0;

    helixAxisPosition_.x = 0;
    helixAxisPosition_.y = 10;
    helixAxisPosition_.z = 0;

    helixAxisDirection_.x = 0;
    helixAxisDirection_.y = 0;
    helixAxisDirection_.z = -1;

    // Keep the new default paths inside the same 18 m x 40 m operating area
    // used by the default serpentine path.
    spiralCentrePoint_.x = 9;
    spiralCentrePoint_.y = 20;
    spiralCentrePoint_.z = 0;

    spiralStartPoint_.x = 17;
    spiralStartPoint_.y = 20;
    spiralStartPoint_.z = 0;

    // NewRacetrack's alternating end turns extend beyond its input polygon.
    // Inset the long edges by half the larger diameter so the generated path stays
    // within the overall x=[0, 18], y=[0, 40] operating area.
    racetrackPolygonVertices_[0].x = 4;
    racetrackPolygonVertices_[0].y = 0;
    racetrackPolygonVertices_[0].z = 0;

    racetrackPolygonVertices_[1].x = 14;
    racetrackPolygonVertices_[1].y = 0;
    racetrackPolygonVertices_[1].z = 0;

    racetrackPolygonVertices_[2].x = 14;
    racetrackPolygonVertices_[2].y = 40;
    racetrackPolygonVertices_[2].z = 0;

    racetrackPolygonVertices_[3].x = 4;
    racetrackPolygonVertices_[3].y = 40;
    racetrackPolygonVertices_[3].z = 0;
}

void InterfaceNode::StartUserInputThread() {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        conditionVariable_.wait(lock, [this] { return isStateInitialized_; });
    }

    while (rclcpp::ok()) {
        int input;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            system("clear");  // Clear the terminal
            std::cout << "Current state: " << currentState_ << std::endl;
            std::cout << "PLEASE CHOOSE NEXT STATE:" << std::endl;
            std::cout << "1. IDLE" << std::endl;
            std::cout << "2. HOLD" << std::endl;
            std::cout << "3. JoyStick Control" << std::endl;
            std::cout << "4. Trajectory Following" << std::endl;
            std::cout << "5. Path Following" << std::endl;
            std::cout << "Enter choice: ";
        }

        std::cin >> input;
        std::string state = GetStateByNumber(input);
        if (!state.empty()) {
            ProcessStateAndSendRequest(state);
        } else {
            std::cin.clear();  // Clear error state
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');  // Discard bad input
            std::cout << "Invalid input. Please try again." << std::endl;
        }

        // Short delay to avoid excessive terminal clearing
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void InterfaceNode::ProcessStateAndSendRequest(const std::string& state) {
    std::vector<double> coordinates;
    double timeToReach = 0;

    if (state == States::TRAJECTORY_FOLLOWING) {
        coordinates.resize(6);  // x, y, z, roll, pitch, yaw
        std::cout << "Enter coordinates (x, y, z, roll, pitch, yaw) or 'c' to cancel: ";
        if (!(std::cin >> coordinates[0] >> coordinates[1] >> coordinates[2] >> coordinates[3] >> coordinates[4] >> coordinates[5])) {
            HandleCancelRequest();
            return;
        }

        std::cout << "Enter desired time to reach the goal (seconds) or 'c' to cancel: ";
        if (!(std::cin >> timeToReach)) {
            HandleCancelRequest();
            return;
        }
        SendServiceRequest(state, coordinates, timeToReach);
    } else if (state == States::PATH_FOLLOWING) {
        GatherPathDetails();
    }
    else {
        SendServiceRequest(state, coordinates, timeToReach);
    }
}

void InterfaceNode::GatherPathDetails() {
    std::cout << "Choose a path (1 for 2D Serpentine, 2 for 3D Serpentine, "
                 "3 for 3D Helix, 4 for 2D Spiral, 5 for 2D Racetrack, or 'c' to cancel): ";
    if (!(std::cin >> pathChoice_) || (pathChoice_ < 1 || pathChoice_ > 5)) {
        HandleCancelRequest();
        return;
    }

    if (pathChoice_ == auv_core_helper::Serpentine2D) {
        char enableAltitudeHold = 'n';
        std::cout << "Enable seabed-altitude hold? (y for yes, n for no, c to cancel): ";
        if (!(std::cin >> enableAltitudeHold)) {
            HandleCancelRequest();
            return;
        }
        if (enableAltitudeHold == 'c' || enableAltitudeHold == 'C') {
            std::cout << "Path request canceled." << std::endl;
            return;
        }
        if (enableAltitudeHold != 'y' && enableAltitudeHold != 'Y' &&
            enableAltitudeHold != 'n' && enableAltitudeHold != 'N') {
            std::cout << "Invalid choice. Path request canceled." << std::endl;
            return;
        }
        seabedAltitudeHold_ = enableAltitudeHold == 'y' || enableAltitudeHold == 'Y';
    }

    char useDefaults = 'y';
    std::cout << "Do you want to use the default path parameters? (y/n or 'c' to cancel): ";
    if (!(std::cin >> useDefaults) || (useDefaults != 'y' && useDefaults != 'n' && useDefaults != 'c')) {
        HandleCancelRequest();
        return;
    }

    if (useDefaults == 'n') {
        if (pathChoice_ == auv_core_helper::Serpentine2D) {
            Gather2DSerpentinePathParameters();
        } else if (pathChoice_ == auv_core_helper::Serpentine3D) {
            Gather3DSerpentinePathParameters();
        } else if (pathChoice_ == auv_core_helper::Helix3D) {
            Gather3DHelixPathParameters();
        } else if (pathChoice_ == auv_core_helper::Spiral2D) {
            Gather2DSpiralPathParameters();
        } else if (pathChoice_ == auv_core_helper::Racetrack2D) {
            Gather2DRacetrackPathParameters();
        }
    }

    if (pathChoice_ == auv_core_helper::Serpentine2D &&
        seabedAltitudeHold_ && useDefaults == 'n') {
            std::cout << "Enter desired seabed altitude in metres (default is 2.0): ";
            if (!(std::cin >> seabedAltitudeGoal_) || seabedAltitudeGoal_ <= 0.0) {
                HandleCancelRequest();
                return;
            }
            std::cout << "Enter shallowest allowed depth as negative Z (default is -0.5): ";
            if (!(std::cin >> shallowDepthLimit_) || shallowDepthLimit_ >= 0.0) {
                HandleCancelRequest();
                return;
            }
    }

    SendPathRequest();
}

void InterfaceNode::Gather2DSerpentinePathParameters() {
    std::cout << "Enter serpentine angle: ";
    if (!(std::cin >> serpentineAngle_)) {
        HandleCancelRequest();
        return;
    }

    std::cout << "Enter serpentine direction (1 for forward, 0 for backward): ";
    if (!(std::cin >> serpentineDirection_)) {
        HandleCancelRequest();
        return;
    }

    std::cout << "Enter serpentine offset: ";
    if (!(std::cin >> serpentineOffset_)) {
        HandleCancelRequest();
        return;
    }

    std::cout << "Enter polygon vertices (x y z) for 4 vertices or 'c' to cancel: ";
    for (auto& vertex : serpentinePolygonVertices_) {
        if (!(std::cin >> vertex.x >> vertex.y >> vertex.z)) {
            HandleCancelRequest();
            return;
        }
    }

}

void InterfaceNode::Gather3DHelixPathParameters() {
    std::cout << "Enter helix start position (x y z): ";
    if (!(std::cin >> helixStartPosition_.x >> helixStartPosition_.y >> helixStartPosition_.z)) {
        HandleCancelRequest();
        return;
    }

    std::cout << "Enter helix axis position (x y z): ";
    if (!(std::cin >> helixAxisPosition_.x >> helixAxisPosition_.y >> helixAxisPosition_.z)) {
        HandleCancelRequest();
        return;
    }

    std::cout << "Enter helix axis direction (x y z): ";
    if (!(std::cin >> helixAxisDirection_.x >> helixAxisDirection_.y >> helixAxisDirection_.z)) {
        HandleCancelRequest();
        return;
    }

    std::cout << "Enter helix frequency: ";
    if (!(std::cin >> helixFrequency_)) {
        HandleCancelRequest();
        return;
    }

    std::cout << "Enter number of quadrants: ";
    if (!(std::cin >> helixNumQuadrants_)) {
        HandleCancelRequest();
        return;
    }

    std::cout << "Enter helix direction (1 for counter clockwise, 0 for clockwise): ";
    if (!(std::cin >> helixCounterClockwise_)) {
        HandleCancelRequest();
        return;
    }
}

void InterfaceNode::SendServiceRequest(const std::string& state, const std::vector<double>& coordinates, double timeToReach) {
    auto request = std::make_shared<auv_core_helper::srv::ControlCommand::Request>();
    request->state = state;

    if (!coordinates.empty() && state == States::TRAJECTORY_FOLLOWING) {
        request->x = coordinates[0];
        request->y = coordinates[1];
        request->z = coordinates[2];
        request->roll = coordinates[3];
        request->pitch = coordinates[4];
        request->yaw = coordinates[5];
        request->time_to_reach = timeToReach;
    }

    auto resultFuture = controlCommandClient_->async_send_request(request,
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

std::string InterfaceNode::GetStateByNumber(int number) const {
    switch (number) {
    case 1: return States::IDLE;
    case 2: return States::HOLD;
    case 3: return States::JOYSTICK;
    case 4: return States::TRAJECTORY_FOLLOWING;
    case 5: return States::PATH_FOLLOWING;
    default: return "";
    }
}

void InterfaceNode::StateCallback(const std_msgs::msg::String::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        currentState_ = msg->data;
        isStateInitialized_ = true;
    }
    conditionVariable_.notify_one();
}

void InterfaceNode::HandleCancelRequest() {
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

void InterfaceNode::Gather3DSerpentinePathParameters() {
    std::cout << "Enter serpentine angle: ";
    if (!(std::cin >> serpentineAngle_)) {
        HandleCancelRequest();
        return;
    }

    std::cout << "Enter serpentine direction (1 for forward, 0 for backward): ";
    if (!(std::cin >> serpentineDirection_)) {
        HandleCancelRequest();
        return;
    }

    std::cout << "Enter serpentine offset: ";
    if (!(std::cin >> serpentineOffset_)) {
        HandleCancelRequest();
        return;
    }

    std::cout << "Enter polygon vertices (x y z) for 4 vertices or 'c' to cancel: ";
    for (auto& vertex : serpentinePolygonVertices_) {
        if (!(std::cin >> vertex.x >> vertex.y >> vertex.z)) {
            HandleCancelRequest();
            return;
        }
    }

    std::cout << "Enter dive depth: ";
    if (!(std::cin >> diveDepth_)) {
        HandleCancelRequest();
        return;
    }

    std::cout << "Enter curvature: ";
    if (!(std::cin >> curvature_)) {
        HandleCancelRequest();
        return;
    }

    std::cout << "Enter number of points for the dip: ";
    if (!(std::cin >> diveNumPoints_)) {
        HandleCancelRequest();
        return;
    }

    std::cout << "Enter dive length: ";
    if (!(std::cin >> diveLength_)) {
        HandleCancelRequest();
        return;
    }
}

void InterfaceNode::Gather2DSpiralPathParameters() {
    std::cout << "Enter spiral centre point (x y z): ";
    if (!(std::cin >> spiralCentrePoint_.x >> spiralCentrePoint_.y >> spiralCentrePoint_.z)) {
        HandleCancelRequest();
        return;
    }

    std::cout << "Enter spiral start point (x y z): ";
    if (!(std::cin >> spiralStartPoint_.x >> spiralStartPoint_.y >> spiralStartPoint_.z)) {
        HandleCancelRequest();
        return;
    }

    std::cout << "Enter spiral radius offset: ";
    if (!(std::cin >> spiralRadiusOffset_) || spiralRadiusOffset_ <= 0.0) {
        HandleCancelRequest();
        return;
    }
}

void InterfaceNode::Gather2DRacetrackPathParameters() {
    std::cout << "Enter racetrack angle in degrees: ";
    if (!(std::cin >> racetrackAngle_)) {
        HandleCancelRequest();
        return;
    }

    std::cout << "Enter racetrack direction (1 for forward, 0 for backward): ";
    if (!(std::cin >> racetrackForward_)) {
        HandleCancelRequest();
        return;
    }

    std::cout << "Enter first and second racetrack diameters: ";
    if (!(std::cin >> racetrackFirstDiameter_ >> racetrackSecondDiameter_) ||
        racetrackFirstDiameter_ <= 0.0 || racetrackSecondDiameter_ <= 0.0 ||
        racetrackFirstDiameter_ == racetrackSecondDiameter_) {
        HandleCancelRequest();
        return;
    }

    std::cout << "Enter polygon vertices (x y z) for 4 vertices or 'c' to cancel: ";
    for (auto& vertex : racetrackPolygonVertices_) {
        if (!(std::cin >> vertex.x >> vertex.y >> vertex.z)) {
            HandleCancelRequest();
            return;
        }
    }
}

void InterfaceNode::SendPathRequest() {
    auto request = std::make_shared<auv_core_helper::srv::ControlCommand::Request>();
    request->state = States::PATH_FOLLOWING;
    request->path_planning_2d_3d = pathChoice_;

    // Use a switch statement to handle the path choice
    switch (pathChoice_) {
        case auv_core_helper::Serpentine2D: { // 2D Serpentine path
            request->serpentine_angle = serpentineAngle_;
            request->serpentine_direction = serpentineDirection_;
            request->serpentine_offset = serpentineOffset_;
            request->serpentine_polygon_vertices = serpentinePolygonVertices_;
            request->seabed_altitude_hold = seabedAltitudeHold_;
            request->seabed_altitude_goal = seabedAltitudeGoal_;
            request->shallow_depth_limit = shallowDepthLimit_;
            break;
        }
        case auv_core_helper::Serpentine3D: { // 3D Serpentine path
            request->serpentine_angle = serpentineAngle_;
            request->serpentine_direction = serpentineDirection_;
            request->serpentine_offset = serpentineOffset_;
            request->serpentine_polygon_vertices = serpentinePolygonVertices_;
            request->dive_depth = diveDepth_;
            request->curvature = curvature_;
            request->dip_num_points = diveNumPoints_;
            request->dive_length = diveLength_;
            break;
        }
        case auv_core_helper::Helix3D: { // 3D Helix path
            request->helix_start_pos = helixStartPosition_;
            request->helix_axis_pos = helixAxisPosition_;
            request->helix_axis_dir = helixAxisDirection_;
            request->helix_frequency = helixFrequency_;
            request->helix_num_quadrants = helixNumQuadrants_;
            request->helix_counter_clockwise = helixCounterClockwise_;
            break;
        }
        case auv_core_helper::Spiral2D: { // 2D inward spiral path
            request->spiral_centre_point = spiralCentrePoint_;
            request->spiral_start_point = spiralStartPoint_;
            request->spiral_radius_offset = spiralRadiusOffset_;
            break;
        }
        case auv_core_helper::Racetrack2D: { // 2D racetrack path
            request->racetrack_angle = racetrackAngle_;
            request->racetrack_forward = racetrackForward_;
            request->racetrack_first_diameter = racetrackFirstDiameter_;
            request->racetrack_second_diameter = racetrackSecondDiameter_;
            request->racetrack_polygon_vertices = racetrackPolygonVertices_;
            break;
        }
        default: {
            std::cout << "Invalid path choice: " << pathChoice_ << std::endl;
            return; // Exit the function early if the path choice is invalid
        }
    }

    auto resultFuture = controlCommandClient_->async_send_request(
        request, [](rclcpp::Client<auv_core_helper::srv::ControlCommand>::SharedFuture future) {
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
