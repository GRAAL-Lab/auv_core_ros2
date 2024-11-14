#include "kcl/kcl_class.hpp"

KCL::KCL(const std::string& config_name) : Node("kcl_fsm_node") {
    this->declare_parameter("config_name", config_name);
    std::string config_name_param;
    this->get_parameter("config_name", config_name_param);
    ctrlData_ = std::make_shared<auv::ControlData>();

    LoadParamsFromConf(
        config_name_param,
        nullptr,    // mass (m)
        nullptr,    // center of gravity (CG)
        nullptr,    // inertia tensor (I)
        nullptr,    // added mass diagonal (M_a_diag)
        nullptr,    // damping diagonal (D_diag)
        nullptr,    // buoyancy (B)
        nullptr,    // center of buoyancy (CB)
        nullptr,    // gravity vector (G)
        nullptr,    // thruster positions
        nullptr,    // thruster orientations
        nullptr,    // thruster upper limits
        nullptr,    // thruster lower limits
        nullptr,    // thruster allocation weights
        &ctrlData_->gainsX_,    // gains for X axis
        &ctrlData_->gainsY_,    // gains for Y axis
        &ctrlData_->gainsZ_,    // gains for Z axis
        &ctrlData_->gainsRoll_, // gains for roll
        &ctrlData_->gainsPitch_,// gains for pitch
        &ctrlData_->gainsYaw_,  // gains for yaw
        &ctrlData_->maxVelocity_, // max linear/angular velocities
        &ctrlData_->minVelocity_  // min linear/angular velocities
        );

    setupTransitions();

    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, std::bind(&KCL::JoyStickCallback, this, std::placeholders::_1));
    fsm_timer_ = this->create_wall_timer(100ms, std::bind(&KCL::executeFSM, this));

    pose_actual_subscriber_ = this->create_subscription<auv_core_helper::msg::PoseStamped>(auv_core_helper::topicnames::pose_actual, 1, std::bind(&KCL::pose_actual_callback, this, _1));
    velocity_actual_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(auv_core_helper::topicnames::velocity_actual, 1, std::bind(&KCL::velocity_actual_callback, this, _1));
    acceleration_actual_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(auv_core_helper::topicnames::acceleration_actual, 1, std::bind(&KCL::acceleration_actual_callback, this, _1));

    pose_goal_publisher_ = this->create_publisher<auv_core_helper::msg::PoseStamped>(auv_core_helper::topicnames::pose_goal, 1);
    velocity_desired_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(auv_core_helper::topicnames::velocity_desired, 1);

    control_command_service_ = this->create_service<auv_core_helper::srv::ControlCommand>(auv_core_helper::topicnames::control_cmd_service, std::bind(&KCL::handleControlCommand, this, _1, _2));
    state_publisher_ = this->create_publisher<std_msgs::msg::String>(auv_core_helper::topicnames::kcl_state, 1);

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 1);
}

void KCL::executeFSM() {
    std::string previous_state = fsm_.GetCurrentStateName();
    fsm_.SwitchState();
    fsm_.ExecuteState();

    std_msgs::msg::String state_msg;
    state_msg.data = fsm_.GetCurrentStateName();
    state_publisher_->publish(state_msg);

    publish_Eigen_pose(pose_goal_publisher_, ctrlData_->pose_goal_, this->get_clock()->now());

    // Calculate scaling factors to not exceed maximum and minimum velocities
    double scaleDown = 1.0;
    for (int i = 0; i < 3; ++i) {
        // Handle scaling for linear velocities
        if (ctrlData_->velocity_desired_[i] > ctrlData_->maxVelocity_[i]) {
            scaleDown = std::min(scaleDown, ctrlData_->maxVelocity_[i] / ctrlData_->velocity_desired_[i]);
        } else if (ctrlData_->velocity_desired_[i] < ctrlData_->minVelocity_[i]) {
            scaleDown = std::min(scaleDown, ctrlData_->minVelocity_[i] / ctrlData_->velocity_desired_[i]);
        }

        // Handle scaling for angular velocities
        if (ctrlData_->velocity_desired_[i + 3] > ctrlData_->maxVelocity_[i + 3]) {
            scaleDown = std::min(scaleDown, ctrlData_->maxVelocity_[i + 3] / ctrlData_->velocity_desired_[i + 3]);
        } else if (ctrlData_->velocity_desired_[i + 3] < ctrlData_->minVelocity_[i + 3]) {
            scaleDown = std::min(scaleDown, ctrlData_->minVelocity_[i + 3] / ctrlData_->velocity_desired_[i + 3]);
        }
    }

    // Scale down velocities if necessary
    if (scaleDown < 1.0) {
        ctrlData_->velocity_desired_.head(6) *= scaleDown;
    }

    // Final clamping to ensure values are within bounds
    for (int i = 0; i < 3; ++i) {
        // Clamp linear velocities
        ctrlData_->velocity_desired_[i] = std::max(ctrlData_->minVelocity_[i], std::min(ctrlData_->velocity_desired_[i], ctrlData_->maxVelocity_[i]));

        // Clamp angular velocities
        ctrlData_->velocity_desired_[i + 3] = std::max(ctrlData_->minVelocity_[i + 3], std::min(ctrlData_->velocity_desired_[i + 3], ctrlData_->maxVelocity_[i + 3]));
    }


    publish_Eigen_velocity(velocity_desired_publisher_, ctrlData_->velocity_desired_);

    // Publish the planned path
    path_publisher_->publish(ctrlData_->planned_path_);
}

void KCL::setupTransitions() {
    idleState_ = std::make_unique<IdleState>(&fsm_);
    holdState_ = std::make_unique<HoldState>(&fsm_);
    joystickState_ = std::make_unique<JoystickState>(&fsm_);
    trajectoryPlanningState_ = std::make_unique<TrajectoryPlanningState>(&fsm_);
    pathPlanningState_ = std::make_unique<PathPlanningState>(&fsm_);

    // Assigning Shared Pointer to states
    idleState_->ctrlData = ctrlData_;
    holdState_->ctrlData = ctrlData_;
    joystickState_->ctrlData = ctrlData_;
    trajectoryPlanningState_->ctrlData = ctrlData_;
    pathPlanningState_->ctrlData = ctrlData_;

    fsm_.AddState(States::IDLE, idleState_.get());
    fsm_.AddState(States::HOLD, holdState_.get());
    fsm_.AddState(States::JOYSTICK, joystickState_.get());
    fsm_.AddState(States::TRAJECTORY_FOLLOWING, trajectoryPlanningState_.get());
    fsm_.AddState(States::PATH_FOLLOWING, pathPlanningState_.get());

    fsm_.EnableTransition(States::IDLE, States::JOYSTICK, true);
    fsm_.EnableTransition(States::IDLE, States::TRAJECTORY_FOLLOWING, true);
    fsm_.EnableTransition(States::IDLE, States::PATH_FOLLOWING, true);
    fsm_.EnableTransition(States::IDLE, States::HOLD, true);
    fsm_.EnableTransition(States::JOYSTICK, States::IDLE, true);
    fsm_.EnableTransition(States::JOYSTICK, States::TRAJECTORY_FOLLOWING, true);
    fsm_.EnableTransition(States::JOYSTICK, States::PATH_FOLLOWING, true);
    fsm_.EnableTransition(States::JOYSTICK, States::HOLD, true);
    fsm_.EnableTransition(States::TRAJECTORY_FOLLOWING, States::IDLE, true);
    fsm_.EnableTransition(States::TRAJECTORY_FOLLOWING, States::JOYSTICK, true);
    fsm_.EnableTransition(States::TRAJECTORY_FOLLOWING, States::PATH_FOLLOWING, true);
    fsm_.EnableTransition(States::TRAJECTORY_FOLLOWING, States::HOLD, true);
    fsm_.EnableTransition(States::PATH_FOLLOWING, States::IDLE, true);
    fsm_.EnableTransition(States::PATH_FOLLOWING, States::JOYSTICK, true);
    fsm_.EnableTransition(States::PATH_FOLLOWING, States::TRAJECTORY_FOLLOWING, true);
    fsm_.EnableTransition(States::PATH_FOLLOWING, States::HOLD, true);
    fsm_.EnableTransition(States::HOLD, States::IDLE, true);
    fsm_.EnableTransition(States::HOLD, States::JOYSTICK, true);
    fsm_.EnableTransition(States::HOLD, States::TRAJECTORY_FOLLOWING, true);
    fsm_.EnableTransition(States::HOLD, States::PATH_FOLLOWING, true);

    fsm_.SetInitState(States::HOLD);
}

void KCL::JoyStickCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    mapJoystickToVelocity(msg->axes, &ctrlData_->joystick_velocity_desired_);
}

void KCL::pose_actual_callback(const auv_core_helper::msg::PoseStamped::SharedPtr msg) {
    ctrlData_->pose_actual_ << msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw;
    ctrlData_->time_actual_ = msg->header.stamp;
}

void KCL::velocity_actual_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    ctrlData_->velocity_actual_ << msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z;
}

void KCL::acceleration_actual_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    ctrlData_->acceleration_actual_ << msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z;
}

void KCL::handleControlCommand(const std::shared_ptr<auv_core_helper::srv::ControlCommand::Request> request, std::shared_ptr<auv_core_helper::srv::ControlCommand::Response> response) {
    if (request->state == States::TRAJECTORY_FOLLOWING) {
        ctrlData_->pose_goal_ = {request->x, request->y, request->z, request->roll, request->pitch, request->yaw};
        ctrlData_->TP_goal_time_ = request->time_to_reach;
    } else if (request->state == States::PATH_FOLLOWING) {
        ctrlData_->path_planning_2d_3d = request->path_planning_2d_3d;
        if (request->path_planning_2d_3d == 3) {
            // 3D Helix Path Following
            ctrlData_->helix_startPos = {request->helix_start_pos.x, request->helix_start_pos.y, request->helix_start_pos.z};
            ctrlData_->helix_axisPos = {request->helix_axis_pos.x, request->helix_axis_pos.y, request->helix_axis_pos.z};
            ctrlData_->helix_axisDir = {request->helix_axis_dir.x, request->helix_axis_dir.y, request->helix_axis_dir.z};
            ctrlData_->helix_frequency = request->helix_frequency;
            ctrlData_->helix_numQuadrants = request->helix_num_quadrants;
            ctrlData_->helix_counterClockwise = request->helix_counter_clockwise;
        } else if (request->path_planning_2d_3d == 1){
            // 2D Serpentine Path Following
            ctrlData_->serpentine_angle_ = request->serpentine_angle;
            ctrlData_->serpentine_direction_ = request->serpentine_direction;
            ctrlData_->serpentine_offset_ = request->serpentine_offset;
            ctrlData_->serpentine_polygon_vertices_.clear();
            for (const auto& vertex : request->serpentine_polygon_vertices) {
                ctrlData_->serpentine_polygon_vertices_.emplace_back(vertex.x, vertex.y, vertex.z);
            }
        }
        else if (request->path_planning_2d_3d == 2) {
            // 3D Serpentine Path Following
            ctrlData_->serpentine_angle_ = request->serpentine_angle;
            ctrlData_->serpentine_direction_ = request->serpentine_direction;
            ctrlData_->serpentine_offset_ = request->serpentine_offset;
            ctrlData_->serpentine_polygon_vertices_.clear();
            for (const auto& vertex : request->serpentine_polygon_vertices) {
                ctrlData_->serpentine_polygon_vertices_.emplace_back(vertex.x, vertex.y, vertex.z);
            }
            ctrlData_->dive_depth_ = request->dive_depth;
            ctrlData_->curvature_ = request->curvature;
            ctrlData_->dip_num_points_ = request->dip_num_points;
            ctrlData_->dive_length_ = request->dive_length;
        }
    }

    std::cout << "Transitioning to state: " << request->state << std::endl;
    fsm_.SetNextState(request->state);
    response->success = true;
}

