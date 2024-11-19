#include "kcl/kcl_class.hpp"
using namespace std::chrono_literals;


KCL::KCL(const std::string& configName)
    : Node("kcl_fsm_node") {
    // Declare and get the "config_name" parameter
    this->declare_parameter<std::string>("config_name", configName);
    std::string configNameParam;
    this->get_parameter("config_name", configNameParam);

    // Initialize control data
    ctrlData_ = std::make_shared<auv::ControlData>();

    // Load configuration parameters into control data
    LoadParamsFromConf(
        configNameParam,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr,
        &ctrlData_->gainsX_, &ctrlData_->gainsY_, &ctrlData_->gainsZ_,
        &ctrlData_->gainsRoll_, &ctrlData_->gainsPitch_, &ctrlData_->gainsYaw_,
        &ctrlData_->maxVelocity_, &ctrlData_->minVelocity_);

    // Set up state transitions
    SetupTransitions();

    // Create FSM timer
    fsmTimer_ = this->create_wall_timer(
        100ms, std::bind(&KCL::executeFSM, this));

    // Create subscriptions
    joystickSubscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1, std::bind(&KCL::JoyStickCallback, this, std::placeholders::_1));

    poseActualSubscription_ = this->create_subscription<auv_core_helper::msg::PoseStamped>(
        auv_core_helper::topicnames::pose_actual, 1,
        std::bind(&KCL::PoseActualCallback, this, std::placeholders::_1));

    velocityActualSubscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        auv_core_helper::topicnames::velocity_actual, 1,
        std::bind(&KCL::VelocityActualCallback, this, std::placeholders::_1));

    accelerationActualSubscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        auv_core_helper::topicnames::acceleration_actual, 1,
        std::bind(&KCL::AccelerationActualCallback, this, std::placeholders::_1));

    // Create publishers
    poseGoalPublisher_ = this->create_publisher<auv_core_helper::msg::PoseStamped>(
        auv_core_helper::topicnames::pose_goal, 1);

    velocityDesiredPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        auv_core_helper::topicnames::velocity_desired, 1);

    statePublisher_ = this->create_publisher<std_msgs::msg::String>(
        auv_core_helper::topicnames::kcl_state, 1);

    pathPublisher_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 1);

    // Create service for control commands
    controlCommandService_ = this->create_service<auv_core_helper::srv::ControlCommand>(
        auv_core_helper::topicnames::control_cmd_service,
        std::bind(&KCL::handleControlCommand, this, std::placeholders::_1, std::placeholders::_2));
}

void KCL::JoyStickCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Map joystick axes data to desired velocity
    MapJoystickToVelocity(msg->axes, &ctrlData_->joystick_velocity_desired_);
}

void KCL::PoseActualCallback(const auv_core_helper::msg::PoseStamped::SharedPtr msg) {
    // Update actual pose in control data
    ctrlData_->pose_actual_ << msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw;
    ctrlData_->time_actual_ = msg->header.stamp;
}

void KCL::VelocityActualCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Update actual velocity in control data
    ctrlData_->velocity_actual_ << msg->linear.x, msg->linear.y, msg->linear.z,
                                   msg->angular.x, msg->angular.y, msg->angular.z;
}

void KCL::AccelerationActualCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Update actual acceleration in control data
    ctrlData_->acceleration_actual_ << msg->linear.x, msg->linear.y, msg->linear.z,
                                       msg->angular.x, msg->angular.y, msg->angular.z;
}

void KCL::handleControlCommand(
    const std::shared_ptr<auv_core_helper::srv::ControlCommand::Request> request,
    std::shared_ptr<auv_core_helper::srv::ControlCommand::Response> response) {
    // Handle different control states
    if (request->state == States::TRAJECTORY_FOLLOWING) {
        ctrlData_->pose_goal_ = {request->x, request->y, request->z, request->roll, request->pitch, request->yaw};
        ctrlData_->TP_goal_time_ = request->time_to_reach;
    } else if (request->state == States::PATH_FOLLOWING) {
        ctrlData_->path_planning_2d_3d = request->path_planning_2d_3d;
        if (request->path_planning_2d_3d == 3) {
            // Handle 3D Helix Path Following
            ctrlData_->helix_startPos = {request->helix_start_pos.x, request->helix_start_pos.y, request->helix_start_pos.z};
            ctrlData_->helix_axisPos = {request->helix_axis_pos.x, request->helix_axis_pos.y, request->helix_axis_pos.z};
            ctrlData_->helix_axisDir = {request->helix_axis_dir.x, request->helix_axis_dir.y, request->helix_axis_dir.z};
            ctrlData_->helix_frequency = request->helix_frequency;
            ctrlData_->helix_numQuadrants = request->helix_num_quadrants;
            ctrlData_->helix_counterClockwise = request->helix_counter_clockwise;
        } else if (request->path_planning_2d_3d == 1) {
            // Handle 2D Serpentine Path Following
            ctrlData_->serpentine_angle_ = request->serpentine_angle;
            ctrlData_->serpentine_direction_ = request->serpentine_direction;
            ctrlData_->serpentine_offset_ = request->serpentine_offset;
            ctrlData_->serpentine_polygon_vertices_.clear();
            for (const auto& vertex : request->serpentine_polygon_vertices) {
                ctrlData_->serpentine_polygon_vertices_.emplace_back(vertex.x, vertex.y, vertex.z);
            }
        } else if (request->path_planning_2d_3d == 2) {
            // Handle 3D Serpentine Path Following
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

    // Transition to the requested state
    std::cout << "Transitioning to state: " << request->state << std::endl;
    fsm_.SetNextState(request->state);

    // Respond with success
    response->success = true;
}

void KCL::SetupTransitions() {
    // Create states
    idleState_ = std::make_unique<IdleState>(&fsm_);
    holdState_ = std::make_unique<HoldState>(&fsm_);
    joystickState_ = std::make_unique<JoystickState>(&fsm_);
    trajectoryPlanningState_ = std::make_unique<TrajectoryPlanningState>(&fsm_);
    pathPlanningState_ = std::make_unique<PathPlanningState>(&fsm_);

    // Share control data with states
    idleState_->ctrlData = ctrlData_;
    holdState_->ctrlData = ctrlData_;
    joystickState_->ctrlData = ctrlData_;
    trajectoryPlanningState_->ctrlData = ctrlData_;
    pathPlanningState_->ctrlData = ctrlData_;

    // Add states and enable transitions
    fsm_.AddState(States::IDLE, idleState_.get());
    fsm_.AddState(States::HOLD, holdState_.get());
    fsm_.AddState(States::JOYSTICK, joystickState_.get());
    fsm_.AddState(States::TRAJECTORY_FOLLOWING, trajectoryPlanningState_.get());
    fsm_.AddState(States::PATH_FOLLOWING, pathPlanningState_.get());

    // Enable transitions
    fsm_.EnableTransition(States::IDLE, States::JOYSTICK, true);
    fsm_.EnableTransition(States::IDLE, States::TRAJECTORY_FOLLOWING, true);
    fsm_.SetInitState(States::HOLD);

    RCLCPP_INFO(this->get_logger(), "FSM transitions set up.");
}

void KCL::executeFSM() {
    // Execute the current FSM state
    std::string previous_state = fsm_.GetCurrentStateName();
    fsm_.SwitchState();
    fsm_.ExecuteState();

    // Publish current state
    std_msgs::msg::String stateMsg;
    stateMsg.data = fsm_.GetCurrentStateName();
    statePublisher_->publish(stateMsg);

    // Publish goal pose
    PublishEigenPose(poseGoalPublisher_, ctrlData_->pose_goal_, this->get_clock()->now());

    // Final clamping of velocities
    for (int i = 0; i < 6; ++i) {
        ctrlData_->velocity_desired_[i] = std::clamp(ctrlData_->velocity_desired_[i], ctrlData_->minVelocity_[i], ctrlData_->maxVelocity_[i]);
    }

    // Publish desired velocity
    PublishEigenVelocity(velocityDesiredPublisher_, ctrlData_->velocity_desired_);

    // Publish the planned path
    pathPublisher_->publish(ctrlData_->planned_path_);
}
