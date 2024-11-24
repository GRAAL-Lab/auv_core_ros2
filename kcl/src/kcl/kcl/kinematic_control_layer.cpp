#include "kcl/kinematic_control_layer.hpp"
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
        &ctrlData_->gainsX, &ctrlData_->gainsY, &ctrlData_->gainsZ,
        &ctrlData_->gainsRoll, &ctrlData_->gainsPitch, &ctrlData_->gainsYaw,
        &ctrlData_->maxVelocity, &ctrlData_->minVelocity);


    // Set up state transitions
    SetupTransitions();

    // Create FSM timer
    fsmTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(ctrlData_->dt * 1000)),
        std::bind(&KCL::ExecuteFSM, this));


    // Create subscriptions
    joystickSubscription_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, std::bind(&KCL::JoyStickCallback, this, std::placeholders::_1));

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
        std::bind(&KCL::HandleControlCommand, this, std::placeholders::_1, std::placeholders::_2));
}

void KCL::JoyStickCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Update joystick data in control data
    ctrlData_->joystickAxes = msg->axes;
}

void KCL::PoseActualCallback(const auv_core_helper::msg::PoseStamped::SharedPtr msg) {
    // Update actual pose in control data
    ctrlData_->poseActual << msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw;
    ctrlData_->timeActual = msg->header.stamp;
}

void KCL::VelocityActualCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Update actual velocity in control data
    ctrlData_->velocityActual << msg->linear.x, msg->linear.y, msg->linear.z,
                                   msg->angular.x, msg->angular.y, msg->angular.z;
}

void KCL::AccelerationActualCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Update actual acceleration in control data
    ctrlData_->accelerationActual << msg->linear.x, msg->linear.y, msg->linear.z,
                                       msg->angular.x, msg->angular.y, msg->angular.z;
}
void KCL::HandleControlCommand(
    const std::shared_ptr<auv_core_helper::srv::ControlCommand::Request> request,
    std::shared_ptr<auv_core_helper::srv::ControlCommand::Response> response) {

    // Handle different control states
    if (request->state == States::TRAJECTORY_FOLLOWING) {
        ctrlData_->poseGoal << request->x, request->y, request->z, request->roll, request->pitch, request->yaw;
        ctrlData_->tpGoalTime = request->time_to_reach;
    } else if (request->state == States::PATH_FOLLOWING) {
        ctrlData_->pathPlanningMode = request->path_planning_2d_3d;

        // Use a switch statement to handle path planning modes
        switch (ctrlData_->pathPlanningMode) {
            case auv_core_helper::Helix3D: {
                // Handle 3D Helix Path Following
                // RCLCPP_INFO(this->get_logger(), "Helix path following requested");
                ctrlData_->helixStartPos = {request->helix_start_pos.x, request->helix_start_pos.y, request->helix_start_pos.z};
                ctrlData_->helixAxisPos = {request->helix_axis_pos.x, request->helix_axis_pos.y, request->helix_axis_pos.z};
                ctrlData_->helixAxisDir = {request->helix_axis_dir.x, request->helix_axis_dir.y, request->helix_axis_dir.z};
                ctrlData_->helixFrequency = request->helix_frequency;
                ctrlData_->helixNumQuadrants = request->helix_num_quadrants;
                ctrlData_->helixCounterClockwise = request->helix_counter_clockwise;
                break;
            }
            case auv_core_helper::Serpentine2D: {
                // Handle 2D Serpentine Path Following
                // RCLCPP_INFO(this->get_logger(), "2D Serpentine path following requested");
                ctrlData_->serpentineAngle = request->serpentine_angle;
                ctrlData_->serpentineDirection = request->serpentine_direction;
                ctrlData_->serpentineOffset = request->serpentine_offset;
                ctrlData_->serpentinePolygonVertices.clear();
                for (const auto& vertex : request->serpentine_polygon_vertices) {
                    ctrlData_->serpentinePolygonVertices.emplace_back(vertex.x, vertex.y, vertex.z);
                }
                break;
            }
            case auv_core_helper::Serpentine3D: {
                // Handle 3D Serpentine Path Following
                // RCLCPP_INFO(this->get_logger(), "3D Serpentine path following requested");
                ctrlData_->serpentineAngle = request->serpentine_angle;
                ctrlData_->serpentineDirection = request->serpentine_direction;
                ctrlData_->serpentineOffset = request->serpentine_offset;
                ctrlData_->serpentinePolygonVertices.clear();
                for (const auto& vertex : request->serpentine_polygon_vertices) {
                    ctrlData_->serpentinePolygonVertices.emplace_back(vertex.x, vertex.y, vertex.z);
                }
                ctrlData_->diveDepth = request->dive_depth;
                ctrlData_->curvature = request->curvature;
                ctrlData_->dipNumPoints = request->dip_num_points;
                ctrlData_->diveLength = request->dive_length;
                break;
            }
            default: {
                RCLCPP_ERROR(this->get_logger(), "Invalid pathPlanningMode: %d", ctrlData_->pathPlanningMode);
                response->success = false;
                return;
            }
        }
    }

    // Transition to the requested state
    RCLCPP_INFO(this->get_logger(), "Transitioning to state: %s", request->state.c_str());
    std::cout << "pathPlanningMode: " << ctrlData_->pathPlanningMode << std::endl;
    fsm_.SetNextState(request->state);
    fsm_.SwitchState();

    // Respond with success
    response->success = true;
}

void KCL::SetupTransitions() {
    // Create states
    idleState_ = std::make_unique<IdleState>(&fsm_);
    holdState_ = std::make_unique<HoldState>(&fsm_);
    joystickState_ = std::make_unique<JoystickState>(&fsm_);
    trajectoryFollowingState_ = std::make_unique<TrajectoryFollowingState>(&fsm_);
    pathFollowingState_ = std::make_unique<PathFollowingState>(&fsm_);

    // Share control data with states
    idleState_->ctrlData = ctrlData_;
    holdState_->ctrlData = ctrlData_;
    joystickState_->ctrlData = ctrlData_;
    trajectoryFollowingState_->ctrlData = ctrlData_;
    pathFollowingState_->ctrlData = ctrlData_;

    // Add states and enable transitions
    fsm_.AddState(States::IDLE, idleState_.get());
    fsm_.AddState(States::HOLD, holdState_.get());
    fsm_.AddState(States::JOYSTICK, joystickState_.get());
    fsm_.AddState(States::TRAJECTORY_FOLLOWING, trajectoryFollowingState_.get());
    fsm_.AddState(States::PATH_FOLLOWING, pathFollowingState_.get());

    // Enable transitions
    fsm_.EnableTransition(States::IDLE, States::HOLD, true);
    fsm_.EnableTransition(States::IDLE, States::JOYSTICK, true);
    fsm_.EnableTransition(States::IDLE, States::TRAJECTORY_FOLLOWING, true);
    fsm_.EnableTransition(States::IDLE, States::PATH_FOLLOWING, true);
    fsm_.EnableTransition(States::HOLD, States::IDLE, true);
    fsm_.EnableTransition(States::HOLD, States::JOYSTICK, true);
    fsm_.EnableTransition(States::HOLD, States::TRAJECTORY_FOLLOWING, true);
    fsm_.EnableTransition(States::HOLD, States::PATH_FOLLOWING, true);
    fsm_.EnableTransition(States::JOYSTICK, States::IDLE, true);
    fsm_.EnableTransition(States::JOYSTICK, States::HOLD, true);
    fsm_.EnableTransition(States::JOYSTICK, States::TRAJECTORY_FOLLOWING, true);
    fsm_.EnableTransition(States::JOYSTICK, States::PATH_FOLLOWING, true);
    fsm_.EnableTransition(States::TRAJECTORY_FOLLOWING, States::IDLE, true);
    fsm_.EnableTransition(States::TRAJECTORY_FOLLOWING, States::HOLD, true);
    fsm_.EnableTransition(States::TRAJECTORY_FOLLOWING, States::JOYSTICK, true);
    fsm_.EnableTransition(States::TRAJECTORY_FOLLOWING, States::PATH_FOLLOWING, true);
    fsm_.EnableTransition(States::PATH_FOLLOWING, States::IDLE, true);
    fsm_.EnableTransition(States::PATH_FOLLOWING, States::HOLD, true);
    fsm_.EnableTransition(States::PATH_FOLLOWING, States::JOYSTICK, true);
    fsm_.EnableTransition(States::PATH_FOLLOWING, States::TRAJECTORY_FOLLOWING, true);
    fsm_.SetInitState(States::HOLD);

    RCLCPP_INFO(this->get_logger(), "FSM transitions set up.");
}

void KCL::ExecuteFSM() {
    // Execute the current FSM state
    // std::string previous_state = fsm_.GetCurrentStateName();
    // fsm_.SwitchState();
    fsm_.ExecuteState();

    // Publish current state
    std_msgs::msg::String stateMsg;
    stateMsg.data = fsm_.GetCurrentStateName();
    statePublisher_->publish(stateMsg);

    // Publish goal pose
    PublishEigenPose(poseGoalPublisher_, ctrlData_->poseGoal, this->get_clock()->now());

    // Scale desired velocity within limits
    rml::SaturateVectorWithinLimits(ctrlData_->maxVelocity, ctrlData_->minVelocity, ctrlData_->velocityDesired);

    // Publish desired velocity
    PublishEigenVelocity(velocityDesiredPublisher_, ctrlData_->velocityDesired);

    // Publish the planned path
    pathPublisher_->publish(ctrlData_->plannedPath);
}
