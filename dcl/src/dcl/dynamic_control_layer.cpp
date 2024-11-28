#include "dcl/dynamic_control_layer.hpp"

DynamicControlLayer::DynamicControlLayer(const std::string& configName)
    : Node("dynamic_control_layer_node") {
    // Declare and retrieve configuration parameter
    this->declare_parameter("config_name", configName);
    std::string configNameParam;
    this->get_parameter("config_name", configNameParam);

    // Load parameters from configuration
    LoadParamsFromConf(
        configNameParam, 
        &mass_, &centerGravity_, &inertiaTensor_, &addedMass_, &dampingCoefficients_,
        &buoyancy_, &centerBuoyancy_, &gravityVector_, &thrusterPositions_, &thrusterOrientations_,
        &thrusterUpperLimits_, &thrusterLowerLimits_, &thrusterAllocationWeights_,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);

    // Subscriptions
    poseDesiredSubscriber_ = this->create_subscription<auv_core_helper::msg::PoseStamped>(
        auv_core_helper::topicnames::pose_desired, 1, 
        std::bind(&DynamicControlLayer::PoseDesiredCallback, this, std::placeholders::_1));

    velocityDesiredSubscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        auv_core_helper::topicnames::velocity_desired, 1, 
        std::bind(&DynamicControlLayer::VelocityDesiredCallback, this, std::placeholders::_1));

    poseActualSubscriber_ = this->create_subscription<auv_core_helper::msg::PoseStamped>(
        auv_core_helper::topicnames::pose_actual, 1, 
        std::bind(&DynamicControlLayer::PoseActualCallback, this, std::placeholders::_1));

    velocityActualSubscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        auv_core_helper::topicnames::velocity_actual, 1, 
        std::bind(&DynamicControlLayer::VelocityActualCallback, this, std::placeholders::_1));

    kclStateSubscription_ = this->create_subscription<std_msgs::msg::String>(
        auv_core_helper::topicnames::kcl_state, 1, 
        std::bind(&DynamicControlLayer::KclStateCallback, this, std::placeholders::_1));

    // Publisher
    forcesDesiredPublisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        auv_core_helper::topicnames::forces_desired, 1);

    // Initialize dynamic model matrices
    inertiaMatrix_ = GetM(mass_, centerGravity_, inertiaTensor_, addedMass_);
    thrustersWrenchMatrix_ = GetThrustersWrenchMatrix(thrusterPositions_, thrusterOrientations_);

    // Timer
    forceComputeTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&DynamicControlLayer::ForceComputeCallback, this));
}

void DynamicControlLayer::PoseDesiredCallback(const auv_core_helper::msg::PoseStamped::SharedPtr msg) {
    poseDesired_ << msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw;
}

void DynamicControlLayer::VelocityDesiredCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    velocityDesired_ << msg->linear.x, msg->linear.y, msg->linear.z, 
                        msg->angular.x, msg->angular.y, msg->angular.z;
}

void DynamicControlLayer::PoseActualCallback(const auv_core_helper::msg::PoseStamped::SharedPtr msg) {
    poseActual_ << msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw;
}

void DynamicControlLayer::VelocityActualCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    velocityActual_ << msg->linear.x, msg->linear.y, msg->linear.z, 
                       msg->angular.x, msg->angular.y, msg->angular.z;
}

void DynamicControlLayer::KclStateCallback(const std_msgs::msg::String::SharedPtr msg) {
    kclCurrentState_ = msg->data;
}

void DynamicControlLayer::ForceComputeCallback() {
    // Compute dynamic model components
    Eigen::Matrix<double, 6, 6> coriolisMatrix = GetC(velocityDesired_, mass_, centerGravity_, inertiaTensor_);
    Eigen::Matrix<double, 6, 6> dampingMatrix = GetD(velocityDesired_, dampingCoefficients_);
    Eigen::Matrix<double, 6, 1> gravityVector = GetG(poseActual_, mass_, buoyancy_, gravityVector_, centerGravity_, centerBuoyancy_);

    // Solve for desired acceleration
    accelerationDesired_ = (velocityDesired_ - velocityActual_) * 1;

    // Compute left-hand side
    Eigen::Matrix<double, 6, 1> lhs = 
        inertiaMatrix_ * accelerationDesired_ + 
        coriolisMatrix * velocityDesired_ + 
        dampingMatrix * velocityDesired_ + 
        gravityVector;

    // Update bounds matrix
    Eigen::MatrixXd upLowBounds(thrustersWrenchMatrix_.cols(), 2);
    upLowBounds.col(0) = thrusterLowerLimits_;
    upLowBounds.col(1) = thrusterUpperLimits_;

    // Solve for thruster forces
    Eigen::VectorXd forces = GetForces(thrustersWrenchMatrix_, lhs, upLowBounds, thrusterAllocationWeights_);

    // Prepare message for publishing
    std_msgs::msg::Float64MultiArray forceMsg;
    forceMsg.data.resize(forces.size());
    std::copy_n(forces.data(), forces.size(), forceMsg.data.begin());

    if (kclCurrentState_ == "IDLE" || kclCurrentState_ == "RESET") {
        std::fill(forceMsg.data.begin(), forceMsg.data.end(), 0.0);
    } else {
        for (size_t i = 0; i < static_cast<size_t>(forces.size()); ++i) {
            forceMsg.data[i] = forces(i);
        }
    }

    forcesDesiredPublisher_->publish(forceMsg);
}
