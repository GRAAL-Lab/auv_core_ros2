#include "sim/simulator.hpp"

constexpr double EPSILON = 0.001; // Define a small threshold value for zero comparison

Simulator::Simulator(const std::string& configName)
    : Node("simulator_node"), simulationTime_(0, 0, RCL_ROS_TIME) {
    // Declare and retrieve configuration parameter
    this->declare_parameter("config_name", configName);
    std::string configNameParam;
    this->get_parameter("config_name", configNameParam);

    // Load parameters from configuration
    LoadParamsFromConf(
        configNameParam, 
        &mass_, &centerGravity_, &inertiaTensor_, &addedMass_, &dampingCoefficients_,
        &buoyancy_, &centerBuoyancy_, &gravityVector_, &thrusterPositions_, 
        &thrusterOrientations_, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, 
        nullptr, nullptr, nullptr, nullptr, nullptr);

    // Publishers
    poseActualPublisher_ = this->create_publisher<auv_core_helper::msg::PoseStamped>(
        auv_core_helper::topicnames::pose_actual, 1);
    velocityActualPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        auv_core_helper::topicnames::velocity_actual, 1);
    accelerationActualPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        auv_core_helper::topicnames::acceleration_actual, 1);

    // Subscriptions
    forcesDesiredSubscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        auv_core_helper::topicnames::forces_desired, 1, 
        std::bind(&Simulator::ForcesDesiredCallback, this, std::placeholders::_1));

    kclStateSubscription_ = this->create_subscription<std_msgs::msg::String>(
        auv_core_helper::topicnames::kcl_state, 1, 
        std::bind(&Simulator::KclStateCallback, this, std::placeholders::_1));

    // Timer for simulation
    simulationTimer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Simulator::Simulate, this));

    // Initialize state vectors
    poseActual_.setZero(6);
    velocityActual_.setZero(6);
    accelerationActual_.setZero(6);

    // Initialize dynamic model matrices
    inertiaMatrix_ = GetM(mass_, centerGravity_, inertiaTensor_, addedMass_);
    thrustersWrenchMatrix_ = GetThrustersWrenchMatrix(thrusterPositions_, thrusterOrientations_);
    forcesDesired_.resize(thrustersWrenchMatrix_.cols());
}

void Simulator::ForcesDesiredCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    forcesDesired_.resize(msg->data.size());
    for (size_t i = 0; i < msg->data.size(); ++i) {
        forcesDesired_(i) = msg->data[i];
    }
}

void Simulator::Simulate() {
    double dt = 0.1; // Time step
    simulationTime_ += rclcpp::Duration::from_seconds(0.1);

    // Relative velocity
    Eigen::Matrix<double, 6, 1> velocityActualRel = velocityActual_; // Example

    Eigen::Matrix<double, 6, 6> C = GetC(velocityActualRel, mass_, centerGravity_, inertiaTensor_);
    Eigen::Matrix<double, 6, 6> D = GetD(velocityActualRel, dampingCoefficients_);
    Eigen::Matrix<double, 6, 1> g = GetG(poseActual_, mass_, buoyancy_, gravityVector_, centerGravity_, centerBuoyancy_);

    // Forces and accelerations
    Eigen::Matrix<double, 6, 1> b = thrustersWrenchMatrix_ * forcesDesired_ 
                                    - (C * velocityActualRel + D * velocityActualRel + g);
    GetAcceleration(inertiaMatrix_, b, accelerationActual_);

    // Update state
    velocityActual_ += accelerationActual_ * dt;
    poseActual_ += velocityActual_ * dt;

    // Reset conditions
    if (kclCurrentState_ == "RESET" || poseActual_.norm() > 1000.0) {
        poseActual_.setZero();
        velocityActual_.setZero();
        accelerationActual_.setZero();
        simulationTime_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    CheckAndSetZero(poseActual_);
    PublishEigenPose(poseActualPublisher_, poseActual_, simulationTime_);
    PublishEigenVelocity(velocityActualPublisher_, velocityActual_);
    PublishEigenAcceleration(accelerationActualPublisher_, accelerationActual_);
}

void Simulator::KclStateCallback(const std_msgs::msg::String::SharedPtr msg) {
    kclCurrentState_ = msg->data;
}

void Simulator::CheckAndSetZero(Eigen::Matrix<double, 6, 1>& vec) {
    for (int i = 0; i < vec.size(); ++i) {
        vec(i) = std::round(vec(i) * 10000.0) / 10000.0;
        if (std::abs(vec(i)) < EPSILON) {
            vec(i) = 0.0;
        }
    }
}

Eigen::Matrix3d Simulator::RotationMatrix(double roll, double pitch, double yaw) {
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}
