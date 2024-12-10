#include "sim/simulator.hpp"

constexpr double EPSILON = 0.001; // Define a small threshold value for zero comparison

Simulator::Simulator()
    : Node("simulator_node"), simulationTime_(0, 0, RCL_ROS_TIME) {
    // Declare and retrieve configuration parameter
    this->declare_parameter<std::string>("config_name", "default_value");
    std::string configNameParam;
    this->get_parameter("config_name", configNameParam);

    // Declare parameters for current velocities and simulation time step
    this->declare_parameter<double>("current_y_velocity", 0.0); // Default y-axis current velocity (world frame)
    this->declare_parameter<double>("current_z_velocity", 0.0); // Default z-axis current velocity (world frame)
    this->declare_parameter<double>("simulation_dt", 0.1);      // Default time step

    this->get_parameter("current_y_velocity", currentYVelocity_);
    this->get_parameter("current_z_velocity", currentZVelocity_);
    this->get_parameter("simulation_dt", dt_);

    std::cout << "Current y velocity (world frame): " << currentYVelocity_ << std::endl;
    std::cout << "Current z velocity (world frame): " << currentZVelocity_ << std::endl;
    std::cout << "Simulation time step: " << dt_ << std::endl;

    // Store parameters
    // currentVelocity_ is initially specified in world frame
    currentVelocity_ << 0.0, currentYVelocity_, currentZVelocity_, 0.0, 0.0, 0.0;

    // Construct the dynamic model parameters path
    std::string packagePath = ament_index_cpp::get_package_share_directory("auv_core_helper");
    std::string dynamicModelParamsPath_ = packagePath + "/param/dynamic_model/" + configNameParam + ".conf";

    // Load configuration file
    libconfig::Config config;
    try {
        config.readFile(dynamicModelParamsPath_.c_str());
    } catch (const libconfig::FileIOException &fioex) {
        RCLCPP_ERROR(this->get_logger(), "I/O error while reading file: %s", dynamicModelParamsPath_.c_str());
        throw std::runtime_error("Failed to load configuration file: I/O error");
    } catch (const libconfig::ParseException &pex) {
        RCLCPP_ERROR(this->get_logger(), "Parse error at %s:%d - %s", pex.getFile(), pex.getLine(), pex.getError());
        throw std::runtime_error("Failed to load configuration file: Parse error");
    }

    RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully from: %s", dynamicModelParamsPath_.c_str());

    // Initialize the dynamics model using a smart pointer
    dynamicsModel_ = std::make_unique<SixDOF::DynamicsModel>(config, configNameParam);

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
    simulationTimer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt_ * 1000)),
                                               std::bind(&Simulator::Simulate, this));

    // Initialize state vectors
    // Pose in world frame (x, y, z, roll, pitch, yaw)
    poseActual_.setZero(6);
    // Velocity in body frame (u, v, w, p, q, r)
    velocityActual_.setZero(6);
    // Acceleration in body frame
    accelerationActual_.setZero(6);

    // Initialize forcesDesired_ with zeros
    forcesDesired_ = Eigen::VectorXd::Zero(dynamicsModel_->GetNumThrusters());

    RCLCPP_INFO(this->get_logger(), "Simulator initialized (body-frame velocities).");
}

void Simulator::ForcesDesiredCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // Ensure that the size matches the number of thrusters
    if (msg->data.size() != dynamicsModel_->GetNumThrusters()) {
    RCLCPP_ERROR(this->get_logger(), "ForcesDesiredCallback: Received forces vector of size %zu, expected %zu", msg->data.size(), dynamicsModel_->GetNumThrusters());
        return;
    }
    forcesDesired_.resize(msg->data.size());
    for (size_t i = 0; i < msg->data.size(); ++i) {
        forcesDesired_(i) = msg->data[i];
    }
}

void Simulator::Simulate() {
    simulationTime_ += rclcpp::Duration::from_seconds(dt_);

    // Use EulerRPY to get the rotation matrix
    rml::EulerRPY rpy(poseActual_(3), poseActual_(4), poseActual_(5));
    Eigen::Matrix3d R = rpy.ToRotationMatrix().matrix();

    // Convert current velocity from world frame to body frame
    Eigen::Vector3d currentVelLinearWorld(currentVelocity_(0), currentVelocity_(1), currentVelocity_(2));
    Eigen::Vector3d currentVelLinearBody = R.transpose() * currentVelLinearWorld;

    // No angular current assumed
    Eigen::Matrix<double,6,1> currentVelocityBody;
    currentVelocityBody << currentVelLinearBody(0), currentVelLinearBody(1), currentVelLinearBody(2), 0.0, 0.0, 0.0;

    // velocityActual_ in body frame, so is currentVelocityBody
    Eigen::Matrix<double, 6, 1> velocityActualRel_ = velocityActual_ - currentVelocityBody;

    // Update the dynamics model (pose in world frame, velocity in body frame)
    dynamicsModel_->UpdateModel(velocityActualRel_, poseActual_);

    // Compute acceleration in body frame
    accelerationActual_ = dynamicsModel_->ComputeAcceleration(forcesDesired_);

    // Integrate velocity in body frame
    velocityActual_ += accelerationActual_ * dt_;

    // Convert body-frame linear velocity to world frame for position integration
    Eigen::Vector3d vBody(velocityActual_(0), velocityActual_(1), velocityActual_(2));
    Eigen::Vector3d vWorld = R * vBody;

    // Integrate position in world frame
    poseActual_.x() += vWorld.x() * dt_;
    poseActual_.y() += vWorld.y() * dt_;
    poseActual_.z() += vWorld.z() * dt_;

    // Convert body angular velocities to Euler angle rates using EulerRPY
    Eigen::Vector3d wBody(velocityActual_(3), velocityActual_(4), velocityActual_(5));
    Eigen::Vector3d eulerRates = rpy.Derivative(wBody);

    // Integrate orientation
    poseActual_(3) += eulerRates(0) * dt_; // roll
    poseActual_(4) += eulerRates(1) * dt_; // pitch
    poseActual_(5) += eulerRates(2) * dt_; // yaw

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
