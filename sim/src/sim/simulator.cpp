#include "sim/simulator.hpp"

constexpr double EPSILON = 0.001; // Define a small threshold value for zero comparison

simulator::simulator(const std::string& config_name) : Node("simulator"), simulation_time_(0, 0, RCL_ROS_TIME) {
    this->declare_parameter("config_name", config_name);
    std::string config_name_param;
    this->get_parameter("config_name", config_name_param);

    LoadParamsFromConf(config_name_param, &m_, &CG_, &I_, &M_a_diag_, &D_diag_, &B_, &CB_, &G_, &thruster_positions_, &thruster_orientations_, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    pose_actual_publisher_ = this->create_publisher<auv_core_helper::msg::PoseStamped>(auv_core_helper::topicnames::pose_actual, 1);
    velocity_actual_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(auv_core_helper::topicnames::velocity_actual, 1);
    acceleration_actual_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(auv_core_helper::topicnames::acceleration_actual, 1);
    forces_desired_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(auv_core_helper::topicnames::forces_desired, 1, std::bind(&simulator::forces_desired_callback, this, _1));
    KCL_state_subscription_ = this->create_subscription<std_msgs::msg::String>(auv_core_helper::topicnames::kcl_state, 1, std::bind(&simulator::KCL_state_callback, this, _1));
    timer_ = this->create_wall_timer(100ms, std::bind(&simulator::simulate, this));
    pose_actual_.setZero(6);
    velocity_actual_.setZero(6);
    acceleration_actual_.setZero(6);
    M = GetM(m_, CG_, I_, M_a_diag_);
    ThrustersWrenchMatrix = GetThrustersWrenchMatrix(thruster_positions_, thruster_orientations_);
    forces_desired_.resize(ThrustersWrenchMatrix.cols());
}

void simulator::forces_desired_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // Resize the vector to match the size of the incoming data
    forces_desired_.resize(msg->data.size());
    for (size_t i = 0; i < msg->data.size(); ++i) {
        forces_desired_(i) = msg->data[i];
    }
}


void simulator::simulate() {
    double dt = 0.1; // Time step
    simulation_time_ += rclcpp::Duration::from_seconds(0.1);

    // Define current velocity in the world frame
    Eigen::Vector3d current_velocity(0.0, 0.1, 0.1); // Example values for current velocity
    // Eigen::Matrix3d R = rotationMatrix(pose_actual_(3), pose_actual_(4), pose_actual_(5));
    Eigen::Matrix<double, 6, 1> current_velocity_body;
    current_velocity_body.head<3>() = current_velocity;
    current_velocity_body.tail<3>().setZero(); // Assuming no angular velocity due to current
    Eigen::Matrix<double, 6, 1> velocity_actual_rel = velocity_actual_ - current_velocity_body;

    // Continue with your dynamics calculation using the relative velocity
    Eigen::Matrix<double, 6, 6> C = GetC(velocity_actual_rel, m_, CG_, I_);
    Eigen::Matrix<double, 6, 6> D = GetD(velocity_actual_rel, D_diag_);
    Eigen::Matrix<double, 6, 1> g = GetG(pose_actual_, m_, B_, G_, CG_, CB_);
    // Compute the forces/accelerations
    Eigen::Matrix<double, 6, 1> b = ThrustersWrenchMatrix * forces_desired_ - (C * velocity_actual_rel + D * velocity_actual_rel + g);
    GetAcceleration(M, b, acceleration_actual_);

    // Update velocity and pose
    Eigen::Vector6d acceleration_actual_rotated;
    acceleration_actual_rotated.head<3>() = acceleration_actual_.head<3>();
    acceleration_actual_rotated.tail<3>() = acceleration_actual_.tail<3>();

    velocity_actual_ += acceleration_actual_rotated * dt;
    pose_actual_ += velocity_actual_ * dt;

    // Handle special cases like resetting
    if (KCL_current_state_ == "RESET" || pose_actual_.norm() > 1000.0) {
        pose_actual_.setZero();
        velocity_actual_.setZero();
        acceleration_actual_.setZero();
        simulation_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        KCL_current_state_ = ""; // Clear the reset state to avoid continuous reset
    }

    pose_actual_(3)=0;
    velocity_actual_(3)=0;
    acceleration_actual_(3)=0;
    check_and_set_zero(pose_actual_);
    PublishEigenPose(pose_actual_publisher_, pose_actual_, simulation_time_);
    PublishEigenVelocity(velocity_actual_publisher_, velocity_actual_);
    PublishEigenAcceleration(acceleration_actual_publisher_, acceleration_actual_);
}


void simulator::KCL_state_callback(const std_msgs::msg::String::SharedPtr msg) {
    KCL_current_state_ = msg->data;
}

void simulator::check_and_set_zero(Eigen::Matrix<double, 6, 1>& vec) {
    for (int i = 0; i < vec.size(); ++i) {
        vec(i) = std::round(vec(i) * 10000.0) / 10000.0; // Round to four decimal places
        if (std::abs(vec(i)) < EPSILON) {
            vec(i) = 0.0; // Set to zero if within EPSILON range
        }
    }
}


Eigen::Matrix3d simulator::rotationMatrix(double roll, double pitch, double yaw) {
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}
