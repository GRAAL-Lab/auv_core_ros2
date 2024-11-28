#include "dcl/dynamic_control_layer.hpp"

DCL::DCL(const std::string& config_name) : Node("dynamic_control_layer_node") {
    this->declare_parameter("config_name", config_name);
    std::string config_name_param;
    this->get_parameter("config_name", config_name_param);
    LoadParamsFromConf(config_name_param, &m_, &CG_, &I_, &M_a_diag_, &D_diag_, &B_, &CB_, &G_, &thruster_positions_, &thruster_orientations_, &thruster_upper_limits, &thruster_lower_limits, &thruster_allocation_weights, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    pose_desired_subscriber_ = this->create_subscription<auv_core_helper::msg::PoseStamped>(auv_core_helper::topicnames::pose_desired, 1, std::bind(&DCL::pose_desired_callback, this, _1));
    velocity_desired_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(auv_core_helper::topicnames::velocity_desired, 1, std::bind(&DCL::velocity_desired_callback, this, _1));

    pose_actual_subscriber_ = this->create_subscription<auv_core_helper::msg::PoseStamped>(auv_core_helper::topicnames::pose_actual, 1, std::bind(&DCL::pose_actual_callback, this, _1));
    velocity_actual_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(auv_core_helper::topicnames::velocity_actual, 1, std::bind(&DCL::velocity_actual_callback, this, _1));
    KCL_state_subscription_ = this->create_subscription<std_msgs::msg::String>(auv_core_helper::topicnames::kcl_state, 1, std::bind(&DCL::KCL_state_callback, this, _1));
    forces_desired_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(auv_core_helper::topicnames::forces_desired, 1);
    M = GetM(m_, CG_, I_, M_a_diag_);
    ThrustersWrenchMatrix = GetThrustersWrenchMatrix(thruster_positions_, thruster_orientations_);

    force_compute_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DCL::force_compute_callback, this));
}

void DCL::pose_desired_callback(const auv_core_helper::msg::PoseStamped::SharedPtr msg) {
    pose_desired_ << msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw;
}

void DCL::velocity_desired_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    velocity_desired_ << msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z;
}

void DCL::pose_actual_callback(const auv_core_helper::msg::PoseStamped::SharedPtr msg) {
    pose_actual_ << msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw;
    //std::cout << "Updated Actual Pose Matrix: \n" << pose_actual_.transpose() << std::endl;
}

void DCL::velocity_actual_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    velocity_actual_ << msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z;
    //std::cout << "Updated Actual Velocity Matrix: \n" << velocity_actual_.transpose() << std::endl;
}

void DCL::KCL_state_callback(const std_msgs::msg::String::SharedPtr msg) {
    KCL_current_state_ = msg->data;
}

void DCL::force_compute_callback() {

    // Compute the matrices involved in the dynamic model equation
    Eigen::Matrix<double, 6, 6> C = GetC(velocity_desired_, m_, CG_, I_);
    Eigen::Matrix<double, 6, 6> D = GetD(velocity_desired_, D_diag_);
    Eigen::Matrix<double, 6, 1> g = GetG(pose_actual_, m_, B_, G_, CG_, CB_);

    //solve for acceleration desired
    acceleration_desired_ = (velocity_desired_-velocity_actual_)*1;

    // Compute the left-hand side of the equation
    Eigen::Matrix<double, 6, 1> lhs = M * acceleration_desired_ + C * velocity_desired_ + D * velocity_desired_ + g;
    // Update bounds matrix
    Eigen::MatrixXd up_low_bounds(ThrustersWrenchMatrix.cols(), 2);
    up_low_bounds.col(0) = thruster_lower_limits;
    up_low_bounds.col(1) = thruster_upper_limits;

    // Use the QP solver to get optimal forces
    Eigen::VectorXd x = GetForces(ThrustersWrenchMatrix, lhs, up_low_bounds, thruster_allocation_weights);
    // Publish the computed forces
    std_msgs::msg::Float64MultiArray force_msg;
    force_msg.data.resize(x.size());
    std::copy_n(x.data(), x.size(), force_msg.data.begin());

    if (KCL_current_state_ == "IDLE" || KCL_current_state_ == "RESET") {
        std::fill(force_msg.data.begin(), force_msg.data.end(), 0.0);
    } else {
        for (size_t i = 0; i < static_cast<size_t>(x.size()); ++i) {
            force_msg.data[i] = x(i);
        }
    }
    forces_desired_publisher_->publish(force_msg);
}
