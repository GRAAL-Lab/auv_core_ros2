#include "dcl/dynamic_control_layer.hpp"

DynamicControlLayer::DynamicControlLayer()
    : Node("dynamic_control_layer_node") {
    // Declare and retrieve configuration parameter
    this->declare_parameter<std::string>("config_name", "default_value");
    std::string configNameParam;
    this->get_parameter("config_name", configNameParam);

    // Load parameters from configuration
    LoadParamsFromConf(
        configNameParam, &thrusterUpperLimits_, &thrusterLowerLimits_, &thrusterAllocationWeights_,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
        
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
    dynamicsModel_ = std::make_unique<mvm::UnderwaterVehicleModel>(config, configNameParam);

    // Subscriptions
    poseDesiredSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        auv_core_helper::topicnames::pose_desired, 1, 
        std::bind(&DynamicControlLayer::PoseDesiredCallback, this, std::placeholders::_1));

    velocityDesiredSubscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        auv_core_helper::topicnames::velocity_desired, 1, 
        std::bind(&DynamicControlLayer::VelocityDesiredCallback, this, std::placeholders::_1));

    poseActualSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
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

    // Timer
    forceComputeTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(125), 
        std::bind(&DynamicControlLayer::ForceComputeCallback, this));
}

void DynamicControlLayer::PoseDesiredCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    poseDesired_ = PoseStampedMsgToEigen(*msg);
}

void DynamicControlLayer::VelocityDesiredCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    velocityDesired_ << msg->linear.x, msg->linear.y, msg->linear.z, 
                        msg->angular.x, msg->angular.y, msg->angular.z;
}

void DynamicControlLayer::PoseActualCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    poseActual_ = PoseStampedMsgToEigen(*msg);
}

void DynamicControlLayer::VelocityActualCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    velocityActual_ << msg->linear.x, msg->linear.y, msg->linear.z, 
                       msg->angular.x, msg->angular.y, msg->angular.z;
}

void DynamicControlLayer::KclStateCallback(const std_msgs::msg::String::SharedPtr msg) {
    kclCurrentState_ = msg->data;
}

void DynamicControlLayer::ForceComputeCallback() {
    // Solve for desired acceleration
    accelerationDesired_ = (velocityDesired_ - velocityActual_) * 1;

    // update the actual model
    dynamicsModel_->UpdateModel(velocityDesired_, poseDesired_);

    // Compute the left-hand side of the dynamics equation
    Eigen::Matrix<double, 6, 1> lhs_ = dynamicsModel_->GetMassMatrix() * accelerationDesired_ + dynamicsModel_->GetCoriolisMatrix() * velocityDesired_ + dynamicsModel_->GetDampingMatrix() * velocityDesired_ + dynamicsModel_->GetRestoringForces();


    // Compute acceleration given the desired forces
    thrustersWrenchMatrix_ = dynamicsModel_->GetThrustersWrenchMatrix();
    

    // Update bounds matrix
    Eigen::MatrixXd upLowBounds_(thrustersWrenchMatrix_.cols(), 2);
    upLowBounds_.col(0) = thrusterLowerLimits_;
    upLowBounds_.col(1) = thrusterUpperLimits_;

    // Solve for thruster forces
    Eigen::VectorXd forces = GetForces(thrustersWrenchMatrix_, lhs_, upLowBounds_, thrusterAllocationWeights_);

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




// Compute the forces for a system using optimization
auto DynamicControlLayer::GetForces(const Eigen::MatrixXd& A, const Eigen::MatrixXd& b, const Eigen::MatrixXd& upLowBounds_, const Eigen::VectorXd& weights) -> Eigen::VectorXd {
    int numVars = A.cols();
    Eigen::MatrixXd H = weights.asDiagonal();
    Eigen::VectorXd f = Eigen::VectorXd::Zero(numVars);
    Eigen::VectorXd lb = upLowBounds_.col(0);
    Eigen::VectorXd ub = upLowBounds_.col(1);
    Eigen::VectorXd lbA = b.cast<double>();
    Eigen::VectorXd ubA = b.cast<double>();
    qpOASES::SQProblem problem(numVars, A.rows(), qpOASES::HST_POSDEF);
    qpOASES::Options options;
    options.enableRegularisation = qpOASES::BT_TRUE;
    options.terminationTolerance = 1e-6;
    options.boundTolerance = 1e-6;
    options.printLevel = qpOASES::PL_NONE;
    problem.setOptions(options);
    qpOASES::int_t nWSR = 10000;
    qpOASES::real_t* H_d = ConvertEigenToQpOASESArray(H);
    qpOASES::real_t* f_d = ConvertEigenToQpOASESArray(f);
    qpOASES::real_t* A_d = ConvertEigenToQpOASESArray(A);
    qpOASES::real_t* lb_d = ConvertEigenToQpOASESArray(lb);
    qpOASES::real_t* ub_d = ConvertEigenToQpOASESArray(ub);
    qpOASES::real_t* lbA_d = ConvertEigenToQpOASESArray(lbA);
    qpOASES::real_t* ubA_d = ConvertEigenToQpOASESArray(ubA);
    Eigen::VectorXd x(numVars);
    if (problem.init(H_d, f_d, A_d, lb_d, ub_d, lbA_d, ubA_d, nWSR) == qpOASES::SUCCESSFUL_RETURN) {
        problem.getPrimalSolution(x.data());
    } else {
        Eigen::MatrixXd A_copy = A;
        double* thrustersWrenchMatrixData = A_copy.data();
        int rows = A.rows();
        int cols = A.cols();
        double* TWPInv = new double[cols * rows];
        double thresholdTW = 1e-4;
        double lambdaTW = 1e-2;
        double prodTW;
        int flagTW;
        rml::GT_RegPinv(thrustersWrenchMatrixData, rows, cols, TWPInv, thresholdTW, lambdaTW, &prodTW, &flagTW);
        Eigen::Map<Eigen::MatrixXd> TWPInvMatrix(TWPInv, cols, rows);
        x = TWPInvMatrix * b;
        delete[] TWPInv;
    }
    delete[] H_d;
    delete[] f_d;
    delete[] A_d;
    delete[] lb_d;
    delete[] ub_d;
    delete[] lbA_d;
    delete[] ubA_d;
    double scaleDown = 1.0;
    for (int i = 0; i < numVars; ++i) {
        if (x[i] > ub[i]) {
            scaleDown = std::min(scaleDown, ub[i] / x[i]);
        }
        if (x[i] < lb[i]) {
            scaleDown = std::min(scaleDown, lb[i] / x[i]);
        }
    }
    if (scaleDown < 1.0) {
        x *= scaleDown;
    }
    for (int i = 0; i < numVars; ++i) {
        x[i] = std::max(lb[i], std::min(x[i], ub[i]));
    }
    return x;
}
