#pragma once

// ROS 2 headers
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

// AUV-specific headers
#include "auv_core_helper/msg/pose_stamped.hpp"
#include "auv_core_helper/helper_lib.hpp"
#include "auv_msgs_ros2/topicnames.hpp"

// Dynamic model
#include "underwater_vehicle_model.hpp"

// QP headers
#include <qpOASES.hpp>

class DynamicControlLayer : public rclcpp::Node {
public:
    DynamicControlLayer();

private:
    // Callback methods
    void PoseDesiredCallback(const auv_core_helper::msg::PoseStamped::SharedPtr msg);
    void VelocityDesiredCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void AccelerationDesiredCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void PoseActualCallback(const auv_core_helper::msg::PoseStamped::SharedPtr msg);
    void VelocityActualCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void KclStateCallback(const std_msgs::msg::String::SharedPtr msg);
    void ForceComputeCallback();

    Eigen::VectorXd GetForces(const Eigen::MatrixXd& A, const Eigen::MatrixXd& b, const Eigen::MatrixXd& upLowBounds, const Eigen::VectorXd& weights);

    /**
     * @brief Convert an Eigen matrix to a qpOASES array.
     * @tparam Derived Eigen matrix type.
     * @param m Eigen matrix.
     * @return Pointer to the qpOASES array.
     */
    template <typename Derived>
    qpOASES::real_t* ConvertEigenToQpOASESArray(const Eigen::MatrixBase<Derived>& m){
        int rows = m.rows();
        int cols = m.cols();
        qpOASES::real_t* B = new qpOASES::real_t[rows * cols];
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                B[i * cols + j] = static_cast<qpOASES::real_t>(m(i, j));
            }
        }
        return B;
    }

    // Dynamics model
    std::unique_ptr<mvm::DynamicsModel> dynamicsModel_; 

    // Dynamic model properties
    double mass_;                                 // Vehicle mass
    Eigen::Vector3d centerGravity_;              // Center of Gravity
    Eigen::Matrix3d inertiaTensor_;              // Inertia tensor
    Eigen::Matrix<double, 6, 1> addedMass_;      // Added mass
    Eigen::Matrix<double, 6, 1> dampingCoefficients_; // Damping coefficients
    double buoyancy_;                            // Buoyancy
    Eigen::Vector3d centerBuoyancy_;             // Center of Buoyancy
    Eigen::Vector3d gravityVector_;              // Gravity vector

    // Thruster-related properties
    Eigen::MatrixXd thrusterPositions_;
    Eigen::MatrixXd thrusterOrientations_;
    Eigen::VectorXd thrusterUpperLimits_;
    Eigen::VectorXd thrusterLowerLimits_;
    Eigen::VectorXd thrusterAllocationWeights_;
    
    // Desired states
    Eigen::Matrix<double, 6, 1> poseDesired_;
    Eigen::Matrix<double, 6, 1> velocityDesired_;
    Eigen::Matrix<double, 6, 1> accelerationDesired_;

    // Actual states
    Eigen::Matrix<double, 6, 1> poseActual_;
    Eigen::Matrix<double, 6, 1> velocityActual_;
    Eigen::Matrix<double, 6, 1> accelerationActual_;

    // Dynamic matrices
    Eigen::Matrix<double, 6, 6> inertiaMatrix_;
    Eigen::MatrixXd thrustersWrenchMatrix_;

    std::string kclCurrentState_; // Current state

    // ROS 2 communication
    rclcpp::Subscription<auv_core_helper::msg::PoseStamped>::SharedPtr poseDesiredSubscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocityDesiredSubscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr accelerationDesiredSubscriber_;

    rclcpp::Subscription<auv_core_helper::msg::PoseStamped>::SharedPtr poseActualSubscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocityActualSubscriber_;

    rclcpp::TimerBase::SharedPtr forceComputeTimer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forcesDesiredPublisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr kclStateSubscription_;
};
