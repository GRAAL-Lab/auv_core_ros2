#ifndef HELPER_LIB_HPP
#define HELPER_LIB_HPP

#include <Eigen/Dense>
#include <string>
#include <qpOASES.hpp>
#include <rclcpp/rclcpp.hpp>
#include "auv_core_helper/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <libconfig.h++>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem> 

//using namespace Eigen;

// Declaration of functions
void LoadParamsFromConf(const std::string& config_name, double* m, Eigen::Vector3d* CG, Eigen::Matrix3d* I, Eigen::Matrix<double, 6, 1>* M_a_diag, Eigen::Matrix<double, 6, 1>* D_diag, double* B, Eigen::Vector3d* CB, Eigen::Vector3d* G, Eigen::MatrixXd* thruster_positions, Eigen::MatrixXd* thruster_orientations, Eigen::VectorXd* thruster_upper_limits, Eigen::VectorXd* thruster_lower_limits, Eigen::VectorXd* thruster_allocation_weights,
                        Eigen::VectorXd* gainsX, Eigen::VectorXd* gainsY, Eigen::VectorXd* gainsZ, Eigen::VectorXd* gainsRoll, Eigen::VectorXd* gainsPitch, Eigen::VectorXd* gainsYaw, Eigen::VectorXd* max_linear_angular_velocities, Eigen::VectorXd* min_linear_angular_velocities);


// Function template for matrix to qpOASES array conversion
template <typename Derived>
qpOASES::real_t* convertEigenToQpOASESArray(const Eigen::MatrixBase<Derived>& m) {
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

void publish_Eigen_pose(const rclcpp::Publisher<auv_core_helper::msg::PoseStamped>::SharedPtr& publisher, const Eigen::Matrix<double, 6, 1>& pose, const rclcpp::Time& time);
void publish_Eigen_velocity(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& publisher, const Eigen::Matrix<double, 6, 1>& velocity);
void publish_Eigen_acceleration(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& publisher, const Eigen::Matrix<double, 6, 1>& acceleration);
void mapJoystickToVelocity(const std::vector<float>& axes, geometry_msgs::msg::Twist* velocity_desired);
#endif // HELPER_LIB_HPP
