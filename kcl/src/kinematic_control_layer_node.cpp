#include <iostream>       // For std::cerr, std::endl
#include <memory>         // For std::make_shared
#include <string>         // For std::string
#include "kcl/kinematic_control_layer.hpp"  // Include KCL class definition (project-specific header)
#include "rclcpp/rclcpp.hpp"  // For ROS 2 API

[[nodiscard]] int main(int argc, char** argv) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Validate input arguments
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config_name>" << std::endl;
        return EXIT_FAILURE;
    }

    // Parse the configuration name argument
    std::string configName = argv[1];

    // Create a shared pointer to the KCL node
    auto node = std::make_shared<KCL>(configName);

    // Spin the ROS 2 node
    rclcpp::spin(node);

    // Shut down the ROS 2 system
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
