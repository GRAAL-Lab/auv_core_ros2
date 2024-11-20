#include "interface/interface.hpp"

/// Main entry point for the InterfaceNode.
[[nodiscard]] int main(int argc, char** argv) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create a shared pointer to the InterfaceNode
    auto node = std::make_shared<InterfaceNode>();

    // Spin the ROS 2 node
    rclcpp::spin(node);

    // Shut down the ROS 2 system
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
