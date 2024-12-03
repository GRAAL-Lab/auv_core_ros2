#include "sim/simulator.hpp"

int main(int argc, char** argv) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create a shared pointer to the Simulator node without arguments
    auto node = std::make_shared<Simulator>();

    // Spin the ROS 2 node
    rclcpp::spin(node);

    // Shut down the ROS 2 system
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
