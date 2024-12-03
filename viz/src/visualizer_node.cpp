#include "viz/visualizer.hpp" // Include the header file for the Visualizer class


[[nodiscard]] int main(int argc, char** argv) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create a shared pointer to the Visualizer node without arguments
    auto node = std::make_shared<Visualizer>();

    // Spin the ROS 2 node
    rclcpp::spin(node);

    // Shut down the ROS 2 system
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
