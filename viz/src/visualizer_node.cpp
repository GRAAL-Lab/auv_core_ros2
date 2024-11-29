#include "viz/visualizer.hpp" // Include the header file for the Visualizer class


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

    // Create a shared pointer to the Visualizer node
    auto node = std::make_shared<Visualizer>(configName);

    // Spin the ROS 2 node
    rclcpp::spin(node);

    // Shut down the ROS 2 system
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
