#include "sim/simulator.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config_name>" << std::endl;
        return 1;
    }

    std::string config_name = argv[1];
    auto node = std::make_shared<simulator>(config_name);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
