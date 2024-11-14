#include "interface/interface_class.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
