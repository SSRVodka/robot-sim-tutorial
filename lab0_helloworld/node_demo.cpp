
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    
    std::string node_name = "demo_cpp_node";
    auto node = std::make_shared<rclcpp::Node>(node_name);
    auto logger = node->get_logger();
    RCLCPP_INFO(logger, "hello from %s", node_name.c_str());

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

