
#include <QtWidgets/QApplication>
#include <rclcpp/rclcpp.hpp>

#include <sys_stat_if/msg/system_stat.hpp>

#include "main_window.h"


class SystemStatListener: public rclcpp::Node {
public:
    explicit SystemStatListener(std::string node_name): rclcpp::Node(node_name) {
        this->sub_ = this->create_subscription<sys_stat_if::msg::SystemStat>(
            "/system/stat", rclcpp::QoS(rclcpp::KeepLast(10)),
            std::bind(&SystemStatListener::on_stat_update, this, std::placeholders::_1));
    }
    ~SystemStatListener() {}
private:

    void on_stat_update(std::shared_ptr<sys_stat_if::msg::SystemStat> stat) {
        RCLCPP_INFO(this->get_logger(), "received sys stat update");
        update_my_window(*stat);
    }

    std::shared_ptr<rclcpp::Subscription<sys_stat_if::msg::SystemStat>> sub_;
};



int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    rclcpp::init(argc, argv);

    std::string node_name = "sys_stat_listener_node";
    auto node = std::make_shared<SystemStatListener>(node_name);

    SSMainWindow::get_instance()->show();
    RCLCPP_INFO(node->get_logger(), "node %s started", node_name.c_str());

    std::thread spin_thread([&]() {rclcpp::spin(node);});

    int ret = app.exec();
    
    rclcpp::shutdown();
    spin_thread.join();

    return ret;
}


