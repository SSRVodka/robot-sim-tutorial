
#include <rclcpp/rclcpp.hpp>

#include <random>

#include <turtle_pd_if/srv/patrol.hpp>
#include <turtle_pd_if/srv/patrol_status.hpp>

using namespace std::chrono_literals;

#define AREA_MIN 0
#define AREA_MAX 12


class RandomPatrolClient: public rclcpp::Node {
public:
    RandomPatrolClient(std::string node_name)
    : rclcpp::Node(node_name), rd(), gen(this->rd()), dis(AREA_MIN, AREA_MAX) {
        // Lab1 TODO
        // 提示：参考 turtle_controller 的服务接口
        // this->client_ = nullptr;
        // this->patrol_status_ = nullptr;
        this->timer_ = this->create_timer(1s, std::bind(&RandomPatrolClient::send_random_request, this));
    }
    ~RandomPatrolClient() {}

private:

    void send_random_request() {
        float rand_x = this->dis(this->gen);
        float rand_y = this->dis(this->gen);
        this->send_patrol_request(rand_x, rand_y);
    }

    void send_patrol_request(float x, float y) {
        RCLCPP_INFO(this->get_logger(), "sending patrol request (%.2f, %.2f) to turtle controller...", x, y);
        
        while (!this->patrol_status_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "rclcpp context error while waiting for patrol status. Abort");
                return;
            }
            RCLCPP_WARN(this->get_logger(), "waiting for patrol status service to be ready...");
        }
        turtle_pd_if::srv::PatrolStatus::Request sreq;
        this->patrol_status_->async_send_request(std::make_shared<decltype(sreq)>(sreq),
            [this, x, y](rclcpp::Client<turtle_pd_if::srv::PatrolStatus>::SharedFuture req_future) {
            auto resp = req_future.get();
            RCLCPP_INFO(this->get_logger(), "patrol status request finish. status=%d", resp->status);

            if (resp->status != resp->IDLE) {
                RCLCPP_WARN(this->get_logger(), "turtle not idle. Skipped random patrol request");
                return;
            }

            while (!this->client_->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "rclcpp context error while waiting for control service. Abort");
                    return;
                }
                RCLCPP_WARN(this->get_logger(), "waiting for turtle control service to be ready...");
            }
            turtle_pd_if::srv::Patrol::Request req;
            req.target_x = x;
            req.target_y = y;
            this->client_->async_send_request(std::make_shared<decltype(req)>(req),
                [this](rclcpp::Client<turtle_pd_if::srv::Patrol>::SharedFuture req_future) {
                auto resp = req_future.get();
                RCLCPP_INFO(this->get_logger(), "patrol request finish. code=%d, msg=%s",
                    resp->code, resp->msg.c_str());
            });
            // auto code = rclcpp::spin_until_future_complete(
            //     std::make_shared<RandomPatrolClient>(*this), req_future, 3min);
            // switch (code) {
            // case rclcpp::FutureReturnCode::SUCCESS:
            //     RCLCPP_INFO(this->get_logger(), "patrol request sent successfully");
            //     break;
            // case rclcpp::FutureReturnCode::TIMEOUT:
            //     RCLCPP_WARN(this->get_logger(), "patrol request wait timeout");
            //     break;
            // case rclcpp::FutureReturnCode::INTERRUPTED:
            //     RCLCPP_ERROR(this->get_logger(), "patrol request interrupted unexpectedly");
            //     break;
            // }
        });
    }

    std::shared_ptr<rclcpp::Client<turtle_pd_if::srv::Patrol>> client_;
    std::shared_ptr<rclcpp::Client<turtle_pd_if::srv::PatrolStatus>> patrol_status_;

    std::shared_ptr<rclcpp::TimerBase> timer_;

    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<float> dis;
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    std::string _node_name = "random_patrol_client_node";
    auto client_node = std::make_shared<RandomPatrolClient>(_node_name);
    RCLCPP_INFO(client_node->get_logger(), "%s initialized", _node_name.c_str());

    rclcpp::spin(client_node);
    rclcpp::shutdown();

    return 0;
}

