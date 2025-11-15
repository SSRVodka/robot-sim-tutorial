
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

#include <turtle_pd_if/srv/patrol.hpp>
#include <turtle_pd_if/srv/patrol_status.hpp>

class TurtleController: public rclcpp::Node {
public:
    explicit TurtleController(
        std::string name, float target_x, float target_y,
        float pos_tol = 0.2, float ang_tol = 0.1, float accel = 0.8,
        float max_speed = 0.5, float max_angular = 0.1)
    : rclcpp::Node(name), target_x(target_x), target_y(target_y),
        pos_tol(pos_tol), ang_tol(ang_tol), accel(accel), max_speed(max_speed), max_angular(max_angular) {

        this->status.status = this->status.MOVING;  // wait for next target reach point
        
        // Lab1 TODO
        // 提示：发布驱动海龟运动数据的话题为 /turtle1/cmd_vel，订阅海龟状态的话题是 /turtle1/pose
        // this->publisher_ = nullptr;
        // this->subscriber_ = nullptr;

        this->control_srv_ = this->create_service<turtle_pd_if::srv::Patrol>(
            "/turtle1/patrol",
            std::bind(&TurtleController::on_control, this, std::placeholders::_1, std::placeholders::_2));
        this->patrol_status_srv_ = this->create_service<turtle_pd_if::srv::PatrolStatus>(
            "/turtle1/patrol/status", std::bind(&TurtleController::get_patrol_status, this, std::placeholders::_1, std::placeholders::_2));
    }
    ~TurtleController() {}

    void get_patrol_status(
        const std::shared_ptr<turtle_pd_if::srv::PatrolStatus::Request>,
        const std::shared_ptr<turtle_pd_if::srv::PatrolStatus::Response> resp) {
        RCLCPP_INFO(this->get_logger(), "received get patrol status request");
        resp->status = this->status.status;
    }

    void on_control(
        const std::shared_ptr<turtle_pd_if::srv::Patrol::Request> request,
        const std::shared_ptr<turtle_pd_if::srv::Patrol::Response> response) {

        this->status.status = this->status.MOVING;

        RCLCPP_INFO(this->get_logger(), "received control request: target (%.2f, %.2f)",
            request->target_x, request->target_y);
        
        // boundary: [0, 12]
        if ((request->target_x < 0 || request->target_x > 12)
            && (request->target_y < 0 || request->target_y > 12)) {
            response->code = response->INVALID_ARGS;
            response->msg = "Invalid arguments";
            return;
        }

        this->target_x = request->target_x;
        this->target_y = request->target_y;
        response->code = response->SUCCESS;
        response->msg = "OK";
        RCLCPP_INFO(this->get_logger(), "turtle target position updated to (%.2f, %.2f)",
            request->target_x, request->target_y);
    }

    void on_pose_update(const std::shared_ptr<turtlesim::msg::Pose> pose) {
        float x = pose->x;
        float y = pose->y;
        float theta = pose->theta;
        RCLCPP_INFO(this->get_logger(), "received pose update: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);

        float delta_x = target_x - x;
        float delta_y = target_y - y;
        float distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        float diff_angle = std::atan2(delta_y, delta_x) - theta;

        geometry_msgs::msg::Twist cont;
        if (distance > this->pos_tol) {
            if (std::abs(diff_angle) > this->ang_tol) {
                cont.angular.z = std::abs(diff_angle);
            } else {
                cont.linear.x = this->accel * distance;
            }
        } else {
            this->status.status = this->status.IDLE;
        }

        if (cont.linear.x > this->max_speed) {
            cont.linear.x = this->max_speed;
        }
        this->publisher_->publish(cont);
    }

private:
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> publisher_;
    std::shared_ptr<rclcpp::Subscription<turtlesim::msg::Pose>> subscriber_;

    std::shared_ptr<rclcpp::Service<turtle_pd_if::srv::Patrol>> control_srv_;
    std::shared_ptr<rclcpp::Service<turtle_pd_if::srv::PatrolStatus>> patrol_status_srv_;

    float target_x;
    float target_y;
    float pos_tol;
    float ang_tol;
    float accel;
    float max_speed;
    float max_angular;

    turtle_pd_if::srv::PatrolStatus::Response status;
};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);

    auto controller_node = std::make_shared<TurtleController>("turtle_cont_node", 5, 5);

    RCLCPP_INFO(controller_node->get_logger(), "controller node initialized");

    rclcpp::spin(controller_node);
    rclcpp::shutdown();
    
    return 0;
}
