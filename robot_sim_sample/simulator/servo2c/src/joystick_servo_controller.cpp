#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>

const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string EEF_FRAME_ID = "panda_hand";
const std::string BASE_FRAME_ID = "panda_link0";

static const std::string NODE_NAME = "servo2c_joy2servo_publisher";

// Enums for button names -> axis/button array index
// For XBOX 1 controller
enum Axis {
    LEFT_STICK_X = 0,
    LEFT_STICK_Y = 1,
    LEFT_TRIGGER = 2,
    RIGHT_STICK_X = 3,
    RIGHT_STICK_Y = 4,
    RIGHT_TRIGGER = 5,
    D_PAD_X = 6,
    D_PAD_Y = 7
};
enum Button {
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    LEFT_BUMPER = 4,
    RIGHT_BUMPER = 5,
    CHANGE_VIEW = 6,
    MENU = 7,
    HOME = 8,
    LEFT_STICK_CLICK = 9,
    RIGHT_STICK_CLICK = 10
};

// Some axes have offsets (e.g. the default trigger position is 1.0 not 0)
// This will map the default values for the axes
std::map<Axis, double> AXIS_DEFAULTS = { { LEFT_TRIGGER, 1.0 }, { RIGHT_TRIGGER, 1.0 } };
std::map<Button, double> BUTTON_DEFAULTS;

// To change controls or setup a new controller, all you should to do is change the above enums and the follow 2
// functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A TwistStamped message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                        std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                        std::unique_ptr<control_msgs::msg::JointJog>& joint) {
    // Give joint jogging priority because it is only buttons
    // If any joint jog command is requested, we are only publishing joint commands
    if (buttons[A] || buttons[B] || buttons[X] || buttons[Y] || axes[D_PAD_X] || axes[D_PAD_Y]) {
        // Map the D_PAD to the proximal joints
        joint->joint_names.push_back("panda_joint1");
        joint->velocities.push_back(axes[D_PAD_X]);
        joint->joint_names.push_back("panda_joint2");
        joint->velocities.push_back(axes[D_PAD_Y]);

        // Map the diamond to the distal joints
        joint->joint_names.push_back("panda_joint7");
        joint->velocities.push_back(buttons[B] - buttons[X]);
        joint->joint_names.push_back("panda_joint6");
        joint->velocities.push_back(buttons[Y] - buttons[A]);
        return false;
    }

    // The bread and butter: map buttons to twist commands
    twist->twist.linear.z = axes[RIGHT_STICK_Y];
    twist->twist.linear.y = axes[RIGHT_STICK_X];

    double lin_x_right = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
    double lin_x_left = 0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
    twist->twist.linear.x = lin_x_right + lin_x_left;

    twist->twist.angular.y = axes[LEFT_STICK_Y];
    twist->twist.angular.x = axes[LEFT_STICK_X];

    double roll_positive = buttons[RIGHT_BUMPER];
    double roll_negative = -1 * (buttons[LEFT_BUMPER]);
    twist->twist.angular.z = roll_positive + roll_negative;

    return true;
}

/** \brief // This should update the frame_to_publish_ as needed for changing command frame via controller
 * @param frame_name Set the command frame to this
 * @param buttons The vector of discrete controller button values
 */
void updateCmdFrame(std::string& frame_name, const std::vector<int>& buttons) {
    if (buttons[CHANGE_VIEW] && frame_name == EEF_FRAME_ID)
        frame_name = BASE_FRAME_ID;
    else if (buttons[MENU] && frame_name == BASE_FRAME_ID)
        frame_name = EEF_FRAME_ID;
}

namespace servo2c_plugins {

/**
 * @brief This class has two uses:
 * 
 * 1. Receives control data (sensor_msgs::msg::Joy) from the JoyStick, converts it into control data
 *  (geometry_msgs::msg::TwistStamped and control_msgs::msg::JointJog) that the Servo Node can recognize,
 *  and publishes it so that the Servo Node can control the machine.
 * 
 * 2. Publish PlanningScene information to the PlanningSceneMonitor listening on the Servo Node,
 *  which coordinates MoveIt2 state space information to assist the planner in decision-making.
 * 
 * 它的作用有 2 点：
 * 
 * 1. 接收 JoyStick 传来的控制数据（sensor_msgs::msg::Joy）转换为 Servo Node 能识别的控制数据
 *  geometry_msgs::msg::TwistStamped 和 control_msgs::msg::JointJog 并发布出去，让 Servo Node 控制机器。
 * 
 * 2. 发布 PlanningScene 信息，提供给 Servo Node 上监听的 PlanningSceneMonitor，后者会统筹 Moveit2 状态空间信息，帮助规划器决策。
 * 
 * @see ${ws_moveit2}/src/moveit2/moveit_ros/moveit_servo/src/servo_node.cpp
 * @related [planning scene monitor](https://moveit.picknik.ai/main/doc/examples/planning_scene_monitor/planning_scene_monitor_tutorial.html#)
 */
class Joy2ServoController : public rclcpp::Node {
public:
    Joy2ServoController(const rclcpp::NodeOptions& options)
    : Node(NODE_NAME, options), frame_to_publish_(BASE_FRAME_ID) {
        // Setup pub/sub
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
            [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { return joyCB(msg); });

        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
        joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
        // collision_pub_ =
        //     this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", rclcpp::SystemDefaultsQoS());

        // Create a service client to start the ServoNode
        servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
        servo_start_client_->wait_for_service(std::chrono::seconds(1));
        servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

        // Load the collision scene asynchronously
        collision_pub_thread_ = std::thread([this]() {
            rclcpp::sleep_for(std::chrono::seconds(3));
            // Create collision object, in the way of servoing
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = "world";
            collision_object.id = "box";

            shape_msgs::msg::SolidPrimitive table_1;
            table_1.type = table_1.BOX;
            // table_1.dimensions = { 0.4, 0.6, 0.03 };
            table_1.dimensions = { 0.03, 0.03, 0.08 };

            geometry_msgs::msg::Pose table_1_pose;
            // table_1_pose.position.x = 0.6;
            // table_1_pose.position.y = 0.0;
            // table_1_pose.position.z = 0.4;
            table_1_pose.position.x = 0.6;
            table_1_pose.position.y = -0.2;
            table_1_pose.position.z = 0.0;

            collision_object.primitives.push_back(table_1);
            collision_object.primitive_poses.push_back(table_1_pose);
            collision_object.operation = collision_object.ADD;

            #include "servo2c/demo"

            moveit_msgs::msg::PlanningSceneWorld psw;
            psw.collision_objects.push_back(collision_object);
            psw.collision_objects.push_back(base_obj);
            psw.collision_objects.push_back(front_obj);
            psw.collision_objects.push_back(back_obj);
            psw.collision_objects.push_back(left_obj);
            psw.collision_objects.push_back(right_obj);

            // auto ps = std::make_unique<moveit_msgs::msg::PlanningScene>();
            // ps->world = psw;
            // ps->is_diff = true;
            // collision_pub_->publish(std::move(ps));
        });
    }

    ~Joy2ServoController() override {
        if (collision_pub_thread_.joinable())
            collision_pub_thread_.join();
    }

    void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg) {
        // Create the messages we might publish
        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

        // This call updates the frame for twist commands
        updateCmdFrame(frame_to_publish_, msg->buttons);

        // Convert the joystick message to Twist or JointJog and publish
        if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg)) {
            // publish the TwistStamped
            twist_msg->header.frame_id = frame_to_publish_;
            twist_msg->header.stamp = this->now();
            twist_pub_->publish(std::move(twist_msg));
        } else {
            // publish the JointJog
            joint_msg->header.stamp = this->now();
            joint_msg->header.frame_id = "panda_link3";
            joint_pub_->publish(std::move(joint_msg));
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    // rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

    std::string frame_to_publish_;

    std::thread collision_pub_thread_;
};  // class Joy2ServoController

}  // namespace servo2c_plugins

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(servo2c_plugins::Joy2ServoController)
