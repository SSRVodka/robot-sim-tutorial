
#include "oh_servo/keyboard_servo.hpp"


// Constants definition
const std::string ServoController::TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string ServoController::JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string ServoController::EEF_FRAME_ID = "panda_hand";
const std::string ServoController::BASE_FRAME_ID = "panda_link0";
const size_t ServoController::ROS_QUEUE_SIZE = 10;


// ServoController implementation
ServoController::ServoController() 
  : current_frame_(BASE_FRAME_ID)
  , joint_velocity_command_(1.0)
  , running_(false)
{
  node_ = rclcpp::Node::make_shared("servo_controller");
  
  twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    TWIST_TOPIC, ROS_QUEUE_SIZE);
  joint_pub_ = node_->create_publisher<control_msgs::msg::JointJog>(
    JOINT_TOPIC, ROS_QUEUE_SIZE);
}

ServoController::~ServoController()
{
  stop();
}

void ServoController::setInputHandler(std::unique_ptr<InputHandler> handler)
{
  input_handler_ = std::move(handler);
}

int ServoController::run()
{
  if (!input_handler_)
  {
    RCLCPP_ERROR(node_->get_logger(), "No input handler set!");
    return -1;
  }
  
  running_ = true;
  input_handler_->start();
  
  // Start ROS spinning in a separate thread
  std::thread ros_spinner([this]() {
    while (running_ && rclcpp::ok())
    {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
  
  // Main command processing loop
  ServoCommand command(CommandType::QUIT);
  while (running_ && rclcpp::ok())
  {
    if (input_handler_->getNextCommand(command))
    {
      if (command.type == CommandType::QUIT)
      {
        break;
      }
      processCommand(command);
    }
  }
  
  running_ = false;
  if (ros_spinner.joinable())
  {
    ros_spinner.join();
  }
  
  input_handler_->stop();
  return 0;
}

void ServoController::stop()
{
  running_ = false;
}

void ServoController::processCommand(const ServoCommand& command)
{
  switch (command.type)
  {
    case CommandType::CARTESIAN_TWIST:
      publishTwistCommand(command);
      break;
      
    case CommandType::JOINT_JOG:
      publishJointCommand(command);
      break;
      
    case CommandType::FRAME_SWITCH:
      switchFrame(command.use_end_effector_frame);
      break;
      
    case CommandType::JOINT_DIRECTION_TOGGLE:
      toggleJointDirection();
      break;
      
    case CommandType::QUIT:
      // Handled in main loop
      break;
  }
}

void ServoController::publishTwistCommand(const ServoCommand& command)
{
  auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  
  twist_msg->header.stamp = node_->now();
  twist_msg->header.frame_id = current_frame_;
  
  twist_msg->twist.linear.x = command.linear_x;
  twist_msg->twist.linear.y = command.linear_y;
  twist_msg->twist.linear.z = command.linear_z;
  twist_msg->twist.angular.x = command.angular_x;
  twist_msg->twist.angular.y = command.angular_y;
  twist_msg->twist.angular.z = command.angular_z;
  
  twist_pub_->publish(std::move(twist_msg));
  
  RCLCPP_DEBUG(node_->get_logger(), "Published twist command in frame: %s", current_frame_.c_str());
}

void ServoController::publishJointCommand(const ServoCommand& command)
{
  if (command.joint_index < 0 || command.joint_index > 6)
  {
    RCLCPP_WARN(node_->get_logger(), "Invalid joint index: %d", command.joint_index);
    return;
  }
  
  auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();
  
  joint_msg->header.stamp = node_->now();
  joint_msg->header.frame_id = BASE_FRAME_ID;
  
  std::string joint_name = "panda_joint" + std::to_string(command.joint_index + 1);
  joint_msg->joint_names.push_back(joint_name);
  joint_msg->velocities.push_back(joint_velocity_command_);
  
  joint_pub_->publish(std::move(joint_msg));
  
  RCLCPP_DEBUG(node_->get_logger(), "Published joint command for %s with velocity %f", 
               joint_name.c_str(), joint_velocity_command_);
}

void ServoController::switchFrame(bool use_end_effector)
{
  current_frame_ = use_end_effector ? EEF_FRAME_ID : BASE_FRAME_ID;
  RCLCPP_INFO(node_->get_logger(), "Switched to frame: %s", current_frame_.c_str());
}

void ServoController::toggleJointDirection()
{
  joint_velocity_command_ *= -1.0;
  RCLCPP_INFO(node_->get_logger(), "Joint velocity direction toggled to: %f", joint_velocity_command_);
}