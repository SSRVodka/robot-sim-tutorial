#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>

// Command types for the servo controller
enum class CommandType
{
  CARTESIAN_TWIST,
  JOINT_JOG,
  FRAME_SWITCH,
  JOINT_DIRECTION_TOGGLE,
  QUIT
};

// Structure to hold command data
struct ServoCommand
{
  CommandType type;
  
  // For Cartesian commands
  double linear_x = 0.0;
  double linear_y = 0.0;
  double linear_z = 0.0;
  double angular_x = 0.0;
  double angular_y = 0.0;
  double angular_z = 0.0;
  
  // For joint commands
  int joint_index = -1;  // 0-6 for joints 1-7
  
  // For frame switching
  bool use_end_effector_frame = false;
  
  ServoCommand(CommandType cmd_type) : type(cmd_type) {}
};

// Abstract base class for input handlers
class InputHandler
{
public:
  virtual ~InputHandler() = default;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual bool getNextCommand(ServoCommand& command) = 0;
};

// Main servo controller class
class ServoController
{
public:
  ServoController();
  ~ServoController();
  
  void setInputHandler(std::unique_ptr<InputHandler> handler);
  int run();
  void stop();

private:
  void processCommand(const ServoCommand& command);
  void publishTwistCommand(const ServoCommand& command);
  void publishJointCommand(const ServoCommand& command);
  void switchFrame(bool use_end_effector);
  void toggleJointDirection();
  
  // ROS2 node and publishers
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  
  // Configuration
  std::string current_frame_;
  double joint_velocity_command_;
  bool running_;
  
  // Input handler
  std::unique_ptr<InputHandler> input_handler_;
  
  // Constants
  static const std::string TWIST_TOPIC;
  static const std::string JOINT_TOPIC;
  static const std::string EEF_FRAME_ID;
  static const std::string BASE_FRAME_ID;
  static const size_t ROS_QUEUE_SIZE;
};
