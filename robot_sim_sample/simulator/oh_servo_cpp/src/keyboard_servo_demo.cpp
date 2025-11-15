
#include "oh_servo/keyboard_servo.hpp"

// Global controller instance for signal handler
std::unique_ptr<ServoController> g_controller;

void signalHandler(int sig)
{
  (void)sig;
  if (g_controller)
  {
    g_controller->stop();
  }
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // Create controller and set up keyboard input
  g_controller = std::make_unique<ServoController>();
  g_controller->setInputHandler(std::make_unique<KeyboardInputHandler>());
  
  // Set up signal handler
  signal(SIGINT, signalHandler);
  
  // Run the controller
  int result = g_controller->run();
  
  // Cleanup
  g_controller.reset();
  rclcpp::shutdown();
  
  return result;
}