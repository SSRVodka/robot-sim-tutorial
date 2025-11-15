#pragma once

#include <termios.h>

#include "oh_servo/servo_controller.hpp"


// Keyboard-specific input handler
class KeyboardInputHandler : public InputHandler
{
public:
  KeyboardInputHandler();
  ~KeyboardInputHandler();
  
  void start() override;
  void stop() override;
  bool getNextCommand(ServoCommand& command) override;

private:
  void setupTerminal();
  void restoreTerminal();
  char readKey();
  
  int keyboard_fd_;
  struct termios original_terminal_settings_;
  bool terminal_configured_;
  
  // Key code definitions
  static const char KEYCODE_RIGHT = 0x43;
  static const char KEYCODE_LEFT = 0x44;
  static const char KEYCODE_UP = 0x41;
  static const char KEYCODE_DOWN = 0x42;
  static const char KEYCODE_PERIOD = 0x2E;
  static const char KEYCODE_SEMICOLON = 0x3B;
  static const char KEYCODE_1 = 0x31;
  static const char KEYCODE_2 = 0x32;
  static const char KEYCODE_3 = 0x33;
  static const char KEYCODE_4 = 0x34;
  static const char KEYCODE_5 = 0x35;
  static const char KEYCODE_6 = 0x36;
  static const char KEYCODE_7 = 0x37;
  static const char KEYCODE_Q = 0x71;
  static const char KEYCODE_W = 0x77;
  static const char KEYCODE_E = 0x65;
  static const char KEYCODE_R = 0x72;
};
