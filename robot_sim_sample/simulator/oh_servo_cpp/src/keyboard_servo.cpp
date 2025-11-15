
#include "oh_servo/keyboard_servo.hpp"


// KeyboardInputHandler implementation
KeyboardInputHandler::KeyboardInputHandler() 
  : keyboard_fd_(0)
  , terminal_configured_(false)
{
}

KeyboardInputHandler::~KeyboardInputHandler()
{
  stop();
}

void KeyboardInputHandler::start()
{
  setupTerminal();
  
  printf("Reading from keyboard\n");
  printf("---------------------------\n");
  printf("Use arrow keys and the '.' and ';' keys to Cartesian jog\n");
  printf("Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame\n");
  printf("Use 1|2|3|4|5|6|7 keys to joint jog. 'R' to reverse the direction of jogging.\n");
  printf("'Q' to quit.\n");
}

void KeyboardInputHandler::stop()
{
  restoreTerminal();
}

bool KeyboardInputHandler::getNextCommand(ServoCommand& command)
{
  try
  {
    char key = readKey();
    
    switch (key)
    {
      case KEYCODE_LEFT:
        command = ServoCommand(CommandType::CARTESIAN_TWIST);
        command.linear_y = -1.0;
        return true;
        
      case KEYCODE_RIGHT:
        command = ServoCommand(CommandType::CARTESIAN_TWIST);
        command.linear_y = 1.0;
        return true;
        
      case KEYCODE_UP:
        command = ServoCommand(CommandType::CARTESIAN_TWIST);
        command.linear_x = 1.0;
        return true;
        
      case KEYCODE_DOWN:
        command = ServoCommand(CommandType::CARTESIAN_TWIST);
        command.linear_x = -1.0;
        return true;
        
      case KEYCODE_PERIOD:
        command = ServoCommand(CommandType::CARTESIAN_TWIST);
        command.linear_z = -1.0;
        return true;
        
      case KEYCODE_SEMICOLON:
        command = ServoCommand(CommandType::CARTESIAN_TWIST);
        command.linear_z = 1.0;
        return true;
        
      case KEYCODE_E:
        command = ServoCommand(CommandType::FRAME_SWITCH);
        command.use_end_effector_frame = true;
        return true;
        
      case KEYCODE_W:
        command = ServoCommand(CommandType::FRAME_SWITCH);
        command.use_end_effector_frame = false;
        return true;
        
      case KEYCODE_1:
      case KEYCODE_2:
      case KEYCODE_3:
      case KEYCODE_4:
      case KEYCODE_5:
      case KEYCODE_6:
      case KEYCODE_7:
        command = ServoCommand(CommandType::JOINT_JOG);
        command.joint_index = key - KEYCODE_1;  // Convert to 0-6 range
        return true;
        
      case KEYCODE_R:
        command = ServoCommand(CommandType::JOINT_DIRECTION_TOGGLE);
        return true;
        
      case KEYCODE_Q:
        command = ServoCommand(CommandType::QUIT);
        return true;
        
      default:
        // Unknown key, don't return a command
        return false;
    }
  }
  catch (const std::runtime_error& e)
  {
    // Error reading key
    return false;
  }
}

void KeyboardInputHandler::setupTerminal()
{
  tcgetattr(keyboard_fd_, &original_terminal_settings_);
  
  struct termios raw_settings;
  memcpy(&raw_settings, &original_terminal_settings_, sizeof(struct termios));
  raw_settings.c_lflag &= ~(ICANON | ECHO);
  raw_settings.c_cc[VEOL] = 1;
  raw_settings.c_cc[VEOF] = 2;
  
  tcsetattr(keyboard_fd_, TCSANOW, &raw_settings);
  terminal_configured_ = true;
}

void KeyboardInputHandler::restoreTerminal()
{
  if (terminal_configured_)
  {
    tcsetattr(keyboard_fd_, TCSANOW, &original_terminal_settings_);
    terminal_configured_ = false;
  }
}

char KeyboardInputHandler::readKey()
{
  char key;
  int result = read(keyboard_fd_, &key, 1);
  if (result < 0)
  {
    throw std::runtime_error("Failed to read key");
  }
  return key;
}
