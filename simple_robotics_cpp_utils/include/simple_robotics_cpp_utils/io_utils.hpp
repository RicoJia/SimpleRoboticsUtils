#pragma once
#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <unordered_map>

namespace SimpleRoboticsCppUtils {
enum class ANSIStrings {
  RESET,
  BOLD,
  UNDERLINE,
  RED,
  GREEN,
  YELLOW,
  BLUE,
  MAGENTA,
  CYAN,
  WHITE
};

// RAII class that stores & restores older terminal settings
class TerminalSettings {
public:
  TerminalSettings() {
    tcgetattr(STDIN_FILENO, &oldt); // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);        // disable buffering and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings
  }
  ~TerminalSettings() {
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
  }

private:
  struct termios oldt, newt;
};

std::string to_color_msg(const ANSIStrings &a, const std::string &str,
                         const bool &bold);
char getch(int timeout_ms);
void sleep_or_pause_on_keystroke(const std::string &str, const int &timeout_ms);
}; // namespace SimpleRoboticsCppUtils
