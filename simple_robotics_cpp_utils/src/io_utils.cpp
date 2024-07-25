#include "simple_robotics_cpp_utils/io_utils.hpp"

namespace SimpleRoboticsCppUtils {

std::unordered_map<ANSIStrings, std::string> ANSI_codes = {
    {ANSIStrings::RESET, "\033[0m"},     {ANSIStrings::BOLD, "\033[1m"},
    {ANSIStrings::UNDERLINE, "\033[4m"}, {ANSIStrings::RED, "\033[31m"},
    {ANSIStrings::GREEN, "\033[32m"},    {ANSIStrings::YELLOW, "\033[33m"},
    {ANSIStrings::BLUE, "\033[34m"},     {ANSIStrings::MAGENTA, "\033[35m"},
    {ANSIStrings::CYAN, "\033[36m"},     {ANSIStrings::WHITE, "\033[37m"}};

char getch(int timeout_ms) {
  fd_set set;
  struct timeval timeout;
  int rv;
  char buff = 0;
  int len = 1;
  int filedesc = STDIN_FILENO;

  FD_ZERO(&set);
  FD_SET(filedesc, &set);

  if (timeout_ms > 0) {
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    if (rv == -1) {
      std::cerr << "Error: select" << std::endl;
    } else if (rv == 0) {
      // Timeout
    } else {
      if (read(filedesc, &buff, len) < 0) {
        std::cerr << "Error: read" << std::endl;
      }
    }
  } else {
    TerminalSettings();
    char ch = getchar(); // read a character
  }

  return buff;
}

std::string to_color_msg(const ANSIStrings &a, const std::string &str,
                         const bool &bold = false) {
  std::string bold_prefix = bold ? ANSI_codes.at(ANSIStrings::BOLD) : "";
  return bold_prefix + ANSI_codes.at(a) + str +
         ANSI_codes.at(ANSIStrings::RESET);
}

void sleep_or_pause_on_keystroke(const std::string &str = "",
                                 const int &timeout_ms = 1000) {
  std::cout << str << std::endl;
  int k = getch(timeout_ms);
  if (k > 0) {
    std::string msg = "Paused. Press any key to unpause";
    msg = to_color_msg(ANSIStrings::BLUE, msg, true);
    std::cout << msg << std::endl;
    getch(0);
  }
}
}; // namespace SimpleRoboticsCppUtils