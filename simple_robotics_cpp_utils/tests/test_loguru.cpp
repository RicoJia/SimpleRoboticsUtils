#include "simple_robotics_cpp_utils/io_utils.hpp"
#include "simple_robotics_cpp_utils/loguru.hpp"

#include <gtest/gtest.h>
#include <iomanip>

TEST(LoguruTest, TestBasicLoggingFunctionalities) {
  int argc = 1; // argument count, the program's name itself
  char *argv[] = {const_cast<char*>("this_test_program"), NULL}; // argument vector
  loguru::init(argc, argv);
  LOG_SCOPE_FUNCTION(INFO);

  // Only show most relevant things on stderr:
  // loguru::g_stderr_verbosity = 1;
  LOG_F(INFO, "Heyo, from main.cpp!");
  LOG_F(ERROR, "Heyo, this is a warning");
  auto colored_str = to_color_msg(SimpleRoboticsCppUtils::ANSIStrings::GREEN,
                                  "colorrrrr!!", true);
  LOG_F(INFO, colored_str.c_str());
  LOG_S(INFO) << "Some float: " << std::setfill('0') << std::setw(5)
              << std::setprecision(3) << 1.12345;
}