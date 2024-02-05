/**
 * Notes: perks of using gtest
    - Provides a single "overridable" main function, for multiple test source files, in gtest_main library. 
    So evetually, you can have one binary for them
 */
#include "simple_robotics_cpp_utils/math_utils.hpp"
#include <gtest/gtest.h>
#include <iostream>

TEST(MathUtilsTest, TestDrawFromPdfNormal){
    auto result = SimpleRoboticsCppUtils::draw_from_pdf_normal(1.0, 1.0);
    std::cout << result << std::endl;
}

