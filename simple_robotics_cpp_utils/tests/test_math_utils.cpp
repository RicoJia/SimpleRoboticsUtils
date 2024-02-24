/**
 * Notes: perks of using gtest
    - Provides a single "overridable" main function, for multiple test source
 files, in gtest_main library. So evetually, you can have one binary for them
 */
#include "simple_robotics_cpp_utils/math_utils.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include <numeric>
#include <vector>

constexpr size_t SAMPLE_SIZE = 300;

void test_univariate(const std::vector<double> &results, const double &mean,
                     const double &std_dev) {
  // getting mean
  double mean_result =
      std::accumulate(results.begin(), results.end(), 0.0) / SAMPLE_SIZE;
  std::vector<double> normalized_results(SAMPLE_SIZE, 0.0);
  std::transform(results.begin(), results.end(), normalized_results.begin(),
                 [mean](double result) { return result - mean; });
  double std_dev_result = std::sqrt(
      std::inner_product(normalized_results.begin(), normalized_results.end(),
                         normalized_results.begin(), 0.0) /
      SAMPLE_SIZE);
  std::cout << "mean = " << mean_result << ", std_dev = " << std_dev_result
            << std::endl;
  // 99.73% probability. Is it "flaky"?
  EXPECT_NEAR(mean_result, mean, 3 * std_dev);
}

TEST(MathUtilsTest, TestDrawFromPdfNormalUnivariate) {
  double mean = 0.0, std_dev = 1.0;
  std::vector<double> results(
      SAMPLE_SIZE, SimpleRoboticsCppUtils::draw_from_pdf_normal(mean, std_dev));
  test_univariate(results, mean, std_dev);
}

TEST(MathUtilsTest, TestDrawFromPdfNormalMultivariate) {
  // draw
  Eigen::Vector3d means = {1.0, 2.0, 3.0};
  Eigen::Matrix3d cov;
  cov << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  std::array<std::vector<double>, 3> results;
  for (size_t i = 0; i < SAMPLE_SIZE; ++i) {
    Eigen::VectorXd return_vec =
        SimpleRoboticsCppUtils::draw_from_pdf_normal(means, cov);
    results[0].push_back(return_vec[0]);
    results[1].push_back(return_vec[1]);
    results[2].push_back(return_vec[2]);
  }

  for (size_t i = 0; i < 3; ++i) {
    test_univariate(results[i], means[i], std::sqrt(cov(i, i)));
  }
}

TEST(MathUtilsTest, test_normal_dist_prob) {
  const double mean = 0, std_dev = 1, val = 0;
  double prob =
      SimpleRoboticsCppUtils::normal_dist_prob(mean, std_dev, val, false);
  EXPECT_DOUBLE_EQ(prob,
                   1 / std_dev * SimpleRoboticsCppUtils::ROOT_2PI_INVERSE);
}

TEST(MathUtilsTest, Transform4dTo3d) {
  // Define a 4D transformation matrix (for simplicity, use a rotation around
  // Z-axis and a translation)
  Eigen::Matrix4d T_4d = Eigen::Matrix4d::Identity();
  double angle = M_PI / 4; // 45 degrees
  T_4d(0, 0) = cos(angle);
  T_4d(0, 1) = -sin(angle);
  T_4d(0, 3) = 1.0; // Translation in x
  T_4d(1, 0) = sin(angle);
  T_4d(1, 1) = cos(angle);
  T_4d(1, 3) = 2.0; // Translation in y

  // Expected 3D transformation matrix
  Eigen::Matrix3d expected_T_3d = Eigen::Matrix3d::Identity();
  expected_T_3d(0, 0) = cos(angle);
  expected_T_3d(0, 1) = -sin(angle);
  expected_T_3d(0, 2) = 1.0;
  expected_T_3d(1, 0) = sin(angle);
  expected_T_3d(1, 1) = cos(angle);
  expected_T_3d(1, 2) = 2.0;

  // Perform the transformation
  Eigen::Matrix3d actual_T_3d =
      SimpleRoboticsCppUtils::transform_4d_to_3d(T_4d);

  // Assertions to check if the transformation was correct
  assert(actual_T_3d.isApprox(expected_T_3d) &&
         "The transformation result does not match the expected output.");

  std::cout << "Test passed: 4D to 3D transformation is correct.\n";
}

TEST(MathUtilsTest, RotationAroundZ) {
  Eigen::Matrix4d transform =
      SimpleRoboticsCppUtils::get_transform_from_2d_rotation(M_PI_2);

  // Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  // // Test 90 degrees (π/2 radians)
  // transform.block<3,3>(0,0) = Eigen::AngleAxisd(M_PI_2,
  // Eigen::Vector3d::UnitZ()).toRotationMatrix();
  assert(
      std::abs(SimpleRoboticsCppUtils::get_2d_rotation_from_z_axis(transform) -
               M_PI_2) < 1e-6);
}