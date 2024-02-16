/**
 * Notes: perks of using gtest
    - Provides a single "overridable" main function, for multiple test source
 files, in gtest_main library. So evetually, you can have one binary for them
 */
#include "simple_robotics_cpp_utils/math_utils.hpp"
#include <algorithm>
#include <array>
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