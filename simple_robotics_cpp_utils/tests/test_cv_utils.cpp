#include "simple_robotics_cpp_utils/cv_utils.hpp"

#include <gtest/gtest.h>

using namespace SimpleRoboticsCppUtils;

TEST(CvUtilsTest, TestFrobeniusNorm) {
  cv::Mat_<double> m(3, 3);
  m << 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0, 2.0, 0.0;
  double norm = getFrobeniusNorm(m);
  EXPECT_EQ(norm, std::sqrt(1 + 4 + 9 + 1 + 4));
}

TEST(CvUtilsTest, TestPixel2cam) {
  cv::Mat K = (cv::Mat_<double>(3, 3) << 800, 0, 320, 0, 800, 240, 0, 0, 1);
  cv::Point2f p(400, 300);
  cv::Point2f expected(0.1, 0.075);
  cv::Point2f result = pixel2cam(p, K);

  EXPECT_NEAR(result.x, expected.x, 1e-4);
  EXPECT_NEAR(result.y, expected.y, 1e-4);
}

TEST(CvUtilsTest, TestRT2EigenIsometry3d) {
  // rotate +90 deg around z axis
  cv::Mat R = (cv::Mat_<double>(3, 3) << 0, -1, 0, 1, 0, 0, 0, 0, 1);
  cv::Mat t = (cv::Mat_<double>(3, 1) << 1, 0, 0);
  auto trans = cv_R_t_to_eigen_isometry3d(R, t);
  auto vec = Eigen::Vector3d(1, 0, 0);
  auto expected = Eigen::Vector3d(1, 1, 0);
  auto rotated_vec = trans * vec;
  // TODO
  std::cout << "trans: " << trans.matrix() << std::endl;
  EXPECT_LT((rotated_vec - expected).norm(), 1e-5);
}

TEST(CvUtilsTest, TestInvertMat) {
  cv::Mat R = (cv::Mat_<double>(3, 3) << 0, -1, 0, 1, 0, 0, 0, 0, 1);
  cv::Mat R_inv_expected =
      (cv::Mat_<double>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 1);
  cv::Mat R_inv;
  bool succeeded = invert_mat(R, R_inv);
  EXPECT_EQ(succeeded, true);
  EXPECT_LT(cv::norm(R_inv_expected - R_inv), 1e-5);
}

TEST(CvUtilsTest, TestSkewMatrixConversion) {
  // Define the input vector t
  cv::Mat t = (cv::Mat_<double>(3, 1) << 1, 2, 3);

  // Expected skew-symmetric matrix
  cv::Mat expected = (cv::Mat_<double>(3, 3) << 0, -3, 2, 3, 0, -1, -2, 1, 0);
  // Call the function
  cv::Mat result = skew_symmetric_mat(t);
  EXPECT_LT(cv::norm(result - expected), 1e-5);
}

TEST(CvUtilsTest, TestImagePyramid) {
  auto image = cv::imread("../tests/data/orb_test_1.png", cv::IMREAD_COLOR);
  auto image_pyramid = build_image_pyramid(image, 3, 1.2);
  //   for (const auto& i: image_pyramid) display_image(i);
}