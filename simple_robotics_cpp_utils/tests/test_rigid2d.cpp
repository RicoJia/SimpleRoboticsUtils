#include "simple_robotics_cpp_utils/rigid2d.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <simple_robotics_cpp_utils/math_utils.hpp>
#include <unordered_map>
#include <utility>
using namespace SimpleRoboticsCppUtils;

TEST(Rigid2DTest, TestAngle) {
  {
    Pose2D p1(1, 1, 0);
    Pose2D p2(0, 0, 0);
    EXPECT_NEAR(angle_of_two_points(p1, p2), -3 * M_PI / 4, 1e-5);
  }
}

TEST(Rigid2DTest, TestDrawingFromIcc) {
  // Testing with zero matrix will make lower cholesky decomposition fail
  const double std_dev = 1e-6;
  // 1e-2 is 10 of standard deviation
  const double WHEEL_DIST = 1, ABS_ERROR = 1e-5;
  auto prev_pose = Pose2D{0, 0, 0};
  Eigen::Matrix2d cov = std::pow(std_dev, 2) * Eigen::Matrix2d::Identity();
  std::vector<std::pair<double, double>> odoms_list{
      // Translation
      std::make_pair(WHEEL_DIST, WHEEL_DIST),
      // Rotation
      std::make_pair(0.5 * M_PI * WHEEL_DIST, -0.5 * M_PI * WHEEL_DIST),
      std::make_pair(0.25 * M_PI * WHEEL_DIST, -0.25 * M_PI * WHEEL_DIST),
      std::make_pair(-0.25 * M_PI * WHEEL_DIST, 0.25 * M_PI * WHEEL_DIST),
      // Translation and rotation
      std::make_pair(1.0 / 4 * M_PI * WHEEL_DIST, 3.0 / 4 * M_PI * WHEEL_DIST),

  };
  std::vector<std::pair<double, double>> screw_displacements_list{
      std::make_pair(WHEEL_DIST, 0.0),
      std::make_pair(0.0, -M_PI),
      std::make_pair(0.0, -M_PI / 2.0),
      std::make_pair(0.0, M_PI / 2.0),
      std::make_pair(1.0 / 2 * M_PI * WHEEL_DIST, M_PI / 2.0),

  };
  std::vector<Pose2D> noiseless_new_pose_ls{
      Pose2D{WHEEL_DIST, 0, 0},
      Pose2D{0, 0, M_PI},
      Pose2D{0, 0, -M_PI / 2.0},
      Pose2D{0, 0, M_PI / 2.0},
      Pose2D{WHEEL_DIST, WHEEL_DIST, M_PI / 2.0},
  };
  ASSERT_EQ(odoms_list.size(), screw_displacements_list.size());
  for (unsigned i = 0; i < odoms_list.size(); i++) {
    auto screw_displacements =
        get_2D_screw_displacement(odoms_list[i], WHEEL_DIST);
    auto [d_v, d_theta] = screw_displacements_list[i];
    auto [d_v_estimate, d_theta_estimate] = screw_displacements;
    EXPECT_NEAR(d_v_estimate, d_v, ABS_ERROR);
    EXPECT_NEAR(normalize_angle_2PI(d_theta_estimate),
                normalize_angle_2PI(d_theta), ABS_ERROR);

    auto [prob, new_pose] = draw_from_icc(prev_pose, screw_displacements, cov);
    //
    EXPECT_NEAR(new_pose.x, noiseless_new_pose_ls[i].x, ABS_ERROR);
    EXPECT_NEAR(new_pose.y, noiseless_new_pose_ls[i].y, ABS_ERROR);
    EXPECT_NEAR(normalize_angle_2PI(new_pose.theta),
                normalize_angle_2PI(noiseless_new_pose_ls[i].theta), ABS_ERROR);
  }
}