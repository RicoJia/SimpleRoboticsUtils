#include "simple_robotics_cpp_utils/performance_utils.hpp"
#include "simple_robotics_cpp_utils/rigid2d.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <simple_robotics_cpp_utils/math_utils.hpp>
#include <unordered_map>
#include <utility>
#include <vector>
using namespace SimpleRoboticsCppUtils;

TEST(Rigid2DTest, TestAngle) {
  {
    Pose2D p1(1, 1, 0);
    Pose2D p2(0, 0, 0);
    EXPECT_NEAR(angle_of_two_points(p1, p2), -3 * M_PI / 4, 1e-5);
  }
}

TEST(Rigid2DTest, TestTo4D) {
  Pose2D p1(1, 2, M_PI / 4);
  Eigen::Matrix4d T = p1.to_se3();
  EXPECT_NEAR(p1.x, T(0, 3), 1e-5);
  EXPECT_NEAR(p1.y, T(1, 3), 1e-5);
  EXPECT_NEAR(0, T(2, 3), 1e-5);
  EXPECT_NEAR(get_2d_rotation_from_z_axis(T), M_PI / 4, 1e-5);

  Pose2D p2(T);
  EXPECT_NEAR(p2.x, 1, 1e-5);
  EXPECT_NEAR(p2.y, 2, 1e-5);
  EXPECT_NEAR(p2.theta, 1, M_PI / 4);
}

TEST(Rigid2DTest, TestPixel) {
  // turns out, this map could be resized after hitting a certain size
  std::unordered_map<int, SimpleRoboticsCppUtils::Pixel2DWithCount> count_map;
  long init_size = get_memory_usage();
  for (unsigned int j = 0; j < 100; j++) {
    count_map.emplace(std::piecewise_construct, std::make_tuple(j),
                      std::make_tuple(j, j));
  }
  auto after_size = get_memory_usage();
  std::cout << "For obstacle count: " << 100
            << ", the map size is: " << after_size << "kb" << std::endl;

  Eigen::Matrix4d T = SimpleRoboticsCppUtils::Pose2D(2.1, 2.1, 0.0).to_se3();
  const double resolution = 1;
  const auto pixel = SimpleRoboticsCppUtils::Pixel2DWithCount(T, resolution);
  EXPECT_EQ(pixel.x, 2);
  EXPECT_EQ(pixel.y, 2);

  auto pose = SimpleRoboticsCppUtils::Pose2D(2.1, 2.1, 0.0);
  const auto pixel2 =
      SimpleRoboticsCppUtils::Pixel2DWithCount(pose, resolution);
  EXPECT_EQ(pixel2.x, 2);
  EXPECT_EQ(pixel2.y, 2);
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

TEST(Rigid2DTest, TestScrewDisplacement2dToTransform) {
  std::vector<std::pair<double, double>> screw_displacements_list{
      {M_PI / 2.0, M_PI / 2.0},
      {M_PI / 2.0, -M_PI / 2.0},
      {1, 0.0},
      {-M_PI / 2.0, M_PI / 2.0},
  };
  std::vector<Pose2D> end_pose_list{
      {1, 1, M_PI / 2.0},
      {1, -1, -M_PI / 2.0},
      {1, 0, 0},
      {-1, -1, M_PI / 2.0},
  };
  assert(screw_displacements_list.size() == end_pose_list.size());
  for (unsigned i = 0; i < screw_displacements_list.size(); i++) {
    auto end_pose = end_pose_list[i];
    auto T = screw_displacement_2d_to_body_frame_transform(
        screw_displacements_list[i]);
    EXPECT_NEAR(T(0, 3), end_pose.x, 1e-5);
    EXPECT_NEAR(T(1, 3), end_pose.y, 1e-5);
    auto T_3d = transform_4d_to_3d(T);
  }
}

TEST(Rigid2DTest, TestUnitVectorPixel) {
  // start, end
  std::vector<std::pair<Pixel2DWithCount, Pixel2DWithCount>> start_ends{
      {{0, 0}, {3, 0}},   {{0, 0}, {3, 1}},   {{0, 0}, {3, 2}},
      {{0, 0}, {3, 3}},

      {{0, 0}, {2, 3}},   {{0, 0}, {1, 3}},   {{0, 0}, {0, 3}},

      {{0, 0}, {-1, 3}},  {{0, 0}, {-2, 3}},  {{0, 0}, {-3, 3}},

      {{0, 0}, {-3, 2}},  {{0, 0}, {-3, 1}},  {{0, 0}, {-3, 0}},

      {{0, 0}, {-3, -1}}, {{0, 0}, {-3, -2}},
  };
  std::vector<Pixel2DWithCount> ground_truths{
      Pixel2DWithCount(1, 0),  Pixel2DWithCount(1, 0),
      Pixel2DWithCount(1, 1),  Pixel2DWithCount(1, 1),

      Pixel2DWithCount(1, 1),  Pixel2DWithCount(0, 1),
      Pixel2DWithCount(0, 1),

      Pixel2DWithCount(0, 1),  Pixel2DWithCount(-1, 1),
      Pixel2DWithCount(-1, 1),

      Pixel2DWithCount(-1, 1), Pixel2DWithCount(-1, 0),
      Pixel2DWithCount(-1, 0),

      Pixel2DWithCount(-1, 0), Pixel2DWithCount(-1, -1),
  };

  for (unsigned i = 0; i < start_ends.size(); i++) {
    auto unit_vector_end_pixel = get_unit_vector_endpoint_pixel(
        start_ends[i].first, start_ends[i].second);
    EXPECT_EQ(unit_vector_end_pixel, ground_truths[i]);
  }
}