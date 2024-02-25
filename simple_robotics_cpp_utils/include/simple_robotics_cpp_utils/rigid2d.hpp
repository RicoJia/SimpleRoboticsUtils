#pragma once
#include <functional>
#include <iostream>
#include <memory>
#include <ostream>
#include <unordered_map>
// for std::make_tuple
#include <tuple>
// vector is more memory friendly than list
#include <utility>
#include <vector>

// Eigen is a header-only library
#include "simple_robotics_cpp_utils/math_utils.hpp"
#include <Eigen/Dense>

namespace SimpleRoboticsCppUtils {
struct Pose2D {
  double x, y, theta;
  Pose2D(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
  Pose2D(const Eigen::Matrix4d &T)
      : x(T(0, 3)), y(T(1, 3)), theta(std::atan2(T(1, 0), T(0, 0))) {}
  inline Eigen::Matrix4d to_se3() const {
    Eigen::Matrix4d T;
    T << cos(theta), -sin(theta), 0, x, sin(theta), cos(theta), 0, y, 0, 0, 1,
        0, 0, 0, 0, 1;
    return T;
  }
};

struct Pixel2DWithCount {
  int x, y;
  unsigned int hit_count_ = 0;
  unsigned int total_count_ = 0;
  Pixel2DWithCount(int x_, int y_) : x(x_), y(y_) {}
  /**
   * @brief Return true if the point is considered occupied, otherwise false.
   This is based on the "reflection map". When added to the PointAccumulator,
   this point's count is at least 1 So, we just use a bool to represent
   occupancy
   */
  bool is_full() const {
    // We set 0.5 as known/unknown boundary. So, if
    return (hit_count_ << 1) > total_count_;
  }
};

inline size_t hash_pixel2d_with_count(const SimpleRoboticsCppUtils::Pixel2DWithCount& p) {
  std::hash<long> _hasher;
// shifting 4 bytes
constexpr size_t _shift_bits_num = sizeof(int) * 8;
return _hasher(static_cast<long>(p.x) << _shift_bits_num |
                static_cast<long>(p.y));
}

inline size_t hash_pixel2d_with_count(const unsigned int &x, const unsigned int &y) {
    std::hash<long> _hasher;
    // shifting 4 bytes
    constexpr size_t _shift_bits_num = sizeof(int) * 8;
    return _hasher(static_cast<long>(x) << _shift_bits_num |
                   static_cast<long>(y));
  }

inline double dist(const Pose2D &p1, const Pose2D &p2) {
  return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                   (p1.y - p2.y) * (p1.y - p2.y));
}

inline double angle_of_two_points(const Pose2D &start, const Pose2D &end) {
  return atan2(end.y - start.y, end.x - start.x);
}

inline std::ostream &operator<<(std::ostream &stream, const Pose2D &p) {
  stream << "(x: " << p.x << ", y:" << p.y << ", theta:" << p.theta << ")"
         << std::endl;
}
inline std::ostream &operator<<(std::ostream &stream,
                                const Pixel2DWithCount &p) {
  stream << "(" << p.x << " " << p.y << ")";
  // return the object to allow chaining
  return stream;
}

inline bool operator==(const Pixel2DWithCount &p1, const Pixel2DWithCount &p2) {
  return p1.x == p2.x && p1.y == p2.y;
}

/**
 * @brief : Assuming the motion current and previous odom pose is a circular arc
 * (2D screw motion), icc (instantaneous center of curvature). We add a noise to
 * the screw displacements v and theta One lesson learned in this function is,
 * we screw_displacement or left or right odom is much better than passing in
 * poses. Due to numerical errors, these poses may not be holonomically feasible
 * for diff drive. So, let's pick our fight right.
 *
 * @param prev_pose : odom pose received in the previous state
 * @param screw_displacement : [d_v, d_theta] of the differential drive
 * @param cov_matrix : covariance of [v, theta]
 * @return std::pair<double, Pose2D> : [probability, pose from motion model]
 */
inline std::pair<double, Pose2D>
draw_from_icc(const Pose2D &prev_pose,
              const std::pair<double, double> &screw_displacement,
              const Eigen::Matrix2d &cov_matrix) {
  // d_v is the arc length, d_theta is the theta needed
  auto [d_v, d_theta] = screw_displacement;
  auto noise_mean = Eigen::Vector2d{0.0, 0.0};
  auto noises = draw_from_pdf_normal(noise_mean, cov_matrix);
  double log_prob = 0;
  for (unsigned i = 0; i < noises.size(); i++) {
    log_prob += normal_dist_prob(noise_mean[i], std::sqrt(cov_matrix(i, i)),
                                 noises[i], true);
  }

  // Note: we are omitting 1/(std_dev * sqrt(2pi))
  double prob = exp(log_prob);
  const double d_theta_before_noise = d_theta;
  d_v += noises[0], d_theta += noises[1];
  double new_theta = normalize_angle_2PI(prev_pose.theta + d_theta);

  if (std::abs(d_theta_before_noise) < 1e-3) {
    Pose2D pose{prev_pose.x + d_v * cos(new_theta),
                prev_pose.y + d_v * sin(new_theta), new_theta};
    return {prob, pose};
  } else {
    double pitch = d_v / d_theta;
    Pose2D pose{prev_pose.x + pitch * (sin(new_theta) - sin(prev_pose.theta)),
                prev_pose.y + pitch * (cos(prev_pose.theta) - cos(new_theta)),
                new_theta};
    return {prob, pose};
  }
}

}; // namespace SimpleRoboticsCppUtils
