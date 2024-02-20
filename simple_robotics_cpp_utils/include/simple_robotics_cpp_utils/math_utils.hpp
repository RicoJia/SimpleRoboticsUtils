//
/**
* Notes:
* the inline keyword:
    - is a hint for compiler to replace function calls with the code directly.
        It's just a hint, compilers may not do it. But it will make sure there's
only one copy of this function
    - So, it can avoid ODR violation when linked against other source files,
inline
* <cmath> has M_PI
* #pragma once preprocessor directive, though not in standard c++ standard, is
portable
*/
#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <eigen3/Eigen/src/Geometry/AngleAxis.h>
#include <iostream>
#include <random>
#include <utility>

namespace SimpleRoboticsCppUtils {

constexpr double TWO_M_PI = (2 * M_PI);
// can't use std::sqrt in C++17 in constexpr
constexpr double ROOT_2PI_INVERSE = 0.3989422804014327;

/**
 * @brief normalize angle to [0, 2pi], with 2pi-1e-5 being considered 2pi
 *
 * @param angle : angle to normalized
 * @return double : normalized angle
 */
inline double normalize_angle_2PI(const double &angle) {
  double return_angle = std::fmod(angle, TWO_M_PI);
  return_angle = (return_angle < 0) ? return_angle + TWO_M_PI : return_angle;
  return_angle = (TWO_M_PI - return_angle) < 1e-5 ? 0 : return_angle;
  return return_angle;
}

/**
 * @brief Returning d_v, and d_theta along a circlar arc, since on 2D, screw
 * axis is the z axis, and translation is 0 along z.
 *
 * @param prev_pose : previous pose
 * @param odoms : [left_odom, right_odom], in meters
 * @param wheel_dist : distance between left and right wheels in meters
 * @return std::pair <double, double> : change in arc length (d_v) and rotation
 * (d_theta)
 */
inline std::pair<double, double>
get_2D_screw_displacement(const std::pair<double, double> &odoms,
                          const double &wheel_dist) {
  auto [l, r] = odoms;
  // TODO
  std::cout << "(r - l) / wheel_dist" << (r - l) / wheel_dist << std::endl;
  double d_theta = normalize_angle_2PI((r - l) / wheel_dist);
  double d_v = (r + l) / 2;
  return {d_v, d_theta};
}

// Returns transform in body frame, from starting point to ending point
inline Eigen::Matrix4d screw_displacement_2d_to_body_frame_transform(
    const std::pair<double, double> &screw_displacement) {
  auto [d_v, d_theta] = screw_displacement;
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d R =
      Eigen::AngleAxisd(d_theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.block<3, 3>(0, 0) = R;
  double tx, ty;
  if (std::abs(d_theta) < 1e-5) {
    tx = d_v;
    ty = 0;
  } else {
    double r = d_v / d_theta;
    tx = r * std::sin(d_theta);
    ty = r * (1 - std::cos(d_theta));
  }
  T(0, 3) = tx;
  T(1, 3) = ty;
  return T;
}

inline Eigen::Matrix3d transform_4d_to_3d(const Eigen::Matrix4d &T) {
  Eigen::Matrix3d T_3d = Eigen::Matrix3d::Identity();
  T_3d << T(0, 0), T(0, 1), T(0, 3), T(1, 0), T(1, 1), T(1, 3), 0.0, 0.0, 1.0;
  return T_3d;
}

/**
 * @brief Draw from univariate normal distribution with specified mean and
 * variance
 *
 * @param mean mean of normal distribution
 * @param variance variance of normal distribution
 * @return randomly generated number
 */
inline double draw_from_pdf_normal(const double &mean, const double &std_dev) {
  // random seed
  static std::random_device rd;
  // Mersenne twister PRNG, seeded with rd
  static std::mt19937 gen(rd());
  std::normal_distribution<double> d(mean, std_dev);
  return d(gen);
}

/**
 * @brief Function to draw from a multivariate Gaussian Distribution.
 *
 * @param means : row or column vector.
 * @param covariances : covariances of the 3 dimensions
 * @return Eigen::VectorXd: random vector that statistically follows the
 * covariances matrix
 */
inline Eigen::VectorXd
draw_from_pdf_normal(const Eigen::VectorXd &means,
                     const Eigen::MatrixXd &covariances) {
  assert(covariances.cols() == covariances.rows());
  assert(std::max(means.rows(), means.cols()) == covariances.rows());
  // cholesky decomposition A=LL^T. That's how Eigen named llt, lol.
  Eigen::LLT<Eigen::MatrixXd> llt(covariances);
  if (llt.info() == Eigen::NumericalIssue) {
    throw std::runtime_error("Lower Cholesky decomposition failed");
  }
  // L is "the square root" of the covariance matrix
  Eigen::MatrixXd L = llt.matrixL();
  // Note, Zero() returns an expression object, which is an rvalue referece.
  Eigen::VectorXd return_vec = Eigen::VectorXd::Zero(means.size());
  for (unsigned int i = 0; i < return_vec.size(); ++i) {
    // construct vector of 3 normally distributed variables. Why?
    return_vec[i] = draw_from_pdf_normal(0, 1);
  }
  return_vec = means + L * return_vec;
  return return_vec;
}

inline double normal_dist_prob(const double &mean, const double &std_dev,
                               const double &val, bool return_log_score) {
  double exponential_term = -0.5 * std::pow((val - mean) / std_dev, 2);
  return (return_log_score)
             ? exponential_term
             : 1.0 / std_dev * ROOT_2PI_INVERSE * exp(exponential_term);
}

} // namespace SimpleRoboticsCppUtils