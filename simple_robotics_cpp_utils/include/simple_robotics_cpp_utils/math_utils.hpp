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
#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>

namespace SimpleRoboticsCppUtils {

constexpr double TWO_M_PI = (2 * M_PI);
// can't use std::sqrt in C++17 in constexpr
constexpr double ROOT_2PI_INVERSE = 0.3989422804014327;

// normalize angles [-pi, pi]
inline double normalize_angle_PI(const double &angle) {
  return std::fmod(angle + M_PI, TWO_M_PI) - M_PI;
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