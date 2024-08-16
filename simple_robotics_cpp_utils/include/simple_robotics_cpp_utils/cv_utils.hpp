#pragma once
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/opencv2.hpp>
#include <Eigen/Geometry>
#include <opencv2/imgproc/imgproc.hpp>

namespace SimpleRoboticsCppUtils {
// TODO: Image Window
void display_image(const cv::Mat &image);
cv::Mat skew_symmetric_mat(const cv::Mat &t);

/**
 * @brief Get the Frobenius (a.k.a Hillbert-Schmidt) Norm of a matrix
 sum_ij(|M_ij|^2)
 *
 * @param m : matrix under calculation
 * @return double : Frobenius norm
 */
double getFrobeniusNorm(const cv::Mat &m);

/**
 * @brief Convert a pixel into canonical coordinates plane where depth=1 unit
    [p_c, p_y, 1] = K^-1 [u, v, 1] , we are returning [p_c, p_y]
 *
 * @param pixel : A single pixel observed
 * @param K : Camera Intrinsics, in cv::Mat_<double>(3, 3)
 * @return cv::Point2f : output canonical coordinates of the pixels
 */
cv::Point2f pixel2cam(const cv::Point2f &pixel, const cv::Mat &K);

/**
 * @brief Convert a vector of pixels to its canonical coordinates
 *
 * @param pixels : list of pixels observed
 * @param K : Camera Intrinsics
 * @return std::vector<cv::Point2f> : canonical coordinates of pixels
 */
std::vector<cv::Point2f> pixel2cam(const std::vector<cv::Point2f> &pixels,
                                   const cv::Mat &K);

/**
 * @brief Convert rotation, translation in cv::Mat to eigen::isometry3D
 *
 * @param R : rotation matrix
 * @param t : translation (matrix)
 * @return Eigen::Isometry3d : resulting transformation
 */
Eigen::Isometry3d cv_R_t_to_eigen_isometry3d(const cv::Mat &R,
                                             const cv::Mat &t);

/**
 * @brief Find an inverse of matrix
 *
 * @return true if an inverse of matrix can be found successfully
 * @return false : if an inverse of matrix cannot be found (determinant == 0)
 */
bool invert_mat(const cv::Mat &K, cv::Mat &K_inv);

/**
 * @brief Build an Image Pyramid (a vector of scaled images)
 *
 * @param image - original image
 * @param num_levels : number of levels
 * @return std::vector<cv::Mat> : image pyramid
 */
std::vector<cv::Mat> build_image_pyramid(const cv::Mat &image,
                                         const unsigned int num_levels,
                                         const double &scale_factor);

template <typename MatrixXd>
const MatrixXd cv_R_to_eigen_matrixXd(const cv::Mat &m) {
  MatrixXd eigen_mat(m.rows, m.cols);
  for (int i = 0; i < m.rows; ++i)
    for (int j = 0; j < m.rows; ++j)
      eigen_mat(i, j) = m.at<double>(i, j);
  return eigen_mat;
}

void draw_feature_points(const cv::Mat &image,
                         const std::vector<cv::KeyPoint> &keypoints);

void visualize_cv_rects(const std::vector<cv::Rect> &rects);

cv::Point to_cv_point(const cv::Point2f &p);

}; // namespace SimpleRoboticsCppUtils
