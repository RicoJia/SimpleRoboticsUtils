#include "simple_robotics_cpp_utils/cv_utils.hpp"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace SimpleRoboticsCppUtils {
// TODO: Image Window
void display_image(const cv::Mat &image) {
  // High Gui provides simple GUI functionalities, like track bar, mouse event
  // handling
  cv::namedWindow("Image window");
  cv::imshow("Image window", image);
  while (1) {
    int key = cv::waitKey(0);
    // esc is hit
    if (key == 27)
      break;
  }

  try {
    cv::destroyWindow("Image window");
  } catch (const cv::Exception &e) {
  }
}

cv::Mat skew_symmetric_mat(const cv::Mat &t) {
  return (cv::Mat_<double>(3, 3) << 0, -t.at<double>(2), t.at<double>(1),
          t.at<double>(2), 0, -t.at<double>(0), -t.at<double>(1),
          t.at<double>(0), 0);
}

double getFrobeniusNorm(const cv::Mat &m) {
  // element-wise matrix multiplication
  // cv::sum sums of elements across all dimensions
  return std::sqrt(cv::sum(m.mul(m))[0]);
}

cv::Point2f pixel2cam(const cv::Point2f &pixel, const cv::Mat &K) {
  // K^-1 [u, v, 1] = [p_c, p_y, 1]. using const float
  // because we are working with Point2f
  const float fx = K.at<double>(0, 0);
  const float fy = K.at<double>(1, 1);
  const float cx = K.at<double>(0, 2);
  const float cy = K.at<double>(1, 2);
  const float x = pixel.x;
  const float y = pixel.y;
  return {(x - cx) / fx, (y - cy) / fy};
}

std::vector<cv::Point2f> pixel2cam(const std::vector<cv::Point2f> &pixels,
                                   const cv::Mat &K) {
  std::vector<cv::Point2f> result;
  result.reserve(pixels.size());
  for (const auto &p : pixels) {
    result.push_back(pixel2cam(p, K));
  }
  return result;
}

Eigen::Isometry3d cv_R_t_to_eigen_isometry3d(const cv::Mat &R,
                                             const cv::Mat &t) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d rotation;
  rotation << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
      R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
      R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
  transform.rotate(rotation);
  Eigen::Vector3d translation(t.at<double>(0, 0), t.at<double>(1, 0),
                              t.at<double>(2, 0));
  transform.pretranslate(translation);
  return transform;
}

bool invert_mat(const cv::Mat &K, cv::Mat &K_inv) {
  auto det = cv::invert(K, K_inv, cv::DECOMP_SVD);
  return det != 0;
}

std::vector<cv::Mat> build_image_pyramid(const cv::Mat &image,
                                         const unsigned int num_levels,
                                         const double &scale_factor) {
  if (scale_factor < 1.0)
    throw std::runtime_error("scale_factor must be above 1.0 for downsizing");
  std::vector<cv::Mat> image_pyramid{image};
  for (unsigned int level = 1; level < num_levels; ++level) {
    double scale = 1 / std::pow(scale_factor, level);
    int width = static_cast<int>(image.cols * scale);
    int height = static_cast<int>(image.rows * scale);
    cv::Mat resized_image;
    // TODO: interarea? Also, need scale, which is
    cv::resize(image, resized_image, cv::Size(width, height), 0, 0,
               cv::INTER_AREA);
    image_pyramid.emplace_back(std::move(resized_image));
  }
  return image_pyramid;
}

void draw_feature_points(const cv::Mat &image,
                         const std::vector<cv::KeyPoint> &keypoints) {
  auto image_cp = image.clone();
  if (image.type() != CV_8UC3) {
    cv::cvtColor(image, image_cp,
                 cv::COLOR_GRAY2BGR); // or another appropriate conversion
  }
  cv::drawKeypoints(image_cp, keypoints, image_cp, cv::Scalar::all(-1),
                    cv::DrawMatchesFlags::DEFAULT);
  SimpleRoboticsCppUtils::display_image(image_cp);
}

void visualize_cv_rects(const std::vector<cv::Rect> &rects) {
  int min_x = rects.at(0).tl().x;
  int max_x = rects.at(0).br().x;
  int min_y = rects.at(0).tl().y;
  int max_y = rects.at(0).br().y;

  // Find the bounds of the rectangles
  for (const auto &r : rects) {
    min_x = std::min(min_x, r.tl().x);
    max_x = std::max(max_x, r.br().x);
    min_y = std::min(min_y, r.tl().y);
    max_y = std::max(max_y, r.br().y);
  }

  // Adjust to create an image that can contain all rectangles
  int width = max_x - min_x;
  int height = max_y - min_y;

  // Create an image large enough to contain all rectangles
  cv::Mat image = cv::Mat::zeros(height, width, CV_8UC1);

  int thickness = 2;
  cv::Scalar color(255); // White rectangles

  // Draw the rectangles
  for (const auto &r : rects) {
    // Shift rectangles to ensure they are within the image bounds
    cv::Rect shifted_rect = r + cv::Point(-min_x, -min_y);
    // This function does NOT take into account the bounds.
    cv::rectangle(image, shifted_rect, color, thickness);
  }

  // Enlarge the image for visualization
  cv::Mat enlarged_img;
  cv::resize(image, enlarged_img, cv::Size(), 10, 10, cv::INTER_NEAREST);

  // Display the image (you need to define this function)
  cv::imshow("Rectangles", enlarged_img);
  cv::waitKey(0); // Wait for key press
}

cv::Point to_cv_point(const cv::Point2f &point_f) {
  return cv::Point(cvRound(point_f.x), cvRound(point_f.y));
}

}; // namespace SimpleRoboticsCppUtils