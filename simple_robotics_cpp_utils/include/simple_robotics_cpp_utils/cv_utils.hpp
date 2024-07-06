#pragma once
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/opencv2.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace SimpleRoboticsCppUtils {
// TODO: Image Window
void display_image(const cv::Mat &image);
cv::Mat skew_symmetric_mat(const cv::Mat &t);
}; // namespace SimpleRoboticsCppUtils
