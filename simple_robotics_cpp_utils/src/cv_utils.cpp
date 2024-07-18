#include "simple_robotics_cpp_utils/cv_utils.hpp"
#include <iostream>

namespace SimpleRoboticsCppUtils{
    // TODO: Image Window
    void display_image(const cv::Mat &image) {
        // High Gui provides simple GUI functionalities, like track bar, mouse event
        // handling
        cv::namedWindow("Image window");
        cv::imshow("Image window", image);
        cv::waitKey(0);
        cv::destroyWindow("Image window");
    }

    cv::Mat skew_symmetric_mat(const cv::Mat & t){
        return (cv::Mat_<double>(3, 3) << 
                0, -t.at<double>(2), t.at<double>(1),
                t.at<double>(2), 0, -t.at<double>(0),
                -t.at<double>(1), t.at<double>(0), 0);
    }

    double getFrobeniusNorm(const cv::Mat &m){
        // element-wise matrix multiplication 
        // cv::sum sums of elements across all dimensions
        return std::sqrt(cv::sum(m.mul(m))[0]);
    }

    cv::Point2f pixel2cam(const cv::Point2f& pixel, const cv::Mat &K){
        // K^-1 [u, v, 1] = [p_c, p_y, 1]. using const float 
        // because we are working with Point2f
        const float fx = K.at<double>(0, 0);
        const float fy = K.at<double>(1, 1);
        const float cx = K.at<double>(0, 2);
        const float cy = K.at<double>(1, 2);
        const float x = pixel.x;
        const float y = pixel.y;
        return {(x - cx)/fx, (y-cy)/fy};
    }

    std::vector<cv::Point2f> pixel2cam(const std::vector<cv::Point2f>& pixels, const cv::Mat &K){
        std::vector<cv::Point2f> result;
        result.reserve(pixels.size());
        for (const auto &p : pixels){
            result.push_back(pixel2cam(p, K));
        }
        return result;
    }

    Eigen::Isometry3d cv_R_t_to_eigen_isometry3d(const cv::Mat& R, const cv::Mat& t){
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        Eigen::Matrix3d rotation;
        rotation << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
        transform.rotate(rotation);
        Eigen::Vector3d translation(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0));
        transform.pretranslate(translation);
        return transform;
    }

    // TODO: test
    bool invert_mat(const cv::Mat& K, cv::Mat& K_inv){
        auto det = cv::invert(K, K_inv, cv::DECOMP_SVD);
        return det != 0;
    }
};