#include "simple_robotics_cpp_utils/cv_utils.hpp"

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
};