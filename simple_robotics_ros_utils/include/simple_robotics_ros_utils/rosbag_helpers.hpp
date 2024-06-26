#pragma once
#include <ros/ros.h>
#include <rosbag/bag.h>

namespace SimpleRoboticsRosUtils{
class BagParser{
    public:
        BagParser(const std::string& bag_file);
        void add_topic(const bool& publish_after_reading);
        void next();
    private:
        std::string bag_file_;

};

inline BagParser::BagParser(const std::string& bag_file){}

};