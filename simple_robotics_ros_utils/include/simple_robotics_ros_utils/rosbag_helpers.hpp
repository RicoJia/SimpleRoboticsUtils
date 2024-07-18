#pragma once
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <optional>
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <unordered_map>

namespace SimpleRoboticsRosUtils {

class BagParser {
public:
  BagParser(ros::NodeHandle nh, const std::string &package_name, const std::string &bag_file);
  ~BagParser();

  // If the topic previously have been declared to publish, it will be done
  // here.
  template <typename T>
  boost::shared_ptr<T> next(const std::optional<std::string> &topic);

private:
  ros::NodeHandle nh_;
  std::string bag_file_;
  rosbag::Bag bag_;
  // view_ cannot be copied or moved
  std::shared_ptr<rosbag::View> view_;
  std::unordered_map<std::string, ros::Publisher> publishers_;
  std::unordered_map<std::string, rosbag::View::iterator> msg_iterators_;
};

inline BagParser::BagParser(ros::NodeHandle nh, const std::string &package_name, const std::string &bag_file) {
  std::string package_path = ros::package::getPath(package_name);
  if (package_path.empty()) 
    throw std::runtime_error(package_path + "does not exist");
  std::string full_bag_path = package_path + "/" + bag_file;
  std::cout << "Opening bag file: " << full_bag_path << std::endl;
  bag_.open(full_bag_path, rosbag::bagmode::Read);
  view_ = std::make_shared<rosbag::View>(bag_);
for (const auto& c: view_->getConnections()){
    msg_iterators_.insert(
        std::make_pair(c->topic, view_->begin()));
    std::cout<<"Added Topic: "<<c->topic<<std::endl;
    }
}
inline BagParser::~BagParser() { bag_.close(); }


// if topic is none, we will return the next topic by the receipt time.
// If a topic is specified, we have an iterator for each topic that will be used
// to find its next message
// TODO: a huge assumption being made is rosbag iterator is based on
// chronological order
template <typename T>
boost::shared_ptr<T> BagParser::next(const std::optional<std::string> &topic) {
  if (topic) {
    if (msg_iterators_.find(*topic) == msg_iterators_.end()){
        throw std::runtime_error(*topic + " could not be found in bags.");
    }
    for (auto &it = msg_iterators_.at(*topic); it != view_->end(); it++) {
      if (it->getTopic() == *topic) {
        rosbag::MessageInstance m = *it;
        it++; // set up for next function call
        return m.instantiate<T>();
      }
    }
  }
  return nullptr;
}

}; // namespace SimpleRoboticsRosUtils