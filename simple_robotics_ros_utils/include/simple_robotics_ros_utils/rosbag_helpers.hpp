#pragma once
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <optional>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <unordered_map>

namespace SimpleRoboticsRosUtils {

class BagParser {
public:
  BagParser(ros::NodeHandle nh, const std::string &bag_file);
  ~BagParser();
  template <typename T>
  void add_topic(const std::string topic, const bool &publish_after_reading);

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
  rosbag::View::iterator general_msg_iterator_;
};

inline BagParser::BagParser(ros::NodeHandle nh, const std::string &bag_file) {
  std::cout << "Opening bag file: " << bag_file << std::endl;
  bag_.open(bag_file, rosbag::bagmode::Read);
  view_ = std::make_shared<rosbag::View>(bag_);
  general_msg_iterator_ = view_->begin();
  for (auto connection_info : view_->getConnections()) {
    std::cout << "BagParser added topic: " << connection_info->topic
              << std::endl;
    msg_iterators_.insert(
        std::make_pair(connection_info->topic, view_->begin()));
  }
}
inline BagParser::~BagParser() { bag_.close(); }

// returning message pointer  TODO: when publish_after_reading is not set yet
template <typename T>
inline void BagParser::add_topic(const std::string topic,
                                 const bool &publish_after_reading) {
  if (publish_after_reading) {
    publishers_.insert(std::make_pair(topic, nh_.advertise<T>(topic, 1)));
  }
}

// if topic is none, we will return the next topic by the receipt time.
// If a topic is specified, we have an iterator for each topic that will be used
// to find its next message
// TODO: a huge assumption being made is rosbag iterator is based on
// chronological order
template <typename T>
boost::shared_ptr<T> BagParser::next(const std::optional<std::string> &topic) {
  if (topic) {
    // would throw an error if topic doesn't exist? TODO
    for (auto &it = msg_iterators_.at(*topic); it != view_->end(); it++) {
      if (it->getTopic() == *topic) {
        rosbag::MessageInstance m = *it;
        it++; // set up for next function call
        return m.instantiate<T>();
      }
    }
  }
  // TODO
  std::cout << "finished next" << std::endl;
  return nullptr;
}

}; // namespace SimpleRoboticsRosUtils