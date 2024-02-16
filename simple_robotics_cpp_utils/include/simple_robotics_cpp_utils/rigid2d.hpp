#pragma once
#include <functional>
#include <iostream>
#include <memory>
#include <unordered_map>
// for std::make_tuple
#include <tuple>
// vector is more memory friendly than list
#include <vector>

// Eigen is a header-only library
#include <Eigen/Dense>

namespace SimpleRoboticsCppUtils {
struct Pose {
  double x, y, theta;
  Pose(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
};

struct Pixel2DWithCount {
  int x, y;
  unsigned int hit_count_;
  unsigned int total_count_ = 1;
  Pixel2DWithCount(int x_, int y_, unsigned int hit_count_)
      : x(x_), y(y_), hit_count_(hit_count_) {}
  /**
   * @brief Return true if the point is considered occupied, otherwise false.
   This is based on the "reflection map". When added to the PointAccumulator,
   this point's count is at least 1 So, we just use a bool to represent
   occupancy
   */
  bool is_full() const {
    // We set 0.5 as known/unknown boundary. So, if
    return (hit_count_ << 1) > total_count_;
  }
};

inline std::ostream &operator<<(std::ostream &stream,
                                const Pixel2DWithCount &p) {
  stream << "(" << p.x << " " << p.y << ")";
  // return the object to allow chaining
  return stream;
}

inline bool operator==(const Pixel2DWithCount &p1, const Pixel2DWithCount &p2) {
  return p1.x == p2.x && p1.y == p2.y;
}

inline void draw_from_icc(const Pose& prev_pose, const Pose& current_pose, const Eigen::Matrix3d& cov_matrix){
}

}; // namespace SimpleRoboticsCppUtils

namespace std {
// TODO: what does this anonymous template func do?
template <> struct hash<SimpleRoboticsCppUtils::Pixel2DWithCount> {
  std::hash<long> _hasher;
  size_t operator()(const SimpleRoboticsCppUtils::Pixel2DWithCount &p) const {
    // shifting 4 bytes
    constexpr size_t _shift_bits_num = sizeof(int) * 8;
    return _hasher(static_cast<long>(p.x) << _shift_bits_num |
                   static_cast<long>(p.y));
  }
};
} // namespace std