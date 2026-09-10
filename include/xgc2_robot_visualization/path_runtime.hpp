#pragma once

#include <cstddef>
#include <deque>
#include <string>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <ros/time.h>

#include "xgc2_robot_visualization/path_history.hpp"

namespace xgc2_robot_visualization {

struct PathRuntimeConfig {
    double sample_rate_hz{kDefaultPathPublishRateHz};
    double max_age_sec{kDefaultPathHistoryDurationSec};
    int max_points{0};
};

// BoundedPathRuntime retains poses exactly as received. It does not transform,
// offset, or synthesize z. Callers must provide poses already in the declared
// Fixed Frame. Ground-vehicle history callers flatten world z to 0 first;
// UAV history keeps the fused pose z.
class BoundedPathRuntime {
  public:
    BoundedPathRuntime(std::string frame_id, PathRuntimeConfig config);

    bool append(const ros::Time& stamp, const geometry_msgs::Pose& pose);
    bool expire(const ros::Time& now);
    void reset();
    const nav_msgs::Path& message() const;

  private:
    std::string frame_id_;
    double sample_rate_hz_;
    double max_age_sec_;
    int max_points_;
    nav_msgs::Path path_;
};

std::string namespacedPathTopic(const std::string& ros_namespace,
                                const std::string& relative_topic);

}  // namespace xgc2_robot_visualization
