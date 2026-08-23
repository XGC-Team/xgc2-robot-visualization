#include "xgc2_robot_visualization/path_runtime.hpp"

#include <algorithm>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "xgc2_robot_visualization/path_history.hpp"

namespace xgc2_robot_visualization {
namespace {

bool canonicalROSIdentifier(const std::string& value) {
    static const std::regex pattern("^[A-Za-z_][A-Za-z0-9_]*$");
    return !value.empty() && value.size() <= 127 && std::regex_match(value, pattern);
}

bool canonicalRelativeROSName(const std::string& value) {
    if (value.empty() || value.front() == '/' || value.back() == '/') {
        return false;
    }
    std::stringstream stream(value);
    std::string segment;
    while (std::getline(stream, segment, '/')) {
        if (!canonicalROSIdentifier(segment)) {
            return false;
        }
    }
    return true;
}

}  // namespace

BoundedPathRuntime::BoundedPathRuntime(std::string frame_id, PathRuntimeConfig config)
    : frame_id_(std::move(frame_id)),
      sample_rate_hz_(config.sample_rate_hz),
      max_age_sec_(config.max_age_sec),
      max_points_(config.max_points) {
    if (frame_id_.empty()) {
        throw std::invalid_argument("Path Fixed Frame is required");
    }
    applyPathHistoryConfig(&sample_rate_hz_, &max_age_sec_, &max_points_);
    path_.header.frame_id = frame_id_;
}

bool BoundedPathRuntime::append(const ros::Time& stamp, const geometry_msgs::Pose& pose) {
    if (stamp.isZero()) {
        return false;
    }
    if (!path_.poses.empty()) {
        const ros::Time previous = path_.poses.back().header.stamp;
        if (stamp < previous) {
            const double reset_threshold_sec = std::max(1.0, max_age_sec_);
            if ((previous - stamp).toSec() > reset_threshold_sec) {
                reset();
            } else {
                return false;
            }
        } else if (stamp == previous || (stamp - previous).toSec() < 1.0 / sample_rate_hz_) {
            return false;
        }
    }
    while (path_.poses.size() >= 2U && max_age_sec_ > 0.0 &&
           (stamp - path_.poses.front().header.stamp).toSec() > max_age_sec_) {
        path_.poses.erase(path_.poses.begin());
    }
    while (static_cast<int>(path_.poses.size()) >= max_points_) {
        path_.poses.erase(path_.poses.begin());
    }
    geometry_msgs::PoseStamped sample;
    sample.header.frame_id = frame_id_;
    sample.header.stamp = stamp;
    sample.pose = pose;
    path_.poses.push_back(std::move(sample));
    path_.header.frame_id = frame_id_;
    path_.header.stamp = stamp;
    return true;
}

void BoundedPathRuntime::reset() {
    path_.poses.clear();
    path_.header.frame_id = frame_id_;
    path_.header.stamp = ros::Time();
}

const nav_msgs::Path& BoundedPathRuntime::message() const {
    return path_;
}

std::string namespacedPathTopic(const std::string& ros_namespace,
                                const std::string& relative_topic) {
    if (ros_namespace.size() < 2U || ros_namespace.front() != '/' ||
        !canonicalROSIdentifier(ros_namespace.substr(1)) ||
        !canonicalRelativeROSName(relative_topic)) {
        throw std::invalid_argument("Path namespace/topic is not canonical");
    }
    return ros_namespace + "/" + relative_topic;
}

}  // namespace xgc2_robot_visualization
