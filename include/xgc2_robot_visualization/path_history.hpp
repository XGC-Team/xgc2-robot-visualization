#pragma once

#include <geometry_msgs/Point.h>
#include <ros/time.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

namespace xgc2_robot_visualization {

// Public world trails are a time window at a stated sample rate, not a handful
// of leftover frames. Default 60 s at 10 Hz is 601 points.
constexpr double kDefaultPathHistoryDurationSec = 60.0;
constexpr double kDefaultPathPublishRateHz = 10.0;

struct PathSample {
    ros::Time stamp;
    geometry_msgs::Point point;
};

inline int pathHistoryPointBudget(double duration_sec, double sample_rate_hz) {
    const double rate = std::max(1.0, sample_rate_hz);
    const double duration = std::max(0.0, duration_sec);
    return std::max(2, static_cast<int>(std::ceil(duration * rate)) + 1);
}

inline void applyPathHistoryConfig(double *sample_rate_hz, double *duration_sec, int *max_points) {
    if (sample_rate_hz == nullptr || duration_sec == nullptr || max_points == nullptr) {
        return;
    }
    *sample_rate_hz = std::max(1.0, *sample_rate_hz);
    *duration_sec = std::max(0.0, *duration_sec);
    if (*duration_sec > 0.0) {
        *max_points = pathHistoryPointBudget(*duration_sec, *sample_rate_hz);
    }
    *max_points = std::max(2, *max_points);
}

inline bool pushPathHistory(std::deque<PathSample> *path, const ros::Time &stamp,
                            const geometry_msgs::Point &point, double sample_rate_hz,
                            double duration_sec, int max_points) {
    if (path == nullptr || stamp.isZero()) {
        return false;
    }
    const double min_dt = 1.0 / std::max(1.0, sample_rate_hz);
    if (!path->empty() && (stamp - path->back().stamp).toSec() < min_dt) {
        return false;
    }
    while (path->size() >= 2 && duration_sec > 0.0 &&
           (stamp - path->front().stamp).toSec() > duration_sec) {
        path->pop_front();
    }
    const int cap = std::max(2, max_points);
    while (static_cast<int>(path->size()) >= cap) {
        path->pop_front();
    }
    PathSample sample;
    sample.stamp = stamp;
    sample.point = point;
    path->push_back(sample);
    return true;
}

inline void assignPathHistoryPoints(const std::deque<PathSample> &path,
                                    std::vector<geometry_msgs::Point> *points) {
    if (points == nullptr) {
        return;
    }
    points->clear();
    points->reserve(path.size());
    for (const PathSample &sample : path) {
        points->push_back(sample.point);
    }
}

}  // namespace xgc2_robot_visualization
