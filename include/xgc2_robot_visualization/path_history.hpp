#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <ros/time.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

namespace xgc2_robot_visualization {

// Public world trails are a time window at a stated sample rate, not a handful
// of leftover frames. Product 3D and AR trails are the last 6 s at 10 Hz (61
// points). This duration must stay equal to Core visualization.ProductRobotHistoryWindowSec.
constexpr double kDefaultPathHistoryDurationSec = 6.0;
constexpr double kDefaultPathPublishRateHz = 10.0;

// Ground-vehicle history is drawn on the world XY plane. Canonical /pose still
// carries mocap marker height; accumulating that z makes the trail float.
// UAV history keeps world z. BoundedPathRuntime stores whatever the caller
// passes; UGV publishers flatten before append.
inline geometry_msgs::Point flattenGroundVehicleHistoryPoint(geometry_msgs::Point point) {
    point.z = 0.0;
    return point;
}

inline geometry_msgs::Pose flattenGroundVehicleHistoryPose(geometry_msgs::Pose pose) {
    pose.position.z = 0.0;
    return pose;
}

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
    if (*max_points <= 0 && *duration_sec > 0.0) {
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
    if (!path->empty()) {
        const ros::Time previous = path->back().stamp;
        if (stamp < previous) {
            const double reset_threshold_sec = std::max(1.0, duration_sec);
            if ((previous - stamp).toSec() > reset_threshold_sec) {
                path->clear();
            } else {
                return false;
            }
        } else if ((stamp - previous).toSec() < min_dt) {
            return false;
        }
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

// Drop samples older than duration_sec relative to now. A frozen now (paused
// sim time) leaves the window intact. A large clock rollback clears the buffer.
// Callers must not append a cached last pose with now to keep the trail alive.
inline bool expirePathHistory(std::deque<PathSample> *path, const ros::Time &now,
                             double duration_sec) {
    if (path == nullptr || now.isZero() || path->empty()) {
        return false;
    }
    const ros::Time last = path->back().stamp;
    if (now < last && (last - now).toSec() > std::max(1.0, duration_sec)) {
        path->clear();
        return true;
    }
    bool changed = false;
    while (!path->empty() && duration_sec > 0.0 &&
           (now - path->front().stamp).toSec() > duration_sec) {
        path->pop_front();
        changed = true;
    }
    return changed;
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
