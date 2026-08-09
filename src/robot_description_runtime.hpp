#pragma once

#include <string>
#include <vector>

namespace xgc2_robot_visualization {

// RobotDescription is the kind-neutral, frozen input of the managed
// description/RSP runtime. Every value is already projected by Core from the
// immutable Experiment Robot roster; no asset payload or credential crosses
// this boundary.
struct RobotDescription {
    std::string name;
    std::string ros_namespace;
    std::string description_package;
    std::string description_file;
    bool robot_state_publisher;
    std::string joint_state_topic;
};

// readRobotVisualizationRoster strictly decodes the canonical JSON array
// passed by the Session-owned process definition. On failure, robots is left
// unchanged so callers cannot accidentally run a partially admitted fleet.
bool readRobotVisualizationRoster(const std::string& raw,
                                  std::vector<RobotDescription>* robots,
                                  std::string* error);

}  // namespace xgc2_robot_visualization
