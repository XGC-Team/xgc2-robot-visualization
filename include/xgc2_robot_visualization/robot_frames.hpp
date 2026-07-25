#pragma once

#include <string>

namespace xgc2_robot_visualization {

// One owner for the transform names every robot kind publishes. A viewer layout
// derives a URDF layer's frame prefix from the robot's name, so the convention
// is part of the product's contract with the viewer rather than a detail of any
// one visualizer.

// The robot's body: its position and its attitude.
inline std::string robotBodyFrame(const std::string& robot_name) {
    return robot_name + "/base_link";
}

// A robot's name label hangs a fixed height above it, and must stay there. The
// body frame cannot carry it: that frame rolls and pitches with the vehicle, so
// an aircraft at thirty degrees of bank would drag its own name a quarter of a
// metre off to the side and tilt the text with it.
//
// This frame follows the robot's position and never its attitude, so anything
// anchored to it keeps a fixed offset in world terms. It exists so that a
// label's position can travel as a transform -- which the viewer re-resolves on
// every rendered frame -- instead of as a message that has to be retransmitted
// before the label can move.
inline std::string robotLabelFrame(const std::string& robot_name) {
    return robot_name + "/label";
}

} // namespace xgc2_robot_visualization
