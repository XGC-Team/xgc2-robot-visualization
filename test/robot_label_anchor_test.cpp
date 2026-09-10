// A robot's name label is the one thing in the scene that is anchored rather
// than positioned: the viewer redraws it wherever its frame currently is. That
// only stays correct if the anchor frame refuses to inherit the vehicle's
// attitude. If it ever did, a banking aircraft would carry its own name off to
// one side and tilt the text with it -- which is exactly what publishing the
// label in the body frame would have done.

#include "xgc2_robot_visualization/fs150_uav_visualizer.hpp"
#include "xgc2_robot_visualization/mecanum_ugv_visualizer.hpp"
#include "xgc2_robot_visualization/robot_frames.hpp"
#include "xgc2_robot_visualization/scout_ugv_visualizer.hpp"

#include <cmath>
#include <string>
#include <set>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <gtest/gtest.h>
#include <visualization_msgs/MarkerArray.h>

namespace xgc2_robot_visualization {
namespace {

const geometry_msgs::TransformStamped* findTransform(
    const std::vector<geometry_msgs::TransformStamped>& transforms, const std::string& child) {
    for (const geometry_msgs::TransformStamped& transform : transforms) {
        if (transform.child_frame_id == child) {
            return &transform;
        }
    }
    return nullptr;
}

const visualization_msgs::Marker* findLabel(const visualization_msgs::MarkerArray& markers) {
    for (const visualization_msgs::Marker& marker : markers.markers) {
        if (marker.type == visualization_msgs::Marker::TEXT_VIEW_FACING) {
            return &marker;
        }
    }
    return nullptr;
}

// Roll 30 degrees and yaw 90: an attitude that would move the label visibly if
// the anchor inherited any of it.
geometry_msgs::Quaternion bankedAttitude() {
    const double roll = 30.0 * M_PI / 180.0;
    const double yaw = 90.0 * M_PI / 180.0;
    geometry_msgs::Quaternion attitude;
    attitude.w = std::cos(roll / 2.0) * std::cos(yaw / 2.0);
    attitude.x = std::sin(roll / 2.0) * std::cos(yaw / 2.0);
    attitude.y = std::sin(roll / 2.0) * std::sin(yaw / 2.0);
    attitude.z = std::cos(roll / 2.0) * std::sin(yaw / 2.0);
    return attitude;
}

void expectUprightAnchorAbove(const geometry_msgs::TransformStamped& anchor,
                              const geometry_msgs::Pose& pose, double height) {
    EXPECT_DOUBLE_EQ(anchor.transform.translation.x, pose.position.x);
    EXPECT_DOUBLE_EQ(anchor.transform.translation.y, pose.position.y);
    EXPECT_DOUBLE_EQ(anchor.transform.translation.z, pose.position.z + height);
    // Identity, not the vehicle's attitude. This is the whole guarantee.
    EXPECT_DOUBLE_EQ(anchor.transform.rotation.x, 0.0);
    EXPECT_DOUBLE_EQ(anchor.transform.rotation.y, 0.0);
    EXPECT_DOUBLE_EQ(anchor.transform.rotation.z, 0.0);
    EXPECT_DOUBLE_EQ(anchor.transform.rotation.w, 1.0);
}

TEST(RobotLabelAnchor, MultirotorLabelStaysOverheadWhileBanking) {
    Fs150UavVisualizer visualizer{Fs150UavVisualizer::Config{}};
    UavVisualState state;
    state.name = "uav3";
    state.pose.position.x = 4.0;
    state.pose.position.y = -2.5;
    state.pose.position.z = 12.0;
    state.pose.orientation = bankedAttitude();
    state.stamp = ros::Time(7, 0);

    visualization_msgs::MarkerArray markers;
    std::vector<geometry_msgs::TransformStamped> transforms;
    visualizer.append(state, &markers, &transforms);

    const geometry_msgs::TransformStamped* anchor =
        findTransform(transforms, robotLabelFrame("uav3"));
    ASSERT_NE(anchor, nullptr) << "the label has no anchor to follow";
    EXPECT_EQ(anchor->header.frame_id, "world");
    expectUprightAnchorAbove(*anchor, state.pose, 0.55);

    // The body frame does carry the attitude; the two must not be confused.
    const geometry_msgs::TransformStamped* body =
        findTransform(transforms, robotBodyFrame("uav3"));
    ASSERT_NE(body, nullptr);
    EXPECT_DOUBLE_EQ(body->transform.rotation.w, state.pose.orientation.w);

    const visualization_msgs::Marker* label = findLabel(markers);
    ASSERT_NE(label, nullptr);
    EXPECT_EQ(label->header.frame_id, robotLabelFrame("uav3"));
    // The offset lives in the anchor now, so the marker sits at its origin.
    EXPECT_DOUBLE_EQ(label->pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(label->pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(label->pose.position.z, 0.0);
}

TEST(RobotLabelAnchor, ScoutLabelStaysOverheadOnASlope) {
    ScoutUgvVisualizer visualizer{ScoutUgvVisualizer::Config{}};
    UgvVisualState state;
    state.name = "ugv2";
    state.pose.position.x = -3.0;
    state.pose.position.y = 1.0;
    state.pose.position.z = 0.2;
    state.pose.orientation = bankedAttitude();
    state.stamp = ros::Time(7, 0);

    visualization_msgs::MarkerArray markers;
    std::vector<geometry_msgs::TransformStamped> transforms;
    visualizer.append(state, &markers, &transforms);

    const geometry_msgs::TransformStamped* anchor =
        findTransform(transforms, robotLabelFrame("ugv2"));
    ASSERT_NE(anchor, nullptr);
    expectUprightAnchorAbove(*anchor, state.pose, 0.65);

    const visualization_msgs::Marker* label = findLabel(markers);
    ASSERT_NE(label, nullptr);
    EXPECT_EQ(label->header.frame_id, robotLabelFrame("ugv2"));
}

TEST(RobotLabelAnchor, MecanumLabelStaysOverheadOnASlope) {
    MecanumUgvVisualizer visualizer{MecanumUgvVisualizer::Config{}};
    MecanumVisualState state;
    state.name = "mecanum1";
    state.pose.position.x = 0.5;
    state.pose.position.y = 6.0;
    state.pose.position.z = 0.05;
    state.pose.orientation = bankedAttitude();
    state.stamp = ros::Time(7, 0);

    visualization_msgs::MarkerArray markers;
    std::vector<geometry_msgs::TransformStamped> transforms;
    visualizer.append(state, &markers, &transforms);

    const geometry_msgs::TransformStamped* anchor =
        findTransform(transforms, robotLabelFrame("mecanum1"));
    ASSERT_NE(anchor, nullptr);
    expectUprightAnchorAbove(*anchor, state.pose, 0.32);

    const visualization_msgs::Marker* label = findLabel(markers);
    ASSERT_NE(label, nullptr);
    EXPECT_EQ(label->header.frame_id, robotLabelFrame("mecanum1"));
}

// The anchor is world-parented on purpose: the publisher forwards every
// world-parented transform at the pose rate and gates the rest down to the
// joint rate. A label parented to its own robot would animate at 5 Hz.
TEST(RobotLabelAnchor, AnchorIsWorldParentedSoItRidesThePoseRate) {
    Fs150UavVisualizer visualizer{Fs150UavVisualizer::Config{}};
    UavVisualState state;
    state.name = "uav1";
    state.pose.orientation.w = 1.0;
    state.stamp = ros::Time(7, 0);

    visualization_msgs::MarkerArray markers;
    std::vector<geometry_msgs::TransformStamped> transforms;
    visualizer.append(state, &markers, &transforms);

    const geometry_msgs::TransformStamped* anchor =
        findTransform(transforms, robotLabelFrame("uav1"));
    ASSERT_NE(anchor, nullptr);
    EXPECT_EQ(anchor->header.frame_id, "world");
    EXPECT_NE(anchor->header.frame_id, robotBodyFrame("uav1"));
}

template <typename Visualizer, typename State>
void expectIsolatedDisplayTree(const std::string& name) {
    Visualizer visualizer{typename Visualizer::Config{}};
    State state;
    state.name = name;
    state.pose.orientation.w = 1.0;
    state.stamp = ros::Time(7, 0);
    visualization_msgs::MarkerArray markers;
    std::vector<geometry_msgs::TransformStamped> transforms;
    visualizer.append(state, &markers, &transforms);
    const std::string prefix = "xgc/robots/" + name + "/";
    std::set<std::string> children;
    ASSERT_GT(transforms.size(), 2u);
    for (const auto& transform : transforms) {
        EXPECT_EQ(transform.child_frame_id.find(prefix), 0u);
        EXPECT_TRUE(children.insert(transform.child_frame_id).second);
        EXPECT_TRUE(transform.header.frame_id == "world" ||
                    transform.header.frame_id.find(prefix) == 0u);
        EXPECT_NE(transform.child_frame_id.find(name + "/"), 0u);
    }
    EXPECT_EQ(children.count(prefix + "base_link"), 1u);
    for (const auto& marker : markers.markers) {
        EXPECT_TRUE(marker.header.frame_id == "world" ||
                    marker.header.frame_id.find(prefix) == 0u);
    }
}

TEST(RobotDisplayTree, EveryBodyJointAndLabelIsIsolatedFromPlantAndOnboardFrames) {
    expectIsolatedDisplayTree<Fs150UavVisualizer, UavVisualState>("uav1");
    expectIsolatedDisplayTree<ScoutUgvVisualizer, UgvVisualState>("ugv1");
    expectIsolatedDisplayTree<MecanumUgvVisualizer, MecanumVisualState>("mecanum1");
}

} // namespace
} // namespace xgc2_robot_visualization

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
