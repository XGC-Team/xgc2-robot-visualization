#include "xgc2_robot_visualization/path_history.hpp"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <gtest/gtest.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "xgc2_robot_visualization/fs150_uav_visualizer.hpp"
#include "xgc2_robot_visualization/mecanum_ugv_visualizer.hpp"
#include "xgc2_robot_visualization/scout_ugv_visualizer.hpp"

namespace xgc2_robot_visualization {
namespace {

const visualization_msgs::Marker *findNs(const visualization_msgs::MarkerArray &markers,
                                         const std::string &ns) {
    for (const visualization_msgs::Marker &marker : markers.markers) {
        if (marker.ns == ns) {
            return &marker;
        }
    }
    return nullptr;
}

TEST(PathHistory, BudgetIsDurationTimesRateNotAFewFrames) {
    EXPECT_EQ(pathHistoryPointBudget(kDefaultPathHistoryDurationSec, kDefaultPathPublishRateHz),
              601);
    EXPECT_EQ(pathHistoryPointBudget(15.0, 10.0), 151);
    EXPECT_GE(pathHistoryPointBudget(kDefaultPathHistoryDurationSec, kDefaultPathPublishRateHz),
              100);
}

TEST(PathHistory, TimeWindowKeepsSixtySecondsAtTenHertz) {
    std::deque<PathSample> path;
    geometry_msgs::Point point;
    point.z = 0.15;
    const int budget = pathHistoryPointBudget(60.0, 10.0);
    for (int index = 0; index < budget + 50; ++index) {
        const ros::Time stamp(static_cast<uint32_t>(index / 10),
                              static_cast<uint32_t>((index % 10) * 100000000U));
        point.x = 0.1 * index;
        pushPathHistory(&path, stamp, point, 10.0, 60.0, budget);
    }
    ASSERT_FALSE(path.empty());
    EXPECT_EQ(static_cast<int>(path.size()), budget);
    EXPECT_LE((path.back().stamp - path.front().stamp).toSec(), 60.0 + 1.0 / 10.0);
    EXPECT_DOUBLE_EQ(path.back().point.z, 0.15);
}

TEST(PathHistory, LateSubscriberStillReceivesTheRetainedWindow) {
    std::deque<PathSample> path;
    geometry_msgs::Point point;
    for (int index = 1; index <= 20; ++index) {
        point.x = index;
        pushPathHistory(&path, ros::Time(index, 0), point, 1.0, 60.0, 61);
    }
    std::vector<geometry_msgs::Point> points;
    assignPathHistoryPoints(path, &points);
    EXPECT_EQ(points.size(), 20U);
    EXPECT_DOUBLE_EQ(points.front().x, 1.0);
    EXPECT_DOUBLE_EQ(points.back().x, 20.0);
}

TEST(Fs150Path, GroundAndTakeoffZMatchWorldPose) {
    Fs150UavVisualizer::Config config;
    config.path_publish_rate = 10.0;
    config.path_history_duration_sec = 60.0;
    Fs150UavVisualizer visualizer(config);
    UavVisualState state;
    state.name = "uav1";
    state.pose.orientation.w = 1.0;
    state.pose.position.z = 0.15;
    visualization_msgs::MarkerArray markers;
    std::vector<geometry_msgs::TransformStamped> transforms;
    for (int index = 0; index < 3; ++index) {
        state.stamp = ros::Time(index, 0);
        state.pose.position.x = 0.2 * index;
        markers.markers.clear();
        transforms.clear();
        visualizer.append(state, &markers, &transforms);
    }
    const visualization_msgs::Marker *path = findNs(markers, "uav1_actual_path");
    ASSERT_NE(path, nullptr);
    ASSERT_GE(path->points.size(), 2U);
    EXPECT_DOUBLE_EQ(path->points.back().z, 0.15);
    EXPECT_EQ(path->header.frame_id, "world");

    state.stamp = ros::Time(3, 0);
    state.pose.position.z = 1.5;
    markers.markers.clear();
    transforms.clear();
    visualizer.append(state, &markers, &transforms);
    path = findNs(markers, "uav1_actual_path");
    ASSERT_NE(path, nullptr);
    EXPECT_DOUBLE_EQ(path->points.back().z, 1.5);
}

TEST(ScoutPath, SharesTimeWindowAndKeepsGroundZ) {
    ScoutUgvVisualizer::Config config;
    config.path_publish_rate = 10.0;
    config.path_history_duration_sec = 60.0;
    ScoutUgvVisualizer visualizer(config);
    UgvVisualState state;
    state.name = "ugv1";
    state.pose.orientation.w = 1.0;
    state.pose.position.z = 0.08;
    visualization_msgs::MarkerArray markers;
    std::vector<geometry_msgs::TransformStamped> transforms;
    for (int index = 0; index < 5; ++index) {
        state.stamp = ros::Time(index, 0);
        state.pose.position.x = 0.3 * index;
        markers.markers.clear();
        transforms.clear();
        visualizer.append(state, &markers, &transforms);
    }
    const visualization_msgs::Marker *path = findNs(markers, "ugv1_actual_path");
    ASSERT_NE(path, nullptr);
    EXPECT_GE(path->points.size(), 2U);
    EXPECT_DOUBLE_EQ(path->points.front().z, 0.08);
    EXPECT_DOUBLE_EQ(path->points.back().z, 0.08);
}

TEST(MecanumPath, SharesTimeWindowAndKeepsGroundZ) {
    MecanumUgvVisualizer::Config config;
    config.path_publish_rate = 10.0;
    config.path_history_duration_sec = 60.0;
    MecanumUgvVisualizer visualizer(config);
    MecanumVisualState state;
    state.name = "ugv2";
    state.pose.orientation.w = 1.0;
    state.pose.position.z = 0.05;
    visualization_msgs::MarkerArray markers;
    std::vector<geometry_msgs::TransformStamped> transforms;
    for (int index = 0; index < 5; ++index) {
        state.stamp = ros::Time(index, 0);
        state.pose.position.x = 0.25 * index;
        markers.markers.clear();
        transforms.clear();
        visualizer.append(state, &markers, &transforms);
    }
    const visualization_msgs::Marker *path = findNs(markers, "ugv2_actual_path");
    ASSERT_NE(path, nullptr);
    EXPECT_GE(path->points.size(), 2U);
    EXPECT_DOUBLE_EQ(path->points.back().z, 0.05);
}

TEST(PathHistory, ThreeKindsShareTheSameDefaultWindow) {
    EXPECT_DOUBLE_EQ(Fs150UavVisualizer::Config{}.path_history_duration_sec,
                     kDefaultPathHistoryDurationSec);
    EXPECT_DOUBLE_EQ(ScoutUgvVisualizer::Config{}.path_history_duration_sec,
                     kDefaultPathHistoryDurationSec);
    EXPECT_DOUBLE_EQ(MecanumUgvVisualizer::Config{}.path_history_duration_sec,
                     kDefaultPathHistoryDurationSec);
    EXPECT_DOUBLE_EQ(Fs150UavVisualizer::Config{}.path_publish_rate, kDefaultPathPublishRateHz);
    EXPECT_DOUBLE_EQ(ScoutUgvVisualizer::Config{}.path_publish_rate, kDefaultPathPublishRateHz);
    EXPECT_DOUBLE_EQ(MecanumUgvVisualizer::Config{}.path_publish_rate, kDefaultPathPublishRateHz);
    Fs150UavVisualizer fs{Fs150UavVisualizer::Config{}};
    ScoutUgvVisualizer scout{ScoutUgvVisualizer::Config{}};
    MecanumUgvVisualizer mecanum{MecanumUgvVisualizer::Config{}};
    (void)fs;
    (void)scout;
    (void)mecanum;
}

TEST(PathHistory, InstancesAndNamesDoNotShareStorage) {
    Fs150UavVisualizer::Config config;
    config.path_publish_rate = 10.0;
    config.path_history_duration_sec = 60.0;
    Fs150UavVisualizer first(config);
    Fs150UavVisualizer second(config);
    UavVisualState a;
    a.name = "uav-a";
    a.pose.orientation.w = 1.0;
    a.pose.position.x = 1.0;
    a.stamp = ros::Time(1, 0);
    UavVisualState b = a;
    b.name = "uav-b";
    b.pose.position.x = 9.0;
    visualization_msgs::MarkerArray markers;
    std::vector<geometry_msgs::TransformStamped> transforms;
    first.append(a, &markers, &transforms);
    markers.markers.clear();
    transforms.clear();
    first.append(b, &markers, &transforms);
    a.stamp = ros::Time(2, 0);
    a.pose.position.x = 1.5;
    markers.markers.clear();
    transforms.clear();
    first.append(a, &markers, &transforms);
    const visualization_msgs::Marker *path_a = findNs(markers, "uav-a_actual_path");
    ASSERT_NE(path_a, nullptr);
    ASSERT_EQ(path_a->points.size(), 2U);
    EXPECT_DOUBLE_EQ(path_a->points.front().x, 1.0);
    EXPECT_DOUBLE_EQ(path_a->points.back().x, 1.5);
    for (const geometry_msgs::Point &point : path_a->points) {
        EXPECT_NE(point.x, 9.0);
    }

    UavVisualState c = a;
    c.name = "uav-c";
    c.pose.position.x = 4.0;
    c.stamp = ros::Time(3, 0);
    markers.markers.clear();
    transforms.clear();
    second.append(c, &markers, &transforms);
    c.stamp = ros::Time(4, 0);
    c.pose.position.x = 4.5;
    markers.markers.clear();
    transforms.clear();
    second.append(c, &markers, &transforms);
    const visualization_msgs::Marker *path_c = findNs(markers, "uav-c_actual_path");
    ASSERT_NE(path_c, nullptr);
    ASSERT_EQ(path_c->points.size(), 2U);
    EXPECT_DOUBLE_EQ(path_c->points.front().x, 4.0);
    EXPECT_DOUBLE_EQ(path_c->points.back().x, 4.5);
    EXPECT_EQ(findNs(markers, "uav-a_actual_path"), nullptr);
}

template <typename Visualizer, typename State>
void expectBoundedWindow(Visualizer *visualizer, State *state, const std::string &ns) {
    visualization_msgs::MarkerArray markers;
    std::vector<geometry_msgs::TransformStamped> transforms;
    const int budget = pathHistoryPointBudget(60.0, 10.0);
    for (int index = 0; index < budget + 40; ++index) {
        state->stamp = ros::Time(static_cast<uint32_t>(index / 10),
                                 static_cast<uint32_t>((index % 10) * 100000000U));
        state->pose.position.x = 0.1 * index;
        markers.markers.clear();
        transforms.clear();
        visualizer->append(*state, &markers, &transforms);
    }
    const visualization_msgs::Marker *path = findNs(markers, ns);
    ASSERT_NE(path, nullptr);
    EXPECT_EQ(static_cast<int>(path->points.size()), budget);
}

TEST(ScoutPath, EnforcesSixtySecondCapacity) {
    ScoutUgvVisualizer::Config config;
    config.path_publish_rate = 10.0;
    config.path_history_duration_sec = 60.0;
    ScoutUgvVisualizer visualizer(config);
    UgvVisualState state;
    state.name = "ugv1";
    state.pose.orientation.w = 1.0;
    state.pose.position.z = 0.08;
    expectBoundedWindow(&visualizer, &state, "ugv1_actual_path");
}

TEST(MecanumPath, EnforcesSixtySecondCapacity) {
    MecanumUgvVisualizer::Config config;
    config.path_publish_rate = 10.0;
    config.path_history_duration_sec = 60.0;
    MecanumUgvVisualizer visualizer(config);
    MecanumVisualState state;
    state.name = "ugv2";
    state.pose.orientation.w = 1.0;
    state.pose.position.z = 0.05;
    expectBoundedWindow(&visualizer, &state, "ugv2_actual_path");
}

TEST(Fs150Path, EnforcesSixtySecondCapacity) {
    Fs150UavVisualizer::Config config;
    config.path_publish_rate = 10.0;
    config.path_history_duration_sec = 60.0;
    Fs150UavVisualizer visualizer(config);
    UavVisualState state;
    state.name = "uav1";
    state.pose.orientation.w = 1.0;
    state.pose.position.z = 0.15;
    expectBoundedWindow(&visualizer, &state, "uav1_actual_path");
}

}  // namespace
}  // namespace xgc2_robot_visualization

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
