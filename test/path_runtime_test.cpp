#include "xgc2_robot_visualization/path_runtime.hpp"

#include <stdexcept>

#include <gtest/gtest.h>

namespace xgc2_robot_visualization {
namespace {

geometry_msgs::Pose pose(double x, double z) {
    geometry_msgs::Pose value;
    value.position.x = x;
    value.position.z = z;
    value.orientation.w = 1.0;
    return value;
}

TEST(PathRuntime, GrowsWithMonotonicSourceTimeAndPreservesWorldPose) {
    BoundedPathRuntime path("world", PathRuntimeConfig{10.0, kDefaultPathHistoryDurationSec, 32});
    ASSERT_TRUE(path.append(ros::Time(10, 0), pose(1.0, 3.25)));
    ASSERT_TRUE(path.append(ros::Time(10, 100000000), pose(2.0, 3.5)));
    const nav_msgs::Path& message = path.message();
    ASSERT_EQ(message.poses.size(), 2U);
    EXPECT_EQ(message.header.frame_id, "world");
    EXPECT_EQ(message.header.stamp, ros::Time(10, 100000000));
    EXPECT_EQ(message.poses.front().header.stamp, ros::Time(10, 0));
    EXPECT_EQ(message.poses.back().header.stamp, ros::Time(10, 100000000));
    EXPECT_DOUBLE_EQ(message.poses.front().pose.position.z, 3.25);
    EXPECT_DOUBLE_EQ(message.poses.back().pose.position.z, 3.5);
}

TEST(PathRuntime, DecimatesAndRejectsDuplicateTime) {
    BoundedPathRuntime path("world", PathRuntimeConfig{5.0, kDefaultPathHistoryDurationSec, 32});
    ASSERT_TRUE(path.append(ros::Time(10, 0), pose(1.0, 0.0)));
    EXPECT_FALSE(path.append(ros::Time(10, 100000000), pose(2.0, 0.0)));
    EXPECT_FALSE(path.append(ros::Time(10, 0), pose(3.0, 0.0)));
    ASSERT_TRUE(path.append(ros::Time(10, 200000000), pose(4.0, 0.0)));
    EXPECT_FALSE(path.append(ros::Time(10, 150000000), pose(5.0, 0.0)));
    ASSERT_EQ(path.message().poses.size(), 2U);
    EXPECT_DOUBLE_EQ(path.message().poses.back().pose.position.x, 4.0);
}

TEST(PathRuntime, EnforcesPointAndTimeBoundsThenResetsOnNewEpoch) {
    BoundedPathRuntime path("world", PathRuntimeConfig{10.0, 0.25, 3});
    for (int index = 0; index < 6; ++index) {
        ASSERT_TRUE(path.append(ros::Time(10, index * 100000000U), pose(index, 0.0)));
    }
    ASSERT_LE(path.message().poses.size(), 3U);
    EXPECT_LE((path.message().poses.back().header.stamp -
               path.message().poses.front().header.stamp).toSec(),
              0.25);

    ASSERT_TRUE(path.append(ros::Time(1, 0), pose(9.0, 0.0)));
    ASSERT_EQ(path.message().poses.size(), 1U);
    EXPECT_EQ(path.message().poses.front().header.stamp, ros::Time(1, 0));
    path.reset();
    EXPECT_TRUE(path.message().poses.empty());
    EXPECT_TRUE(path.message().header.stamp.isZero());
}

TEST(PathRuntime, ResolvesOnlyRosterOwnedRelativeTopics) {
    EXPECT_EQ(namespacedPathTopic("/uav7", "path"), "/uav7/path");
    EXPECT_THROW(namespacedPathTopic("uav7", "path"), std::invalid_argument);
    EXPECT_THROW(namespacedPathTopic("/uav7", "/path"), std::invalid_argument);
}

TEST(PathRuntime, DefaultConfigIsTheProductSixSecondWindow) {
    EXPECT_DOUBLE_EQ(PathRuntimeConfig{}.max_age_sec, kDefaultPathHistoryDurationSec);
    EXPECT_DOUBLE_EQ(kDefaultPathHistoryDurationSec, 6.0);
}

TEST(PathRuntime, ExpireClipsToSixSecondsWithoutSynthesizingNewSamples) {
    BoundedPathRuntime path("world", PathRuntimeConfig{});
    ASSERT_TRUE(path.append(ros::Time(1, 0), pose(1.0, 1.5)));
    ASSERT_TRUE(path.append(ros::Time(4, 0), pose(2.0, 1.5)));
    ASSERT_TRUE(path.append(ros::Time(7, 0), pose(3.0, 1.5)));
    EXPECT_FALSE(path.expire(ros::Time(7, 0)));
    ASSERT_EQ(path.message().poses.size(), 3U);
    ASSERT_TRUE(path.expire(ros::Time(10, 0)));
    ASSERT_EQ(path.message().poses.size(), 2U);
    EXPECT_EQ(path.message().poses.front().header.stamp, ros::Time(4, 0));
    ASSERT_TRUE(path.expire(ros::Time(10, 1)));
    ASSERT_EQ(path.message().poses.size(), 1U);
    EXPECT_EQ(path.message().poses.front().header.stamp, ros::Time(7, 0));
    EXPECT_EQ(path.message().header.stamp, ros::Time(7, 0));
    EXPECT_DOUBLE_EQ(path.message().poses.front().pose.position.x, 3.0);
    ASSERT_TRUE(path.expire(ros::Time(14, 0)));
    EXPECT_TRUE(path.message().poses.empty());
}

TEST(PathRuntime, ClockRollbackClearsBufferInsteadOfKeepingCachedPoses) {
    BoundedPathRuntime path("world", PathRuntimeConfig{});
    ASSERT_TRUE(path.append(ros::Time(20, 0), pose(1.0, 0.0)));
    ASSERT_TRUE(path.append(ros::Time(21, 0), pose(2.0, 0.0)));
    ASSERT_TRUE(path.append(ros::Time(1, 0), pose(9.0, 0.0)));
    ASSERT_EQ(path.message().poses.size(), 1U);
    EXPECT_EQ(path.message().poses.front().header.stamp, ros::Time(1, 0));
    EXPECT_DOUBLE_EQ(path.message().poses.front().pose.position.x, 9.0);
}

TEST(PathRuntime, FrozenNowDoesNotExpireALiveWindow) {
    BoundedPathRuntime path("world", PathRuntimeConfig{});
    ASSERT_TRUE(path.append(ros::Time(10, 0), pose(1.0, 0.0)));
    ASSERT_TRUE(path.append(ros::Time(12, 0), pose(2.0, 0.0)));
    EXPECT_FALSE(path.expire(ros::Time(12, 0)));
    EXPECT_EQ(path.message().poses.size(), 2U);
}

}  // namespace
}  // namespace xgc2_robot_visualization

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
