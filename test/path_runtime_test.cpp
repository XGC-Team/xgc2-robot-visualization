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

TEST(PathRuntime, BulkAppendPruningPreservesTheContinuityAnchorAndQuaternion) {
    BoundedPathRuntime path("world", PathRuntimeConfig{10.0, 30.0, 500});
    for (unsigned int index = 0; index <= 120; ++index) {
        auto value = pose(index, 3.25);
        value.orientation.z = 0.6;
        value.orientation.w = 0.8;
        ASSERT_TRUE(path.append(ros::Time(10 + index / 10, (index % 10) * 100000000U), value));
    }
    ASSERT_EQ(path.message().poses.size(), 121U);
    const auto anchor = path.message().poses.back();
    ASSERT_TRUE(path.append(ros::Time(100, 0), pose(999.0, 4.0)));
    ASSERT_EQ(path.message().poses.size(), 2U);
    const auto& retained = path.message().poses.front();
    EXPECT_EQ(retained.header.stamp, anchor.header.stamp);
    EXPECT_EQ(retained.header.frame_id, "world");
    EXPECT_DOUBLE_EQ(retained.pose.position.x, 120.0);
    EXPECT_DOUBLE_EQ(retained.pose.position.z, 3.25);
    EXPECT_DOUBLE_EQ(retained.pose.orientation.z, 0.6);
    EXPECT_DOUBLE_EQ(retained.pose.orientation.w, 0.8);
    EXPECT_EQ(path.message().header.stamp, ros::Time(100, 0));
    EXPECT_DOUBLE_EQ(path.message().poses.back().pose.position.x, 999.0);
}

TEST(PathRuntime, BulkExpiryPreservesEveryBoundarySampleAndItsPose) {
    BoundedPathRuntime path("world", PathRuntimeConfig{10.0, 30.0, 500});
    for (unsigned int index = 0; index <= 120; ++index) {
        auto value = pose(index, 3.25);
        value.orientation.z = 0.6;
        value.orientation.w = 0.8;
        ASSERT_TRUE(path.append(ros::Time(10 + index / 10, (index % 10) * 100000000U), value));
    }
    ASSERT_TRUE(path.expire(ros::Time(45, 0)));
    ASSERT_EQ(path.message().poses.size(), 71U);
    for (unsigned int index = 50; index <= 120; ++index) {
        const auto& sample = path.message().poses[index - 50];
        EXPECT_EQ(sample.header.stamp, ros::Time(10 + index / 10, (index % 10) * 100000000U));
        EXPECT_EQ(sample.header.frame_id, "world");
        EXPECT_DOUBLE_EQ(sample.pose.position.x, index);
        EXPECT_DOUBLE_EQ(sample.pose.position.z, 3.25);
        EXPECT_DOUBLE_EQ(sample.pose.orientation.z, 0.6);
        EXPECT_DOUBLE_EQ(sample.pose.orientation.w, 0.8);
    }
    EXPECT_EQ(path.message().header.stamp, ros::Time(22, 0));
    EXPECT_FALSE(path.expire(ros::Time(45, 0)));
    ASSERT_TRUE(path.expire(ros::Time(52, 0)));
    ASSERT_EQ(path.message().poses.size(), 1U);
    EXPECT_EQ(path.message().poses.front().header.stamp, ros::Time(22, 0));
    ASSERT_TRUE(path.expire(ros::Time(52, 1)));
    EXPECT_TRUE(path.message().poses.empty());
    EXPECT_EQ(path.message().header.stamp, ros::Time(52, 1));
}

TEST(PathRuntime, CountOnlyPruningDoesNotChangeSamplingOrPoseOrder) {
    BoundedPathRuntime path("world", PathRuntimeConfig{10.0, 0.0, 4});
    for (unsigned int index = 0; index < 20; ++index) {
        ASSERT_TRUE(path.append(ros::Time(10 + index / 10, (index % 10) * 100000000U),
                                pose(index, 2.0)));
    }
    ASSERT_EQ(path.message().poses.size(), 4U);
    for (unsigned int offset = 0; offset < 4; ++offset) {
        EXPECT_DOUBLE_EQ(path.message().poses[offset].pose.position.x, 16 + offset);
    }
    EXPECT_FALSE(path.expire(ros::Time(100, 0)));
    EXPECT_FALSE(path.append(ros::Time(11, 950000000), pose(20.0, 2.0)));
    EXPECT_EQ(path.message().poses.size(), 4U);
}

}  // namespace
}  // namespace xgc2_robot_visualization

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
