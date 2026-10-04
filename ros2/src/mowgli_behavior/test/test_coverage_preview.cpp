#include <gtest/gtest.h>

#include "mowgli_behavior/coverage_preview.hpp"

namespace
{
nav_msgs::msg::Path path(std::initializer_list<std::pair<double, double>> points)
{
  nav_msgs::msg::Path result;
  for (const auto & [x, y] : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    result.poses.push_back(pose);
  }
  return result;
}
}

TEST(CoveragePreview, SimplifiesWithinToleranceAndPreservesEndpoints)
{
  const auto input = path({{0, 0}, {0.25, 0.001}, {0.5, 0.002}, {1, 0}});
  const auto preview = mowgli_behavior::makeCoveragePreview({input}, 0.01);
  ASSERT_EQ(preview.subpath_offsets, (std::vector<std::uint32_t>{0, 2}));
  ASSERT_EQ(preview.xy.size(), 4u);
  EXPECT_FLOAT_EQ(preview.xy[0], 0.0f);
  EXPECT_FLOAT_EQ(preview.xy[1], 0.0f);
  EXPECT_FLOAT_EQ(preview.xy[2], 1.0f);
  EXPECT_FLOAT_EQ(preview.xy[3], 0.0f);
  EXPECT_EQ(input.poses.size(), 4u);  // preview never mutates the executor path
}

TEST(CoveragePreview, KeepsSubpathsSeparateAndUsesFloat32Coordinates)
{
  const auto preview = mowgli_behavior::makeCoveragePreview(
    {path({{0.123456789, 0}, {1, 0}}), path({{10, 5}, {11, 5}})});
  EXPECT_EQ(preview.subpath_offsets, (std::vector<std::uint32_t>{0, 2, 4}));
  EXPECT_EQ(preview.xy.size(), 8u);
  EXPECT_FLOAT_EQ(preview.xy.front(), static_cast<float>(0.123456789));
  EXPECT_FLOAT_EQ(preview.xy[4], 10.0f);
}

TEST(CoveragePreview, RetainsVerticesBeyondTolerance)
{
  const auto preview = mowgli_behavior::makeCoveragePreview(
    {path({{0, 0}, {0.5, 0.02}, {1, 0}})}, 0.01);
  EXPECT_EQ(preview.subpath_offsets, (std::vector<std::uint32_t>{0, 3}));
}
