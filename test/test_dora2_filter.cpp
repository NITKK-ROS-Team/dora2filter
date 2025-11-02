#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "dora2filter/dora2_filter.hpp"

namespace
{

sensor_msgs::msg::LaserScan make_scan(const std::vector<float> & ranges)
{
  sensor_msgs::msg::LaserScan scan;
  constexpr float kAngleIncrement = 0.1F;
  scan.header.frame_id = "laser";
  scan.angle_min = 0.0F;
  scan.angle_max = static_cast<float>(ranges.size() - 1) * kAngleIncrement;
  scan.angle_increment = kAngleIncrement;
  scan.range_min = 0.0F;
  scan.range_max = 10.0F;
  scan.ranges = ranges;
  return scan;
}

void expect_nan_or_value(float actual, float expected)
{
  if (std::isnan(expected)) {
    EXPECT_TRUE(std::isnan(actual));
  } else {
    EXPECT_NEAR(actual, expected, 1e-6);
  }
}

}  // namespace

TEST(Dora2Filter, FiltersAlignedNeighbors1)
{
  auto node = std::make_shared<rclcpp::Node>("test_dora2_filter_aligned");
  dora2filter::Dora2Filter filter;
  filter.configure(
    "", "dora2_filter",
    node->get_node_logging_interface(), node->get_node_parameters_interface());

  sensor_msgs::msg::LaserScan input = make_scan({0.9F, 1.0F, 0.5F, 0.9F, 0.9F});
  sensor_msgs::msg::LaserScan output;

  ASSERT_TRUE(filter.update(input, output));

  std::vector<float> expected = {
    0.9F,
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN(),
    0.9F,
    0.9F};

  ASSERT_EQ(output.ranges.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    expect_nan_or_value(output.ranges[i], expected[i]);
  }
}

TEST(Dora2Filter, HonorsThresholdParameter1)
{
  auto node = std::make_shared<rclcpp::Node>("test_dora2_filter_threshold");
  node->declare_parameter("custom.threshold", 1.1);

  dora2filter::Dora2Filter filter;
  filter.configure(
    "custom",
    "dora2_filter",
    node->get_node_logging_interface(),
    node->get_node_parameters_interface());

  sensor_msgs::msg::LaserScan input = make_scan({0.9F, 1.0F, 0.5F, 0.9F, 0.9F});
  sensor_msgs::msg::LaserScan output;

  ASSERT_TRUE(filter.update(input, output));

  std::vector<float> expected = {0.9F, 1.0F, 0.5F, 0.9F, 0.9F};

  ASSERT_EQ(output.ranges.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    expect_nan_or_value(output.ranges[i], expected[i]);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
