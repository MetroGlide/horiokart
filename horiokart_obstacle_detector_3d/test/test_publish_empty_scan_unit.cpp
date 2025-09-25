#include <gtest/gtest.h>

#include <cmath>
#include <sensor_msgs/msg/laser_scan.hpp>

// Helper that mirrors node behavior for empty scan creation
static sensor_msgs::msg::LaserScan makeEmptyScan(
  double angle_min, double angle_max, double angle_inc, double range_max)
{
  sensor_msgs::msg::LaserScan s;
  s.angle_min = angle_min;
  s.angle_max = angle_max;
  s.angle_increment = angle_inc;
  s.range_max = range_max;
  int buckets = static_cast<int>(std::ceil((angle_max - angle_min) / angle_inc));
  if (buckets <= 0) {
    buckets = 1;
  }
  s.ranges.assign(buckets, std::numeric_limits<float>::infinity());
  return s;
}

TEST(PublishEmptyScanUnit, RangeCountAndInf)
{
  auto s = makeEmptyScan(-1.57, 1.57, 0.01, 10.0);
  EXPECT_GT(s.ranges.size(), 0u);
  for (auto r : s.ranges) {
    EXPECT_TRUE(std::isinf(r));
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
