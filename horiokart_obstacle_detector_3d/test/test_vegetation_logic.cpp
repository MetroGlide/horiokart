#include <gtest/gtest.h>

#include <cmath>

// Use shared core helpers
#include "horiokart_obstacle_detector_3d/core/color_utils.hpp"

TEST(VegetationLogic, GreenDetection)
{
  // create a packed RGB float for greenish color (R=200,G=255,B=50) -> hue ~76 deg
  uint8_t r = 200, g = 255, b = 50;
  uint32_t u = (r << 16) | (g << 8) | b;
  float packed = *reinterpret_cast<float *>(&u);

  double rd, gd, bd, h, s, v;
  obstacle_detector::unpackFloatRGB(packed, rd, gd, bd);
  obstacle_detector::rgbToHsv(rd, gd, bd, h, s, v);

  // vegetation ranges used in node: h in [35,85], s>=0.3, v>=0.2
  EXPECT_GE(h, 35.0);
  EXPECT_LE(h, 85.0);
  EXPECT_GE(s, 0.3);
  EXPECT_GE(v, 0.2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
