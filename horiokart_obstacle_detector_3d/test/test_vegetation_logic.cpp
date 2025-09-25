#include <gtest/gtest.h>

#include <cmath>

// Reimplement small helpers: unpackFloatRGB and rgbToHsv to mirror node behavior
static void unpackFloatRGB(float rgb_float, double & r, double & g, double & b)
{
  uint32_t u = *reinterpret_cast<uint32_t *>(&rgb_float);
  uint8_t ri = (u >> 16) & 0xFF;
  uint8_t gi = (u >> 8) & 0xFF;
  uint8_t bi = u & 0xFF;
  r = ri / 255.0;
  g = gi / 255.0;
  b = bi / 255.0;
}

static void rgbToHsv(double r, double g, double b, double & h, double & s, double & v)
{
  double mx = std::max(r, std::max(g, b));
  double mn = std::min(r, std::min(g, b));
  v = mx;
  double d = mx - mn;
  s = (mx == 0.0) ? 0.0 : d / mx;
  if (d == 0.0) {
    h = 0.0;
    return;
  }
  if (mx == r) {
    h = 60.0 * (fmod(((g - b) / d), 6.0));
  } else if (mx == g) {
    h = 60.0 * (((b - r) / d) + 2.0);
  } else {
    h = 60.0 * (((r - g) / d) + 4.0);
  }
  if (h < 0.0) {
    h += 360.0;
  }
}

TEST(VegetationLogic, GreenDetection)
{
  // create a packed RGB float for greenish color (R=200,G=255,B=50) -> hue ~76 deg
  uint8_t r = 200, g = 255, b = 50;
  uint32_t u = (r << 16) | (g << 8) | b;
  float packed = *reinterpret_cast<float *>(&u);

  double rd, gd, bd, h, s, v;
  unpackFloatRGB(packed, rd, gd, bd);
  rgbToHsv(rd, gd, bd, h, s, v);

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
