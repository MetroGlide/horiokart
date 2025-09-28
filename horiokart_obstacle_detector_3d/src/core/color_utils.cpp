// SPDX-License-Identifier: Apache-2.0

#include "horiokart_obstacle_detector_3d/core/color_utils.hpp"

#include <cmath>
#include <cstdint>

namespace obstacle_detector {
void unpackFloatRGB(float rgb_float, double &r, double &g, double &b) {
  uint32_t u = *reinterpret_cast<uint32_t *>(&rgb_float);
  uint8_t ri = (u >> 16) & 0xFF;
  uint8_t gi = (u >> 8) & 0xFF;
  uint8_t bi = u & 0xFF;
  r = ri / 255.0;
  g = gi / 255.0;
  b = bi / 255.0;
}

void rgbToHsv(double r, double g, double b, double &h, double &s, double &v) {
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
    h = 60.0 * std::fmod(((g - b) / d), 6.0);
  } else if (mx == g) {
    h = 60.0 * (((b - r) / d) + 2.0);
  } else {
    h = 60.0 * (((r - g) / d) + 4.0);
  }
  if (h < 0.0) {
    h += 360.0;
  }
}

} // namespace obstacle_detector
