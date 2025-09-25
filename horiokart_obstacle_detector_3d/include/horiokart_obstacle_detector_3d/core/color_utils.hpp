// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

namespace obstacle_detector
{
// Unpack a packed float RGB (IEEE 754 representation of a uint32 packed as float)
// into r,g,b components in the range [0,1].
void unpackFloatRGB(float rgb_float, double & r, double & g, double & b);

// Convert RGB (r,g,b in [0,1]) to HSV. h in degrees [0,360), s and v in [0,1].
void rgbToHsv(double r, double g, double b, double & h, double & s, double & v);

}  // namespace obstacle_detector
