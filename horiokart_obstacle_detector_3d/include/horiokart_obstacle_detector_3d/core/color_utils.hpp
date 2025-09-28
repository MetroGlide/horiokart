// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

namespace obstacle_detector {
// パックされた float (IEEE 754 表現で uint32 を float に詰めた形式) から
// r,g,b 成分 ([0,1]) を取り出します。
void unpackFloatRGB(float rgb_float, double &r, double &g, double &b);

// RGB (r,g,b ∈ [0,1]) を HSV に変換します。h は度単位 [0,360)、s,v は [0,1]。
void rgbToHsv(double r, double g, double b, double &h, double &s, double &v);

} // namespace obstacle_detector
