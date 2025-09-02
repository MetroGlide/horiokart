#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

// Simple sanity test for coordinate conversion helpers used by the layer.
TEST(CoordinateConversion, RotationTranslation)
{
    Eigen::Vector2d pt(1.0, 0.0);
    double yaw = M_PI_2; // 90 deg
    Eigen::Rotation2D<double> rot(yaw);
    Eigen::Vector2d rotated = rot * pt;
    EXPECT_NEAR(rotated.x(), 0.0, 1e-6);
    EXPECT_NEAR(rotated.y(), 1.0, 1e-6);

    Eigen::Vector2d trans = rotated + Eigen::Vector2d(0.5, -0.5);
    EXPECT_NEAR(trans.x(), 0.5, 1e-6);
    EXPECT_NEAR(trans.y(), 0.5, 1e-6);
}
