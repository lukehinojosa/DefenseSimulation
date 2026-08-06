#include <gtest/gtest.h>

#include "sim/Vector3.hpp"

using sim::Vector3;

namespace {
constexpr double kEps = 1e-9;
}

TEST(Vector3, AddSubtractAndScale) {
    Vector3 a{1.0, 2.0, 3.0};
    Vector3 b{4.0, -5.0, 6.0};

    Vector3 sum = a + b;
    EXPECT_NEAR(sum.x, 5.0, kEps);
    EXPECT_NEAR(sum.y, -3.0, kEps);
    EXPECT_NEAR(sum.z, 9.0, kEps);

    Vector3 diff = a - b;
    EXPECT_NEAR(diff.x, -3.0, kEps);
    EXPECT_NEAR(diff.y, 7.0, kEps);
    EXPECT_NEAR(diff.z, -3.0, kEps);

    Vector3 scaled = a * 2.0;
    EXPECT_NEAR(scaled.x, 2.0, kEps);
    EXPECT_NEAR(scaled.y, 4.0, kEps);
    EXPECT_NEAR(scaled.z, 6.0, kEps);

    // Scalar-on-left overload.
    Vector3 scaledLeft = 3.0 * a;
    EXPECT_NEAR(scaledLeft.z, 9.0, kEps);
}

TEST(Vector3, DotProduct) {
    Vector3 a{1.0, 2.0, 3.0};
    Vector3 b{4.0, 5.0, 6.0};
    EXPECT_NEAR(a.dot(b), 32.0, kEps); // 4 + 10 + 18

    // Orthogonal vectors dot to zero.
    Vector3 x{1.0, 0.0, 0.0};
    Vector3 y{0.0, 1.0, 0.0};
    EXPECT_NEAR(x.dot(y), 0.0, kEps);
}

TEST(Vector3, CrossProductFollowsRightHandRule) {
    Vector3 x{1.0, 0.0, 0.0};
    Vector3 y{0.0, 1.0, 0.0};
    Vector3 z = x.cross(y);
    EXPECT_NEAR(z.x, 0.0, kEps);
    EXPECT_NEAR(z.y, 0.0, kEps);
    EXPECT_NEAR(z.z, 1.0, kEps);

    // Anti-commutativity: y x x = -z.
    Vector3 negZ = y.cross(x);
    EXPECT_NEAR(negZ.z, -1.0, kEps);

    // Cross product is orthogonal to both inputs.
    Vector3 a{2.0, -3.0, 1.0};
    Vector3 b{-1.0, 4.0, 2.0};
    Vector3 c = a.cross(b);
    EXPECT_NEAR(c.dot(a), 0.0, kEps);
    EXPECT_NEAR(c.dot(b), 0.0, kEps);
}

TEST(Vector3, MagnitudeAndNormalization) {
    Vector3 v{3.0, 4.0, 0.0};
    EXPECT_NEAR(v.magnitude(), 5.0, kEps);
    EXPECT_NEAR(v.magnitudeSquared(), 25.0, kEps);

    Vector3 n = v.normalized();
    EXPECT_NEAR(n.magnitude(), 1.0, kEps);
    EXPECT_NEAR(n.x, 0.6, kEps);
    EXPECT_NEAR(n.y, 0.8, kEps);
}

TEST(Vector3, NormalizingZeroVectorIsSafe) {
    Vector3 zero{};
    Vector3 n = zero.normalized();
    EXPECT_NEAR(n.x, 0.0, kEps);
    EXPECT_NEAR(n.y, 0.0, kEps);
    EXPECT_NEAR(n.z, 0.0, kEps);
}

TEST(Vector3, Distance) {
    Vector3 a{0.0, 0.0, 0.0};
    Vector3 b{1.0, 2.0, 2.0};
    EXPECT_NEAR(a.distanceTo(b), 3.0, kEps);
    EXPECT_NEAR(a.distanceSquaredTo(b), 9.0, kEps);
}

TEST(Vector3, ConstexprEvaluation) {
    // Confirm the core operations are usable in constant expressions.
    constexpr Vector3 a{1.0, 0.0, 0.0};
    constexpr Vector3 b{0.0, 1.0, 0.0};
    constexpr Vector3 c = a.cross(b);
    static_assert(c.z == 1.0, "cross product must be constexpr");
    static_assert(a.dot(b) == 0.0, "dot product must be constexpr");
    SUCCEED();
}
