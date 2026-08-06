#include <gtest/gtest.h>

#include "sim/BoundingBox.hpp"

using sim::BoundingBox;
using sim::Vector3;

TEST(BoundingBox, ContainsUsesHalfOpenUpperBound) {
    BoundingBox box{{0.0, 0.0, 0.0}, {10.0, 10.0, 10.0}};

    EXPECT_TRUE(box.contains({5.0, 5.0, 5.0}));
    EXPECT_TRUE(box.contains({0.0, 0.0, 0.0}));      // min face is inside
    EXPECT_FALSE(box.contains({10.0, 5.0, 5.0}));    // max face is outside
    EXPECT_FALSE(box.contains({-0.1, 5.0, 5.0}));
    EXPECT_FALSE(box.contains({5.0, 5.0, 10.0}));
}

TEST(BoundingBox, Intersects) {
    BoundingBox a{{0.0, 0.0, 0.0}, {10.0, 10.0, 10.0}};
    BoundingBox b{{5.0, 5.0, 5.0}, {15.0, 15.0, 15.0}};
    BoundingBox c{{20.0, 20.0, 20.0}, {30.0, 30.0, 30.0}};
    BoundingBox touching{{10.0, 0.0, 0.0}, {20.0, 10.0, 10.0}};

    EXPECT_TRUE(a.intersects(b));
    EXPECT_TRUE(b.intersects(a));
    EXPECT_FALSE(a.intersects(c));
    EXPECT_TRUE(a.intersects(touching)); // shared face counts as overlap
}

TEST(BoundingBox, CenterSizeAndFromCenterHalf) {
    BoundingBox box{{0.0, 0.0, 0.0}, {10.0, 20.0, 40.0}};
    Vector3 c = box.center();
    EXPECT_DOUBLE_EQ(c.x, 5.0);
    EXPECT_DOUBLE_EQ(c.y, 10.0);
    EXPECT_DOUBLE_EQ(c.z, 20.0);

    Vector3 s = box.size();
    EXPECT_DOUBLE_EQ(s.x, 10.0);
    EXPECT_DOUBLE_EQ(s.y, 20.0);
    EXPECT_DOUBLE_EQ(s.z, 40.0);

    BoundingBox cube = BoundingBox::fromCenterHalf({100.0, 100.0, 100.0}, 25.0);
    EXPECT_DOUBLE_EQ(cube.min.x, 75.0);
    EXPECT_DOUBLE_EQ(cube.max.z, 125.0);
}
