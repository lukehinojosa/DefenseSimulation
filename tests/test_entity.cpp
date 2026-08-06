#include <gtest/gtest.h>

#include "sim/Entity.hpp"

using namespace sim;

namespace {
constexpr double kEps = 1e-9;
}

TEST(Entity, KinematicIntegration) {
    Entity e;
    e.position = {0.0, 0.0, 1000.0};
    e.velocity = {100.0, -50.0, 0.0}; // m/s
    e.status   = EntityStatus::Active;

    e.integrate(0.5); // half a second
    EXPECT_NEAR(e.position.x, 50.0, kEps);
    EXPECT_NEAR(e.position.y, -25.0, kEps);
    EXPECT_NEAR(e.position.z, 1000.0, kEps);
}

TEST(Entity, DestroyedEntitiesDoNotMove) {
    Entity e;
    e.position = {10.0, 20.0, 30.0};
    e.velocity = {1.0, 1.0, 1.0};
    e.status   = EntityStatus::Destroyed;

    e.integrate(10.0);
    EXPECT_NEAR(e.position.x, 10.0, kEps);
    EXPECT_NEAR(e.position.y, 20.0, kEps);
    EXPECT_NEAR(e.position.z, 30.0, kEps);
    EXPECT_FALSE(e.isActive());
}

TEST(Entity, FilterBitmaskMatching) {
    EXPECT_TRUE(typeMatchesFilter(EntityType::Hostile, FILTER_HOSTILE));
    EXPECT_TRUE(typeMatchesFilter(EntityType::Hostile, FILTER_ALL));
    EXPECT_FALSE(typeMatchesFilter(EntityType::Hostile, FILTER_FRIENDLY));

    EXPECT_TRUE(typeMatchesFilter(EntityType::Friendly, FILTER_FRIENDLY));
    EXPECT_TRUE(typeMatchesFilter(EntityType::Neutral, FILTER_NEUTRAL));

    // QUERY_HOSTILE_ONLY excludes friendlies and neutrals.
    EXPECT_TRUE(typeMatchesFilter(EntityType::Hostile, QUERY_HOSTILE_ONLY));
    EXPECT_FALSE(typeMatchesFilter(EntityType::Friendly, QUERY_HOSTILE_ONLY));
    EXPECT_FALSE(typeMatchesFilter(EntityType::Neutral, QUERY_HOSTILE_ONLY));

    // Combined masks.
    const std::uint32_t hostileOrNeutral = FILTER_HOSTILE | FILTER_NEUTRAL;
    EXPECT_TRUE(typeMatchesFilter(EntityType::Hostile, hostileOrNeutral));
    EXPECT_TRUE(typeMatchesFilter(EntityType::Neutral, hostileOrNeutral));
    EXPECT_FALSE(typeMatchesFilter(EntityType::Friendly, hostileOrNeutral));
}

TEST(Entity, IsTriviallyCopyable) {
    // Guarantees the struct can be memcpy'd into the telemetry pipeline.
    static_assert(std::is_trivially_copyable<Entity>::value,
                  "Entity must stay trivially copyable");
    SUCCEED();
}
