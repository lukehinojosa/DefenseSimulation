#include <gtest/gtest.h>

#include <atomic>
#include <numeric>
#include <vector>

#include "sim/SimulationEngine.hpp"
#include "sim/ThreadPool.hpp"

using namespace sim;

TEST(ThreadPool, ParallelForVisitsEveryIndexExactlyOnce) {
    ThreadPool pool(4);
    const std::size_t n = 100000;
    std::vector<int> hits(n, 0);

    pool.parallelFor(0, n, [&hits](std::size_t i) { hits[i] += 1; });

    for (std::size_t i = 0; i < n; ++i) {
        ASSERT_EQ(hits[i], 1) << "index " << i << " visited " << hits[i] << " times";
    }
}

TEST(ThreadPool, ParallelForEmptyRangeIsNoOp) {
    ThreadPool pool(4);
    std::atomic<int> calls{0};
    pool.parallelFor(0, 0, [&calls](std::size_t) { calls++; });
    pool.parallelFor(50, 50, [&calls](std::size_t) { calls++; });
    EXPECT_EQ(calls.load(), 0);
}

TEST(ThreadPool, ParallelForConcurrentSumIsCorrect) {
    ThreadPool pool(0); // hardware concurrency
    const std::size_t n = 1000000;
    std::atomic<std::uint64_t> sum{0};
    pool.parallelFor(0, n, [&sum](std::size_t i) {
        sum.fetch_add(i, std::memory_order_relaxed);
    });
    const std::uint64_t expected = (std::uint64_t(n - 1) * n) / 2;
    EXPECT_EQ(sum.load(), expected);
}

TEST(Engine, ParallelStepIntegratesAllEntities) {
    SimulationEngine engine(defaultAirspace());
    const std::size_t n = 20000;

    for (std::size_t i = 0; i < n; ++i) {
        Entity e;
        e.position = {1000.0, 2000.0, 3000.0};
        e.velocity = {10.0, 0.0, -5.0};
        e.type     = EntityType::Hostile;
        engine.spawn(e);
    }
    ASSERT_EQ(engine.entityCount(), n);

    const double dt = 1.0 / 60.0;
    engine.step(dt);

    // Every entity must have advanced by exactly V*dt on the shared trajectory.
    for (const Entity& e : engine.entities()) {
        EXPECT_NEAR(e.position.x, 1000.0 + 10.0 * dt, 1e-6);
        EXPECT_NEAR(e.position.y, 2000.0, 1e-6);
        EXPECT_NEAR(e.position.z, 3000.0 - 5.0 * dt, 1e-6);
    }
}

TEST(Engine, RebuildIndexReflectsPositionsAndSkipsDestroyed) {
    SimulationEngine engine(defaultAirspace());

    Entity a; a.position = {5000.0, 5000.0, 1000.0}; a.type = EntityType::Hostile;
    Entity b; b.position = {5100.0, 5100.0, 1000.0}; b.type = EntityType::Friendly;
    Entity c; c.position = {5200.0, 5200.0, 1000.0}; c.type = EntityType::Hostile;
    c.status = EntityStatus::Destroyed;

    engine.spawn(a);
    engine.spawn(b);
    engine.spawn(c);
    engine.rebuildIndex();

    // Destroyed entity c is excluded from the index.
    EXPECT_EQ(engine.index().size(), 2u);

    BoundingBox region = BoundingBox::fromCenterHalf({5050.0, 5050.0, 1000.0}, 1000.0);
    auto hostiles = engine.queryRange(region, QUERY_HOSTILE_ONLY);
    ASSERT_EQ(hostiles.size(), 1u); // only the active hostile a
    EXPECT_EQ(hostiles.front().id, 0u);
}

TEST(Engine, SpawnAssignsSequentialIds) {
    SimulationEngine engine(defaultAirspace());
    Entity e;
    e.position = {1.0, 1.0, 1.0};
    EXPECT_EQ(engine.spawn(e), 0u);
    EXPECT_EQ(engine.spawn(e), 1u);
    EXPECT_EQ(engine.spawn(e), 2u);
}
