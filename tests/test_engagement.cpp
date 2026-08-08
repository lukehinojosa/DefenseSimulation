#include <gtest/gtest.h>

#include "sim/EngagementManager.hpp"
#include "sim/SimulationEngine.hpp"

using namespace sim;

namespace {

Entity hostile(const Vector3& pos, const Vector3& vel) {
    Entity e;
    e.position = pos;
    e.velocity = vel;
    e.type     = EntityType::Hostile;
    return e;
}

} // namespace

TEST(Engagement, ThreatQueueOrdersByTimeToImpact) {
    SimulationEngine engine(defaultAirspace());
    EngagementManager::Config cfg;
    cfg.defendedAsset = {0.0, 0.0, 0.0};
    EngagementManager mgr(engine, cfg);

    // Far but fast, closing straight on the asset: low TTI.
    EntityId fast = engine.spawn(hostile({10000.0, 0.0, 0.0}, {-1000.0, 0.0, 0.0}));
    // Near but slow: higher TTI than the fast one despite being closer.
    EntityId slow = engine.spawn(hostile({2000.0, 0.0, 0.0}, {-100.0, 0.0, 0.0}));
    // A friendly that must never appear in the threat queue.
    Entity friendly;
    friendly.position = {1000.0, 0.0, 0.0};
    friendly.type     = EntityType::Friendly;
    engine.spawn(friendly);

    engine.rebuildIndex();
    auto threats = mgr.buildThreatQueue();

    ASSERT_EQ(threats.size(), 2u); // friendly filtered out
    EXPECT_EQ(threats[0].id, fast); // 10 s vs 20 s -> fast is more urgent
    EXPECT_EQ(threats[1].id, slow);
    EXPECT_LT(threats[0].timeToImpact, threats[1].timeToImpact);
}

TEST(Engagement, AssignsDistinctTargetsToInterceptors) {
    SimulationEngine engine(defaultAirspace());
    EngagementManager mgr(engine);

    engine.spawn(hostile({20000.0, 20000.0, 1000.0}, {-100.0, -100.0, 0.0}));
    engine.spawn(hostile({30000.0, 30000.0, 1000.0}, {-100.0, -100.0, 0.0}));

    mgr.deployInterceptor({0.0, 0.0, 0.0}, 1000.0);
    mgr.deployInterceptor({0.0, 0.0, 0.0}, 1000.0);

    engine.rebuildIndex();
    mgr.assignTargets();

    const auto& is = mgr.interceptors();
    ASSERT_EQ(is.size(), 2u);
    EXPECT_TRUE(is[0].hasTarget());
    EXPECT_TRUE(is[1].hasTarget());
    EXPECT_NE(is[0].targetId, is[1].targetId); // no double assignment
}

TEST(Engagement, ProximityFuzeDestroysBothEntities) {
    SimulationEngine engine(defaultAirspace());
    EngagementManager::Config cfg;
    cfg.fuzeRadius = 5.0;
    EngagementManager mgr(engine, cfg);

    EntityId threat = engine.spawn(hostile({1000.0, 1000.0, 1000.0}, {0.0, 0.0, 0.0}));
    // Interceptor placed 3 m away — inside the 5 m fuze radius.
    EntityId icept = mgr.deployInterceptor({1000.0, 1000.0, 1003.0}, 1000.0);
    engine.entityById(icept)->velocity = Vector3{};

    engine.rebuildIndex();
    mgr.assignTargets();
    const int hits = mgr.processDetonations();

    EXPECT_EQ(hits, 1);
    EXPECT_EQ(mgr.interceptCount(), 1);
    EXPECT_FALSE(engine.entityById(threat)->isActive());
    EXPECT_FALSE(engine.entityById(icept)->isActive()); // both despawn
    EXPECT_EQ(engine.index().size(), 0u);               // removed from index
}

TEST(Engagement, FuzeIgnoresTargetsOutsideRadius) {
    SimulationEngine engine(defaultAirspace());
    EngagementManager mgr(engine); // default 5 m fuze

    EntityId threat = engine.spawn(hostile({1000.0, 1000.0, 1000.0}, {0.0, 0.0, 0.0}));
    // 50 m away — well outside the fuze radius.
    mgr.deployInterceptor({1000.0, 1000.0, 1050.0}, 1000.0);

    engine.rebuildIndex();
    const int hits = mgr.processDetonations();
    EXPECT_EQ(hits, 0);
    EXPECT_TRUE(engine.entityById(threat)->isActive());
}

TEST(Engagement, ClosedLoopInterceptDestroysThreat) {
    SimulationEngine engine(defaultAirspace());
    EngagementManager::Config cfg;
    cfg.defendedAsset = {50000.0, 50000.0, 0.0};
    cfg.fuzeRadius    = 10.0;
    EngagementManager mgr(engine, cfg);

    // Inbound hostile crossing toward the asset.
    EntityId threat = engine.spawn(
        hostile({60000.0, 55000.0, 8000.0}, {-300.0, -100.0, -100.0}));
    mgr.deployInterceptor({50000.0, 50000.0, 0.0}, 1400.0);

    bool destroyed = false;
    for (int frame = 0; frame < 60 * 60; ++frame) { // up to 60 s at 60 Hz
        mgr.update(1.0 / 60.0);
        if (!engine.entityById(threat)->isActive()) {
            destroyed = true;
            break;
        }
    }

    EXPECT_TRUE(destroyed);
    EXPECT_EQ(mgr.interceptCount(), 1);
}

TEST(Engagement, InterceptorReleasesTargetWhenThreatDestroyed) {
    SimulationEngine engine(defaultAirspace());
    EngagementManager mgr(engine);

    EntityId threat = engine.spawn(hostile({20000.0, 0.0, 1000.0}, {-100.0, 0.0, 0.0}));
    mgr.deployInterceptor({0.0, 0.0, 1000.0}, 1000.0);

    engine.rebuildIndex();
    mgr.assignTargets();
    ASSERT_TRUE(mgr.interceptors()[0].hasTarget());

    // Externally destroy the threat; the interceptor must drop its lock.
    engine.entityById(threat)->status = EntityStatus::Destroyed;
    engine.rebuildIndex();
    mgr.assignTargets();

    EXPECT_FALSE(mgr.interceptors()[0].hasTarget());
}
