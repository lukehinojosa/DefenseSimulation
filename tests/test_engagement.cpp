#include <gtest/gtest.h>

#include <algorithm>

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
    // Interceptor placed 3 m away -- inside the 5 m fuze radius.
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
    // 50 m away -- well outside the fuze radius.
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

TEST(Engagement, LowThreatInterceptedWithoutDivingThroughGround) {
    // A low, fast inbound skimming toward the asset. The interceptor must reach it
    // on a shallow glide (arriving at the target's altitude flying level) rather
    // than diving through the ground -- so the threat dies AND no interceptor ever
    // dips below the ground plane during the run.
    SimulationEngine engine(defaultAirspace());
    EngagementManager::Config cfg;
    cfg.defendedAsset = {50000.0, 50000.0, 0.0};
    cfg.fuzeRadius    = 15.0;
    EngagementManager mgr(engine, cfg);

    EntityId threat =
        engine.spawn(hostile({62000.0, 50000.0, 250.0}, {-330.0, 0.0, -5.0}));
    const EntityId icId = mgr.deployInterceptor({50000.0, 50000.0, 0.0}, 1400.0);

    bool destroyed = false;
    double minInterceptorZ = 1e9;
    for (int frame = 0; frame < 60 * 60; ++frame) {
        mgr.update(1.0 / 60.0);
        const Entity* ic = engine.entityById(icId);
        if (ic != nullptr && ic->isActive() && ic->type != EntityType::Hostile) {
            minInterceptorZ = std::min(minInterceptorZ, ic->position.z);
        }
        if (!engine.entityById(threat)->isActive()) {
            destroyed = true;
            break;
        }
    }

    EXPECT_TRUE(destroyed);
    // The descent-rate cap guarantees the round can always arrest before the deck.
    EXPECT_GE(minInterceptorZ, cfg.groundZ - 1.0);
}

TEST(Engagement, InterceptorClearsCitySkylineChasingLowTarget) {
    // A tall building sits between the interceptor and a low target beyond it.
    // Terrain-following must lift the round over the skyline: it clears the roof
    // and records zero ground/city losses, rather than gliding into the building.
    SimulationEngine engine(defaultAirspace());
    EngagementManager::Config cfg;
    cfg.defendedAsset = {0.0, 0.0, 0.0};
    // Place the building OUTSIDE the protected radius so the *skyline* floor (not
    // the protected-core minimum) is what must lift the round: a 500 m tower at
    // x = 8 km (R = 8 km > protectedRadius 6 km).
    BoundingBox tower({7900.0, -100.0, 0.0}, {8100.0, 100.0, 500.0});
    cfg.city.push_back(CityStructure{tower, MapKind::Skyscraper});
    EngagementManager mgr(engine, cfg);

    // Low target 12 km out, level -- the interceptor must transit the tower's
    // location to reach it, and its glide would otherwise sink it into the tower.
    engine.spawn(hostile({12000.0, 0.0, 5.0}, {-40.0, 0.0, 0.0}));
    const EntityId icId = mgr.deployInterceptor({0.0, 0.0, 1500.0}, 1400.0);

    bool crossedTowerFootprint = false;
    double minZOverTower = 1e9;
    for (int frame = 0; frame < 60 * 40; ++frame) {
        mgr.update(1.0 / 60.0);
        const Entity* ic = engine.entityById(icId);
        if (ic == nullptr) break;
        if (ic->isActive() && ic->type != EntityType::Hostile &&
            ic->position.x > 7700.0 && ic->position.x < 8300.0 &&
            ic->position.y > -300.0 && ic->position.y < 300.0) {
            crossedTowerFootprint = true;
            minZOverTower = std::min(minZOverTower, ic->position.z);
        }
    }

    EXPECT_EQ(mgr.interceptorCityLosses(), 0);
    EXPECT_EQ(mgr.interceptorGroundLosses(), 0);
    // It actually transited the tower's location and did so above the roof.
    EXPECT_TRUE(crossedTowerFootprint);
    EXPECT_GE(minZOverTower, 500.0);
}

TEST(Engagement, InterceptorDivingOnGroundTargetNeverCrossesFloor) {
    // Force the worst case for terrain avoidance: an interceptor high above a
    // target sitting just off the deck, so guidance commands a full dive. The
    // per-step descent cap must keep it above Z = 0 every frame -- the continuous
    // sqrt(2*a*AGL) bound alone overshoots the floor in a single Euler step once
    // the altitude is small.
    SimulationEngine engine(defaultAirspace());
    EngagementManager::Config cfg;
    cfg.defendedAsset = {50000.0, 50000.0, 0.0};
    EngagementManager mgr(engine, cfg);

    // Near-ground target (level flight, so the ground fail-safe leaves it be) and
    // an interceptor 2.5 km directly above the approach.
    engine.spawn(hostile({52000.0, 50000.0, 5.0}, {-60.0, 0.0, 0.0}));
    const EntityId icId =
        mgr.deployInterceptor({50000.0, 50000.0, 2500.0}, 1400.0);

    double minInterceptorZ = 1e9;
    for (int frame = 0; frame < 60 * 30; ++frame) {
        mgr.update(1.0 / 60.0);
        const Entity* ic = engine.entityById(icId);
        if (ic == nullptr) break;            // fuzed against the target: fine
        if (ic->isActive() && ic->type != EntityType::Hostile) {
            minInterceptorZ = std::min(minInterceptorZ, ic->position.z);
        }
    }

    // Never dipped through the ground plane (skim margin holds it a hair above).
    EXPECT_GE(minInterceptorZ, cfg.groundZ - 1e-6);
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
