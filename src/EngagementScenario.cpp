#include "sim/EngagementScenario.hpp"

#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "sim/Guidance.hpp"

namespace sim {

namespace {

EngagementManager::Config makeConfig(const SimConfig& scfg,
                                     std::vector<CityStructure> city) {
    EngagementManager::Config cfg = scfg.toEngagementConfig();
    cfg.city = std::move(city);
    return cfg;
}

} // namespace

EngagementScenario::EngagementScenario(const SimConfig& scfg,
                                       std::vector<CityStructure> city)
    : scfg_(scfg),
      asset_(scfg.defendedAsset),
      handoffAltitude_(scfg.launchHandoffAltitude),
      engine_(defaultAirspace()),
      mgr_(engine_, makeConfig(scfg, std::move(city))),
      rng_(scfg.rngSeed) {
    // Hostile salvo: each threat starts on a random bearing at a random range
    // and a random altitude. Ground launches boost vertically before pitching
    // over onto the city; ones that spawn already above the hand-off altitude
    // cruise straight in -- a mixed-profile raid rather than a uniform wave.
    std::uniform_real_distribution<double> bearing(0.0, 2.0 * 3.14159265);
    std::uniform_real_distribution<double> range(scfg_.threatRangeMin,
                                                 scfg_.threatRangeMax);
    std::uniform_real_distribution<double> height(scfg_.threatAltitudeMin,
                                                  scfg_.threatAltitudeMax);
    for (int i = 0; i < scfg_.threatCount; ++i) {
        const double a = bearing(rng_);
        const double r = range(rng_);
        Entity h;
        h.position = {asset_.x + r * std::cos(a), asset_.y + r * std::sin(a),
                      height(rng_)};
        h.velocity = {0.0, 0.0, scfg_.threatBoostSpeed}; // straight up off the pad
        h.type     = EntityType::Hostile;
        h.flags    = EFLAG_LAUNCHING | EFLAG_BOOSTING;
        engine_.spawn(h);
    }

    // Friendly interceptors launch from ground pads on demand rather than in one
    // mass salvo: a small standing battery fires first, then more rounds ripple
    // up over time to cover any threat that lacks a shooter. This keeps idle
    // rounds from boosting straight up with no job to do.
    for (int i = 0; i < scfg_.friendlyStandingBattery; ++i) launchFriendly();

    // Neutral (allied) interceptors stream in from range on random bearings,
    // well above the deck, each already on an inbound cruise so it flies a
    // natural approach instead of accelerating from a dead stop before ProNav
    // takes over. The allegiance matrix lets these non-hostile defenders engage
    // the same hostile set.
    std::uniform_real_distribution<double> nbearing(0.0, 2.0 * 3.14159265);
    std::uniform_real_distribution<double> nrange(scfg_.neutralRangeMin,
                                                  scfg_.neutralRangeMax);
    std::uniform_real_distribution<double> alt(scfg_.neutralAltitudeMin,
                                               scfg_.neutralAltitudeMax);
    for (int i = 0; i < scfg_.neutralCount; ++i) {
        const double a = nbearing(rng_);
        const double r = nrange(rng_);
        const Vector3 pos{asset_.x + r * std::cos(a), asset_.y + r * std::sin(a),
                          alt(rng_)};
        const EntityId id = mgr_.deployInterceptor(
            pos, scfg_.neutralCruiseSpeed, EntityType::Neutral, /*launching=*/false);
        if (Entity* e = engine_.entityById(id)) {
            e->velocity = (asset_ - pos).normalized() * scfg_.neutralInboundSpeed;
        }
    }
}

void EngagementScenario::launchFriendly() {
    const double a = padAngle_(rng_);
    mgr_.deployInterceptor(
        asset_ + Vector3{scfg_.friendlyPadRadius * std::cos(a),
                         scfg_.friendlyPadRadius * std::sin(a), 0.0},
        scfg_.friendlyCruiseSpeed, EntityType::Friendly, /*launching=*/true);
    ++friendlyLaunched_;
}

// Per-frame threat director: hostiles boost straight up, then pitch over and
// cruise onto the defended asset. The pitch-over is rate-limited, so a threat
// arcs from vertical toward its target rather than snapping heading.
void EngagementScenario::directThreats(double step) {
    const double maxLat = scfg_.threatMaxLatAccelG * SimConfig::kG;
    const double axial  = scfg_.threatAxialAccelG * SimConfig::kG;
    for (Entity& e : engine_.entities()) {
        if (e.type != EntityType::Hostile || !e.isActive()) {
            continue;
        }
        if (e.flags & EFLAG_LAUNCHING) {
            if (e.position.z >= handoffAltitude_) {
                e.flags &= static_cast<std::uint8_t>(
                    ~(EFLAG_LAUNCHING | EFLAG_BOOSTING));
                // fall through to steer this frame
            } else {
                e.velocity = {0.0, 0.0, scfg_.threatBoostSpeed};
                e.flags |= EFLAG_BOOSTING;
                continue;
            }
        }
        e.velocity = guidance::steer(e.velocity, asset_ - e.position,
                                     scfg_.threatCruiseSpeed, maxLat, axial, step);
    }
}

// Demand-driven launch controller: ripple up fresh interceptors whenever the
// live threat count outpaces the ready shooters, so coverage keeps pace with the
// raid (and late leakers still draw a shot) instead of firing one big salvo up
// front.
void EngagementScenario::serviceLaunches(double step) {
    launchCooldown_ -= step;
    int activeHostiles = 0;
    for (const Entity& e : engine_.entities()) {
        if (e.type == EntityType::Hostile && e.isActive()) ++activeHostiles;
    }
    // Count only our OWN (friendly) ready shooters -- allied-neutral rounds are
    // treated as unreliable and are not counted toward coverage. That way the
    // friendly battery is always sized to neutralize every threat by itself; any
    // threat an ally happens to kill just frees a friendly to loiter as a ready
    // reserve (which then backfills if an ally fails).
    int readyFriendly = 0;
    for (const auto& ic : mgr_.interceptors()) {
        const Entity* e = engine_.entityById(ic.id);
        if (e != nullptr && e->isActive() &&
            e->type == EntityType::Friendly && !ic.disposing) {
            ++readyFriendly;
        }
    }
    // Launch only when a threat has no friendly shooter yet -- never fire a
    // round that would have no target.
    if (launchCooldown_ <= 0.0 && friendlyLaunched_ < scfg_.friendlyMaxLaunched &&
        readyFriendly < activeHostiles) {
        launchFriendly();
        launchCooldown_ = scfg_.friendlyLaunchCooldown; // stagger the ripple
    }
}

void EngagementScenario::step(double simDt) {
    directThreats(simDt);   // advance the threat launch/cruise state machine
    serviceLaunches(simDt); // ripple up interceptors to match the live threat
    mgr_.update(simDt);
}

void EngagementScenario::buildRecords(std::vector<telemetry::TelemetryRecord>& out) {
    namespace tlm = telemetry;

    // Map interceptor entity id -> assigned target id for LOS lines.
    std::unordered_map<EntityId, EntityId> targetOf;
    for (const auto& ic : mgr_.interceptors()) {
        if (ic.hasTarget()) targetOf[ic.id] = ic.targetId;
    }

    out.clear();
    for (const Entity& e : engine_.entities()) {
        if (!e.isActive()) continue;
        std::uint32_t targetId = tlm::kNoTargetId;
        auto it = targetOf.find(e.id);
        if (it != targetOf.end()) targetId = it->second;
        const std::uint8_t rflags = e.isBoosting() ? tlm::FLAG_BOOSTER : tlm::FLAG_NONE;
        out.push_back(tlm::makeRecord(e, tlm::classifyThreat(e, asset_), targetId,
                                      rflags));
    }
    // One-frame detonation events so the visualizer can burst FX. Threats that
    // leaked through and struck the city are tagged as asset losses.
    std::unordered_set<EntityId> assetLoss(mgr_.lastAssetLosses().begin(),
                                           mgr_.lastAssetLosses().end());
    for (EntityId id : mgr_.lastDestroyed()) {
        const Entity* e = engine_.entityById(id);
        if (e != nullptr) {
            std::uint8_t f = tlm::FLAG_DESTROYED;
            if (assetLoss.count(id) != 0) f |= tlm::FLAG_ASSET_HIT;
            out.push_back(tlm::makeRecord(*e, tlm::ThreatLevel::None,
                                          tlm::kNoTargetId, f));
        }
    }
}

} // namespace sim
