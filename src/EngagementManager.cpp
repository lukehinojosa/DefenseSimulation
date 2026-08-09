#include "sim/EngagementManager.hpp"

#include <algorithm>
#include <unordered_set>

#include "sim/BoundingBox.hpp"
#include "sim/Octree.hpp"

namespace sim {

bool EngagementManager::isActiveHostile(EntityId id) const {
    const Entity* e = engine_.entityById(id);
    return e != nullptr && e->isActive() && e->type == EntityType::Hostile;
}

std::vector<Threat> EngagementManager::buildThreatQueue() const {
    // Filter to hostiles only via the octree bitmask query; friendlies and
    // neutrals never enter the targeting solution.
    const std::vector<OctreeItem> hostiles =
        engine_.queryRange(engine_.index().bounds(), QUERY_HOSTILE_ONLY);

    std::vector<Threat> threats;
    threats.reserve(hostiles.size());
    for (const OctreeItem& h : hostiles) {
        const Entity* e = engine_.entityById(h.id);
        if (e == nullptr || !e->isActive()) {
            continue;
        }
        const Vector3 toAsset = config_.defendedAsset - e->position;
        // TTI: time for the hostile to reach the asset (relative velocity of
        // the asset w.r.t. the hostile is -hostileVelocity).
        const double tti =
            guidance::timeToGo(toAsset, Vector3{} - e->velocity);
        threats.push_back(Threat{h.id, tti, toAsset.magnitude()});
    }

    // Most urgent first: smallest TTI, then nearest to the asset.
    std::sort(threats.begin(), threats.end(),
              [](const Threat& a, const Threat& b) {
                  if (a.timeToImpact != b.timeToImpact) {
                      return a.timeToImpact < b.timeToImpact;
                  }
                  return a.distanceToAsset < b.distanceToAsset;
              });
    return threats;
}

void EngagementManager::assignTargets() {
    // Drop assignments whose target is gone, and record the ones still valid
    // so two interceptors are not slaved to the same hostile.
    std::unordered_set<EntityId> claimed;
    for (Interceptor& ic : interceptors_) {
        releaseIfTargetLost(ic);
        if (ic.hasTarget()) {
            claimed.insert(ic.targetId);
        }
    }

    const std::vector<Threat> threats = buildThreatQueue();

    for (Interceptor& ic : interceptors_) {
        const Entity* self = engine_.entityById(ic.id);
        if (self == nullptr || !self->isActive() || ic.hasTarget()) {
            continue;
        }
        for (const Threat& t : threats) {
            if (claimed.count(t.id) == 0) {
                ic.targetId = t.id;
                claimed.insert(t.id);
                break;
            }
        }
    }
}

void EngagementManager::guide(double dt) {
    for (Interceptor& ic : interceptors_) {
        Entity* self = engine_.entityById(ic.id);
        if (self == nullptr || !self->isActive()) {
            continue;
        }
        releaseIfTargetLost(ic);
        if (!ic.hasTarget()) {
            continue;
        }
        const Entity* tgt = engine_.entityById(ic.targetId);

        const Vector3 accel = guidance::proNavAcceleration(
            self->position, self->velocity, tgt->position, tgt->velocity,
            config_.navConstant);

        // Integrate the lateral command, then renormalize to cruise speed so
        // guidance steers heading without changing the interceptor's speed.
        Vector3 v = self->velocity + accel * dt;
        const double speed = v.magnitude();
        if (speed > 1e-9) {
            v = v * (ic.cruiseSpeed / speed);
        }
        self->velocity = v;
    }
}

int EngagementManager::processDetonations() {
    int hits = 0;
    const double r = config_.fuzeRadius;
    lastDestroyed_.clear();

    for (Interceptor& ic : interceptors_) {
        Entity* self = engine_.entityById(ic.id);
        if (self == nullptr || !self->isActive()) {
            continue;
        }

        // Proximity fuze: query a fuze-radius cube around the interceptor for
        // hostiles, then confirm with an exact spherical range test.
        const BoundingBox fuze =
            BoundingBox::fromCenterHalf(self->position, r);
        const std::vector<OctreeItem> near =
            engine_.queryRange(fuze, QUERY_HOSTILE_ONLY);

        for (const OctreeItem& h : near) {
            Entity* target = engine_.entityById(h.id);
            if (target == nullptr || !target->isActive()) {
                continue;
            }
            if (self->position.distanceTo(target->position) <= r) {
                // Detonation destroys both the interceptor and the threat.
                self->status   = EntityStatus::Destroyed;
                target->status = EntityStatus::Destroyed;
                lastDestroyed_.push_back(self->id);
                lastDestroyed_.push_back(target->id);
                ic.targetId    = kNoTarget;
                ++hits;
                break;
            }
        }
    }

    interceptCount_ += hits;
    if (hits > 0) {
        // Reflect the despawns in the spatial index immediately.
        engine_.rebuildIndex();
    }
    return hits;
}

void EngagementManager::update(double dt) {
    // The spatial index must exist before the first targeting query.
    if (!primed_) {
        engine_.rebuildIndex();
        primed_ = true;
    }

    assignTargets();
    guide(dt);
    engine_.step(dt);      // integrate all entities + rebuild octree
    processDetonations();  // fuze check against the fresh index
}

int EngagementManager::activeEngagements() const {
    int n = 0;
    for (const Interceptor& ic : interceptors_) {
        const Entity* self = engine_.entityById(ic.id);
        if (self != nullptr && self->isActive() && ic.hasTarget() &&
            isActiveHostile(ic.targetId)) {
            ++n;
        }
    }
    return n;
}

void EngagementManager::releaseIfTargetLost(Interceptor& ic) {
    if (ic.hasTarget() && !isActiveHostile(ic.targetId)) {
        ic.targetId = kNoTarget;
    }
}

} // namespace sim
