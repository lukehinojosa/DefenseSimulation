#include "sim/EngagementManager.hpp"

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include "sim/BoundingBox.hpp"
#include "sim/Octree.hpp"

namespace sim {

bool EngagementManager::isActiveHostile(EntityId id) const {
    const Entity* e = engine_.entityById(id);
    return e != nullptr && e->isActive() && e->type == EntityType::Hostile;
}

std::vector<Threat> EngagementManager::buildThreatQueue() const {
    // Query the engageable-threat set via the allegiance matrix. Interceptors
    // (Friendly or Neutral) never target one another — only hostiles are
    // engageable — but the mask, not the entity type, decides that.
    const std::vector<OctreeItem> hostiles =
        engine_.queryRange(engine_.index().bounds(), QUERY_ENGAGEABLE_THREATS);

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
    // Friendly and allied-neutral defenders claim threats INDEPENDENTLY: a
    // friendly round engages a hostile even if an ally is already on it — we do
    // not trust the ally to finish the job — so the friendly battery covers
    // every threat by itself while allied intercepts are pure redundancy. Within
    // one allegiance, assignments stay distinct (no two friendlies on one
    // threat, no two allies on one threat). Drop dead targets first so a round
    // whose threat was neutralized before impact is free to retarget this frame.
    std::unordered_set<EntityId> claimedFriendly, claimedNeutral;
    for (Interceptor& ic : interceptors_) {
        releaseIfTargetLost(ic);
        if (!ic.hasTarget()) {
            continue;
        }
        const Entity* self = engine_.entityById(ic.id);
        if (self != nullptr) {
            (self->type == EntityType::Friendly ? claimedFriendly
                                                : claimedNeutral)
                .insert(ic.targetId);
        }
    }

    const std::vector<Threat> threats = buildThreatQueue();

    for (Interceptor& ic : interceptors_) {
        const Entity* self = engine_.entityById(ic.id);
        if (self == nullptr || !self->isActive() || ic.hasTarget()) {
            continue;
        }
        std::unordered_set<EntityId>& claimed =
            (self->type == EntityType::Friendly) ? claimedFriendly
                                                 : claimedNeutral;
        for (const Threat& t : threats) {
            if (claimed.count(t.id) == 0) {
                ic.targetId = t.id;
                claimed.insert(t.id);
                break;
            }
        }
    }
}

Vector3 EngagementManager::separationAccel(
    const Entity& self, long long selfPriority,
    const std::unordered_map<EntityId, long long>& priority) const {
    const double R = config_.separationRadius;
    const BoundingBox box = BoundingBox::fromCenterHalf(self.position, R);
    std::vector<OctreeItem> near;
    engine_.queryRange(box, FILTER_FRIENDLY | FILTER_NEUTRAL, near);

    Vector3 push{};
    for (const OctreeItem& it : near) {
        if (it.id == self.id) {
            continue;
        }
        // Right-of-way: only maneuver around a neighbor that outranks us; the
        // higher-priority round holds its (optimal) path and we go around it, so
        // a crossing pair never both deviate. Unknown neighbors: yield (safe).
        const auto p = priority.find(it.id);
        const long long neighborPriority =
            (p != priority.end()) ? p->second
                                  : std::numeric_limits<long long>::max();
        if (neighborPriority <= selfPriority) {
            continue;
        }
        const Entity* other = engine_.entityById(it.id);
        if (other == nullptr || !other->isActive()) {
            continue;
        }
        const Vector3 delta = self.position - other->position;
        const double d = delta.magnitude();
        if (d > 1e-3 && d < R) {
            // Unit vector away from the neighbor, weighted 0..1 by closeness.
            push += delta * ((R - d) / (d * R));
        }
    }
    return push * config_.separationAccel;
}

void EngagementManager::guide(double dt) {
    // Collision-avoidance right-of-way ranking, recomputed each frame:
    // attacking (2) outranks loitering (1) outranks disposing (0); a lower id
    // wins ties. A round yields only to strictly higher priorities, so an
    // engaged interceptor holds its intercept path and others route around it.
    std::unordered_map<EntityId, long long> priority;
    priority.reserve(interceptors_.size());
    for (const Interceptor& q : interceptors_) {
        const int rank = q.disposing ? 0 : (q.hasTarget() ? 2 : 1);
        priority[q.id] = static_cast<long long>(rank) * 1000000000LL -
                         static_cast<long long>(q.id);
    }

    for (Interceptor& ic : interceptors_) {
        Entity* self = engine_.entityById(ic.id);
        if (self == nullptr || !self->isActive()) {
            continue;
        }

        ic.fuel -= dt; // motor burns whether boosting, cruising, or coasting

        // --- Launch phase: boost straight up until clear of the pad ----------
        // Steering is suspended so the round rises cleanly before ProNav pulls
        // it over onto the target; the motor stays lit (EFLAG_BOOSTING) for FX.
        if (self->flags & EFLAG_LAUNCHING) {
            if (self->position.z >= config_.launchHandoffAltitude) {
                self->flags &= static_cast<std::uint8_t>(
                    ~(EFLAG_LAUNCHING | EFLAG_BOOSTING)); // hand off to guidance
            } else {
                self->velocity = Vector3{0.0, 0.0, ic.cruiseSpeed};
                self->flags |= EFLAG_BOOSTING;
                continue; // no ProNav while boosting vertically
            }
        }

        releaseIfTargetLost(ic);

        // Repulsion from nearby fellow defenders, folded into every steering
        // mode below so two interceptors never fly through one another. Only
        // lower-priority rounds actually deviate (see the ranking above).
        const Vector3 sep = separationAccel(*self, priority[ic.id], priority);
        // Steer toward a desired heading under airframe limits, plus avoidance.
        auto steerAvoiding = [&](const Vector3& desiredDir) {
            const Vector3 dd = desiredDir.normalized();
            const Vector3 accelCmd =
                (dd * ic.cruiseSpeed - self->velocity) / dt + sep;
            return guidance::applyAirframeLimits(self->velocity, accelCmd,
                                                 ic.cruiseSpeed,
                                                 config_.maxLateralAccel,
                                                 config_.axialAccel, dt);
        };

        // --- Out of fuel: spend the round clear of the city ------------------
        // Turn away from the defended zone (with a slight climb so it never
        // dives on the city) and coast outward; processInterceptorDisposal()
        // detonates it once it is past the safe radius.
        if (ic.fuel <= 0.0) {
            ic.disposing = true;
        }
        if (ic.disposing) {
            ic.targetId = kNoTarget;
            Vector3 outward = self->position - config_.defendedAsset;
            outward.z = 0.0;
            outward = outward.normalized();
            if (outward.magnitudeSquared() < 1e-9) {
                outward = Vector3{1.0, 0.0, 0.0}; // sitting on the asset: pick one
            }
            self->velocity = steerAvoiding(outward + Vector3{0.0, 0.0, 0.15});
            continue; // heading outbound to a safe detonation point
        }

        // --- No target: hold a CAP orbit at a safe distance -----------------
        // A round with nothing to prosecute (an uneven leftover) flies a
        // holding circle around the defended zone rather than orbiting over the
        // city or being thrown away; it is re-tasked the moment assignTargets()
        // hands it a threat. Each slot holds a distinct radius/altitude so the
        // stack of loiterers never collapses onto one orbit.
        if (!ic.hasTarget()) {
            const double slotRad = config_.loiterRadius + (ic.slot % 3) * 1200.0;
            const double slotAlt = config_.loiterAltitude + (ic.slot % 6) * 500.0;
            Vector3 radial = self->position - config_.defendedAsset;
            radial.z = 0.0;
            const double rr = radial.magnitude();
            const Vector3 out =
                (rr > 1e-6) ? radial * (1.0 / rr) : Vector3{1.0, 0.0, 0.0};
            const Vector3 tangent{-out.y, out.x, 0.0}; // circle the zone
            Vector3 desired = tangent + out * ((slotRad - rr) / slotRad);
            desired.z = (slotAlt - self->position.z) / slotAlt;
            self->velocity = steerAvoiding(desired);
            continue;
        }
        const Entity* tgt = engine_.entityById(ic.targetId);

        // Below a working airspeed (e.g. straight off a standing deployment)
        // ProNav has no velocity to steer, so fly pure pursuit toward the target
        // to build up speed; once flying, hand over to ProNav lead guidance.
        if (self->velocity.magnitudeSquared() <
            (0.3 * ic.cruiseSpeed) * (0.3 * ic.cruiseSpeed)) {
            self->velocity = steerAvoiding(tgt->position - self->position);
            continue;
        }

        // ProNav command (plus avoidance), then clamp to the airframe's
        // lateral-G and thrust limits: the interceptor arcs through the turn and
        // bleeds/gains speed over time instead of instantly re-pointing.
        const Vector3 accel = guidance::proNavAcceleration(
            self->position, self->velocity, tgt->position, tgt->velocity,
            config_.navConstant);
        self->velocity = guidance::applyAirframeLimits(
            self->velocity, accel + sep, ic.cruiseSpeed, config_.maxLateralAccel,
            config_.axialAccel, dt);
    }
}

int EngagementManager::processDetonations() {
    int hits = 0;
    const double r = config_.fuzeRadius;
    const double dt = lastDt_;
    // At Mach-3+ closing speeds the per-frame travel dwarfs the fuze radius, so
    // a point test at the frame boundary can sample either side of the actual
    // pass and miss. Query a box enlarged by the worst-case relative travel, then
    // confirm with a swept closest-approach test over the last step.
    constexpr double kMaxClosureSpeed = 1600.0; // m/s allowance for the box pad
    lastDestroyed_.clear();

    for (Interceptor& ic : interceptors_) {
        Entity* self = engine_.entityById(ic.id);
        if (self == nullptr || !self->isActive()) {
            continue;
        }

        const double pad = (self->velocity.magnitude() + kMaxClosureSpeed) * dt;
        const BoundingBox fuze =
            BoundingBox::fromCenterHalf(self->position, r + pad);
        const std::vector<OctreeItem> near =
            engine_.queryRange(fuze, QUERY_ENGAGEABLE_THREATS);

        for (const OctreeItem& h : near) {
            Entity* target = engine_.entityById(h.id);
            if (target == nullptr || !target->isActive()) {
                continue;
            }
            // Closest approach of the two straight-line segments traversed this
            // step (parameter s in [-dt, 0]); catches high-speed fly-throughs a
            // frame-boundary point test would tunnel past.
            const Vector3 relP = target->position - self->position;
            const Vector3 relV = target->velocity - self->velocity;
            const double vv = relV.magnitudeSquared();
            double s = 0.0;
            if (vv > 1e-9) {
                s = -relP.dot(relV) / vv;
                if (s > 0.0)   s = 0.0;
                if (s < -dt)   s = -dt;
            }
            const double missDist = (relP + relV * s).magnitude();
            if (missDist <= r) {
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

int EngagementManager::processGroundAndAssets() {
    int removed = 0;
    lastAssetLosses_.clear();

    // Build the static-structure broad phase once, the first time we need it.
    // config_.city is fixed after construction, so a single build is enough; the
    // grid then answers "inside a building?" in O(1) instead of scanning them all.
    if (!cityIndexed_) {
        cityIndex_.build(config_.city);
        cityIndexed_ = true;
    }

    for (Entity& e : engine_.entities()) {
        if (!e.isActive()) {
            continue;
        }
        // A round still boosting off its pad legitimately sits at/near Z = 0;
        // exempt it so launches are not self-destructed on frame one.
        if (e.flags & EFLAG_LAUNCHING) {
            continue;
        }

        // Ground contact: descended to/through the floor. Requiring a downward
        // velocity avoids grounding a combat-ready interceptor parked at Z = 0.
        const bool hitGround =
            e.position.z <= config_.groundZ && e.velocity.z < 0.0;

        // City contact: inside any static structure volume. A missile that flies
        // into a building is destroyed regardless of allegiance (interceptors are
        // launched from batteries outside the protected zone so they never climb
        // through the skyline); a leaked hostile striking a structure is an asset
        // loss (tallied below).
        const bool hitCity = cityIndex_.firstContaining(e.position) != nullptr;

        if (!hitGround && !hitCity) {
            continue;
        }

        if (hitGround) {
            e.position.z = config_.groundZ; // clamp so FX/last frame sit on deck
        }
        e.status = EntityStatus::Destroyed;
        lastDestroyed_.push_back(e.id); // reuse the detonation-event channel

        // A leaked hostile that strikes the city, or grounds within the
        // protected footprint, is a defended-asset loss.
        if (e.type == EntityType::Hostile) {
            const double dx = e.position.x - config_.defendedAsset.x;
            const double dy = e.position.y - config_.defendedAsset.y;
            const bool nearAsset =
                (dx * dx + dy * dy) <=
                config_.protectedRadius * config_.protectedRadius;
            if (hitCity || (hitGround && nearAsset)) {
                ++assetFailures_;
                lastAssetLosses_.push_back(e.id);
            }
        }
        ++removed;
    }

    if (removed > 0) {
        // Any interceptor slaved to a downed threat drops its lock, and the
        // spatial index is refreshed to reflect the despawns.
        for (Interceptor& ic : interceptors_) {
            releaseIfTargetLost(ic);
        }
        engine_.rebuildIndex();
    }
    return removed;
}

int EngagementManager::processInterceptorDisposal() {
    int disposed = 0;
    const double r2 =
        config_.safeDisposalRadius * config_.safeDisposalRadius;

    for (Interceptor& ic : interceptors_) {
        Entity* self = engine_.entityById(ic.id);
        if (self == nullptr || !self->isActive() || !ic.disposing) {
            continue;
        }
        const double dx = self->position.x - config_.defendedAsset.x;
        const double dy = self->position.y - config_.defendedAsset.y;
        if (dx * dx + dy * dy >= r2) {
            self->status = EntityStatus::Destroyed; // safe self-detonation
            lastDestroyed_.push_back(ic.id);        // reuse detonation FX channel
            ic.targetId = kNoTarget;
            ++disposed;
        }
    }

    if (disposed > 0) {
        engine_.rebuildIndex();
    }
    return disposed;
}

void EngagementManager::update(double dt) {
    // The spatial index must exist before the first targeting query.
    if (!primed_) {
        engine_.rebuildIndex();
        primed_ = true;
    }
    lastDt_ = dt; // used by the swept proximity fuze

    assignTargets();
    guide(dt);
    engine_.step(dt);           // integrate all entities + rebuild octree
    processDetonations();       // fuze check against the fresh index
    processGroundAndAssets();   // ground plane + city fail-safe (appends events)
    processInterceptorDisposal(); // spent rounds self-detonate clear of the city
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
