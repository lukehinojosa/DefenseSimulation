#ifndef SIM_ENGAGEMENTMANAGER_HPP
#define SIM_ENGAGEMENTMANAGER_HPP

#include <cstdint>
#include <limits>
#include <vector>

#include "sim/Entity.hpp"
#include "sim/Guidance.hpp"
#include "sim/SimulationEngine.hpp"
#include "sim/Vector3.hpp"

namespace sim {

/// Sentinel meaning "no target currently assigned".
constexpr EntityId kNoTarget = std::numeric_limits<EntityId>::max();

/// A friendly interceptor flying ProNav guidance against an assigned hostile.
struct Interceptor {
    EntityId id{0};                 ///< Entity id in the SimulationEngine.
    double   cruiseSpeed{0.0};      ///< Steering keeps speed at this value (m/s).
    EntityId targetId{kNoTarget};   ///< Assigned hostile, or kNoTarget.

    bool hasTarget() const { return targetId != kNoTarget; }
};

/// A ranked hostile track produced by the threat evaluator.
struct Threat {
    EntityId id{0};
    double   timeToImpact{0.0};     ///< Seconds to the defended asset (TTI).
    double   distanceToAsset{0.0};  ///< Range to the defended asset (m).
};

/**
 * @brief Closed-loop threat engagement: prioritize, assign, guide, detonate.
 *
 * Each update() advances one 60 Hz frame:
 *   1. Rank hostiles by time-to-impact against the defended asset (proximity
 *      breaks ties), filtering out friendlies/neutrals via QUERY_HOSTILE_ONLY.
 *   2. Assign the most urgent unengaged threats to idle interceptors.
 *   3. Steer each interceptor with Proportional Navigation.
 *   4. Advance the simulation, then run the proximity fuze: any interceptor
 *      within the fuze radius of its target detonates, destroying both.
 *
 * The manager mutates entities owned by the referenced SimulationEngine; it
 * does not own them.
 */
class EngagementManager {
public:
    struct Config {
        Vector3 defendedAsset{50000.0, 50000.0, 0.0}; ///< Point threats close on.
        double  fuzeRadius{5.0};                       ///< Proximity fuze (m).
        double  navConstant{guidance::kDefaultNavConstant};
    };

    explicit EngagementManager(SimulationEngine& engine)
        : engine_(engine), config_() {}
    EngagementManager(SimulationEngine& engine, Config config)
        : engine_(engine), config_(config) {}

    /// Spawn a friendly interceptor entity at @p pos and register it.
    EntityId deployInterceptor(const Vector3& pos, double cruiseSpeed) {
        Entity e;
        e.position = pos;
        e.velocity = Vector3{};
        e.type     = EntityType::Friendly;
        const EntityId id = engine_.spawn(e);
        interceptors_.push_back(Interceptor{id, cruiseSpeed, kNoTarget});
        return id;
    }

    /// Rank all active hostiles most-urgent-first (lowest TTI, then nearest).
    std::vector<Threat> buildThreatQueue() const;

    /// Greedily assign the top unclaimed threats to idle interceptors.
    void assignTargets();

    /// Apply one ProNav guidance step to every engaged interceptor.
    void guide(double dt);

    /// Run the proximity fuze; returns the number of intercepts this frame.
    int processDetonations();

    /// Full frame: assign -> guide -> step -> detonate.
    void update(double dt);

    // --- Accessors ---------------------------------------------------------
    const std::vector<Interceptor>& interceptors() const { return interceptors_; }
    int  interceptCount() const { return interceptCount_; }
    const Config& config() const { return config_; }

    /// Entity ids destroyed during the most recent processDetonations() call
    /// (interceptor + target pairs). Consumed by the telemetry publisher to
    /// emit one-frame detonation events.
    const std::vector<EntityId>& lastDestroyed() const { return lastDestroyed_; }

    /// Active interceptors that still have an assigned, living target.
    int activeEngagements() const;

private:
    bool isActiveHostile(EntityId id) const;
    void releaseIfTargetLost(Interceptor& ic);

    SimulationEngine&        engine_;
    Config                   config_;
    std::vector<Interceptor> interceptors_;
    std::vector<EntityId>    lastDestroyed_;
    int                      interceptCount_{0};
    bool                     primed_{false};
};

} // namespace sim

#endif // SIM_ENGAGEMENTMANAGER_HPP
