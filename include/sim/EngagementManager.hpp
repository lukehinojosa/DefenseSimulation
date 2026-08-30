#ifndef SIM_ENGAGEMENTMANAGER_HPP
#define SIM_ENGAGEMENTMANAGER_HPP

#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include "sim/CityLayout.hpp"
#include "sim/Entity.hpp"
#include "sim/Guidance.hpp"
#include "sim/SimulationEngine.hpp"
#include "sim/StructureIndex.hpp"
#include "sim/Vector3.hpp"

namespace sim {

/// Sentinel meaning "no target currently assigned".
constexpr EntityId kNoTarget = std::numeric_limits<EntityId>::max();

/**
 * @brief Engageable-threat mask (allegiance matrix).
 *
 * Targeting is expressed as "which allegiances a defender may fire on" rather
 * than a single hard-coded hostile filter, so both Friendly and Neutral
 * interceptors can prosecute the same Hostile track set. Today only hostiles
 * are engageable; widening the alliance is a one-line change here.
 */
constexpr std::uint32_t QUERY_ENGAGEABLE_THREATS = FILTER_HOSTILE;

/// An interceptor flying ProNav guidance against an assigned hostile.
struct Interceptor {
    EntityId id{0};                 ///< Entity id in the SimulationEngine.
    double   cruiseSpeed{0.0};      ///< Steering keeps speed at this value (m/s).
    EntityId targetId{kNoTarget};   ///< Assigned hostile, or kNoTarget.
    double   fuel{0.0};             ///< Seconds of motor life remaining.
    bool     disposing{false};      ///< Steering clear to self-detonate safely.
    int      slot{0};               ///< Deployment index; picks a distinct CAP
                                    ///< holding altitude/radius so loiterers stack
                                    ///< instead of piling on one orbit.

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

        // --- Airframe limits (inertia / momentum) ---------------------------
        /// Peak lateral maneuver (m/s^2). ~40 g is typical for a SAM; this caps
        /// the ProNav turn command so an interceptor cannot pivot instantly.
        double  maxLateralAccel{40.0 * 9.81};
        /// Peak thrust/drag along the velocity (m/s^2) used to converge on the
        /// cruise speed -- finite, so speed changes are not instantaneous.
        double  axialAccel{50.0 * 9.81};

        // --- Ground plane & launch dynamics ---------------------------------
        double  groundZ{0.0};              ///< World floor; below this is a crash.
        /// Skim guard (m AGL): below this an interceptor adds a gentle climb so a
        /// round that has bled down onto the deck is lifted back to usable
        /// altitude. It is deliberately low so it does not block the glide-slope
        /// guidance from prosecuting genuinely low targets above it; the hard
        /// no-penetration guarantee is the descent-rate cap in guide(), not this.
        double  groundAvoidAltitude{60.0};
        /// Seconds over which the below-floor climb closes the altitude deficit up
        /// to the skyline floor (larger = gentler pull-up).
        double  altSettleTime{3.0};
        /// Interceptor terrain-following floor. The effective floor at any (x,y) is
        /// the tallest building top within skylineLookahead metres (from the city
        /// index) plus skylineMargin, so an interceptor levels off above the real
        /// skyline everywhere buildings exist -- it never descends into a building
        /// or the ground, on the city side or over New Jersey. Out over open
        /// ground/water (no structures in range) the floor is just the ground, so
        /// genuinely low threats there stay engageable. skylineClearance is an
        /// additional minimum over the protected core (tapering out over
        /// skylineTaperBand), covering the dense city even if the grid under-reads.
        double  skylineMargin{60.0};
        double  skylineLookahead{900.0};
        double  skylineClearance{600.0};
        double  skylineTaperBand{3000.0};
        double  launchHandoffAltitude{1000.0}; ///< Boost until clearing this AGL.
        double  protectedRadius{6000.0};   ///< Threats grounded within this XY
                                           ///< range of the asset are losses.

        // --- Interceptor fuel, loiter & safe self-disposal ------------------
        double  interceptorFuel{70.0};      ///< Seconds of interceptor motor life.
        double  safeDisposalRadius{12000.0};///< An interceptor that is out of fuel
                                            ///< steers to at least this XY range
                                            ///< from the asset and self-detonates,
                                            ///< clear of the city.
        double  loiterRadius{12000.0};      ///< A round with no target holds a CAP
        double  loiterAltitude{6000.0};     ///< orbit at this XY range / altitude
                                            ///< until a threat needs it (or fuel
                                            ///< runs out).

        // --- Friendly-on-friendly collision avoidance -----------------------
        double  separationRadius{600.0};    ///< Begin avoiding a fellow defender
                                            ///< within this range (m).
        double  separationAccel{60.0 * 9.81};///< Avoidance authority so interceptors
                                            ///< never fly through one another.
        /// Static city structures (skyscrapers/hospital/suburb) as hard
        /// collision volumes. Empty by default so unit tests see no city.
        std::vector<CityStructure> city;
    };

    explicit EngagementManager(SimulationEngine& engine)
        : engine_(engine), config_() {}
    EngagementManager(SimulationEngine& engine, Config config)
        : engine_(engine), config_(config) {}

    /**
     * @brief Spawn an interceptor entity at @p pos and register it.
     * @param allegiance Friendly or Neutral -- both prosecute hostile threats
     *        (the allegiance matrix lets neutrals defend too).
     * @param launching  When true the interceptor starts on the ground boosting
     *        straight up (EFLAG_LAUNCHING) until it clears the hand-off altitude,
     *        then ProNav takes over. When false it is combat-ready at @p pos with
     *        zero initial velocity (the original behavior).
     */
    EntityId deployInterceptor(const Vector3& pos, double cruiseSpeed,
                               EntityType allegiance = EntityType::Friendly,
                               bool launching = false) {
        Entity e;
        e.position = pos;
        if (launching) {
            e.velocity = Vector3{0.0, 0.0, cruiseSpeed}; // straight up off the pad
            e.flags    = EFLAG_LAUNCHING | EFLAG_BOOSTING;
        } else {
            e.velocity = Vector3{};
        }
        e.type = allegiance;
        const EntityId id = engine_.spawn(e);
        const int slot = static_cast<int>(interceptors_.size());
        interceptors_.push_back(Interceptor{
            id, cruiseSpeed, kNoTarget, config_.interceptorFuel, false, slot});
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

    /**
     * @brief Low-altitude fail-safe: enforce the ground plane and city volumes.
     *
     * Any active entity (that is not still boosting off its pad) which has
     * descended to/through the ground, or entered a static city volume, is
     * destroyed in place -- this is what stops interceptors and leaked threats
     * from clipping below Z = 0. A hostile grounded on the city or within the
     * protected radius of the asset is additionally tallied as an asset loss.
     * @return number of entities removed this frame.
     */
    int processGroundAndAssets();

    /**
     * @brief Self-disposal for interceptors that have finished their job.
     *
     * An interceptor with no target left to chase (no engageable threats
     * remain) or with an empty fuel tank is steered radially away from the
     * defended zone by guide(); once it is at least Config::safeDisposalRadius
     * from the asset it self-detonates here, so spent rounds never loiter over
     * or fall onto the city. @return number self-detonated this frame.
     */
    int processInterceptorDisposal();

    /// Full frame: direct launches -> assign -> guide -> step -> detonate ->
    /// ground/asset fail-safe -> spent-interceptor disposal.
    void update(double dt);

    // --- Accessors ---------------------------------------------------------
    const std::vector<Interceptor>& interceptors() const { return interceptors_; }
    int  interceptCount() const { return interceptCount_; }
    int  assetFailures() const { return assetFailures_; }
    const Config& config() const { return config_; }

    /// Entity ids destroyed during the most recent frame (fuze kills plus
    /// ground/city fail-safe removals). Consumed by the telemetry publisher to
    /// emit one-frame detonation events.
    const std::vector<EntityId>& lastDestroyed() const { return lastDestroyed_; }

    /// Subset of lastDestroyed() that were leaked threats striking the defended
    /// city/asset this frame (so the display can tag them as asset losses).
    const std::vector<EntityId>& lastAssetLosses() const { return lastAssetLosses_; }

    /// Active interceptors that still have an assigned, living target.
    int activeEngagements() const;

    // --- Terrain-collision instrumentation ---------------------------------
    /// Entities removed this frame by the ground plane / by a city structure
    /// (parallel to lastDestroyed(); a diagnostic consumer can tag the cause).
    const std::vector<EntityId>& lastGroundHits() const { return lastGroundHits_; }
    const std::vector<EntityId>& lastCityHits()   const { return lastCityHits_; }
    /// Running totals of OWN/allied interceptors (Friendly or Neutral) lost to a
    /// ground or city collision -- these should stay zero with terrain-following on.
    int interceptorGroundLosses() const { return interceptorGroundLosses_; }
    int interceptorCityLosses()   const { return interceptorCityLosses_; }

private:
    bool isActiveHostile(EntityId id) const;
    void releaseIfTargetLost(Interceptor& ic);
    /// Repulsion acceleration away from nearby fellow defenders so interceptors
    /// never collide. Asymmetric via @p priority: @p self only yields to
    /// higher-priority neighbors, so of any crossing pair exactly one maneuvers
    /// (the other holds its optimal path). @p selfPriority is @p self's rank.
    Vector3 separationAccel(const Entity& self, long long selfPriority,
                            const std::unordered_map<EntityId, long long>&
                                priority) const;

    SimulationEngine&        engine_;
    Config                   config_;
    StructureIndex           cityIndex_;      ///< Broad phase over config_.city.
    bool                     cityIndexed_{false}; ///< Lazily built on first use.
    std::vector<Interceptor> interceptors_;
    std::vector<EntityId>    lastDestroyed_;
    std::vector<EntityId>    lastAssetLosses_;
    std::vector<EntityId>    lastGroundHits_;
    std::vector<EntityId>    lastCityHits_;
    int                      interceptCount_{0};
    int                      assetFailures_{0};
    int                      interceptorGroundLosses_{0};
    int                      interceptorCityLosses_{0};
    double                   lastDt_{1.0 / 60.0};  ///< Last step, for the swept fuze.
    bool                     primed_{false};
};

} // namespace sim

#endif // SIM_ENGAGEMENTMANAGER_HPP
