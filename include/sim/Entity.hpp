#ifndef SIM_ENTITY_HPP
#define SIM_ENTITY_HPP

#include <cstdint>

#include "sim/Vector3.hpp"

namespace sim {

using EntityId = std::uint32_t;

/// Allegiance classification used for targeting and display filtering.
enum class EntityType : std::uint8_t {
    Hostile  = 0,
    Friendly = 1,
    Neutral  = 2
};

/// Lifecycle state of an entity within the simulation.
enum class EntityStatus : std::uint8_t {
    Active    = 0,
    Destroyed = 1
};

/**
 * @brief Per-entity behavioral flags (bitmask, stored in Entity::flags).
 *
 * These drive the boost/launch phase of a missile: an entity marked
 * EFLAG_LAUNCHING climbs vertically off its pad and has its steering law
 * suspended until it clears the hand-off altitude, at which point the flag is
 * cleared and normal guidance (ProNav for interceptors, ballistic cruise for
 * threats) resumes. EFLAG_BOOSTING mirrors "motor lit" and is surfaced to the
 * display as a launch-flame / hot-trail cue.
 */
enum EntityFlags : std::uint8_t {
    EFLAG_NONE      = 0u,
    EFLAG_LAUNCHING = 1u << 0, ///< Boosting off the pad; guidance suspended.
    EFLAG_BOOSTING  = 1u << 1  ///< Motor lit (drives launch FX in the display).
};

/**
 * @brief Bitmask flags for spatial queries so callers can restrict results
 *        to a subset of allegiances (e.g. QUERY_HOSTILE_ONLY).
 *
 * The bit index for a type equals its EntityType enumerator value, which
 * lets typeMatchesFilter() translate a type into its flag with a shift.
 */
enum EntityFilter : std::uint32_t {
    FILTER_NONE     = 0u,
    FILTER_HOSTILE  = 1u << 0,
    FILTER_FRIENDLY = 1u << 1,
    FILTER_NEUTRAL  = 1u << 2,
    FILTER_ALL      = FILTER_HOSTILE | FILTER_FRIENDLY | FILTER_NEUTRAL
};

/// Convenience alias matching the guidance-layer vocabulary in later phases.
constexpr std::uint32_t QUERY_HOSTILE_ONLY = FILTER_HOSTILE;

/// @return true if @p type passes @p filter (FILTER_ALL passes everything).
constexpr bool typeMatchesFilter(EntityType type, std::uint32_t filter) {
    return (filter & (1u << static_cast<std::uint32_t>(type))) != 0u;
}

/**
 * @brief A tracked object in the airspace.
 *
 * Kept a plain aggregate (standard-layout, trivially copyable) so large
 * arrays of entities are cache friendly and can be memcpy'd into the
 * telemetry pipeline without serialization overhead.
 */
struct Entity {
    EntityId      id{0};
    Vector3       position{};
    Vector3       velocity{};
    EntityType    type{EntityType::Neutral};
    EntityStatus  status{EntityStatus::Active};
    std::uint8_t  flags{EFLAG_NONE}; ///< EntityFlags bitmask (launch/boost state).

    bool isActive()  const { return status == EntityStatus::Active; }
    bool isBoosting() const { return (flags & EFLAG_BOOSTING) != 0u; }

    /**
     * @brief Advance the entity by one time step using first-order
     *        kinematics: P_new = P_old + V * dt.
     * @param dt Time step in seconds.
     *
     * Destroyed entities are frozen and ignored.
     */
    void integrate(double dt) {
        if (status != EntityStatus::Active) {
            return;
        }
        position += velocity * dt;
    }
};

} // namespace sim

#endif // SIM_ENTITY_HPP
