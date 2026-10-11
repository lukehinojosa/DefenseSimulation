#ifndef SIM_ENGAGEMENTSCENARIO_HPP
#define SIM_ENGAGEMENTSCENARIO_HPP

#include <cstdint>
#include <random>
#include <vector>

#include "sim/CityLayout.hpp"
#include "sim/EngagementManager.hpp"
#include "sim/SimConfig.hpp"
#include "sim/SimulationEngine.hpp"
#include "sim/Telemetry.hpp"

namespace sim {

/**
 * @brief The data-driven raid scenario the C2 display shows, as a steppable unit.
 *
 * Owns the engine, the engagement manager, and the scenario's own launch logic
 * (threat boost/pitch-over director, demand-driven friendly launches, the
 * allied-neutral wave). The desktop publisher (defense_sim telemetry) and the
 * in-browser WebAssembly viewer both drive this one class, so the two front ends
 * show the identical engagement from the identical code.
 *
 * Deterministic: the same SimConfig (seed included) and city replay the same
 * raid, so "restart" is simply constructing a fresh scenario.
 */
class EngagementScenario {
public:
    /// @param city Collision skyline (see cityStructuresFromInstances); moved in.
    EngagementScenario(const SimConfig& scfg, std::vector<CityStructure> city);

    EngagementScenario(const EngagementScenario&) = delete;
    EngagementScenario& operator=(const EngagementScenario&) = delete;

    /// Advance one tick of @p simDt simulated seconds.
    void step(double simDt);

    /// Fill @p out with one telemetry frame: a record per active entity, then a
    /// one-frame FLAG_DESTROYED record per detonation this tick (FLAG_ASSET_HIT
    /// marks a threat that leaked through and struck the city).
    void buildRecords(std::vector<telemetry::TelemetryRecord>& out);

    SimulationEngine&              engine()        { return engine_; }
    const SimulationEngine&        engine()  const { return engine_; }
    EngagementManager&             manager()       { return mgr_; }
    const EngagementManager&       manager() const { return mgr_; }
    const Vector3&                 defendedAsset() const { return asset_; }
    int                            friendlyLaunched() const { return friendlyLaunched_; }

private:
    void launchFriendly();
    void directThreats(double step);
    void serviceLaunches(double step);

    SimConfig         scfg_;
    Vector3           asset_;
    double            handoffAltitude_;
    SimulationEngine  engine_;
    EngagementManager mgr_;
    std::mt19937      rng_;
    std::uniform_real_distribution<double> padAngle_{0.0, 2.0 * 3.14159265};
    int               friendlyLaunched_{0};
    double            launchCooldown_{0.0};
};

} // namespace sim

#endif // SIM_ENGAGEMENTSCENARIO_HPP
