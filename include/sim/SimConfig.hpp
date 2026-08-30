#ifndef SIM_SIMCONFIG_HPP
#define SIM_SIMCONFIG_HPP

#include <string>

#include "sim/CityLayout.hpp"
#include "sim/EngagementManager.hpp"
#include "sim/Vector3.hpp"

namespace sim {

/**
 * @brief Data-driven constants for the telemetry engagement scenario.
 *
 * Every field carries the built-in default. loadSimConfig() overrides only the
 * keys present in the YAML file, so a partial file is fine and a missing or
 * unparseable file simply runs the defaults. Accelerations are expressed in g
 * (Earth gravities) here and converted to m/s^2 (via kG) where the engine needs
 * them — that keeps the YAML readable ("55 g") rather than "539.55".
 */
struct SimConfig {
    static constexpr double kG = 9.81;

    // --- World -------------------------------------------------------------
    Vector3 defendedAsset{50000.0, 50000.0, 0.0};

    // --- Time & determinism ------------------------------------------------
    double       timeScale{0.6};        ///< Physics slow-motion factor (display stays 60 Hz).
    double       durationSeconds{70.0}; ///< Wall-clock run length of the demo.
    unsigned int rngSeed{11};           ///< Seed for the (deterministic) raid.

    // --- Engagement / interceptor airframe ---------------------------------
    double fuzeRadius{20.0};
    double navConstant{4.0};
    double interceptorMaxLatAccelG{55.0};
    double interceptorAxialAccelG{60.0};
    double launchHandoffAltitude{1000.0};
    double protectedRadius{6000.0};
    double interceptorFuelSeconds{70.0};
    double safeDisposalRadius{12000.0};
    double loiterRadius{12000.0};
    double loiterAltitude{6000.0};
    double separationRadius{600.0};
    double separationAccelG{60.0};

    // --- Threats -----------------------------------------------------------
    int    threatCount{16};
    double threatRangeMin{23000.0};
    double threatRangeMax{36000.0};
    double threatAltitudeMin{0.0};
    double threatAltitudeMax{7000.0};
    double threatBoostSpeed{600.0};
    double threatCruiseSpeed{950.0};
    double threatMaxLatAccelG{12.0};
    double threatAxialAccelG{25.0};

    // --- Friendly interceptors (ground-launched) ---------------------------
    int    friendlyStandingBattery{8};
    int    friendlyMaxLaunched{40};
    double friendlyPadRadius{11000.0}; // batteries outside the city + protected zone
    double friendlyCruiseSpeed{1300.0};
    double friendlyLaunchCooldown{0.15};

    // --- Allied-neutral interceptors (airborne) ----------------------------
    int    neutralCount{8};
    double neutralRangeMin{18000.0};
    double neutralRangeMax{22000.0};
    double neutralAltitudeMin{2500.0};
    double neutralAltitudeMax{9000.0};
    double neutralCruiseSpeed{1400.0};
    double neutralInboundSpeed{900.0};

    /// Build the engine-facing engagement config from these constants.
    EngagementManager::Config toEngagementConfig() const {
        EngagementManager::Config c;
        c.defendedAsset         = defendedAsset;
        c.fuzeRadius            = fuzeRadius;
        c.navConstant           = navConstant;
        c.maxLateralAccel       = interceptorMaxLatAccelG * kG;
        c.axialAccel            = interceptorAxialAccelG * kG;
        c.launchHandoffAltitude = launchHandoffAltitude;
        c.protectedRadius       = protectedRadius;
        c.interceptorFuel       = interceptorFuelSeconds;
        c.safeDisposalRadius    = safeDisposalRadius;
        c.loiterRadius          = loiterRadius;
        c.loiterAltitude        = loiterAltitude;
        c.separationRadius      = separationRadius;
        c.separationAccel       = separationAccelG * kG;
        // c.city is populated by the scenario after loading the map asset (see
        // main.cpp / cityStructuresFromInstances) so the collision skyline and
        // the rendered skyline come from the same 1:1 data.
        return c;
    }
};

/**
 * @brief Load a SimConfig from a YAML file.
 * @param path        Path to the YAML file.
 * @param loadedFrom  If non-null, receives the path actually used (empty when
 *                    the file was missing/unparseable and defaults were used).
 * @return the parsed config; any key absent from the file keeps its default.
 */
SimConfig loadSimConfig(const std::string& path,
                        std::string* loadedFrom = nullptr);

} // namespace sim

#endif // SIM_SIMCONFIG_HPP
