#include "sim/SimConfig.hpp"

#include <cstdio>
#include <exception>

#include <yaml-cpp/yaml.h>

namespace sim {
namespace {

/// Read a scalar node into @p def's type, keeping @p def when the key is absent
/// or the value cannot be converted. Keeps the loader tolerant of partial files.
template <typename T>
T scalar(const YAML::Node& node, T def) {
    if (node) {
        try {
            return node.as<T>();
        } catch (const std::exception&) {
            // fall through to default on a malformed value
        }
    }
    return def;
}

} // namespace

SimConfig loadSimConfig(const std::string& path, std::string* loadedFrom) {
    SimConfig c;

    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception& e) {
        std::fprintf(stderr,
                     "[config] could not load '%s' (%s); using built-in defaults\n",
                     path.c_str(), e.what());
        if (loadedFrom != nullptr) {
            *loadedFrom = "";
        }
        return c;
    }
    if (loadedFrom != nullptr) {
        *loadedFrom = path;
    }

    if (const YAML::Node world = root["world"]) {
        const YAML::Node asset = world["defended_asset"];
        if (asset && asset.size() == 3) {
            c.defendedAsset = {asset[0].as<double>(), asset[1].as<double>(),
                               asset[2].as<double>()};
        }
    }

    if (const YAML::Node time = root["time"]) {
        c.timeScale       = scalar(time["scale"], c.timeScale);
        c.durationSeconds = scalar(time["duration_seconds"], c.durationSeconds);
    }
    c.rngSeed = scalar(root["rng_seed"], c.rngSeed);

    if (const YAML::Node e = root["engagement"]) {
        c.fuzeRadius              = scalar(e["fuze_radius"], c.fuzeRadius);
        c.navConstant             = scalar(e["nav_constant"], c.navConstant);
        c.interceptorMaxLatAccelG = scalar(e["interceptor_max_lateral_accel_g"],
                                           c.interceptorMaxLatAccelG);
        c.interceptorAxialAccelG  = scalar(e["interceptor_axial_accel_g"],
                                           c.interceptorAxialAccelG);
        c.launchHandoffAltitude   = scalar(e["launch_handoff_altitude"],
                                           c.launchHandoffAltitude);
        c.protectedRadius         = scalar(e["protected_radius"], c.protectedRadius);
        c.interceptorFuelSeconds  = scalar(e["interceptor_fuel_seconds"],
                                           c.interceptorFuelSeconds);
        c.safeDisposalRadius      = scalar(e["safe_disposal_radius"],
                                           c.safeDisposalRadius);
        c.loiterRadius            = scalar(e["loiter_radius"], c.loiterRadius);
        c.loiterAltitude          = scalar(e["loiter_altitude"], c.loiterAltitude);
        c.separationRadius        = scalar(e["separation_radius"], c.separationRadius);
        c.separationAccelG        = scalar(e["separation_accel_g"], c.separationAccelG);
    }

    if (const YAML::Node t = root["threats"]) {
        c.threatCount        = scalar(t["count"], c.threatCount);
        c.threatRangeMin     = scalar(t["spawn_range_min"], c.threatRangeMin);
        c.threatRangeMax     = scalar(t["spawn_range_max"], c.threatRangeMax);
        c.threatAltitudeMin  = scalar(t["spawn_altitude_min"], c.threatAltitudeMin);
        c.threatAltitudeMax  = scalar(t["spawn_altitude_max"], c.threatAltitudeMax);
        c.threatBoostSpeed   = scalar(t["boost_speed"], c.threatBoostSpeed);
        c.threatCruiseSpeed  = scalar(t["cruise_speed"], c.threatCruiseSpeed);
        c.threatMaxLatAccelG = scalar(t["max_lateral_accel_g"], c.threatMaxLatAccelG);
        c.threatAxialAccelG  = scalar(t["axial_accel_g"], c.threatAxialAccelG);
    }

    if (const YAML::Node ic = root["interceptors"]) {
        if (const YAML::Node f = ic["friendly"]) {
            c.friendlyStandingBattery = scalar(f["standing_battery"],
                                               c.friendlyStandingBattery);
            c.friendlyMaxLaunched     = scalar(f["max_launched"],
                                               c.friendlyMaxLaunched);
            c.friendlyPadRadius       = scalar(f["pad_radius"], c.friendlyPadRadius);
            c.friendlyCruiseSpeed     = scalar(f["cruise_speed"],
                                               c.friendlyCruiseSpeed);
            c.friendlyLaunchCooldown  = scalar(f["launch_cooldown"],
                                               c.friendlyLaunchCooldown);
        }
        if (const YAML::Node n = ic["neutral"]) {
            c.neutralCount        = scalar(n["count"], c.neutralCount);
            c.neutralRangeMin     = scalar(n["spawn_range_min"], c.neutralRangeMin);
            c.neutralRangeMax     = scalar(n["spawn_range_max"], c.neutralRangeMax);
            c.neutralAltitudeMin  = scalar(n["spawn_altitude_min"],
                                           c.neutralAltitudeMin);
            c.neutralAltitudeMax  = scalar(n["spawn_altitude_max"],
                                           c.neutralAltitudeMax);
            c.neutralCruiseSpeed  = scalar(n["cruise_speed"], c.neutralCruiseSpeed);
            c.neutralInboundSpeed = scalar(n["inbound_speed"], c.neutralInboundSpeed);
        }
    }

    return c;
}

} // namespace sim
