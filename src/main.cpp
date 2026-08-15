// Defense Simulation — demonstration harness (Phases 1 & 2).
//
// Two scenarios are run end-to-end from the command line:
//   1. Performance: a large mixed population advanced by the multithreaded
//      engine, reporting per-frame timing and a sample spatial query.
//   2. Engagement: inbound hostiles defended by ProNav interceptors, reporting
//      intercepts over time.

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>

#include "sim/EngagementManager.hpp"
#include "sim/SimConfig.hpp"
#include "sim/SimulationEngine.hpp"

// Telemetry (shared memory + UDP) is cross-platform.
#include <algorithm>
#include <memory>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "sim/SharedMemoryChannel.hpp"
#include "sim/Telemetry.hpp"
#include "sim/UdpTelemetry.hpp"
#define SIM_HAVE_TELEMETRY 1

namespace {

using sim::Entity;
using sim::EntityType;
using sim::Vector3;

Entity makeRandomEntity(std::mt19937& rng, const sim::BoundingBox& air) {
    std::uniform_real_distribution<double> ux(air.min.x, air.max.x);
    std::uniform_real_distribution<double> uy(air.min.y, air.max.y);
    std::uniform_real_distribution<double> uz(air.min.z, air.max.z);
    std::uniform_real_distribution<double> uv(-250.0, 250.0); // m/s per axis
    std::uniform_int_distribution<int>     ut(0, 2);

    Entity e;
    e.position = {ux(rng), uy(rng), uz(rng)};
    e.velocity = {uv(rng), uv(rng), uv(rng)};
    e.type     = static_cast<EntityType>(ut(rng));
    return e;
}

void runPerformanceDemo(std::size_t entityCount, int frames) {
    const double dt = 1.0 / 60.0; // 60 Hz
    sim::SimulationEngine engine(sim::defaultAirspace());
    std::mt19937 rng(1337);

    for (std::size_t i = 0; i < entityCount; ++i) {
        engine.spawn(makeRandomEntity(rng, engine.index().bounds()));
    }

    std::cout << "== Performance demo ==\n"
              << "  entities : " << entityCount << "\n"
              << "  frames   : " << frames << "\n"
              << "  workers  : " << engine.threadPool().threadCount() << "\n";

    using clock = std::chrono::steady_clock;
    const auto start = clock::now();
    for (int f = 0; f < frames; ++f) {
        engine.step(dt);
    }
    const auto end = clock::now();

    const double totalMs =
        std::chrono::duration<double, std::milli>(end - start).count();
    const double perFrameMs = totalMs / frames;

    std::cout << std::fixed << std::setprecision(3)
              << "  per frame: " << perFrameMs << " ms  ("
              << (1000.0 / perFrameMs) << " fps headroom)\n";

    const Vector3 c = engine.index().bounds().center();
    const sim::BoundingBox region = sim::BoundingBox::fromCenterHalf(c, 5000.0);
    const auto hostiles = engine.queryRange(region, sim::QUERY_HOSTILE_ONLY);
    std::cout << "  octree nodes: " << engine.index().nodeCount()
              << ", hostiles in 10km cube: " << hostiles.size() << "\n\n";
}

void runEngagementDemo() {
    const double dt = 1.0 / 60.0;
    sim::SimulationEngine engine(sim::defaultAirspace());

    sim::EngagementManager::Config cfg;
    cfg.defendedAsset = {50000.0, 50000.0, 0.0};
    // Fuze radius exceeds the per-frame travel (~17 m at 60 Hz for a Mach-3
    // interceptor) so terminal-phase fly-bys register rather than being
    // stepped over by the discrete integrator.
    cfg.fuzeRadius    = 15.0;
    sim::EngagementManager mgr(engine, cfg);

    std::mt19937 rng(7);
    std::uniform_real_distribution<double> jitter(-6000.0, 6000.0);

    // A salvo of hostiles inbound to the defended asset from the north-east.
    const int threatCount = 12;
    for (int i = 0; i < threatCount; ++i) {
        Entity h;
        h.position = {80000.0 + jitter(rng), 80000.0 + jitter(rng),
                      9000.0 + jitter(rng) * 0.3};
        const Vector3 toAsset = (cfg.defendedAsset - h.position).normalized();
        h.velocity = toAsset * 320.0; // ~Mach 1 inbound
        h.type     = EntityType::Hostile;
        engine.spawn(h);
    }

    // A picket of interceptors ringing the asset.
    const int interceptorCount = 12;
    for (int i = 0; i < interceptorCount; ++i) {
        const double a = (2.0 * 3.14159265 * i) / interceptorCount;
        Vector3 pos = cfg.defendedAsset +
                      Vector3{8000.0 * std::cos(a), 8000.0 * std::sin(a), 500.0};
        mgr.deployInterceptor(pos, 1000.0); // ~Mach 3
    }

    std::cout << "== Engagement demo ==\n"
              << "  threats     : " << threatCount << "\n"
              << "  interceptors: " << interceptorCount << "\n"
              << "  fuze radius : " << cfg.fuzeRadius << " m,  N = "
              << cfg.navConstant << "\n";

    for (int frame = 0; frame < 60 * 90; ++frame) { // up to 90 s
        mgr.update(dt);
        if (mgr.activeEngagements() == 0 && frame > 5) {
            std::cout << "  all engagements resolved at t = "
                      << std::fixed << std::setprecision(2) << (frame * dt)
                      << " s\n";
            break;
        }
    }

    int destroyed = 0;
    for (const Entity& e : engine.entities()) {
        if (e.type == EntityType::Hostile && !e.isActive()) {
            ++destroyed;
        }
    }
    std::cout << "  intercepts  : " << mgr.interceptCount() << "\n"
              << "  threats neutralized: " << destroyed << " / " << threatCount
              << "\n";
}

#if defined(SIM_HAVE_TELEMETRY)
// Real-time engagement scenario that publishes a telemetry frame every tick to
// POSIX shared memory for the separate telemetry_monitor process to consume.
// When udpHost is non-empty it also streams a prioritized subset over UDP for a
// remote visualizer (e.g. the Windows C2 display over Tailscale).
void runTelemetryPublisher(const sim::SimConfig& scfg, const std::string& udpHost,
                           std::uint16_t udpPort) {
    namespace tlm = sim::telemetry;
    const double dt = 1.0 / 60.0;   // wall-clock frame period (telemetry stays 60 Hz)
    // Run physics in slow motion: the display still refreshes at 60 Hz, but each
    // frame advances less simulated time, so fast (Mach 3+) missiles and their
    // rate-limited turns are legible instead of a 1:1 real-time blur.
    const double simDt = dt * scfg.timeScale;

    sim::SimulationEngine engine(sim::defaultAirspace());
    sim::EngagementManager::Config cfg = scfg.toEngagementConfig();
    sim::EngagementManager mgr(engine, cfg);

    // Threat launch/cruise dynamics (g -> m/s^2). Hostiles boost vertically off
    // ground sites, then dive on the defended city under a sluggish lateral
    // limit so they arc over gradually rather than turning on a dime.
    const double kThreatBoost       = scfg.threatBoostSpeed;
    const double kThreatCruise      = scfg.threatCruiseSpeed;
    const double kThreatMaxLatAccel = scfg.threatMaxLatAccelG * sim::SimConfig::kG;
    const double kThreatAxialAccel  = scfg.threatAxialAccelG * sim::SimConfig::kG;

    std::mt19937 rng(scfg.rngSeed);

    // Hostile salvo: each threat starts on a random bearing at a random range
    // and a random altitude. Ground launches boost vertically before pitching
    // over onto the city; ones that spawn already above the hand-off altitude
    // cruise straight in — a mixed-profile raid rather than a uniform wave.
    std::uniform_real_distribution<double> bearing(0.0, 2.0 * 3.14159265);
    std::uniform_real_distribution<double> range(scfg.threatRangeMin,
                                                 scfg.threatRangeMax);
    std::uniform_real_distribution<double> height(scfg.threatAltitudeMin,
                                                  scfg.threatAltitudeMax);
    for (int i = 0; i < scfg.threatCount; ++i) {
        const double a = bearing(rng);
        const double r = range(rng);
        Entity h;
        h.position = {cfg.defendedAsset.x + r * std::cos(a),
                      cfg.defendedAsset.y + r * std::sin(a), height(rng)};
        h.velocity = {0.0, 0.0, kThreatBoost}; // straight up off the pad
        h.type     = EntityType::Hostile;
        h.flags    = sim::EFLAG_LAUNCHING | sim::EFLAG_BOOSTING;
        engine.spawn(h);
    }

    // Friendly interceptors launch from ground pads on demand rather than in one
    // mass salvo: a small standing battery fires first, then more rounds ripple
    // up over time to cover any threat that lacks a shooter. This keeps idle
    // rounds from boosting straight up with no job to do.
    std::uniform_real_distribution<double> padAngle(0.0, 2.0 * 3.14159265);
    const int kMaxFriendly = scfg.friendlyMaxLaunched;
    int    friendlyLaunched = 0;
    double launchCooldown   = 0.0;
    auto launchFriendly = [&]() {
        const double a = padAngle(rng);
        mgr.deployInterceptor(
            cfg.defendedAsset + Vector3{scfg.friendlyPadRadius * std::cos(a),
                                        scfg.friendlyPadRadius * std::sin(a), 0.0},
            scfg.friendlyCruiseSpeed, EntityType::Friendly, /*launching=*/true);
        ++friendlyLaunched;
    };
    for (int i = 0; i < scfg.friendlyStandingBattery; ++i) launchFriendly();

    // Neutral (allied) interceptors stream in from range on random bearings,
    // well above the deck, each already on an inbound cruise so it flies a
    // natural approach instead of accelerating from a dead stop before ProNav
    // takes over. The allegiance matrix lets these non-hostile defenders engage
    // the same hostile set.
    std::uniform_real_distribution<double> nbearing(0.0, 2.0 * 3.14159265);
    std::uniform_real_distribution<double> nrange(scfg.neutralRangeMin,
                                                  scfg.neutralRangeMax);
    std::uniform_real_distribution<double> alt(scfg.neutralAltitudeMin,
                                               scfg.neutralAltitudeMax);
    for (int i = 0; i < scfg.neutralCount; ++i) {
        const double a = nbearing(rng);
        const double r = nrange(rng);
        const Vector3 pos{cfg.defendedAsset.x + r * std::cos(a),
                          cfg.defendedAsset.y + r * std::sin(a), alt(rng)};
        const sim::EntityId id = mgr.deployInterceptor(
            pos, scfg.neutralCruiseSpeed, EntityType::Neutral, /*launching=*/false);
        if (Entity* e = engine.entityById(id)) {
            e->velocity =
                (cfg.defendedAsset - pos).normalized() * scfg.neutralInboundSpeed;
        }
    }

    // Per-frame threat director: hostiles boost straight up, then pitch over
    // and cruise onto the defended asset. The pitch-over is rate-limited, so a
    // threat arcs from vertical toward its target rather than snapping heading.
    auto directThreats = [&](double step) {
        for (Entity& e : engine.entities()) {
            if (e.type != EntityType::Hostile || !e.isActive()) {
                continue;
            }
            if (e.flags & sim::EFLAG_LAUNCHING) {
                if (e.position.z >= cfg.launchHandoffAltitude) {
                    e.flags &= static_cast<std::uint8_t>(
                        ~(sim::EFLAG_LAUNCHING | sim::EFLAG_BOOSTING));
                    // fall through to steer this frame
                } else {
                    e.velocity = {0.0, 0.0, kThreatBoost};
                    e.flags |= sim::EFLAG_BOOSTING;
                    continue;
                }
            }
            e.velocity = sim::guidance::steer(
                e.velocity, cfg.defendedAsset - e.position, kThreatCruise,
                kThreatMaxLatAccel, kThreatAxialAccel, step);
        }
    };

    // Demand-driven launch controller: ripple up fresh interceptors whenever the
    // live threat count outpaces the ready shooters, so coverage keeps pace with
    // the raid (and late leakers still draw a shot) instead of firing one big
    // salvo up front.
    auto serviceLaunches = [&](double step) {
        launchCooldown -= step;
        int activeHostiles = 0;
        for (const Entity& e : engine.entities()) {
            if (e.type == EntityType::Hostile && e.isActive()) ++activeHostiles;
        }
        // Count only our OWN (friendly) ready shooters — allied-neutral rounds
        // are treated as unreliable and are not counted toward coverage. That
        // way the friendly battery is always sized to neutralize every threat by
        // itself; any threat an ally happens to kill just frees a friendly to
        // loiter as a ready reserve (which then backfills if an ally fails).
        int readyFriendly = 0;
        for (const auto& ic : mgr.interceptors()) {
            const Entity* e = engine.entityById(ic.id);
            if (e != nullptr && e->isActive() &&
                e->type == EntityType::Friendly && !ic.disposing) {
                ++readyFriendly;
            }
        }
        // Launch only when a threat has no friendly shooter yet — never fire a
        // round that would have no target.
        if (launchCooldown <= 0.0 && friendlyLaunched < kMaxFriendly &&
            readyFriendly < activeHostiles) {
            launchFriendly();
            launchCooldown = scfg.friendlyLaunchCooldown; // stagger the ripple
        }
    };

    tlm::ShmPublisher publisher(tlm::kDefaultShmName, /*unlinkOnClose=*/true);
    std::vector<tlm::TelemetryRecord> records; // reused; no per-frame alloc
    records.reserve(tlm::kMaxRecordsPerFrame);

    std::unique_ptr<tlm::UdpTelemetrySender> udp;
    if (!udpHost.empty()) {
        udp = std::make_unique<tlm::UdpTelemetrySender>(udpHost, udpPort);
    }
    std::vector<tlm::TelemetryRecord> udpScratch; // reused for prioritization
    udpScratch.reserve(tlm::kMaxRecordsPerFrame);

    std::cout << "== Telemetry publisher ==\n"
              << "  shm      : " << publisher.name() << "\n"
              << "  duration : " << scfg.durationSeconds << " s @ 60 Hz"
              << " (time scale " << scfg.timeScale << ")\n"
              << "  threats  : " << scfg.threatCount << "\n";
    if (udp) {
        std::cout << "  udp      : streaming to " << udpHost << ":" << udpPort
                  << " (top " << tlm::kUdpMaxRecords << " records/frame)\n";
    }
    std::cout << "  run the monitor:  ./telemetry_monitor\n\n";

    const auto start = std::chrono::steady_clock::now();
    std::uint64_t frameId = 0;
    const int totalFrames = static_cast<int>(scfg.durationSeconds * 60.0);
    for (int f = 0; f < totalFrames; ++f) {
        directThreats(simDt);   // advance the threat launch/cruise state machine
        serviceLaunches(simDt); // ripple up interceptors to match the live threat
        mgr.update(simDt);

        // Map interceptor entity id -> assigned target id for LOS lines.
        std::unordered_map<sim::EntityId, sim::EntityId> targetOf;
        for (const auto& ic : mgr.interceptors()) {
            if (ic.hasTarget()) targetOf[ic.id] = ic.targetId;
        }

        records.clear();
        for (const Entity& e : engine.entities()) {
            if (!e.isActive()) continue;
            std::uint32_t targetId = tlm::kNoTargetId;
            auto it = targetOf.find(e.id);
            if (it != targetOf.end()) targetId = it->second;
            const std::uint8_t rflags =
                e.isBoosting() ? tlm::FLAG_BOOSTER : tlm::FLAG_NONE;
            records.push_back(tlm::makeRecord(
                e, tlm::classifyThreat(e, cfg.defendedAsset), targetId, rflags));
        }
        // One-frame detonation events so the visualizer can burst FX. Threats
        // that leaked through and struck the city are tagged as asset losses.
        std::unordered_set<sim::EntityId> assetLoss(
            mgr.lastAssetLosses().begin(), mgr.lastAssetLosses().end());
        for (sim::EntityId id : mgr.lastDestroyed()) {
            const Entity* e = engine.entityById(id);
            if (e != nullptr) {
                std::uint8_t f = tlm::FLAG_DESTROYED;
                if (assetLoss.count(id) != 0) f |= tlm::FLAG_ASSET_HIT;
                records.push_back(tlm::makeRecord(*e, tlm::ThreatLevel::None,
                                                  tlm::kNoTargetId, f));
            }
        }

        const std::uint64_t ts = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
        publisher.publish(frameId, ts,
                          static_cast<std::uint32_t>(mgr.interceptCount()),
                          records.data(),
                          static_cast<std::uint32_t>(records.size()));

        if (udp) {
            // Send the highest-priority records that fit one datagram.
            tlm::FrameHeader hdr{};
            hdr.magic          = tlm::kMagic;
            hdr.version        = tlm::kProtocolVersion;
            hdr.frameId        = frameId;
            hdr.timestampNs    = ts;
            hdr.recordCount    = static_cast<std::uint32_t>(records.size());
            hdr.interceptCount = static_cast<std::uint32_t>(mgr.interceptCount());

            // Priority-order all records; the codec packs the most important
            // ones that fit the datagram byte budget and drops the rest.
            udpScratch = records;
            std::sort(udpScratch.begin(), udpScratch.end(),
                      [](const tlm::TelemetryRecord& a,
                         const tlm::TelemetryRecord& b) {
                          return tlm::recordPriority(a) > tlm::recordPriority(b);
                      });
            udp->send(hdr, udpScratch.data(),
                      static_cast<std::uint32_t>(udpScratch.size()));
        }

        ++frameId;
        std::this_thread::sleep_until(start + std::chrono::duration<double>(
                                                  (f + 1) * dt));
    }

    std::cout << "Telemetry publisher finished: " << frameId << " frames, "
              << mgr.interceptCount() << " intercepts, "
              << mgr.assetFailures() << " asset losses.\n";
}

// Locate the scenario config: $SIM_CONFIG if set, else the first of a few
// conventional locations that exists (so it resolves whether the binary is run
// from the repo root or a build subdirectory). Falls back to the default path,
// where the loader will warn and use built-in defaults.
std::string resolveConfigPath() {
    if (const char* env = std::getenv("SIM_CONFIG")) {
        if (env[0] != '\0') return env;
    }
    for (const char* p : {"config/simulation.yaml", "../config/simulation.yaml",
                          "../../config/simulation.yaml",
                          "../../../config/simulation.yaml"}) {
        if (std::ifstream(p).good()) return p;
    }
    return "config/simulation.yaml";
}
#endif // SIM_HAVE_TELEMETRY

} // namespace

int main(int argc, char** argv) {
    const std::string mode = (argc > 1) ? argv[1] : "";

    if (mode == "telemetry") {
        // Usage: defense_sim telemetry [seconds] [udpHost] [udpPort]
        // Scenario constants come from config/simulation.yaml (or $SIM_CONFIG);
        // an optional [seconds] argument overrides the file's duration.
        std::string usedPath;
        sim::SimConfig scfg = sim::loadSimConfig(resolveConfigPath(), &usedPath);
        std::cout << "  config   : "
                  << (usedPath.empty() ? "(built-in defaults)" : usedPath) << "\n";
        if (argc > 2) scfg.durationSeconds = std::stod(argv[2]);
        const std::string udp = (argc > 3) ? argv[3] : "";
        const std::uint16_t port =
            (argc > 4) ? static_cast<std::uint16_t>(std::stoi(argv[4])) : 9090;
        runTelemetryPublisher(scfg, udp, port);
        return 0;
    }

    const std::size_t entityCount = (argc > 1) ? std::stoul(argv[1]) : 10000;
    const int         frames      = (argc > 2) ? std::stoi(argv[2]) : 60;

    std::cout << "Defense Simulation - Phases 1-3\n\n";
    runPerformanceDemo(entityCount, frames);
    runEngagementDemo();
    return 0;
}
