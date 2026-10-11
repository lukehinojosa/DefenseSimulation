// Defense Simulation -- demonstration harness (engine + engagement demos).
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
#include "sim/EngagementScenario.hpp"
#include "sim/MapData.hpp"
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

    // Load the 1:1 New York map asset and build the collision skyline from the
    // same instances the visualizer renders (so buildings collide where drawn).
    std::vector<sim::CityStructure> city;
    sim::MapData map;
    const std::string mapPath = sim::resolveMapPath();
    if (sim::loadMapData(mapPath, map)) {
        city = sim::cityStructuresFromInstances(map.instances, scfg.defendedAsset,
                                                /*scale=*/1.0);
        std::cout << "  map      : " << mapPath << "  ("
                  << map.instances.size() << " instances)\n";
    } else {
        std::cout << "  map      : <none> (" << mapPath << " failed to load)\n";
    }

    // The raid itself (spawns, threat director, demand-driven launches) lives in
    // EngagementScenario, shared with the in-browser WebAssembly viewer.
    sim::EngagementScenario scenario(scfg, std::move(city));
    sim::SimulationEngine&  engine = scenario.engine();
    sim::EngagementManager& mgr    = scenario.manager();
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

    // Optional diagnostic CSV dump (env SIM_LOG=path): one row per active entity
    // per frame plus a row per destruction event, enough to trace every trajectory
    // (e.g. an interceptor that reaches the deck) offline.
    //   * SIM_LOG alone   -> log the NORMAL real-time run (the exact session the
    //                        visualizer is showing), for scfg.durationSeconds.
    //   * + SIM_LOG_SECONDS -> headless-fast: skip real-time pacing and cover that
    //                        many seconds of SIMULATED time regardless of the run
    //                        length / time scale (bulk offline analysis).
    const char* logPath = std::getenv("SIM_LOG");
    const bool logging = (logPath != nullptr && logPath[0] != '\0');
    const char* logSecEnv = std::getenv("SIM_LOG_SECONDS");
    const bool fastLog = logging && (logSecEnv != nullptr);
    std::ofstream log;
    if (logging) {
        log.open(logPath);
        log << "t,frame,id,type,launching,booster,x,y,z,vx,vy,vz,"
               "targetId,destroyed,assethit,cause\n"; // cause: 0 none/fuze,1 ground,2 city
        log << std::fixed << std::setprecision(3);
    }
    const double logSeconds = fastLog ? std::atof(logSecEnv) : 0.0;

    const auto start = std::chrono::steady_clock::now();
    std::uint64_t frameId = 0;
    const int totalFrames = fastLog
        ? static_cast<int>(logSeconds / simDt) + 1
        : static_cast<int>(scfg.durationSeconds * 60.0);
    for (int f = 0; f < totalFrames; ++f) {
        scenario.step(simDt);           // threat director, launches, engagement
        scenario.buildRecords(records); // active tracks + this tick's detonations

        if (logging) {
            // Interceptor -> assigned target and the leaked-threat set, for the log.
            std::unordered_map<sim::EntityId, sim::EntityId> targetOf;
            for (const auto& ic : mgr.interceptors()) {
                if (ic.hasTarget()) targetOf[ic.id] = ic.targetId;
            }
            const std::unordered_set<sim::EntityId> assetLoss(
                mgr.lastAssetLosses().begin(), mgr.lastAssetLosses().end());
            const double simTime = (f + 1) * simDt;
            for (const Entity& e : engine.entities()) {
                if (!e.isActive()) continue;
                std::uint32_t tid = tlm::kNoTargetId;
                auto it = targetOf.find(e.id);
                if (it != targetOf.end()) tid = it->second;
                log << simTime << ',' << frameId << ',' << e.id << ','
                    << static_cast<int>(e.type) << ','
                    << ((e.flags & sim::EFLAG_LAUNCHING) ? 1 : 0) << ','
                    << (e.isBoosting() ? 1 : 0) << ','
                    << e.position.x << ',' << e.position.y << ',' << e.position.z << ','
                    << e.velocity.x << ',' << e.velocity.y << ',' << e.velocity.z << ','
                    << static_cast<long long>(tid) << ",0,0,0\n";
            }
            const std::unordered_set<sim::EntityId> groundSet(
                mgr.lastGroundHits().begin(), mgr.lastGroundHits().end());
            const std::unordered_set<sim::EntityId> citySet(
                mgr.lastCityHits().begin(), mgr.lastCityHits().end());
            for (sim::EntityId id : mgr.lastDestroyed()) {
                const Entity* e = engine.entityById(id);
                if (e == nullptr) continue;
                const int cause = groundSet.count(id) ? 1 : (citySet.count(id) ? 2 : 0);
                log << simTime << ',' << frameId << ',' << id << ','
                    << static_cast<int>(e->type) << ",0,0,"
                    << e->position.x << ',' << e->position.y << ',' << e->position.z << ','
                    << e->velocity.x << ',' << e->velocity.y << ',' << e->velocity.z
                    << ",-1,1," << (assetLoss.count(id) != 0 ? 1 : 0) << ',' << cause << '\n';
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
        if (!fastLog) { // real-time pacing for normal runs (logged or not)
            std::this_thread::sleep_until(start + std::chrono::duration<double>(
                                                      (f + 1) * dt));
        }
    }
    if (logging) log.close();

    std::cout << "Telemetry publisher finished: " << frameId << " frames, "
              << mgr.interceptCount() << " intercepts, "
              << mgr.assetFailures() << " asset losses.\n"
              << "  interceptor terrain losses: "
              << (mgr.interceptorGroundLosses() + mgr.interceptorCityLosses())
              << " (ground " << mgr.interceptorGroundLosses()
              << ", city " << mgr.interceptorCityLosses() << ")\n";
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

    std::cout << "Defense Simulation\n\n";
    runPerformanceDemo(entityCount, frames);
    runEngagementDemo();
    return 0;
}
