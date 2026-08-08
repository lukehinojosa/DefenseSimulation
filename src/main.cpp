// Defense Simulation — demonstration harness (Phases 1 & 2).
//
// Two scenarios are run end-to-end from the command line:
//   1. Performance: a large mixed population advanced by the multithreaded
//      engine, reporting per-frame timing and a sample spatial query.
//   2. Engagement: inbound hostiles defended by ProNav interceptors, reporting
//      intercepts over time.

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>

#include "sim/EngagementManager.hpp"
#include "sim/SimulationEngine.hpp"

#if defined(__linux__)
#include <thread>
#include <vector>
#include "sim/SharedMemoryChannel.hpp"
#include "sim/Telemetry.hpp"
#define SIM_HAVE_TELEMETRY 1
#endif

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
void runTelemetryPublisher(double durationSec) {
    namespace tlm = sim::telemetry;
    const double dt = 1.0 / 60.0;

    sim::SimulationEngine engine(sim::defaultAirspace());
    sim::EngagementManager::Config cfg;
    cfg.defendedAsset = {50000.0, 50000.0, 0.0};
    cfg.fuzeRadius    = 15.0;
    sim::EngagementManager mgr(engine, cfg);

    std::mt19937 rng(11);
    std::uniform_real_distribution<double> jitter(-8000.0, 8000.0);
    for (int i = 0; i < 24; ++i) {
        Entity h;
        // Closer, faster inbound salvo so intercepts resolve within the demo
        // window (and the intercept counter is visibly exercised in the feed).
        h.position = {64000.0 + jitter(rng), 64000.0 + jitter(rng),
                      8000.0 + jitter(rng) * 0.3};
        h.velocity = (cfg.defendedAsset - h.position).normalized() * 380.0;
        h.type     = EntityType::Hostile;
        engine.spawn(h);
    }
    // Background friendly air traffic so filtering is visible in the feed.
    for (int i = 0; i < 40; ++i) {
        Entity f;
        f.position = {jitter(rng) + 40000.0, jitter(rng) + 40000.0,
                      6000.0 + jitter(rng) * 0.2};
        f.velocity = {jitter(rng) * 0.02, jitter(rng) * 0.02, 0.0};
        f.type     = EntityType::Neutral;
        engine.spawn(f);
    }
    for (int i = 0; i < 24; ++i) {
        const double a = (2.0 * 3.14159265 * i) / 24.0;
        mgr.deployInterceptor(
            cfg.defendedAsset +
                Vector3{8000.0 * std::cos(a), 8000.0 * std::sin(a), 500.0},
            1200.0);
    }

    tlm::ShmPublisher publisher(tlm::kDefaultShmName, /*unlinkOnClose=*/true);
    std::vector<tlm::TelemetryRecord> records; // reused; no per-frame alloc
    records.reserve(tlm::kMaxRecordsPerFrame);

    std::cout << "== Telemetry publisher ==\n"
              << "  shm      : " << publisher.name() << "\n"
              << "  duration : " << durationSec << " s @ 60 Hz\n"
              << "  run the monitor:  ./telemetry_monitor\n\n";

    const auto start = std::chrono::steady_clock::now();
    std::uint64_t frameId = 0;
    const int totalFrames = static_cast<int>(durationSec * 60.0);
    for (int f = 0; f < totalFrames; ++f) {
        mgr.update(dt);

        records.clear();
        for (const Entity& e : engine.entities()) {
            if (!e.isActive()) continue;
            records.push_back(
                tlm::makeRecord(e, tlm::classifyThreat(e, cfg.defendedAsset)));
        }

        const std::uint64_t ts = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
        publisher.publish(frameId++, ts,
                          static_cast<std::uint32_t>(mgr.interceptCount()),
                          records.data(),
                          static_cast<std::uint32_t>(records.size()));

        std::this_thread::sleep_until(start + std::chrono::duration<double>(
                                                  (f + 1) * dt));
    }

    std::cout << "Telemetry publisher finished: " << frameId << " frames, "
              << mgr.interceptCount() << " intercepts.\n";
}
#endif // SIM_HAVE_TELEMETRY

} // namespace

int main(int argc, char** argv) {
    const std::string mode = (argc > 1) ? argv[1] : "";

    if (mode == "telemetry") {
#if defined(SIM_HAVE_TELEMETRY)
        const double seconds = (argc > 2) ? std::stod(argv[2]) : 20.0;
        runTelemetryPublisher(seconds);
        return 0;
#else
        std::cerr << "Telemetry (POSIX shared memory) is Linux-only; "
                     "this build does not include it.\n";
        return 1;
#endif
    }

    const std::size_t entityCount = (argc > 1) ? std::stoul(argv[1]) : 10000;
    const int         frames      = (argc > 2) ? std::stoi(argv[2]) : 60;

    std::cout << "Defense Simulation - Phases 1-3\n\n";
    runPerformanceDemo(entityCount, frames);
    runEngagementDemo();
    return 0;
}
