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

#include "sim/EngagementManager.hpp"
#include "sim/SimulationEngine.hpp"

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

} // namespace

int main(int argc, char** argv) {
    const std::size_t entityCount = (argc > 1) ? std::stoul(argv[1]) : 10000;
    const int         frames      = (argc > 2) ? std::stoi(argv[2]) : 60;

    std::cout << "Defense Simulation - Phases 1 & 2\n\n";
    runPerformanceDemo(entityCount, frames);
    runEngagementDemo();
    return 0;
}
