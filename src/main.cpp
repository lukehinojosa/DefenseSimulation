// Defense Simulation — Phase 1 demonstration harness.
//
// Spawns a large mixed population of hostile/friendly/neutral tracks inside
// the standard airspace, advances them with the multithreaded engine, and
// reports per-frame timing plus a sample spatial query so the core engine can
// be exercised end-to-end from the command line.

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <random>

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

} // namespace

int main(int argc, char** argv) {
    const std::size_t entityCount = (argc > 1) ? std::stoul(argv[1]) : 10000;
    const int         frames      = (argc > 2) ? std::stoi(argv[2]) : 60;
    const double      dt          = 1.0 / 60.0; // 60 Hz

    sim::SimulationEngine engine(sim::defaultAirspace());
    std::mt19937 rng(1337);

    for (std::size_t i = 0; i < entityCount; ++i) {
        engine.spawn(makeRandomEntity(rng, engine.index().bounds()));
    }

    std::cout << "Defense Simulation - Phase 1\n"
              << "  entities : " << entityCount << "\n"
              << "  frames   : " << frames << "\n"
              << "  workers  : " << engine.threadPool().threadCount() << "\n\n";

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
              << "Timing:\n"
              << "  total     : " << totalMs << " ms\n"
              << "  per frame : " << perFrameMs << " ms  ("
              << (1000.0 / perFrameMs) << " fps headroom)\n\n";

    // Sample query: hostile tracks within a 10 km cube at the airspace center.
    const Vector3 c = engine.index().bounds().center();
    const sim::BoundingBox region = sim::BoundingBox::fromCenterHalf(c, 5000.0);
    const auto hostiles = engine.queryRange(region, sim::QUERY_HOSTILE_ONLY);

    std::cout << "Spatial query (hostiles in 10km cube at airspace center):\n"
              << "  octree nodes : " << engine.index().nodeCount() << "\n"
              << "  hostiles hit : " << hostiles.size() << "\n";

    return 0;
}
