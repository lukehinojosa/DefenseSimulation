#include "sim/SimulationEngine.hpp"

namespace sim {

SimulationEngine::SimulationEngine(const BoundingBox& airspace,
                                   std::size_t threadCount)
    : airspace_(airspace),
      octree_(airspace),
      pool_(threadCount) {}

EntityId SimulationEngine::spawn(Entity e) {
    e.id = nextId_++;
    entities_.push_back(e);
    return e.id;
}

void SimulationEngine::step(double dt) {
    // Phase 1 — parallel kinematic integration. Each task index maps to a
    // single entity element, so writes never overlap and no locking is needed.
    pool_.parallelFor(0, entities_.size(), [this, dt](std::size_t i) {
        entities_[i].integrate(dt);
    });

    // Phase 2 — single-writer spatial rebuild from the updated positions.
    rebuildIndex();
}

void SimulationEngine::rebuildIndex() {
    octree_.clear();
    for (const Entity& e : entities_) {
        if (e.isActive()) {
            octree_.insert(e);
        }
    }
}

} // namespace sim
