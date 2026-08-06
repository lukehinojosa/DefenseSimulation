#ifndef SIM_SIMULATIONENGINE_HPP
#define SIM_SIMULATIONENGINE_HPP

#include <cstddef>
#include <memory>
#include <vector>

#include "sim/BoundingBox.hpp"
#include "sim/Entity.hpp"
#include "sim/Octree.hpp"
#include "sim/ThreadPool.hpp"

namespace sim {

/// Standard airspace volume: 100 km x 100 km laterally, 20 km vertically.
inline BoundingBox defaultAirspace() {
    return BoundingBox{{0.0, 0.0, 0.0}, {100000.0, 100000.0, 20000.0}};
}

/**
 * @brief Owns the entity population and drives the per-frame update.
 *
 * Each step() advances every active entity's kinematic state in parallel
 * across a worker pool and then rebuilds the spatial octree from the fresh
 * positions. The parallel phase writes only to disjoint entity elements, so
 * it requires no locking; the rebuild is the single-writer phase, after which
 * queryRange() may be called read-only.
 */
class SimulationEngine {
public:
    /**
     * @param airspace    World bounds for the octree.
     * @param threadCount Worker threads (0 = hardware concurrency).
     */
    explicit SimulationEngine(const BoundingBox& airspace = defaultAirspace(),
                              std::size_t threadCount = 0);

    /// Add an entity. Its id field is assigned by the engine and returned.
    EntityId spawn(Entity e);

    /// Advance the simulation by @p dt seconds and rebuild the spatial index.
    void step(double dt);

    /// Rebuild the octree from current entity positions without integrating.
    void rebuildIndex();

    // --- Spatial queries (valid after step()/rebuildIndex()) ---------------
    std::vector<OctreeItem> queryRange(const BoundingBox& range,
                                       std::uint32_t filter = FILTER_ALL) const {
        return octree_.queryRange(range, filter);
    }
    void queryRange(const BoundingBox& range,
                    std::uint32_t filter,
                    std::vector<OctreeItem>& out) const {
        octree_.queryRange(range, filter, out);
    }

    // --- Accessors ---------------------------------------------------------
    std::vector<Entity>&       entities()       { return entities_; }
    const std::vector<Entity>& entities() const { return entities_; }
    std::size_t                entityCount() const { return entities_.size(); }
    const Octree&              index() const { return octree_; }
    ThreadPool&                threadPool() { return pool_; }

private:
    std::vector<Entity> entities_;
    BoundingBox         airspace_;
    Octree              octree_;
    ThreadPool          pool_;
    EntityId            nextId_{0};
};

} // namespace sim

#endif // SIM_SIMULATIONENGINE_HPP
