#ifndef SIM_OCTREE_HPP
#define SIM_OCTREE_HPP

#include <cstdint>
#include <memory>
#include <vector>

#include "sim/BoundingBox.hpp"
#include "sim/Entity.hpp"
#include "sim/Vector3.hpp"

namespace sim {

/**
 * @brief A single spatial record held by the Octree.
 *
 * The tree stores a self-contained copy of the fields it needs for spatial
 * filtering (id, position, type) rather than a pointer back into the entity
 * array. This keeps queries cache-local and free of aliasing hazards while
 * the entity array is being mutated on other threads.
 */
struct OctreeItem {
    EntityId   id{0};
    Vector3    position{};
    EntityType type{EntityType::Neutral};
};

/**
 * @brief Recursive, axis-aligned 3D octree for point data.
 *
 * The tree partitions a fixed world volume into eight octants per level. A
 * node holds items directly until it exceeds its capacity, at which point it
 * subdivides and pushes its items down to the children -- unless the maximum
 * depth has been reached, in which case the node is allowed to overflow
 * (necessary because many coincident points cannot be separated by further
 * subdivision).
 *
 * Concurrency model: the tree is built single-writer per frame (or in parallel
 * across disjoint subtrees) and then queried read-only by many threads. It is
 * not internally synchronized for concurrent insert + query.
 */
class Octree {
public:
    /// Default fan-out threshold before a leaf subdivides.
    static constexpr std::size_t kDefaultCapacity = 16;
    /// Hard recursion cap to bound memory on degenerate/clustered inputs.
    static constexpr int kDefaultMaxDepth = 12;

    Octree(const BoundingBox& bounds,
           std::size_t capacity = kDefaultCapacity,
           int maxDepth = kDefaultMaxDepth);

    /**
     * @brief Insert a point item.
     * @return true if inserted; false if the point lies outside world bounds.
     */
    bool insert(const OctreeItem& item);

    /// Convenience overload building the item from an Entity.
    bool insert(const Entity& e) {
        return insert(OctreeItem{e.id, e.position, e.type});
    }

    /// Remove all items and collapse the tree back to a single empty root.
    void clear();

    /**
     * @brief Collect every item whose position lies inside @p range and whose
     *        type passes @p filter.
     * @param range  Query volume (AABB).
     * @param filter EntityFilter bitmask; FILTER_ALL returns all types.
     * @param out    Destination vector; results are appended (not cleared).
     */
    void queryRange(const BoundingBox& range,
                    std::uint32_t filter,
                    std::vector<OctreeItem>& out) const;

    /// Allocating convenience wrapper around queryRange().
    std::vector<OctreeItem> queryRange(const BoundingBox& range,
                                       std::uint32_t filter = FILTER_ALL) const {
        std::vector<OctreeItem> out;
        queryRange(range, filter, out);
        return out;
    }

    const BoundingBox& bounds() const { return root_->bounds; }
    std::size_t size() const { return count_; }
    bool empty() const { return count_ == 0; }

    /// Total number of allocated nodes; primarily for tests/instrumentation.
    std::size_t nodeCount() const;

private:
    struct Node {
        BoundingBox              bounds;
        std::vector<OctreeItem>  items;
        std::unique_ptr<Node>    children[8];
        bool                     leaf{true};

        explicit Node(const BoundingBox& b) : bounds(b) {}
    };

    void insertInto(Node& node, const OctreeItem& item, int depth);
    void subdivide(Node& node);
    static int octantOf(const Node& node, const Vector3& p);
    void queryNode(const Node& node,
                   const BoundingBox& range,
                   std::uint32_t filter,
                   std::vector<OctreeItem>& out) const;
    static std::size_t countNodes(const Node& node);

    std::unique_ptr<Node> root_;
    std::size_t           capacity_;
    int                   maxDepth_;
    std::size_t           count_{0};
};

} // namespace sim

#endif // SIM_OCTREE_HPP
