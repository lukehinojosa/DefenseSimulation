#ifndef SIM_OCTREE_HPP
#define SIM_OCTREE_HPP

#include <cstdint>
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
 *
 * Storage: nodes live in one contiguous pool (a std::vector) and reference their
 * children by index, not by owning pointer. clear() resets a live-node counter
 * instead of freeing, so the node objects -- and their item-vector capacity --
 * are reused across frames. This removes the per-frame malloc/free churn of a
 * pointer-linked tree (the dominant cache-miss source when rebuilding 60x/sec)
 * and lays the nodes out contiguously for a warmer traversal.
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

    const BoundingBox& bounds() const { return nodes_[0].bounds; }
    std::size_t size() const { return count_; }
    bool empty() const { return count_ == 0; }

    /// Number of nodes in the current tree; primarily for tests/instrumentation.
    std::size_t nodeCount() const;

private:
    // A node references its 8 children by pool index (kNone when it is a leaf).
    static constexpr int kNone = -1;
    struct Node {
        BoundingBox              bounds;
        std::vector<OctreeItem>  items;
        int                      children[8];
        bool                     leaf{true};
    };

    /// Claim a node from the pool (reusing storage), initialised as an empty
    /// leaf with the given bounds. Returns its pool index. May reallocate the
    /// pool, so callers must address nodes by index, never by a held reference.
    int  allocNode(const BoundingBox& b);
    void insertInto(int nodeIdx, const OctreeItem& item, int depth);
    void subdivide(int nodeIdx);
    static int octantOf(const Node& node, const Vector3& p);
    void queryNode(int nodeIdx,
                   const BoundingBox& range,
                   std::uint32_t filter,
                   std::vector<OctreeItem>& out) const;
    std::size_t countNodes(int nodeIdx) const;

    std::vector<Node> nodes_;        ///< Contiguous node pool; root is index 0.
    std::size_t       nodeCount_{0}; ///< Live nodes this frame (<= nodes_.size()).
    std::size_t       capacity_;
    int               maxDepth_;
    std::size_t       count_{0};
};

} // namespace sim

#endif // SIM_OCTREE_HPP
