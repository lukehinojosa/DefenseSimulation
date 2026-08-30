#include "sim/Octree.hpp"

namespace sim {

Octree::Octree(const BoundingBox& bounds, std::size_t capacity, int maxDepth)
    : capacity_(capacity == 0 ? 1 : capacity),
      maxDepth_(maxDepth < 0 ? 0 : maxDepth) {
    allocNode(bounds); // root at index 0
}

int Octree::allocNode(const BoundingBox& b) {
    // Reuse a pooled node if one is free; only grow the pool when it is full.
    // Growing may reallocate nodes_, which is why the tree is addressed purely by
    // index -- no Node& is held across an allocation.
    if (nodeCount_ == nodes_.size()) {
        nodes_.emplace_back();
    }
    const int idx = static_cast<int>(nodeCount_++);
    Node& n = nodes_[idx];
    n.bounds = b;
    n.items.clear(); // keeps the vector's capacity for reuse across frames
    for (int i = 0; i < 8; ++i) n.children[i] = kNone;
    n.leaf = true;
    return idx;
}

bool Octree::insert(const OctreeItem& item) {
    if (!nodes_[0].bounds.contains(item.position)) {
        return false;
    }
    insertInto(0, item, 0);
    ++count_;
    return true;
}

void Octree::clear() {
    // Reset the pool to just the root, keeping every node object (and its
    // item-vector capacity) allocated for reuse -- no per-frame free/alloc.
    const BoundingBox rootBounds = nodes_[0].bounds;
    nodeCount_ = 0;
    allocNode(rootBounds);
    count_ = 0;
}

void Octree::insertInto(int nodeIdx, const OctreeItem& item, int depth) {
    // Internal node: descend into the owning octant.
    if (!nodes_[nodeIdx].leaf) {
        const int octant = octantOf(nodes_[nodeIdx], item.position);
        insertInto(nodes_[nodeIdx].children[octant], item, depth + 1);
        return;
    }

    // Leaf with headroom, or a leaf at max depth that must absorb overflow.
    if (nodes_[nodeIdx].items.size() < capacity_ || depth >= maxDepth_) {
        nodes_[nodeIdx].items.push_back(item);
        return;
    }

    // Leaf is full and may still split: subdivide, redistribute, then retry.
    // subdivide() may reallocate the pool, so re-address by index afterward.
    subdivide(nodeIdx);
    {
        Node& node = nodes_[nodeIdx];
        for (const OctreeItem& existing : node.items) {
            const int octant = octantOf(node, existing.position);
            // Pushing into a child's own item vector never reallocates nodes_,
            // so `node` (and the range being iterated) stay valid here.
            nodes_[node.children[octant]].items.push_back(existing);
        }
        node.items.clear(); // keep capacity; the buffer is reused next frame
    }

    const int octant = octantOf(nodes_[nodeIdx], item.position);
    insertInto(nodes_[nodeIdx].children[octant], item, depth + 1);
}

void Octree::subdivide(int nodeIdx) {
    // Capture the parent's bounds by value first: allocating the children below
    // can reallocate the pool and invalidate any reference into it.
    const BoundingBox pb = nodes_[nodeIdx].bounds;
    const Vector3 mn = pb.min;
    const Vector3 mx = pb.max;
    const Vector3 c  = pb.center();

    // Child i is selected by bit flags: bit0 = X high, bit1 = Y high,
    // bit2 = Z high. octantOf() uses the same encoding.
    int kids[8];
    for (int i = 0; i < 8; ++i) {
        const double loX = (i & 1) ? c.x : mn.x;
        const double hiX = (i & 1) ? mx.x : c.x;
        const double loY = (i & 2) ? c.y : mn.y;
        const double hiY = (i & 2) ? mx.y : c.y;
        const double loZ = (i & 4) ? c.z : mn.z;
        const double hiZ = (i & 4) ? mx.z : c.z;
        kids[i] = allocNode(BoundingBox{{loX, loY, loZ}, {hiX, hiY, hiZ}});
    }
    // Wire the children in only after every allocation is done.
    Node& node = nodes_[nodeIdx];
    for (int i = 0; i < 8; ++i) node.children[i] = kids[i];
    node.leaf = false;
}

int Octree::octantOf(const Node& node, const Vector3& p) {
    const Vector3 c = node.bounds.center();
    int octant = 0;
    if (p.x >= c.x) octant |= 1;
    if (p.y >= c.y) octant |= 2;
    if (p.z >= c.z) octant |= 4;
    return octant;
}

void Octree::queryRange(const BoundingBox& range,
                        std::uint32_t filter,
                        std::vector<OctreeItem>& out) const {
    queryNode(0, range, filter, out);
}

void Octree::queryNode(int nodeIdx,
                       const BoundingBox& range,
                       std::uint32_t filter,
                       std::vector<OctreeItem>& out) const {
    const Node& node = nodes_[nodeIdx];
    // Prune whole subtrees whose bounds miss the query volume.
    if (!node.bounds.intersects(range)) {
        return;
    }

    if (node.leaf) {
        for (const OctreeItem& item : node.items) {
            if (range.contains(item.position) &&
                typeMatchesFilter(item.type, filter)) {
                out.push_back(item);
            }
        }
        return;
    }

    for (const int child : node.children) {
        queryNode(child, range, filter, out);
    }
}

std::size_t Octree::nodeCount() const {
    return countNodes(0);
}

std::size_t Octree::countNodes(int nodeIdx) const {
    const Node& node = nodes_[nodeIdx];
    std::size_t total = 1;
    if (!node.leaf) {
        for (const int child : node.children) {
            total += countNodes(child);
        }
    }
    return total;
}

} // namespace sim
