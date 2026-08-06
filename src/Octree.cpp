#include "sim/Octree.hpp"

namespace sim {

Octree::Octree(const BoundingBox& bounds, std::size_t capacity, int maxDepth)
    : root_(std::make_unique<Node>(bounds)),
      capacity_(capacity == 0 ? 1 : capacity),
      maxDepth_(maxDepth < 0 ? 0 : maxDepth) {}

bool Octree::insert(const OctreeItem& item) {
    if (!root_->bounds.contains(item.position)) {
        return false;
    }
    insertInto(*root_, item, 0);
    ++count_;
    return true;
}

void Octree::clear() {
    root_ = std::make_unique<Node>(root_->bounds);
    count_ = 0;
}

void Octree::insertInto(Node& node, const OctreeItem& item, int depth) {
    // Internal node: descend into the owning octant.
    if (!node.leaf) {
        const int octant = octantOf(node, item.position);
        insertInto(*node.children[octant], item, depth + 1);
        return;
    }

    // Leaf with headroom, or a leaf at max depth that must absorb overflow.
    if (node.items.size() < capacity_ || depth >= maxDepth_) {
        node.items.push_back(item);
        return;
    }

    // Leaf is full and may still split: subdivide, redistribute, then retry.
    subdivide(node);
    for (const OctreeItem& existing : node.items) {
        const int octant = octantOf(node, existing.position);
        node.children[octant]->items.push_back(existing);
    }
    node.items.clear();
    node.items.shrink_to_fit();

    const int octant = octantOf(node, item.position);
    insertInto(*node.children[octant], item, depth + 1);
}

void Octree::subdivide(Node& node) {
    const Vector3 mn = node.bounds.min;
    const Vector3 mx = node.bounds.max;
    const Vector3 c  = node.bounds.center();

    // Child i is selected by bit flags: bit0 = X high, bit1 = Y high,
    // bit2 = Z high. octantOf() uses the same encoding.
    for (int i = 0; i < 8; ++i) {
        const double loX = (i & 1) ? c.x : mn.x;
        const double hiX = (i & 1) ? mx.x : c.x;
        const double loY = (i & 2) ? c.y : mn.y;
        const double hiY = (i & 2) ? mx.y : c.y;
        const double loZ = (i & 4) ? c.z : mn.z;
        const double hiZ = (i & 4) ? mx.z : c.z;
        node.children[i] = std::make_unique<Node>(
            BoundingBox{{loX, loY, loZ}, {hiX, hiY, hiZ}});
    }
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
    queryNode(*root_, range, filter, out);
}

void Octree::queryNode(const Node& node,
                       const BoundingBox& range,
                       std::uint32_t filter,
                       std::vector<OctreeItem>& out) const {
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

    for (const auto& child : node.children) {
        queryNode(*child, range, filter, out);
    }
}

std::size_t Octree::nodeCount() const {
    return countNodes(*root_);
}

std::size_t Octree::countNodes(const Node& node) {
    std::size_t total = 1;
    if (!node.leaf) {
        for (const auto& child : node.children) {
            total += countNodes(*child);
        }
    }
    return total;
}

} // namespace sim
