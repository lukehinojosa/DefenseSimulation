#ifndef SIM_BOUNDINGBOX_HPP
#define SIM_BOUNDINGBOX_HPP

#include "sim/Vector3.hpp"

namespace sim {

/**
 * @brief Axis-aligned bounding box (AABB) defined by its min and max corners.
 *
 * Used both as the spatial extent of an Octree node and as the query volume
 * passed to queryRange(). The box is treated as a half-open region on its
 * upper faces so that an entity lying exactly on a subdivision plane lands
 * in exactly one child octant.
 */
struct BoundingBox {
    Vector3 min{};
    Vector3 max{};

    constexpr BoundingBox() = default;
    constexpr BoundingBox(const Vector3& lo, const Vector3& hi) : min(lo), max(hi) {}

    /// Construct from a center point and a half-extent (equal on all axes).
    static constexpr BoundingBox fromCenterHalf(const Vector3& center, double half) {
        return {
            {center.x - half, center.y - half, center.z - half},
            {center.x + half, center.y + half, center.z + half}
        };
    }

    constexpr Vector3 center() const {
        return {(min.x + max.x) * 0.5, (min.y + max.y) * 0.5, (min.z + max.z) * 0.5};
    }

    constexpr Vector3 size() const { return max - min; }

    /**
     * @brief Point-containment test using half-open upper bounds.
     *
     * A point on a min face is inside; a point on a max face is not. This
     * keeps octant assignment unambiguous during subdivision.
     */
    constexpr bool contains(const Vector3& p) const {
        return p.x >= min.x && p.x < max.x &&
               p.y >= min.y && p.y < max.y &&
               p.z >= min.z && p.z < max.z;
    }

    /// @return true if this box overlaps @p other (touching faces count).
    constexpr bool intersects(const BoundingBox& other) const {
        return min.x <= other.max.x && max.x >= other.min.x &&
               min.y <= other.max.y && max.y >= other.min.y &&
               min.z <= other.max.z && max.z >= other.min.z;
    }
};

} // namespace sim

#endif // SIM_BOUNDINGBOX_HPP
