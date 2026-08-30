#ifndef SIM_CITYLAYOUT_HPP
#define SIM_CITYLAYOUT_HPP

#include <cmath>
#include <vector>

#include "sim/BoundingBox.hpp"
#include "sim/MapData.hpp"
#include "sim/Vector3.hpp"

namespace sim {

/// A static structure volume for the engine: the world-axis bounding box of a
/// placed building instance. The engine treats it as a hard collision boundary
/// and ignores @c kind; the visualizer draws the true oriented primitive.
struct CityStructure {
    BoundingBox box;
    MapKind     kind;
};

/**
 * @brief Collision volumes from loaded flyweight instances.
 *
 * Each instance (an oriented box or cylinder in local metres about the map
 * origin) is placed at @p center and scaled by @p scale, then reduced to its
 * world-axis-aligned bounding box for the engine's coarse "did a missile enter
 * a building" test. At @p scale = 1 the city is rendered 1:1.
 */
inline std::vector<CityStructure> cityStructuresFromInstances(
    const std::vector<MapInstance>& instances, const Vector3& center,
    double scale = 1.0) {
    std::vector<CityStructure> city;
    city.reserve(instances.size());
    for (const MapInstance& m : instances) {
        const double c = std::cos(m.angle), s = std::sin(m.angle);
        // Half-extents of the oriented footprint projected onto the world axes.
        const double ax = (std::abs(m.hx * c) + std::abs(m.hy * s)) * scale;
        const double ay = (std::abs(m.hx * s) + std::abs(m.hy * c)) * scale;
        const double ex = center.x + m.cx * scale;
        const double ey = center.y + m.cy * scale;
        city.push_back({BoundingBox{{ex - ax, ey - ay, 0.0},
                                    {ex + ax, ey + ay, m.height * scale}},
                        m.kind});
    }
    return city;
}

} // namespace sim

#endif // SIM_CITYLAYOUT_HPP
