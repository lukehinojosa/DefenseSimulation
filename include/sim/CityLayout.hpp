#ifndef SIM_CITYLAYOUT_HPP
#define SIM_CITYLAYOUT_HPP

#include <cstdint>
#include <vector>

#include "sim/BoundingBox.hpp"
#include "sim/Vector3.hpp"

namespace sim {

/// Kind of static ground structure inside the defended area. Kind drives the
/// display style and whether a leaked threat striking it counts as a defended-
/// asset loss (skyscrapers and the hospital are protected; the suburb blocks
/// are populated ground the shield exists to cover as well).
enum class StructureKind : std::uint8_t {
    Skyscraper = 0,
    Hospital   = 1,
    Suburb     = 2
};

/// A static, axis-aligned structure volume anchored at ground level (z = 0).
/// The engine treats it as a hard collision boundary; the visualizer draws it
/// as a stylized block. Heights are display-exaggerated relative to real life
/// so the skyline reads at a 1 unit = 1 km world scale.
struct CityStructure {
    BoundingBox   box;
    StructureKind kind;
};

/// Footprint/half-extent helper (meters) centered at @p c on the ground.
inline BoundingBox groundBox(const Vector3& c, double halfX, double halfY,
                             double height) {
    return BoundingBox{{c.x - halfX, c.y - halfY, 0.0},
                       {c.x + halfX, c.y + halfY, height}};
}

/**
 * @brief The default protected city clustered around the defended asset.
 *
 * A downtown of skyscrapers over the battery, a hospital just to the north,
 * and a residential suburb grid to the south-west. Returned in world meters so
 * the engine collides against it and the display maps it through worldToRay().
 */
inline std::vector<CityStructure> defaultCityLayout(const Vector3& center) {
    std::vector<CityStructure> city;

    // Downtown: a tight cluster of tall towers right over the defended point.
    const double towerH[] = {1500.0, 1200.0, 1800.0, 1000.0, 1350.0};
    const double towerX[] = {-600.0,  700.0,   0.0,  -900.0,  1000.0};
    const double towerY[] = { 500.0, -300.0, 1100.0,  -800.0,  -100.0};
    for (int i = 0; i < 5; ++i) {
        city.push_back({groundBox({center.x + towerX[i], center.y + towerY[i], 0.0},
                                  260.0, 260.0, towerH[i]),
                        StructureKind::Skyscraper});
    }

    // Hospital: a broad, lower block to the north of downtown.
    city.push_back({groundBox({center.x - 200.0, center.y + 3200.0, 0.0},
                              900.0, 600.0, 600.0),
                    StructureKind::Hospital});

    // Residential suburb: a low grid of blocks to the south-west.
    for (int gx = 0; gx < 4; ++gx) {
        for (int gy = 0; gy < 3; ++gy) {
            const Vector3 c{center.x - 4200.0 + gx * 1100.0,
                            center.y - 4200.0 + gy * 1100.0, 0.0};
            city.push_back({groundBox(c, 380.0, 380.0, 250.0),
                            StructureKind::Suburb});
        }
    }
    return city;
}

} // namespace sim

#endif // SIM_CITYLAYOUT_HPP
