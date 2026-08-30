#include <gtest/gtest.h>

#include <random>
#include <vector>

#include "sim/BoundingBox.hpp"
#include "sim/CityLayout.hpp"
#include "sim/StructureIndex.hpp"
#include "sim/Vector3.hpp"

using sim::BoundingBox;
using sim::CityStructure;
using sim::MapKind;
using sim::StructureIndex;
using sim::Vector3;

namespace {

// A building footprint [x0,x1) x [y0,y1) rising to `height`.
CityStructure block(double x0, double y0, double x1, double y1, double height,
                    MapKind kind = MapKind::Block) {
    return CityStructure{BoundingBox{{x0, y0, 0.0}, {x1, y1, height}}, kind};
}

// Reference O(n) containment, for cross-checking the grid against brute force.
const CityStructure* bruteContaining(const std::vector<CityStructure>& city,
                                     const Vector3& p) {
    for (const CityStructure& s : city)
        if (s.box.contains(p)) return &s;
    return nullptr;
}

} // namespace

TEST(StructureIndex, EmptyCityAlwaysMisses) {
    StructureIndex idx;
    idx.build({});
    EXPECT_TRUE(idx.empty());
    EXPECT_EQ(idx.firstContaining({0.0, 0.0, 0.0}), nullptr);
}

TEST(StructureIndex, HitsInsideAndMissesOutsideAndAboveRoof) {
    std::vector<CityStructure> city = {
        block(100.0, 100.0, 140.0, 140.0, 200.0),
        block(500.0, 300.0, 560.0, 360.0, 90.0),
    };
    StructureIndex idx;
    idx.build(city);

    // Inside the first tower.
    const CityStructure* hit = idx.firstContaining({120.0, 120.0, 50.0});
    ASSERT_NE(hit, nullptr);
    EXPECT_DOUBLE_EQ(hit->box.max.z, 200.0);

    // Between the two buildings: a miss.
    EXPECT_EQ(idx.firstContaining({300.0, 200.0, 10.0}), nullptr);
    // Over the second building's roof (z above its height): a miss.
    EXPECT_EQ(idx.firstContaining({530.0, 330.0, 120.0}), nullptr);
    // Far outside the whole grid: a miss (no crash on out-of-range cells).
    EXPECT_EQ(idx.firstContaining({-9999.0, -9999.0, 5.0}), nullptr);
}

TEST(StructureIndex, BuildingSpanningManyCellsIsFoundEverywhere) {
    // A footprint far larger than the cell size must register in every cell it
    // overlaps, so a point anywhere inside it is a hit.
    std::vector<CityStructure> city = {block(0.0, 0.0, 1000.0, 40.0, 60.0)};
    StructureIndex idx;
    idx.build(city, /*cellSize=*/50.0);
    for (double x = 5.0; x < 1000.0; x += 37.0)
        EXPECT_NE(idx.firstContaining({x, 20.0, 10.0}), nullptr) << "x=" << x;
}

TEST(StructureIndex, MatchesBruteForceOnRandomCity) {
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> pos(0.0, 4000.0);
    std::uniform_real_distribution<double> ext(10.0, 80.0);
    std::uniform_real_distribution<double> hgt(20.0, 300.0);

    std::vector<CityStructure> city;
    for (int i = 0; i < 800; ++i) {
        const double x = pos(rng), y = pos(rng);
        city.push_back(block(x, y, x + ext(rng), y + ext(rng), hgt(rng)));
    }
    StructureIndex idx;
    idx.build(city);
    EXPECT_EQ(idx.structureCount(), city.size());

    // The grid answer must agree with brute force on containment for every
    // probe (a hit is the *same* box; a miss is a miss in both).
    std::uniform_real_distribution<double> probe(-200.0, 4200.0);
    std::uniform_real_distribution<double> zp(0.0, 320.0);
    for (int i = 0; i < 20000; ++i) {
        const Vector3 p{probe(rng), probe(rng), zp(rng)};
        const CityStructure* g = idx.firstContaining(p);
        const bool bruteHit = bruteContaining(city, p) != nullptr;
        EXPECT_EQ(g != nullptr, bruteHit);
        if (g) EXPECT_TRUE(g->box.contains(p));
    }
}

TEST(StructureIndex, RebuildReplacesContents) {
    StructureIndex idx;
    idx.build({block(0.0, 0.0, 100.0, 100.0, 50.0)});
    EXPECT_NE(idx.firstContaining({50.0, 50.0, 10.0}), nullptr);

    idx.build({block(1000.0, 1000.0, 1100.0, 1100.0, 50.0)});
    EXPECT_EQ(idx.firstContaining({50.0, 50.0, 10.0}), nullptr);   // old gone
    EXPECT_NE(idx.firstContaining({1050.0, 1050.0, 10.0}), nullptr); // new present
}
