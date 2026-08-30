#ifndef SIM_STRUCTUREINDEX_HPP
#define SIM_STRUCTUREINDEX_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "sim/BoundingBox.hpp"
#include "sim/CityLayout.hpp"
#include "sim/Vector3.hpp"

namespace sim {

/**
 * @brief Uniform-grid broad phase over the static city skyline.
 *
 * The engine's per-frame "did this missile enter a building?" test used to scan
 * every CityStructure for every active entity — O(entities * buildings), which
 * is fine at a few thousand boxes but collapses at the 1:1 city's target scale
 * (~150k). This index hashes each building's ground footprint into a uniform XY
 * grid once at build time; a point query then hashes to a single cell and tests
 * only the handful of boxes registered there, making containment queries O(1)
 * on average regardless of city size.
 *
 * Why a grid and not the Octree: the Octree stores dynamic *points* (entities)
 * and is rebuilt every frame. The city is static *area* data lying on the
 * ground plane, so a build-once uniform grid — the classic broad-phase for many
 * small, similarly-sized static AABBs — is both simpler and faster here. Two
 * structures, each matched to the data it indexes.
 *
 * Correctness: a building is registered in every cell its footprint overlaps, so
 * if a point falls inside a building it necessarily shares that building's cell;
 * querying the single cell containing the point can therefore never miss a hit.
 */
class StructureIndex {
public:
    /// Default grid resolution (m). ~120 m keeps a Manhattan block to a handful
    /// of cells while holding the per-cell candidate count low.
    static constexpr double kDefaultCellSize = 120.0;

    /// (Re)build the grid from @p city. Safe to call with an empty city (the
    /// index then reports empty() and every query misses).
    void build(const std::vector<CityStructure>& city,
               double cellSize = kDefaultCellSize) {
        structures_ = city;
        cells_.clear();
        nx_ = ny_ = 0;
        if (structures_.empty()) return;

        cell_ = (cellSize > 0.0) ? cellSize : kDefaultCellSize;
        // World extent of all footprints on the XY (ground) plane.
        minX_ = minY_ = 1e300;
        double maxX = -1e300, maxY = -1e300;
        for (const CityStructure& s : structures_) {
            minX_ = std::min(minX_, s.box.min.x);
            minY_ = std::min(minY_, s.box.min.y);
            maxX  = std::max(maxX,  s.box.max.x);
            maxY  = std::max(maxY,  s.box.max.y);
        }
        nx_ = std::max(1, static_cast<int>(std::floor((maxX - minX_) / cell_)) + 1);
        ny_ = std::max(1, static_cast<int>(std::floor((maxY - minY_) / cell_)) + 1);
        cells_.assign(static_cast<std::size_t>(nx_) * ny_, {});

        for (std::size_t i = 0; i < structures_.size(); ++i) {
            const BoundingBox& b = structures_[i].box;
            const int x0 = clampX(cellX(b.min.x)), x1 = clampX(cellX(b.max.x));
            const int y0 = clampY(cellY(b.min.y)), y1 = clampY(cellY(b.max.y));
            for (int y = y0; y <= y1; ++y)
                for (int x = x0; x <= x1; ++x)
                    cells_[index(x, y)].push_back(static_cast<std::uint32_t>(i));
        }
    }

    /**
     * @brief First static structure whose volume contains @p p, or nullptr.
     *
     * Hashes @p p to its cell and tests only that cell's candidates, so cost is
     * independent of the total building count. The z (altitude) check is the
     * box's own containment test, so a missile clearing a building's roof is not
     * a hit.
     */
    const CityStructure* firstContaining(const Vector3& p) const {
        if (cells_.empty()) return nullptr;
        const int cx = cellX(p.x), cy = cellY(p.y);
        if (cx < 0 || cx >= nx_ || cy < 0 || cy >= ny_) return nullptr;
        for (std::uint32_t i : cells_[index(cx, cy)]) {
            const CityStructure& s = structures_[i];
            if (s.box.contains(p)) return &s;
        }
        return nullptr;
    }

    bool        empty() const { return structures_.empty(); }
    std::size_t structureCount() const { return structures_.size(); }
    int         gridWidth() const { return nx_; }
    int         gridHeight() const { return ny_; }

    /// Largest per-cell candidate count — a health check on cell sizing (tests).
    std::size_t maxBucket() const {
        std::size_t m = 0;
        for (const auto& c : cells_) m = std::max(m, c.size());
        return m;
    }

private:
    int cellX(double x) const { return static_cast<int>(std::floor((x - minX_) / cell_)); }
    int cellY(double y) const { return static_cast<int>(std::floor((y - minY_) / cell_)); }
    int clampX(int x) const { return std::min(std::max(x, 0), nx_ - 1); }
    int clampY(int y) const { return std::min(std::max(y, 0), ny_ - 1); }
    std::size_t index(int x, int y) const {
        return static_cast<std::size_t>(y) * nx_ + x;
    }

    std::vector<CityStructure>              structures_;
    std::vector<std::vector<std::uint32_t>> cells_;
    double cell_{kDefaultCellSize};
    double minX_{0.0}, minY_{0.0};
    int    nx_{0}, ny_{0};
};

} // namespace sim

#endif // SIM_STRUCTUREINDEX_HPP
