#include <gtest/gtest.h>

#include <algorithm>
#include <random>
#include <vector>

#include "sim/BoundingBox.hpp"
#include "sim/Entity.hpp"
#include "sim/Octree.hpp"

using namespace sim;

namespace {

BoundingBox worldBounds() {
    return BoundingBox{{0.0, 0.0, 0.0}, {1000.0, 1000.0, 1000.0}};
}

// Reference O(n) range query used as ground truth for the octree.
std::vector<EntityId> bruteForce(const std::vector<OctreeItem>& items,
                                 const BoundingBox& range,
                                 std::uint32_t filter) {
    std::vector<EntityId> ids;
    for (const auto& it : items) {
        if (range.contains(it.position) && typeMatchesFilter(it.type, filter)) {
            ids.push_back(it.id);
        }
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

std::vector<EntityId> sortedIds(std::vector<OctreeItem> items) {
    std::vector<EntityId> ids;
    ids.reserve(items.size());
    for (const auto& it : items) ids.push_back(it.id);
    std::sort(ids.begin(), ids.end());
    return ids;
}

} // namespace

TEST(Octree, InsertionCountsAndBoundsRejection) {
    Octree tree(worldBounds());
    EXPECT_TRUE(tree.empty());

    EXPECT_TRUE(tree.insert(OctreeItem{1, {500.0, 500.0, 500.0}, EntityType::Hostile}));
    EXPECT_TRUE(tree.insert(OctreeItem{2, {10.0, 10.0, 10.0}, EntityType::Friendly}));
    EXPECT_EQ(tree.size(), 2u);

    // Outside world bounds -> rejected, count unchanged.
    EXPECT_FALSE(tree.insert(OctreeItem{3, {-1.0, 0.0, 0.0}, EntityType::Neutral}));
    EXPECT_FALSE(tree.insert(OctreeItem{4, {1000.0, 0.0, 0.0}, EntityType::Neutral}));
    EXPECT_EQ(tree.size(), 2u);
}

TEST(Octree, SubdivisionOccursOnceCapacityExceeded) {
    const std::size_t capacity = 4;
    Octree tree(worldBounds(), capacity, /*maxDepth=*/8);

    // Before exceeding capacity the tree is a single leaf (1 node).
    for (std::size_t i = 0; i < capacity; ++i) {
        tree.insert(OctreeItem{static_cast<EntityId>(i),
                               {static_cast<double>(i) * 100.0 + 10.0, 10.0, 10.0},
                               EntityType::Hostile});
    }
    EXPECT_EQ(tree.nodeCount(), 1u);

    // Spread points across octants so the split actually separates them.
    tree.clear();
    const Vector3 pts[] = {
        {100.0, 100.0, 100.0}, {900.0, 100.0, 100.0},
        {100.0, 900.0, 100.0}, {900.0, 900.0, 100.0},
        {100.0, 100.0, 900.0}, {900.0, 900.0, 900.0}
    };
    EntityId id = 0;
    for (const auto& p : pts) {
        tree.insert(OctreeItem{id++, p, EntityType::Hostile});
    }
    // Exceeding capacity=4 must have produced children (root + 8 = 9 nodes).
    EXPECT_GT(tree.nodeCount(), 1u);
    EXPECT_EQ(tree.size(), 6u);
}

TEST(Octree, HandlesCoincidentPointsBeyondCapacity) {
    // Many identical points cannot be separated by subdivision; the tree must
    // still store them all rather than loop forever or drop them.
    Octree tree(worldBounds(), /*capacity=*/2, /*maxDepth=*/4);
    for (EntityId i = 0; i < 50; ++i) {
        EXPECT_TRUE(tree.insert(OctreeItem{i, {500.0, 500.0, 500.0}, EntityType::Neutral}));
    }
    EXPECT_EQ(tree.size(), 50u);

    auto hits = tree.queryRange(worldBounds(), FILTER_ALL);
    EXPECT_EQ(hits.size(), 50u);
}

TEST(Octree, RangeQueryMatchesBruteForce) {
    std::mt19937 rng(2024);
    std::uniform_real_distribution<double> pos(0.0, 999.999);
    std::uniform_int_distribution<int> typ(0, 2);

    std::vector<OctreeItem> items;
    Octree tree(worldBounds(), /*capacity=*/8, /*maxDepth=*/10);
    for (EntityId i = 0; i < 5000; ++i) {
        OctreeItem it{i, {pos(rng), pos(rng), pos(rng)},
                      static_cast<EntityType>(typ(rng))};
        items.push_back(it);
        ASSERT_TRUE(tree.insert(it));
    }
    ASSERT_EQ(tree.size(), 5000u);

    // Check many random query volumes and every filter against ground truth.
    std::uniform_real_distribution<double> lo(0.0, 800.0);
    std::uniform_real_distribution<double> ext(20.0, 300.0);
    const std::uint32_t filters[] = {FILTER_ALL, FILTER_HOSTILE,
                                     FILTER_FRIENDLY, FILTER_NEUTRAL,
                                     FILTER_HOSTILE | FILTER_NEUTRAL};

    for (int q = 0; q < 200; ++q) {
        Vector3 mn{lo(rng), lo(rng), lo(rng)};
        Vector3 mx{mn.x + ext(rng), mn.y + ext(rng), mn.z + ext(rng)};
        BoundingBox range{mn, mx};
        const std::uint32_t filter = filters[q % 5];

        auto got = sortedIds(tree.queryRange(range, filter));
        auto want = bruteForce(items, range, filter);
        ASSERT_EQ(got, want) << "mismatch on query " << q << " filter " << filter;
    }
}

TEST(Octree, ClearResetsTree) {
    Octree tree(worldBounds(), 2, 6);
    for (EntityId i = 0; i < 100; ++i) {
        tree.insert(OctreeItem{i, {static_cast<double>(i) * 5.0, 1.0, 1.0},
                               EntityType::Hostile});
    }
    EXPECT_EQ(tree.size(), 100u);
    EXPECT_GT(tree.nodeCount(), 1u);

    tree.clear();
    EXPECT_EQ(tree.size(), 0u);
    EXPECT_EQ(tree.nodeCount(), 1u);
    EXPECT_TRUE(tree.queryRange(worldBounds()).empty());
}

TEST(Octree, FilterExcludesNonMatchingTypes) {
    Octree tree(worldBounds(), 4, 8);
    tree.insert(OctreeItem{1, {100.0, 100.0, 100.0}, EntityType::Hostile});
    tree.insert(OctreeItem{2, {110.0, 110.0, 110.0}, EntityType::Friendly});
    tree.insert(OctreeItem{3, {120.0, 120.0, 120.0}, EntityType::Neutral});

    auto hostiles = tree.queryRange(worldBounds(), QUERY_HOSTILE_ONLY);
    ASSERT_EQ(hostiles.size(), 1u);
    EXPECT_EQ(hostiles.front().id, 1u);
}
