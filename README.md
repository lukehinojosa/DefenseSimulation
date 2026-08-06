# Real-Time Defense Simulation & Telemetry Suite

A production-grade C++17 systems project modeling airspace threat tracking and
interception. Built to demonstrate the low-level engineering that defense
programs care about: cache-friendly data layout, hand-written spatial math,
lock-free-style parallelism, and rigorous unit testing.

> **Status:** Phase 1 complete — core engine, spatial math, and the
> multithreaded octree. Phases 2 (ProNav guidance) and 3 (Linux IPC telemetry)
> are planned; see [`Plan.md`](Plan.md).

---

## Phase 1 — Core Engine & Spatial Math

| Deliverable | Where |
|---|---|
| 3D vector math (dot, cross, magnitude, normalize, scale) | [`include/sim/Vector3.hpp`](include/sim/Vector3.hpp) |
| Entity model (id, position, velocity, type, status) + kinematics | [`include/sim/Entity.hpp`](include/sim/Entity.hpp) |
| Axis-aligned bounding box (containment, intersection) | [`include/sim/BoundingBox.hpp`](include/sim/BoundingBox.hpp) |
| Recursive 3D octree with filtered range queries | [`include/sim/Octree.hpp`](include/sim/Octree.hpp) · [`src/Octree.cpp`](src/Octree.cpp) |
| Reusable worker-thread pool (`parallelFor`) | [`include/sim/ThreadPool.hpp`](include/sim/ThreadPool.hpp) |
| Simulation engine: parallel update + spatial rebuild | [`include/sim/SimulationEngine.hpp`](include/sim/SimulationEngine.hpp) · [`src/SimulationEngine.cpp`](src/SimulationEngine.cpp) |
| GoogleTest suite (26 tests) | [`tests/`](tests) |

### Design highlights

- **Cache-friendly entities.** `Entity` is a standard-layout, trivially
  copyable aggregate so 10k+ tracks live in a flat contiguous array and can be
  `memcpy`'d straight into the Phase 3 telemetry pipeline.
- **Octree that stays correct under stress.** Nodes subdivide only when they
  exceed capacity and are below a depth cap; coincident points that cannot be
  separated by further subdivision are absorbed rather than dropped or looped
  on. Half-open box bounds keep every point in exactly one octant.
- **Query correctness is proven, not asserted.** `test_octree.cpp` validates
  200 randomized range queries across every allegiance filter against a brute
  force O(n) reference over 5,000 points.
- **Parallelism without locks on the hot path.** The per-frame kinematic update
  partitions the entity array into disjoint chunks across a persistent thread
  pool, so worker threads never touch overlapping memory — no mutex on the
  update itself. The pool reuses a single completion primitive per batch to
  stay portable across pthread implementations.

### Performance (20-thread machine, Release)

| Entities | Per-frame (integrate + octree rebuild) | 60 Hz headroom |
|---|---|---|
| 10,000 | ~1.8 ms | ~9x |
| 50,000 | ~5.6 ms | ~3x |

The 60 Hz frame budget is 16.7 ms; Phase 1 clears it with room to spare.

---

## Building

Requires CMake 3.20+ and a C++17 compiler. GoogleTest is fetched
automatically via CMake `FetchContent` (needs network access on first
configure).

### Linux / macOS

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/defense_sim            # defaults: 10000 entities, 60 frames
./build/defense_sim 50000 120  # entities, frames
```

### Windows (MinGW)

```bash
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/defense_sim.exe
```

Build without tests with `-DSIM_BUILD_TESTS=OFF`.

---

## Layout

```
include/sim/   Public headers (math, entities, octree, thread pool, engine)
src/           Octree / engine implementation and the demo executable
tests/         GoogleTest suites (one per component)
Plan.md        Full three-phase roadmap
```

## Target environment

Written to be portable C++17. The reference target for later phases is
Linux (Ubuntu 22.04 / RHEL), where Phase 3 adds POSIX shared-memory and UDP
telemetry and the project is profiled with Valgrind and `perf`.
