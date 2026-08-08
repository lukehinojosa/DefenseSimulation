# Real-Time Defense Simulation & Telemetry Suite

A production-grade C++17 systems project modeling airspace threat tracking and
interception. Built to demonstrate the low-level engineering that defense
programs care about: cache-friendly data layout, hand-written spatial math,
lock-free-style parallelism, and rigorous unit testing.

> **Status:** All three phases complete — core engine + spatial math,
> closed-loop ProNav guidance with automated engagement, and a Linux IPC
> telemetry pipeline (POSIX shared memory + UDP) feeding a separate monitor
> process. See [`Plan.md`](Plan.md).

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

## Phase 2 — ProNav Guidance & Intercept

| Deliverable | Where |
|---|---|
| Proportional Navigation guidance law + LOS-rate / closing-speed / time-to-go math | [`include/sim/Guidance.hpp`](include/sim/Guidance.hpp) |
| Automated engagement manager: threat priority queue, target assignment, proximity fuze | [`include/sim/EngagementManager.hpp`](include/sim/EngagementManager.hpp) · [`src/EngagementManager.cpp`](src/EngagementManager.cpp) |
| ProNav convergence + engagement GTest suites (13 tests) | [`tests/test_guidance.cpp`](tests/test_guidance.cpp) · [`tests/test_engagement.cpp`](tests/test_engagement.cpp) |

### Design highlights

- **True Proportional Navigation.** `a_c = N·(V_r × Ω)` with
  `Ω = (R × V_r)/(R·R)`. The command vanishes on a collision course (zero LOS
  rate) and otherwise nulls the LOS rotation, driving intercept. Interceptors
  steer at constant cruise speed (the lateral command rotates the velocity
  vector). Convergence is proven in closed-loop tests against crossing,
  inbound-3D, and sinusoidally-weaving maneuvering targets for N ∈ {3,4,5}.
- **Threat prioritization.** Hostiles are ranked by time-to-impact against the
  defended asset (proximity breaks ties). Targeting queries go through the
  octree with the `QUERY_HOSTILE_ONLY` bitmask, so friendlies and neutrals
  never enter a firing solution; interceptors are assigned distinct threats.
- **Octree proximity fuze.** Each frame a fuze-radius cube around every
  interceptor is queried for hostiles and confirmed with an exact spherical
  range test; a hit destroys (despawns) both the interceptor and the threat.
- **Terminal-phase realism.** Achievable miss distance at a fixed 60 Hz step is
  bounded by roughly half the per-frame travel, which is why proximity fuzes
  are radius-sized rather than requiring exact contact — reflected in both the
  demo and the test tolerances.

The demo's 12-vs-12 salvo (Mach-3 interceptors, Mach-1 inbound threats)
neutralizes all 12 threats, resolving in ~46 s of simulated time.

---

## Phase 3 — Linux IPC Telemetry

> **Linux-only.** These targets build on Unix (developed/tested on a Raspberry
> Pi 4, aarch64, Debian 12). On Windows the CMake project silently omits them
> and still builds Phases 1–2.

| Deliverable | Where |
|---|---|
| Byte-packed binary packet protocol (record + frame header) | [`include/sim/Telemetry.hpp`](include/sim/Telemetry.hpp) |
| POSIX shared-memory seqlock ring buffer (zero-alloc, lock-free) | [`include/sim/SharedMemoryChannel.hpp`](include/sim/SharedMemoryChannel.hpp) · [`src/SharedMemoryChannel.cpp`](src/SharedMemoryChannel.cpp) |
| UDP sender/receiver (remote transport fallback) | [`include/sim/UdpTelemetry.hpp`](include/sim/UdpTelemetry.hpp) · [`src/UdpTelemetry.cpp`](src/UdpTelemetry.cpp) |
| Secondary monitoring executable | [`src/telemetry_monitor.cpp`](src/telemetry_monitor.cpp) |
| IPC GTest suite (round-trip, seqlock stress, UDP) | [`tests/test_telemetry.cpp`](tests/test_telemetry.cpp) |

### Design highlights

- **Fixed wire format.** `TelemetryRecord` (30 B) and `FrameHeader` (32 B) are
  `#pragma pack`-ed and guarded by `static_assert` on their sizes, so the
  protocol can't silently drift across compilers. Positions narrow to `float`
  to keep datagrams compact.
- **Lock-free shared memory via a seqlock ring.** The producer maps a fixed
  POD region (`shm_open` + `mmap`), and publishes each frame into the next of
  N ring slots guarded by a per-slot sequence counter (odd = writing). Readers
  snapshot the latest slot and re-check the counter; with ring depth ≥ 3 the
  writer has always moved on, so reads never tear. Cross-process atomics are
  `static_assert`-ed lock-free. **No allocation occurs on the publish path** —
  records are `memcpy`-ed straight into the mapping.
- **Decoupled monitor process.** `telemetry_monitor` is a pure consumer: it
  attaches read-only and reports live threat tallies, intercept count, and the
  measured telemetry frame rate — the command-and-display process from the
  architecture diagram.
- **UDP fallback.** A single datagram carries the header plus up to 45 records
  (kept under a typical MTU); oversize frames are clamped rather than
  fragmented.

### Running the pipeline (two terminals)

```bash
# Terminal 1 — run the sim, publishing telemetry to /defsim_telemetry for 20 s
./build/defense_sim telemetry 20

# Terminal 2 — attach the monitor
./build/telemetry_monitor
```

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

### Profiling (Linux)

Verified on a Raspberry Pi 4 (Cortex-A72, 4 cores, Debian 12).

**Valgrind — zero leaks** across the engine, thread pool, octree, engagement
manager, and the shared-memory publisher (`shm_open`/`mmap`/`munmap`/
`shm_unlink`):

```bash
valgrind --leak-check=full --error-exitcode=42 ./build/defense_sim telemetry 2
# ==> in use at exit: 0 bytes in 0 blocks
# ==> ERROR SUMMARY: 0 errors from 0 contexts
```

**perf — cache utilization** for 20,000 entities × 60 frames:

```bash
perf stat -e instructions,cycles,cache-references,cache-misses \
          ./build/defense_sim 20000 60
```

| Metric | Value |
|---|---|
| L1 d-cache miss rate | **3.89%** (≈96% hit rate) |
| Per-frame time (Pi, 4 cores) | ~12.3 ms — within the 16.7 ms / 60 Hz budget |

The low miss rate reflects the standard-layout entity array and the octree's
contiguous per-node item storage.

---

## Layout

```
include/sim/   Public headers (math, entities, octree, thread pool, engine,
               guidance, engagement, telemetry, shm channel, UDP)
src/           Implementation, the demo (defense_sim), and telemetry_monitor
tests/         GoogleTest suites (one per component)
Plan.md        Full three-phase roadmap
```

Phase 3's shared-memory and UDP targets build only under `UNIX`; the
`defense_sim telemetry` mode and `telemetry_monitor` executable appear only in
Linux builds.

## Target environment

Written to be portable C++17. The reference target for later phases is
Linux (Ubuntu 22.04 / RHEL), where Phase 3 adds POSIX shared-memory and UDP
telemetry and the project is profiled with Valgrind and `perf`.
