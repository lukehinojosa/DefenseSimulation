# Real-Time Defense Simulation & Telemetry Suite

## Technical Stack & Environment

* **Language:** C++17
* **Build System:** CMake (3.20+)
* **Testing Framework:** GoogleTest (GTest)
* **Target OS:** Linux (Ubuntu 22.04 LTS or RHEL via WSL2/Native)
* **Profiling Tools:** Valgrind (`memcheck`), Linux `perf`, GDB

---

## Architecture Overview

```
                      ┌─────────────────────────────────────────┐
                      │          SIMULATION ENGINE              │
                      │  • Octree 3D Spatial Partitioning       │
                      │  • Kinematic Targets (Hostile)          │
                      │  • ProNav Interceptors (Friendly)       │
                      └────────────────────┬────────────────────┘
                                           │
                                           │ Shared Memory / UDP
                                           ▼
                      ┌─────────────────────────────────────────┐
                      │          TELEMETRY PIPELINE             │
                      │  • Fixed-size Binary Structs            │
                      │  • Zero-Allocation Circular Buffer      │
                      │  • Lock-Free Threading / Mutexes        │
                      └────────────────────┬────────────────────┘
                                           │
                                           ▼
                      ┌─────────────────────────────────────────┐
                      │       COMMAND & DISPLAY PROCESS         │
                      │  • 60Hz Telemetry Ingestion            │
                      │  • Console / Log Display Monitor        │
                      └─────────────────────────────────────────┘

```

---

## Phase 1: Core Engine & Spatial Math

**Goal:** Build the 3D math foundation, entity management system, and spatial partitioning Octree.

### Key Deliverables:

1. **Math & Physics Primitives:**
* Implement a lightweight 3D Vector class (`Vector3`) with essential operations (dot product, cross product, magnitude, normalization, scalar multiplication).
* Implement basic kinematic state updates: $\vec{P}_{\text{new}} = \vec{P}_{\text{old}} + \vec{V} \cdot \Delta t$.


2. **Entity Component Architecture:**
* Create an `Entity` struct/class containing `EntityID`, `Position`, `Velocity`, `Type` (`Hostile`, `Friendly`, `Neutral`), and `Status` (`Active`, `Destroyed`).


3. **Multithreaded 3D Octree:**
* Build a recursive 3D Octree that spatially partitions a $100\text{ km} \times 100\text{ km} \times 20\text{ km}$ airspace.
* Implement spatial query functions: `queryRange(BoundingBox box, EntityTypeFilter filter)`.
* Parallelize the spatial update loop using `std::thread` worker pools so 10,000+ entities update without frame drops.


4. **GoogleTest Integration (GTest):**
* Write unit tests verifying Octree insertion, boundary splitting, and range query accuracy.



---

## Phase 2: ProNav Guidance & Intercept Logic

**Goal:** Implement closed-loop threat engagement, Proportional Navigation guidance, and collision detection.

### Key Deliverables:

1. **Proportional Navigation (ProNav) Algorithm:**
* Implement continuous Line-of-Sight (LOS) angle rate calculation between interceptor and target.
* Calculate commanded acceleration: $\vec{a}_c = N \cdot (\vec{V}_r \times \vec{\Omega})$ (where $N = 3$ to $5$).
* Apply acceleration to update interceptor velocity vector at 60Hz.


2. **Automated Engagement Manager:**
* Implement a threat priority queue sorting hostile targets by proximity and time-to-impact (TTI).
* Filter out friendly entities during targeting queries using bitmask flags (`QUERY_HOSTILE_ONLY`).


3. **Proximity Detonation & Destructor:**
* Implement a proximity fuze query ($< 5\text{ meters}$) using the Octree to trigger target destruction and despawn both entities.


4. **Unit Testing:**
* Write GTest suites testing ProNav convergence against straight-line and maneuvering target paths.



---

## Phase 3: Linux IPC Telemetry & CMake Integration

**Goal:** Connect a secondary display/monitoring process over Linux Inter-Process Communication (IPC).

### Key Deliverables:

1. **Binary Packet Protocol:**
* Define a compact, memory-aligned C++ struct representing telemetry state (`uint32_t entityID`, `float posX, posY, posZ`, `float velX, velY, velZ`, `uint8_t entityType`, `uint68_t threatLevel`).


2. **Linux IPC Pipeline:**
* Implement **POSIX Shared Memory** (`shm_open`, `mmap`) for ultra-fast local process data transfer.
* Add a secondary **UDP Socket** sender/receiver module as a remote transport fallback.
* Design a lock-free or double-buffered ring buffer to ensure zero dynamic memory allocations (`new`/`malloc`) during the execution loop.


3. **Secondary Monitoring Executable:**
* Build a separate lightweight C++ console executable that consumes the shared memory stream and prints active threat levels, intercept counts, and telemetry frame rates.


4. **CMake & Profiling:**
* Configure a clean `CMakeLists.txt` managing executable targets, libraries, and GoogleTest dependency links.
* Run Linux **Valgrind** to verify zero memory leaks and **perf** to verify CPU cache utilization.