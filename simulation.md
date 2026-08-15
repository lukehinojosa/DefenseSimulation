# Simulation Behavior Reference

This document describes **what the simulation does** — the world model, the
per-frame update pipeline, and the behavior of every missile and system in it.
It is a behavioral spec, not an API reference; for build/run instructions see
[README.md](README.md), and for the phase plan see [Plan.md](Plan.md).

Values in **bold** with units are the current defaults. Where the demo scenario
(`defense_sim telemetry`) overrides an engine default, both are given.

> **Data-driven.** Every constant that shapes the engagement scenario lives in
> [`config/simulation.yaml`](config/simulation.yaml) and is read at **runtime** —
> edit and re-run, no rebuild. The binary reads `$SIM_CONFIG` if set, otherwise
> it searches `config/simulation.yaml` and a few parent directories. Any key you
> omit keeps its built-in default, and a missing file just runs the defaults. See
> §14 for the full key map.

---

## 1. World model

| Property | Value |
| --- | --- |
| Coordinate frame | Right-handed, **X = East, Y = North, Z = Up**, meters |
| Airspace volume | **100 km × 100 km × 20 km** — `(0,0,0)` to `(100000,100000,20000)` |
| Defended asset | Center of the map, **(50000, 50000, 0)** (ground level) |
| Ground plane | `Z = 0`; nothing may live below it |
| Frame rate | **60 Hz** fixed step |
| Time base | Real-time 1:1 for the headless demos. The telemetry publisher runs **slow motion** (`kTimeScale = 0.6`): the display still refreshes at 60 Hz, but each frame advances `simDt = (1/60)·0.6` of simulated time, so Mach-3+ missiles and their rate-limited turns are legible instead of a blur. |

Integration is first-order: `P_new = P_old + V · dt`. Destroyed entities are
frozen. The kinematic update runs in parallel across a worker pool (each task
owns one entity, no locking); the spatial octree is then rebuilt single-writer.

---

## 2. Entities

Every tracked object is a plain `Entity` (position, velocity, type, status,
flags). Large arrays of them stay cache-friendly and copy straight into the
telemetry pipeline.

**Allegiance** (`EntityType`):

| Type | Role |
| --- | --- |
| `Hostile` | Inbound threat closing on the defended asset. |
| `Friendly` | Own interceptor — ground-launched from the battery. |
| `Neutral` | Allied interceptor — airborne, engages hostiles too. |

**Status** (`EntityStatus`): `Active` or `Destroyed` (destroyed entities are
removed from the spatial index and frozen in place).

**Flags** (`EntityFlags`, bitmask):

| Flag | Meaning |
| --- | --- |
| `EFLAG_LAUNCHING` | Boosting straight up off the pad; steering law suspended. |
| `EFLAG_BOOSTING` | Motor lit — drives the launch-flame / hot-trail display cue. |

---

## 3. Frame update pipeline

Each 60 Hz tick of the engagement scenario runs, in order:

1. **`directThreats`** — advance every hostile's launch/cruise state machine.
2. **`serviceLaunches`** — demand-driven interceptor launches from the pads.
3. **`assignTargets`** — rank threats and assign them to defenders.
4. **`guide`** — steer every interceptor (guidance + airframe limits + collision avoidance).
5. **`engine.step`** — integrate all entities, then rebuild the octree.
6. **`processDetonations`** — swept proximity fuze (interceptor ↔ hostile).
7. **`processGroundAndAssets`** — ground-plane and city fail-safe.
8. **`processInterceptorDisposal`** — spent rounds self-detonate clear of the city.

Ordering matters: threats are steered before the shared integration, and all
three destruction passes (fuze, ground/city, disposal) run *after* the step so
they act on fresh positions and their events reach the telemetry feed.

---

## 4. Threat (hostile) missile behavior

Threats are **ballistic**: they are not guided by ProNav, they simply fly toward
the defended asset under a rate-limited turn.

1. **Launch.** Spawns on a random bearing at a random range and altitude from the
   asset (demo: **range 23–36 km, altitude 0–7 km**), flagged
   `EFLAG_LAUNCHING | EFLAG_BOOSTING`, with velocity straight up at
   **`kThreatBoost` = 600 m/s**. Ground launches (altitude 0) boost visibly off
   the deck; ones that spawn already above the hand-off altitude cruise straight
   in — a mixed-profile raid, not a uniform wave.
2. **Boost.** Climbs vertically until it clears **`launchHandoffAltitude` = 1 km**.
3. **Pitch-over & cruise.** Clears the launch flags, then each frame steers toward
   the asset at **`kThreatCruise` = 950 m/s** under a sluggish
   **~12 g** lateral limit (`kThreatMaxLatAccel`). The result is a natural arc
   from vertical onto the target rather than an instant turn.
4. **Terminal.** Descends onto the city. If it reaches the ground within the
   protected radius, or enters a city structure, it is destroyed and counted as
   an **asset loss** (see §10).

Threats never maneuver evasively and never target interceptors.

---

## 5. Interceptor missile behavior

Interceptors (both friendly and allied-neutral) share one flight model; they
differ only in how they enter the fight.

### 5.1 Deployment

| | Friendly | Neutral (allied) |
| --- | --- | --- |
| Origin | Ground pads on a **5 km ring** around the asset | Airborne, **~18–22 km** out, random bearing |
| Start altitude | 0 (launches) | **2.5–9 km** |
| Start velocity | Straight up at cruise (`EFLAG_LAUNCHING`) | Inbound cruise toward the asset (**900 m/s**), no boost |
| Cruise speed | **1300 m/s** | **1400 m/s** |

### 5.2 Flight phases

1. **Boost (launched rounds only).** Climbs vertically at cruise speed until
   clearing **1 km AGL**, motor lit. Steering is suspended so it rises cleanly.
2. **Low-speed pursuit.** Below **0.3 × cruise** airspeed (e.g. straight off a
   standing deployment), flies pure pursuit toward the target to build speed —
   ProNav has no velocity to work with from rest.
3. **Proportional Navigation.** Once flying, guides with true ProNav
   `a_c = N·(V_r × Ω)`, `Ω = (R × V_r)/(R·R)`, `N = 4`. The command nulls the
   line-of-sight rotation, driving a collision course.
4. **Terminal.** Detonates via the swept proximity fuze (§9), destroying both
   itself and the threat.

### 5.3 Momentum / airframe limits

The ProNav (or pursuit/loiter) command is **never applied raw**. Every steering
mode passes through `applyAirframeLimits`:

- **Lateral G-limit** caps the turn command (**40 g** engine default,
  **55 g** in the demo). The round *arcs* through a turn instead of pivoting
  instantly — real inertia, no "UFO" snap.
- **Axial thrust/drag** draws speed toward cruise only as fast as
  `axialAccel` allows (**50 g** default / **60 g** demo), so speed changes are
  gradual, not instantaneous.

### 5.4 Fuel, loiter, and disposal

- **Fuel.** Each interceptor carries **`interceptorFuel` = 70 s** of motor life
  (simulated seconds). It burns whether boosting, cruising, or coasting.
- **Loiter / CAP.** A round with **no target** flies a holding orbit at
  **`loiterRadius` = 12 km / `loiterAltitude` = 6 km** around the defended zone,
  circling tangentially. Each round holds a **distinct slot** (staggered radius
  and altitude by deployment index) so a stack of loiterers spreads into a proper
  CAP rather than piling onto one orbit. It is re-tasked the instant
  `assignTargets` hands it a threat.
- **Self-disposal.** On **fuel exhaustion** the round turns radially away from the
  city (with a slight climb so it never dives on the asset) and self-detonates
  once it is past **`safeDisposalRadius` = 12 km** — spent rounds never fall on
  the city. Disposal is triggered *only* by fuel, never by simply lacking a target
  (that case loiters).

### 5.5 Friendly-on-friendly collision avoidance

Interceptors never fly through one another. `separationAccel` queries nearby
friendly/neutral defenders (within **`separationRadius` = 600 m**) and adds a
repulsion acceleration (up to **`separationAccel` = 60 g**), folded into pursuit,
loiter, **and** disposal steering through the same airframe limits.

Avoidance is **asymmetric (right-of-way)**. Each frame every interceptor is
ranked — **attacking (2) > loitering (1) > disposing (0)**, lower entity id
breaks ties — and a round yields **only to higher-priority neighbors**. So of any
crossing pair exactly one maneuvers around the other; an engaged interceptor
holds its optimal intercept path and is never dragged off-solution by a loiterer.
(Note: interceptors also cannot *fuze* each other — the proximity fuze is
hostile-only — so a friendly touch is non-destructive regardless.)

---

## 6. Target prioritization & assignment

- **Threat queue.** Active hostiles are ranked most-urgent-first by
  **time-to-impact** (TTI) on the asset, with range breaking ties.
- **Allegiance matrix.** Targeting uses the `QUERY_ENGAGEABLE_THREATS` mask (=
  hostiles) rather than a hard-coded friendly filter, so **any** defender —
  friendly or allied-neutral — can prosecute the same hostile set, and defenders
  never target one another.
- **Independent claims (don't trust allies).** Friendly and neutral defenders keep
  **separate** claim sets. A friendly engages a threat **even if an ally is
  already on it**, so the friendly battery covers every threat by itself and
  allied intercepts are pure redundancy. Within one allegiance, assignments stay
  distinct (no two friendlies on one threat, no two allies on one threat) — a
  threat can carry at most one friendly + one ally.
- **Retargeting.** Dead targets are dropped at the top of each assignment pass, so
  a round whose threat was neutralized before impact immediately retargets to the
  next uncovered threat that same frame; if none remains, it loiters.

---

## 7. Launch control

Interceptors are launched on demand, not in one mass salvo:

- A small **standing battery** (demo: **8**) fires at the start.
- Each frame, `serviceLaunches` counts **only friendly** ready shooters (allies
  are treated as unreliable and not counted toward coverage) and launches one
  more friendly round — **only** when `readyFriendly < activeHostiles`, i.e. a
  threat still has no friendly shooter. A round is therefore **never launched
  without a target**.
- Launches are **staggered** (**0.15 s** apart) so the pads fire in a visible
  ripple rather than a wall.
- Because friendlies must cover every threat regardless of allies, the friendly
  count is sized to the threat count; allied kills simply free friendlies to
  loiter as a ready reserve that backfills if an ally fails.

Once all hostiles are gone, `activeHostiles == 0` and no further rounds launch.

---

## 8. Allied (neutral) defenders

Allied interceptors stream in from ~20 km out at altitude and engage the same
hostile set via the allegiance matrix. They fly the identical flight model
(ProNav, airframe limits, avoidance, loiter, disposal). They are **not** relied
upon: friendly coverage is complete without them, and any threat they down early
just frees the friendly that was also assigned to it.

---

## 9. Proximity fuze (swept)

At Mach-3+ closing speeds the per-frame travel (~30 m at 60 Hz) dwarfs the fuze
radius, so a naive point test at the frame boundary can sample either side of the
actual pass and miss ("tunneling"). The fuze instead:

1. Queries a box around the interceptor **enlarged by the worst-case per-frame
   closing travel**, so a target that flew past this frame is still found.
2. Confirms with the **closest approach of the two motion segments** over the last
   step (not just the endpoint distance).

If that closest approach is within **`fuzeRadius`** (demo: **20 m**; engine
default 5 m) the round detonates, destroying **both** the interceptor and the
threat. The fuze only ever fires interceptor-against-hostile.

---

## 10. Ground plane, city assets, and failure

- **Ground fail-safe.** Any active, non-launching entity that descends to/through
  `Z = 0` is destroyed in place (clamped to the deck first). This is what stops
  interceptors and leaked threats from clipping below the ground plane.
- **Defended city.** A static skyline of **skyscrapers, a hospital, and a
  residential suburb** (from `CityLayout`) sits around the asset as hard collision
  volumes. Any entity entering a structure is destroyed.
- **Asset loss.** A **hostile** that grounds within **`protectedRadius` = 6 km**
  of the asset, or strikes a city structure, is tallied as an **asset failure**
  and flagged `FLAG_ASSET_HIT` so the display can mark the loss. Interceptors that
  crash simply despawn (no penalty).

---

## 11. Demo scenario at a glance

The `defense_sim telemetry` scenario (seed-fixed, deterministic):

| Element | Value |
| --- | --- |
| Threats | **16**, random bearing, **23–36 km** out, **0–7 km** altitude |
| Threat speed | boost **600 m/s** → cruise **950 m/s**, ~12 g |
| Friendly interceptors | **8** standing + demand-driven ripple, **1300 m/s**, 55 g |
| Allied interceptors | **8**, inbound from ~20 km / 2.5–9 km altitude, **1400 m/s** |
| Fuze radius | **20 m** (swept) |
| Time scale | **0.6×** (slow motion) |
| Default duration | **70 s** wall-clock (`~42 s` simulated) |
| Typical result | **16 / 16 intercepts, 0 asset losses** |

---

## 12. Telemetry & visualization behavior

### 12.1 Wire protocol (v2)

Fixed-layout records feed a decoupled display over two interchangeable
transports: a lock-free **shared-memory seqlock ring** (local) and a compact
**UDP** stream (remote; zig-zag + VLQ codec, whole scene in one datagram). Per
record: id, position, velocity, assigned target id, allegiance, threat level,
and a flags byte:

| Flag | Meaning |
| --- | --- |
| `FLAG_DESTROYED` | Entity was destroyed this frame → spawn detonation FX. |
| `FLAG_BOOSTER` | Motor lit → launch flame + hot trail. |
| `FLAG_ASSET_HIT` | Leaked threat struck the city → count as an asset loss. |

### 12.2 Threat classification

Hostiles are colored by **time-to-impact** on the asset:

| TTI | Level | Color |
| --- | --- | --- |
| < 10 s | Critical | Red |
| < 30 s | High | Orange |
| < 60 s | Medium | Gold |
| < 120 s | Low | Yellow |
| ≥ 120 s / receding | None | Maroon |

### 12.3 Display legend

| Element | Appearance |
| --- | --- |
| Hostile | Threat-colored sphere + red heading dart + trail |
| Friendly interceptor | Green (engaged) / sky-blue (idle) + heading dart + green LOS line to target |
| Allied-neutral interceptor | Dark green + LOS line |
| Boosting round | Orange launch flame; trail runs hot-orange → cool-blue as it transitions to cruise |
| Detonation | Expanding fading wireframe sphere + lingering ground ring |
| Defended city | Blue skyscrapers, red hospital, green suburb blocks; teal asset core + battery ring |
| Ground | Solid terrain deck + 100 × 100 km grid + 20 km altitude ceiling wireframe |

The C2 HUD reports active hostiles, neutralized count, asset losses, success
rate, live telemetry rate/throughput, and the current frame id.

### 12.4 Controls

Drag to orbit · wheel to zoom · `WASD`/`QE` to pan · `Tab` to cycle
target-tracking · `C` for free camera · `--dist <units>` sets the opening zoom.

---

## 13. Parameter reference

Engine defaults (`EngagementManager::Config`) unless a §11 demo override applies:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `fuzeRadius` | 5 m (demo 20 m) | Proximity fuze miss-distance threshold |
| `navConstant` | 4 | ProNav gain `N` |
| `maxLateralAccel` | 40 g (demo 55 g) | Interceptor turn G-limit |
| `axialAccel` | 50 g (demo 60 g) | Interceptor thrust/drag limit |
| `launchHandoffAltitude` | 1 km | Boost ceiling before guidance takes over |
| `protectedRadius` | 6 km | Threat grounding within this = asset loss |
| `interceptorFuel` | 70 s | Motor life before self-disposal |
| `safeDisposalRadius` | 12 km | Minimum range to self-detonate a spent round |
| `loiterRadius` / `loiterAltitude` | 12 km / 6 km | CAP holding orbit for idle rounds |
| `separationRadius` | 600 m | Begin avoiding a fellow defender |
| `separationAccel` | 60 g | Collision-avoidance authority |

---

## 14. Data-driven configuration (`config/simulation.yaml`)

The scenario is defined entirely by [`config/simulation.yaml`](config/simulation.yaml),
loaded at startup (see the box at the top of this doc for path resolution). The
`[seconds]` CLI argument, if given, overrides `time.duration_seconds`. Keys:

| YAML path | Drives |
| --- | --- |
| `world.defended_asset` | Center of the defended city / battery |
| `time.scale` | Physics slow-motion factor (display stays 60 Hz) |
| `time.duration_seconds` | Wall-clock run length |
| `rng_seed` | Deterministic raid layout |
| `engagement.fuze_radius` | Proximity fuze miss distance |
| `engagement.nav_constant` | ProNav gain `N` |
| `engagement.interceptor_max_lateral_accel_g` | Interceptor turn G-limit |
| `engagement.interceptor_axial_accel_g` | Interceptor thrust/drag limit |
| `engagement.launch_handoff_altitude` | Boost ceiling before guidance |
| `engagement.protected_radius` | Asset-loss radius |
| `engagement.interceptor_fuel_seconds` | Motor life before self-disposal |
| `engagement.safe_disposal_radius` | Min range to self-detonate a spent round |
| `engagement.loiter_radius` / `loiter_altitude` | CAP holding orbit |
| `engagement.separation_radius` / `separation_accel_g` | Collision avoidance |
| `threats.count` / `spawn_range_*` / `spawn_altitude_*` | Raid size and geometry |
| `threats.boost_speed` / `cruise_speed` | Threat speeds |
| `threats.max_lateral_accel_g` / `axial_accel_g` | Threat maneuverability |
| `interceptors.friendly.*` | Standing battery, launch cap, pad radius, speed, cooldown |
| `interceptors.neutral.*` | Ally count, spawn range/altitude, speed, inbound cruise |

Accelerations are in **g**, distances in **meters**, speeds in **m/s**. Engine
defaults (used by anything not driven by the YAML, e.g. the unit tests) live in
`EngagementManager::Config`; the loader and struct live in `sim/SimConfig.hpp`.
