// Decoupled 3D tactical C2 visualizer (Raylib).
//
// A standalone, read-only client that consumes the simulation's telemetry via
// the ITelemetryConsumer abstraction (POSIX shared memory locally, or UDP
// remotely) and renders the airspace, entity tracks, ProNav intercept geometry,
// detonations, and a Command-and-Control HUD. It injects zero overhead into the
// engine process.
//
// Usage:
//   c2_visualizer --source shm [--shm /defsim_telemetry]
//   c2_visualizer --source udp [--port 9090]
//   c2_visualizer ... --frames N --screenshot out.png   (headless/CI capture)

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "raylib.h"
#include "raymath.h"

#include "sim/CityLayout.hpp"
#include "sim/Entity.hpp"
#include "sim/ITelemetryConsumer.hpp"
#include "sim/ShmTelemetryConsumer.hpp"
#include "sim/Telemetry.hpp"
#include "sim/UdpTelemetryConsumer.hpp"

namespace {

using namespace sim::telemetry;

// --- World scale: 1 Raylib unit = 1 km. Airspace 100x100x20 km -> 100x100x20.
constexpr float kScale = 1000.0f;
// Recenter the airspace on the origin so the ground grid (drawn centered at 0)
// lines up with the world; the airspace center (50 km, 50 km) maps to (0,0).
constexpr float kHalfExtent = 50000.0f;
// Sim frame is X-east, Y-north, Z-up; Raylib is Y-up. Map z(up) -> Raylib Y.
// Positions are recentered on the origin so the ground grid lines up.
Vector3 worldToRay(float x, float y, float z) {
    return Vector3{ (x - kHalfExtent) / kScale, z / kScale,
                    (y - kHalfExtent) / kScale };
}
// Directions (velocities) must NOT be recentered -- only the axis remap and
// scale apply. (Recentering a direction was the bug that made every velocity
// vector point at the same distant corner.)
Vector3 dirToRay(float x, float y, float z) {
    return Vector3{ x / kScale, z / kScale, y / kScale };
}
Vector3 recPos(const TelemetryRecord& r) { return worldToRay(r.posX, r.posY, r.posZ); }
Vector3 recVel(const TelemetryRecord& r) { return dirToRay(r.velX, r.velY, r.velZ); }

// Map a world-space AABB (meters) to a Raylib cube (center + extents). The
// world Y (north) and Z (up) axes are swapped to match the display frame.
struct RayBox { Vector3 center; Vector3 size; };
RayBox worldBoxToRay(const sim::BoundingBox& b) {
    const Vector3 c = worldToRay(static_cast<float>((b.min.x + b.max.x) * 0.5),
                                 static_cast<float>((b.min.y + b.max.y) * 0.5),
                                 static_cast<float>((b.min.z + b.max.z) * 0.5));
    const Vector3 s{static_cast<float>((b.max.x - b.min.x) / kScale),
                    static_cast<float>((b.max.z - b.min.z) / kScale),
                    static_cast<float>((b.max.y - b.min.y) / kScale)};
    return {c, s};
}

// A fixed-length heading dart in the direction of travel: shows where a track
// is pointing without the raw velocity magnitude dominating the view.
void drawHeading(Vector3 p, Vector3 velRay, float len, Color c) {
    if (Vector3Length(velRay) < 1e-6f) return;
    DrawLine3D(p, Vector3Add(p, Vector3Scale(Vector3Normalize(velRay), len)), c);
}

// --- Orbit / arcball camera state --------------------------------------------
struct OrbitCamera {
    Vector3 target{0.0f, 4.0f, 0.0f}; // airspace center maps to origin
    float   distance{58.0f};          // framed on the defended city by default
    float   yaw{0.7f};
    float   pitch{0.45f};

    Camera3D toCamera() const {
        Camera3D cam{};
        const float cp = cosf(pitch), sp = sinf(pitch);
        cam.position = Vector3{
            target.x + distance * cp * sinf(yaw),
            target.y + distance * sp,
            target.z + distance * cp * cosf(yaw)};
        cam.target   = target;
        cam.up       = Vector3{0.0f, 1.0f, 0.0f};
        cam.fovy     = 55.0f;
        cam.projection = CAMERA_PERSPECTIVE;
        return cam;
    }
};

void updateCamera(OrbitCamera& c, bool trackingActive) {
    // Zoom (wheel up = zoom in).
    const float wheel = GetMouseWheelMove();
    if (wheel != 0.0f) {
        c.distance *= (1.0f - wheel * 0.1f);
        c.distance = Clamp(c.distance, 8.0f, 400.0f);
    }
    // Orbit: left/right mouse drag, or arrow keys.
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) ||
        IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        const Vector2 d = GetMouseDelta();
        c.yaw   -= d.x * 0.005f;
        c.pitch += d.y * 0.005f;
    }
    const float ks = GetFrameTime() * 1.5f;
    if (IsKeyDown(KEY_LEFT))  c.yaw   += ks;
    if (IsKeyDown(KEY_RIGHT)) c.yaw   -= ks;
    if (IsKeyDown(KEY_UP))    c.pitch += ks;
    if (IsKeyDown(KEY_DOWN))  c.pitch -= ks;
    c.pitch = Clamp(c.pitch, -1.5f, 1.5f);

    // Pan the look-at target (disabled while auto-tracking an entity).
    if (!trackingActive) {
        const float pan = GetFrameTime() * c.distance * 0.5f;
        const Vector3 fwd = Vector3Normalize(Vector3{sinf(c.yaw), 0.0f, cosf(c.yaw)});
        const Vector3 right = Vector3{fwd.z, 0.0f, -fwd.x};
        if (IsKeyDown(KEY_W)) c.target = Vector3Subtract(c.target, Vector3Scale(fwd, pan));
        if (IsKeyDown(KEY_S)) c.target = Vector3Add(c.target, Vector3Scale(fwd, pan));
        if (IsKeyDown(KEY_D)) c.target = Vector3Add(c.target, Vector3Scale(right, pan));
        if (IsKeyDown(KEY_A)) c.target = Vector3Subtract(c.target, Vector3Scale(right, pan));
        if (IsKeyDown(KEY_E)) c.target.y += pan;
        if (IsKeyDown(KEY_Q)) c.target.y -= pan;
    }
}

struct Detonation {
    Vector3 pos;
    float   age{0.0f};
};

// A trail sample remembers whether the motor was lit when it was laid down, so
// the ribbon can shade the hot launch boost differently from the cool cruise.
struct TrailPt {
    Vector3 p;
    bool    hot{false};
};

bool isHostile(const TelemetryRecord& r) {
    return r.entityType == static_cast<std::uint8_t>(sim::EntityType::Hostile);
}
bool isFriendly(const TelemetryRecord& r) {
    return r.entityType == static_cast<std::uint8_t>(sim::EntityType::Friendly);
}
bool isNeutral(const TelemetryRecord& r) {
    return r.entityType == static_cast<std::uint8_t>(sim::EntityType::Neutral);
}
bool isBooster(const TelemetryRecord& r) { return (r.flags & FLAG_BOOSTER) != 0u; }
// Every neutral track in the scenario is an allied (neutral) interceptor.
bool isInterceptor(const TelemetryRecord& r) {
    return isFriendly(r) || isNeutral(r);
}

Color threatColor(std::uint8_t level) {
    switch (static_cast<ThreatLevel>(level)) {
        case ThreatLevel::Critical: return RED;
        case ThreatLevel::High:     return ORANGE;
        case ThreatLevel::Medium:   return GOLD;
        case ThreatLevel::Low:      return YELLOW;
        default:                    return MAROON;
    }
}

// --- CLI parsing -------------------------------------------------------------
struct Options {
    std::string source{"shm"};
    std::string shmName{kDefaultShmName};
    std::uint16_t port{9090};
    int         maxFrames{0};      // 0 = run until window closed
    std::string screenshot;        // capture on exit if set
    float       distance{0.0f};    // initial camera distance (0 = default)
};

Options parseArgs(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* def) -> std::string {
            return (i + 1 < argc) ? argv[++i] : def;
        };
        if (a == "--source")          o.source = next("shm");
        else if (a == "--shm")        o.shmName = next(kDefaultShmName);
        else if (a == "--port")       o.port = static_cast<std::uint16_t>(std::stoi(next("9090")));
        else if (a == "--frames")     o.maxFrames = std::stoi(next("0"));
        else if (a == "--screenshot") o.screenshot = next("");
        else if (a == "--dist")       o.distance = std::stof(next("0"));
    }
    return o;
}

std::unique_ptr<ITelemetryConsumer> makeConsumer(const Options& o) {
    if (o.source == "udp") {
        return std::make_unique<UdpTelemetryConsumer>(o.port);
    }
    if (o.source == "shm") {
        return std::make_unique<ShmTelemetryConsumer>(o.shmName);
    }
    TraceLog(LOG_ERROR, "unknown --source '%s' (expected shm or udp)",
             o.source.c_str());
    return nullptr;
}

} // namespace

int main(int argc, char** argv) {
    const Options opt = parseArgs(argc, argv);

    SetTraceLogLevel(LOG_WARNING);
    const int screenW = 1280, screenH = 720;
    InitWindow(screenW, screenH, "Defense Simulation - C2 Tactical Display");
    SetTargetFPS(60);

    // Connect to the telemetry source (retry briefly for shm/producer startup).
    std::unique_ptr<ITelemetryConsumer> consumer;
    for (int attempt = 0; attempt < 50 && !WindowShouldClose(); ++attempt) {
        try {
            consumer = makeConsumer(opt);
            if (consumer) break;
        } catch (const std::exception& e) {
            TraceLog(LOG_WARNING, "connect retry: %s", e.what());
        }
        BeginDrawing();
        ClearBackground(Color{10, 12, 18, 255});
        DrawText("Waiting for telemetry producer...", 40, 40, 20, RAYWHITE);
        EndDrawing();
        WaitTime(0.1);
    }
    if (!consumer) {
        if (!opt.screenshot.empty()) { /* fall through to allow headless exit */ }
        CloseWindow();
        std::fprintf(stderr, "could not create telemetry consumer\n");
        return 1;
    }

    OrbitCamera cam;
    if (opt.distance > 0.0f) cam.distance = Clamp(opt.distance, 8.0f, 400.0f);
    TelemetrySnapshot snap;
    std::unordered_map<std::uint32_t, std::deque<TrailPt>> trails;
    std::vector<Detonation> detonations;

    // Static defended city (skyscrapers, hospital, suburb), precomputed once in
    // display space. The engine collides against the same world-space layout.
    const sim::Vector3 assetWorld{50000.0, 50000.0, 0.0};
    const std::vector<sim::CityStructure> city = sim::defaultCityLayout(assetWorld);

    std::uint64_t lastFrameId = UINT64_MAX;
    std::uint64_t lastTsNs = 0;
    double telemetryFps = 0.0;
    double throughputKBs = 0.0;

    int trackIndex = -1; // -1 = free camera; otherwise index into trackables
    int neutralizedPeak = 0;
    int assetLosses = 0; // leaked threats that struck the city (running total)
    int frameCounter = 0;

    while (!WindowShouldClose()) {
        // --- Ingest -----------------------------------------------------------
        const bool haveNew = consumer->PollLatestFrame(snap);
        const bool freshFrame = haveNew && snap.header.frameId != lastFrameId;

        if (freshFrame) {
            if (lastTsNs != 0 && snap.header.timestampNs > lastTsNs) {
                const double dt = (snap.header.timestampNs - lastTsNs) / 1e9;
                const double inst = (dt > 0.0) ? 1.0 / dt : 0.0;
                telemetryFps = (telemetryFps == 0.0) ? inst
                                                     : 0.9 * telemetryFps + 0.1 * inst;
                const double bytes = sizeof(FrameHeader) +
                    snap.records.size() * sizeof(TelemetryRecord);
                throughputKBs = telemetryFps * bytes / 1024.0;
            }
            lastTsNs = snap.header.timestampNs;
            lastFrameId = snap.header.frameId;
            neutralizedPeak = std::max<int>(
                neutralizedPeak, static_cast<int>(snap.header.interceptCount));

            // Update trails and spawn detonation FX from this frame.
            for (const auto& r : snap.records) {
                if (r.flags & FLAG_DESTROYED) {
                    detonations.push_back(Detonation{recPos(r), 0.0f});
                    trails.erase(r.entityId);
                    if (r.flags & FLAG_ASSET_HIT) ++assetLosses;
                    continue;
                }
                auto& tr = trails[r.entityId];
                tr.push_back(TrailPt{recPos(r), isBooster(r)});
                if (tr.size() > 60) tr.pop_front();
            }
        }

        // --- Trackable set (hostiles then interceptors) for target-follow -----
        std::vector<std::uint32_t> trackables;
        for (const auto& r : snap.records)
            if (isHostile(r) && !(r.flags & FLAG_DESTROYED)) trackables.push_back(r.entityId);
        for (const auto& r : snap.records)
            if (isInterceptor(r) && r.targetId != kNoTargetId) trackables.push_back(r.entityId);

        if (IsKeyPressed(KEY_TAB)) {
            if (trackables.empty()) trackIndex = -1;
            else trackIndex = (trackIndex + 1) % static_cast<int>(trackables.size());
        }
        if (IsKeyPressed(KEY_C)) trackIndex = -1; // free camera

        const bool tracking = trackIndex >= 0 &&
                              trackIndex < static_cast<int>(trackables.size());
        if (tracking) {
            const std::uint32_t id = trackables[trackIndex];
            for (const auto& r : snap.records) {
                if (r.entityId == id) {
                    // Smoothly chase the tracked entity.
                    cam.target = Vector3Lerp(cam.target, recPos(r), 0.2f);
                    break;
                }
            }
        }
        updateCamera(cam, tracking);

        // Age detonations.
        const float dt = GetFrameTime();
        for (auto& d : detonations) d.age += dt;
        detonations.erase(
            std::remove_if(detonations.begin(), detonations.end(),
                           [](const Detonation& d) { return d.age > 1.2f; }),
            detonations.end());

        // --- Render -----------------------------------------------------------
        BeginDrawing();
        ClearBackground(Color{8, 10, 16, 255});

        BeginMode3D(cam.toCamera());
        {
            // Solid terrain deck just below Z = 0 hides anything that grazes the
            // ground for a frame and gives the airspace a floor to read against.
            DrawPlane(Vector3{0.0f, -0.02f, 0.0f}, Vector2{100.0f, 100.0f},
                      Color{16, 22, 30, 255});
            // Airspace grid (100 x 100 km) and altitude ceiling wireframe,
            // both centered on the origin to match the recentered world.
            DrawGrid(100, 1.0f);
            DrawCubeWires(Vector3{0.0f, 10.0f, 0.0f}, 100.0f, 20.0f, 100.0f,
                          Color{40, 60, 90, 255});

            // Defended city: skyscrapers, hospital, suburb as stylized blocks.
            for (const sim::CityStructure& s : city) {
                const RayBox rb = worldBoxToRay(s.box);
                // Anchor the block on the deck (worldToRay puts the box center
                // at half height, which is already correct for a ground box).
                Color face, edge;
                switch (s.kind) {
                    case sim::StructureKind::Skyscraper:
                        face = Color{58, 76, 112, 245}; edge = Color{150, 200, 255, 255}; break;
                    case sim::StructureKind::Hospital:
                        face = Color{92, 54, 62, 245}; edge = Color{255, 140, 150, 255}; break;
                    default: // Suburb
                        face = Color{46, 64, 54, 235}; edge = Color{140, 210, 160, 255}; break;
                }
                DrawCubeV(rb.center, rb.size, face);
                DrawCubeWiresV(rb.center, rb.size, edge);
            }

            // Defended asset (ground) + battery ring at the city core.
            const Vector3 asset = worldToRay(50000.0f, 50000.0f, 0.0f);
            DrawCube(asset, 1.6f, 0.8f, 1.6f, Color{0, 220, 235, 255});
            DrawCircle3D(asset, 8.0f, Vector3{1, 0, 0}, 90.0f,
                         Color{0, 120, 160, 180});

            int activeHostiles = 0, activeInterceptors = 0;

            // Trails: hot orange for the boost phase, cool blue/green for cruise.
            for (const auto& kv : trails) {
                const auto& pts = kv.second;
                for (std::size_t i = 1; i < pts.size(); ++i) {
                    const float fade = static_cast<float>(i) / pts.size();
                    const unsigned char a =
                        static_cast<unsigned char>(60.0f + 140.0f * fade);
                    const Color c = pts[i].hot ? Color{255, 150, 40, a}
                                               : Color{90, 150, 190, a};
                    DrawLine3D(pts[i - 1].p, pts[i].p, c);
                }
            }

            // Entities.
            for (const auto& r : snap.records) {
                if (r.flags & FLAG_DESTROYED) continue;
                const Vector3 p = recPos(r);
                const Vector3 v = recVel(r);

                // Launch-flame cue: a hot dart trailing the boosting motor.
                if (isBooster(r) && Vector3Length(v) > 1e-6f) {
                    const Vector3 back =
                        Vector3Scale(Vector3Normalize(v), -1.4f);
                    DrawSphere(Vector3Add(p, back), 0.35f, Color{255, 140, 30, 230});
                    DrawSphere(Vector3Add(p, Vector3Scale(back, 1.6f)), 0.2f,
                               Color{255, 90, 20, 160});
                }

                if (isHostile(r)) {
                    ++activeHostiles;
                    DrawSphere(p, 0.6f, threatColor(r.threatLevel));
                    // Heading dart (fixed length so it reads as direction).
                    drawHeading(p, v, 3.5f, RED);
                } else if (isInterceptor(r)) {
                    ++activeInterceptors;
                    const bool engaged = r.targetId != kNoTargetId;
                    const bool neutral = isNeutral(r);
                    // Friendly = bright green/skyblue; allied-neutral = dark green.
                    const Color body =
                        neutral ? (engaged ? Color{30, 130, 50, 255}
                                           : Color{22, 95, 38, 255})
                                : (engaged ? GREEN : SKYBLUE);
                    DrawSphere(p, 0.5f, body);
                    drawHeading(p, v, 3.0f, neutral ? Color{40, 150, 60, 255}
                                                    : Color{120, 200, 255, 255});
                    // ProNav LOS line to assigned hostile.
                    if (engaged) {
                        const Color los = neutral ? Color{40, 150, 60, 140}
                                                  : Color{0, 255, 120, 140};
                        for (const auto& t : snap.records) {
                            if (t.entityId == r.targetId &&
                                !(t.flags & FLAG_DESTROYED)) {
                                DrawLine3D(p, recPos(t), los);
                                break;
                            }
                        }
                    }
                } else {
                    DrawSphere(p, 0.35f, Color{90, 100, 110, 255}); // neutral traffic
                }
            }
            (void)activeInterceptors;

            // Detonation FX: an expanding fading wireframe sphere plus a slower
            // ground ring that lingers as a tactical signature of the kill.
            for (const auto& d : detonations) {
                const float radius = 0.5f + d.age * 6.0f;
                const unsigned char a = static_cast<unsigned char>(
                    Clamp(255.0f * (1.0f - d.age / 1.2f), 0.0f, 255.0f));
                DrawSphereWires(d.pos, radius, 8, 8, ColorAlpha(Color{255, 160, 40, 255},
                                                                a / 255.0f));
                const float ringR = 0.5f + d.age * 9.0f;
                const unsigned char ra = static_cast<unsigned char>(
                    Clamp(200.0f * (1.0f - d.age / 1.2f), 0.0f, 255.0f));
                DrawCircle3D(Vector3{d.pos.x, 0.02f, d.pos.z}, ringR,
                             Vector3{1, 0, 0}, 90.0f, Color{255, 120, 30, ra});
            }
        }
        EndMode3D();

        // --- C2 HUD (2D overlay) ---------------------------------------------
        int activeHostiles = 0;
        for (const auto& r : snap.records)
            if (isHostile(r) && !(r.flags & FLAG_DESTROYED)) ++activeHostiles;
        const int neutralized = neutralizedPeak;
        const int totalEngaged = neutralized + activeHostiles;
        const float successRate =
            totalEngaged > 0 ? 100.0f * neutralized / totalEngaged : 0.0f;

        DrawRectangle(0, 0, 360, 216, Color{0, 0, 0, 160});
        DrawText("C2 TACTICAL DISPLAY", 16, 12, 20, RAYWHITE);
        char line[128];
        std::snprintf(line, sizeof(line), "source        : %s", consumer->SourceName());
        DrawText(line, 16, 40, 18, LIGHTGRAY);
        std::snprintf(line, sizeof(line), "active hostiles : %d", activeHostiles);
        DrawText(line, 16, 62, 18, activeHostiles > 0 ? ORANGE : GREEN);
        std::snprintf(line, sizeof(line), "neutralized     : %d", neutralized);
        DrawText(line, 16, 84, 18, GREEN);
        std::snprintf(line, sizeof(line), "asset losses    : %d", assetLosses);
        DrawText(line, 16, 106, 18, assetLosses > 0 ? RED : GRAY);
        std::snprintf(line, sizeof(line), "success rate    : %.0f%%", successRate);
        DrawText(line, 16, 128, 18, RAYWHITE);
        std::snprintf(line, sizeof(line), "telemetry       : %.1f Hz  (%.0f KB/s)",
                      telemetryFps, throughputKBs);
        DrawText(line, 16, 150, 18, SKYBLUE);
        std::snprintf(line, sizeof(line), "frame           : %llu",
                      static_cast<unsigned long long>(snap.header.frameId));
        DrawText(line, 16, 172, 18, GRAY);

        DrawText("[Tab] track  [C] free-cam  [drag] orbit  [wheel] zoom  [WASD/QE] pan",
                 16, screenH - 26, 16, Color{160, 170, 180, 255});
        DrawFPS(screenW - 90, 12);

        if (tracking) {
            std::snprintf(line, sizeof(line), "TRACKING #%u",
                          trackables[trackIndex]);
            DrawText(line, screenW - 220, 40, 20, GREEN);
        }

        EndDrawing();

        // --- Headless capture / bounded run ----------------------------------
        ++frameCounter;
        if (opt.maxFrames > 0 && frameCounter >= opt.maxFrames) {
            if (!opt.screenshot.empty()) {
                TakeScreenshot(opt.screenshot.c_str());
                TraceLog(LOG_INFO, "screenshot saved: %s", opt.screenshot.c_str());
            }
            break;
        }
    }

    CloseWindow();
    return 0;
}
