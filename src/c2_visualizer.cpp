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

// A fixed-length heading dart in the direction of travel: shows where a track
// is pointing without the raw velocity magnitude dominating the view.
void drawHeading(Vector3 p, Vector3 velRay, float len, Color c) {
    if (Vector3Length(velRay) < 1e-6f) return;
    DrawLine3D(p, Vector3Add(p, Vector3Scale(Vector3Normalize(velRay), len)), c);
}

// --- Orbit / arcball camera state --------------------------------------------
struct OrbitCamera {
    Vector3 target{0.0f, 5.0f, 0.0f}; // airspace center maps to origin
    float   distance{120.0f};
    float   yaw{0.7f};
    float   pitch{0.55f};

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
    // Zoom.
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
        if (IsKeyDown(KEY_W)) c.target = Vector3Add(c.target, Vector3Scale(fwd, pan));
        if (IsKeyDown(KEY_S)) c.target = Vector3Subtract(c.target, Vector3Scale(fwd, pan));
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

bool isHostile(const TelemetryRecord& r) {
    return r.entityType == static_cast<std::uint8_t>(sim::EntityType::Hostile);
}
bool isFriendly(const TelemetryRecord& r) {
    return r.entityType == static_cast<std::uint8_t>(sim::EntityType::Friendly);
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
    TelemetrySnapshot snap;
    std::unordered_map<std::uint32_t, std::deque<Vector3>> trails;
    std::vector<Detonation> detonations;

    std::uint64_t lastFrameId = UINT64_MAX;
    std::uint64_t lastTsNs = 0;
    double telemetryFps = 0.0;
    double throughputKBs = 0.0;

    int trackIndex = -1; // -1 = free camera; otherwise index into trackables
    int neutralizedPeak = 0;
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
                    continue;
                }
                auto& tr = trails[r.entityId];
                tr.push_back(recPos(r));
                if (tr.size() > 60) tr.pop_front();
            }
        }

        // --- Trackable set (hostiles then interceptors) for target-follow -----
        std::vector<std::uint32_t> trackables;
        for (const auto& r : snap.records)
            if (isHostile(r) && !(r.flags & FLAG_DESTROYED)) trackables.push_back(r.entityId);
        for (const auto& r : snap.records)
            if (isFriendly(r) && r.targetId != kNoTargetId) trackables.push_back(r.entityId);

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
            // Airspace grid (100 x 100 km) and altitude ceiling wireframe,
            // both centered on the origin to match the recentered world.
            DrawGrid(100, 1.0f);
            DrawCubeWires(Vector3{0.0f, 10.0f, 0.0f}, 100.0f, 20.0f, 100.0f,
                          Color{40, 60, 90, 255});

            // Defended asset (ground) + battery ring.
            const Vector3 asset = worldToRay(50000.0f, 50000.0f, 0.0f);
            DrawCube(asset, 3.0f, 1.5f, 3.0f, Color{0, 180, 200, 255});
            DrawCircle3D(asset, 8.0f, Vector3{1, 0, 0}, 90.0f,
                         Color{0, 120, 160, 180});

            int activeHostiles = 0, activeInterceptors = 0;

            // Trails.
            for (const auto& kv : trails) {
                const auto& pts = kv.second;
                for (std::size_t i = 1; i < pts.size(); ++i) {
                    DrawLine3D(pts[i - 1], pts[i], Color{80, 80, 90, 120});
                }
            }

            // Entities.
            for (const auto& r : snap.records) {
                if (r.flags & FLAG_DESTROYED) continue;
                const Vector3 p = recPos(r);
                const Vector3 v = recVel(r);

                if (isHostile(r)) {
                    ++activeHostiles;
                    DrawSphere(p, 0.6f, threatColor(r.threatLevel));
                    // Heading dart (fixed length so it reads as direction).
                    drawHeading(p, v, 3.5f, RED);
                } else if (isFriendly(r)) {
                    ++activeInterceptors;
                    const bool engaged = r.targetId != kNoTargetId;
                    DrawSphere(p, 0.5f, engaged ? GREEN : SKYBLUE);
                    drawHeading(p, v, 3.0f, Color{120, 200, 255, 255});
                    // ProNav LOS line to assigned hostile.
                    if (engaged) {
                        for (const auto& t : snap.records) {
                            if (t.entityId == r.targetId &&
                                !(t.flags & FLAG_DESTROYED)) {
                                DrawLine3D(p, recPos(t), Color{0, 255, 120, 140});
                                break;
                            }
                        }
                    }
                } else {
                    DrawSphere(p, 0.35f, Color{90, 100, 110, 255}); // neutral
                }
            }
            (void)activeInterceptors;

            // Detonation FX: expanding fading wireframe spheres.
            for (const auto& d : detonations) {
                const float radius = 0.5f + d.age * 6.0f;
                const unsigned char a =
                    static_cast<unsigned char>(Clamp(255.0f * (1.0f - d.age / 1.2f), 0.0f, 255.0f));
                DrawSphereWires(d.pos, radius, 8, 8, Color{255, 160, 40, a});
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

        DrawRectangle(0, 0, 360, 172, Color{0, 0, 0, 160});
        DrawText("C2 TACTICAL DISPLAY", 16, 12, 20, RAYWHITE);
        char line[128];
        std::snprintf(line, sizeof(line), "source        : %s", consumer->SourceName());
        DrawText(line, 16, 40, 18, LIGHTGRAY);
        std::snprintf(line, sizeof(line), "active hostiles : %d", activeHostiles);
        DrawText(line, 16, 62, 18, activeHostiles > 0 ? ORANGE : GREEN);
        std::snprintf(line, sizeof(line), "neutralized     : %d", neutralized);
        DrawText(line, 16, 84, 18, GREEN);
        std::snprintf(line, sizeof(line), "success rate    : %.0f%%", successRate);
        DrawText(line, 16, 106, 18, RAYWHITE);
        std::snprintf(line, sizeof(line), "telemetry       : %.1f Hz  (%.0f KB/s)",
                      telemetryFps, throughputKBs);
        DrawText(line, 16, 128, 18, SKYBLUE);
        std::snprintf(line, sizeof(line), "frame           : %llu",
                      static_cast<unsigned long long>(snap.header.frameId));
        DrawText(line, 16, 150, 18, GRAY);

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
