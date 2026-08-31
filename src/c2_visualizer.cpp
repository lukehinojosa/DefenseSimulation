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
#include <unordered_set>
#include <vector>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

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

// A fixed-length heading dart in the direction of travel: shows where a track
// is pointing without the raw velocity magnitude dominating the view.
void drawHeading(Vector3 p, Vector3 velRay, float len, Color c) {
    if (Vector3Length(velRay) < 1e-6f) return;
    DrawLine3D(p, Vector3Add(p, Vector3Scale(Vector3Normalize(velRay), len)), c);
}

// --- To-scale landmark rendering ---------------------------------------------
// Every structure is drawn at its real dimensions (meters / kScale = units).
// The bounding box gives the ground footprint and height; kind selects the
// silhouette. worldToRay maps world (x east, y north, z up) to Raylib
// (x east, y up, z north), so a box's world-Y half-extent becomes the ray-Z
// half-extent below.
// Double-sided triangle: draw both windings so the face is visible from any
// orbit angle (Raylib back-face culls a single-wound DrawTriangle3D).
void tri2(Vector3 a, Vector3 b, Vector3 c, Color col) {
    DrawTriangle3D(a, b, c, col);
    DrawTriangle3D(a, c, b, col);
}

// Per-building-class color (the instancing shader adds directional shading).
Color kindColor(sim::MapKind k) {
    switch (k) {
        case sim::MapKind::Secretariat: return Color{ 70, 175, 185, 255}; // glass slab
        case sim::MapKind::UnPlaza:     return Color{ 42,  96, 145, 255}; // campus plaza
        case sim::MapKind::UnComplex:   return Color{100, 145, 185, 255};
        case sim::MapKind::Skyscraper:  return Color{110, 128, 160, 255};
        default:                        return Color{ 82,  94, 114, 255}; // Midtown block
    }
}

// Build a raylib Mesh (VBO + IBO) from unique vertices + a triangle index list,
// then upload it once to the GPU. All buildings share one of these base meshes
// and are drawn with per-instance transforms (flyweight instancing).
Mesh finishMesh(const std::vector<float>& pos, const std::vector<float>& nrm,
                const std::vector<unsigned short>& idx) {
    Mesh m{};
    m.vertexCount   = static_cast<int>(pos.size() / 3);
    m.triangleCount = static_cast<int>(idx.size() / 3);
    m.vertices = static_cast<float*>(MemAlloc(
        static_cast<unsigned int>(pos.size() * sizeof(float))));
    std::memcpy(m.vertices, pos.data(), pos.size() * sizeof(float));
    m.normals = static_cast<float*>(MemAlloc(
        static_cast<unsigned int>(nrm.size() * sizeof(float))));
    std::memcpy(m.normals, nrm.data(), nrm.size() * sizeof(float));
    m.indices = static_cast<unsigned short*>(MemAlloc(
        static_cast<unsigned int>(idx.size() * sizeof(unsigned short))));
    std::memcpy(m.indices, idx.data(), idx.size() * sizeof(unsigned short));
    UploadMesh(&m, false);
    return m;
}

// Unit box: x,z in [-0.5, 0.5], y in [0, 1] (base on the ground). Per-face
// normals; winding auto-corrected to face outward so backface culling is safe.
Mesh makeBoxMesh() {
    std::vector<float> pos, nrm;
    std::vector<unsigned short> idx;
    auto quad = [&](Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector3 n) {
        const Vector3 fn = Vector3CrossProduct(Vector3Subtract(b, a),
                                               Vector3Subtract(c, a));
        if (Vector3DotProduct(fn, n) < 0.0f) std::swap(b, d); // ensure CCW-outward
        const unsigned short base = static_cast<unsigned short>(pos.size() / 3);
        for (const Vector3& p : {a, b, c, d}) {
            pos.push_back(p.x); pos.push_back(p.y); pos.push_back(p.z);
            nrm.push_back(n.x); nrm.push_back(n.y); nrm.push_back(n.z);
        }
        for (unsigned short e : {0, 1, 2, 0, 2, 3})
            idx.push_back(static_cast<unsigned short>(base + e));
    };
    const float lo = -0.5f, hi = 0.5f, y0 = 0.0f, y1 = 1.0f;
    const Vector3 A{lo,y0,lo},B{hi,y0,lo},C{hi,y0,hi},D{lo,y0,hi};
    const Vector3 E{lo,y1,lo},F{hi,y1,lo},G{hi,y1,hi},H{lo,y1,hi};
    quad(E,F,G,H,{0,1,0});   quad(A,B,C,D,{0,-1,0});
    quad(D,C,G,H,{0,0,1});   quad(A,B,F,E,{0,0,-1});
    quad(B,C,G,F,{1,0,0});   quad(A,D,H,E,{-1,0,0});
    return finishMesh(pos, nrm, idx);
}

// Unit cylinder: radius 0.5 (x,z), y in [0, 1]. Smooth radial side normals + a
// flat top cap. Scaled by hx/hy per instance, so it becomes a circle or ellipse.
Mesh makeCylinderMesh(int sides) {
    std::vector<float> pos, nrm;
    std::vector<unsigned short> idx;
    auto add = [&](Vector3 p, Vector3 n) {
        const unsigned short id = static_cast<unsigned short>(pos.size() / 3);
        pos.push_back(p.x); pos.push_back(p.y); pos.push_back(p.z);
        nrm.push_back(n.x); nrm.push_back(n.y); nrm.push_back(n.z);
        return id;
    };
    std::vector<unsigned short> bId(sides), tId(sides);
    for (int i = 0; i < sides; ++i) {
        const float a = 2.0f * PI * i / sides, c = cosf(a), s = sinf(a);
        bId[i] = add({0.5f * c, 0.0f, 0.5f * s}, {c, 0.0f, s});
        tId[i] = add({0.5f * c, 1.0f, 0.5f * s}, {c, 0.0f, s});
    }
    for (int i = 0; i < sides; ++i) {
        const int j = (i + 1) % sides;
        for (unsigned short e : {bId[i], tId[j], bId[j]}) idx.push_back(e);
        for (unsigned short e : {bId[i], tId[i], tId[j]}) idx.push_back(e);
    }
    const unsigned short cap = add({0.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
    std::vector<unsigned short> tc(sides);
    for (int i = 0; i < sides; ++i) {
        const float a = 2.0f * PI * i / sides;
        tc[i] = add({0.5f * cosf(a), 1.0f, 0.5f * sinf(a)}, {0.0f, 1.0f, 0.0f});
    }
    for (int i = 0; i < sides; ++i) {
        const int j = (i + 1) % sides;
        for (unsigned short e : {cap, tc[j], tc[i]}) idx.push_back(e);
    }
    return finishMesh(pos, nrm, idx);
}

// Instancing shader (GLSL 330): each instance supplies a model transform; the
// fragment stage applies simple directional shading over a per-group color.
const char* kInstVS =
    "#version 330\n"
    "in vec3 vertexPosition;\n"
    "in vec3 vertexNormal;\n"
    "in mat4 instanceTransform;\n"
    "uniform mat4 mvp;\n"
    "out vec3 fragNormal;\n"
    "void main() {\n"
    "    fragNormal = mat3(instanceTransform) * vertexNormal;\n"
    "    gl_Position = mvp * instanceTransform * vec4(vertexPosition, 1.0);\n"
    "}\n";
const char* kInstFS =
    "#version 330\n"
    "in vec3 fragNormal;\n"
    "uniform vec4 colDiffuse;\n"
    "out vec4 finalColor;\n"
    "void main() {\n"
    "    vec3 L = normalize(vec3(0.35, 0.85, 0.30));\n"
    "    float d = 0.55 + 0.45 * abs(dot(normalize(fragNormal), L));\n"
    "    finalColor = vec4(colDiffuse.rgb * d, colDiffuse.a);\n"
    "}\n";

// --- Stage 3: view-frustum culling -------------------------------------------
// The whole 1:1 city is far larger than any one view, so submitting every
// instance every frame wastes vertex work on buildings behind the camera or off
// to the sides. We extract the six frustum planes from the current view-proj
// matrix and keep only the instances whose bounding sphere survives, rebuilding
// a compact transform list per group each frame. Sphere test is rotation-proof,
// so it's exact for the box/cylinder primitives regardless of instance yaw.
struct Frustum { Vector4 plane[6]; }; // a*x + b*y + c*z + d, outward-normalised

// Gribb-Hartmann extraction from M = modelview * projection (raylib row layout).
// Call inside BeginMode3D, after the camera matrices are pushed.
Frustum extractFrustum() {
    const Matrix m = MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());
    Frustum f;
    f.plane[0] = { m.m3 - m.m0, m.m7 - m.m4, m.m11 - m.m8,  m.m15 - m.m12 }; // right
    f.plane[1] = { m.m3 + m.m0, m.m7 + m.m4, m.m11 + m.m8,  m.m15 + m.m12 }; // left
    f.plane[2] = { m.m3 + m.m1, m.m7 + m.m5, m.m11 + m.m9,  m.m15 + m.m13 }; // bottom
    f.plane[3] = { m.m3 - m.m1, m.m7 - m.m5, m.m11 - m.m9,  m.m15 - m.m13 }; // top
    f.plane[4] = { m.m3 + m.m2, m.m7 + m.m6, m.m11 + m.m10, m.m15 + m.m14 }; // near
    f.plane[5] = { m.m3 - m.m2, m.m7 - m.m6, m.m11 - m.m10, m.m15 - m.m14 }; // far
    for (Vector4& p : f.plane) {
        const float len = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        if (len > 0.0f) { p.x /= len; p.y /= len; p.z /= len; p.w /= len; }
    }
    return f;
}

// A sphere is visible unless it lies fully behind any one plane.
bool sphereVisible(const Frustum& f, Vector3 c, float r) {
    for (const Vector4& p : f.plane)
        if (p.x * c.x + p.y * c.y + p.z * c.z + p.w < -r) return false;
    return true;
}

// One flyweight group: all instances sharing a (kind, prim). Transforms upload
// to the GPU; the parallel bounding spheres drive per-frame frustum culling.
// @c visible is scratch, refilled each frame with the surviving transforms.
struct InstanceGroup {
    std::vector<Matrix>  xform;
    std::vector<Vector3> center;
    std::vector<float>   radius;
    std::vector<Matrix>  visible;
    unsigned int vaoId = 0;   // persistent instanced VAO (static fast path)
    unsigned int instVbo = 0; // persistent per-instance transform VBO
};

// Bake a set of flat (local-metre, z = 0) triangles into one static mesh at a
// fixed display height, single uniform color. Used for the East River surface.
// Local metres map to display units by /kScale (the map origin is the asset,
// which sits at the ray-space origin), so east->x, north->z.
Mesh buildFlatMesh(const std::vector<sim::Vector3>& tris, float y, Color col) {
    std::vector<float> verts;
    std::vector<unsigned char> cols;
    verts.reserve(tris.size() * 3);
    cols.reserve(tris.size() * 4);
    for (const sim::Vector3& v : tris) {
        verts.push_back(static_cast<float>(v.x) / kScale);
        verts.push_back(y);
        verts.push_back(static_cast<float>(v.y) / kScale);
        cols.push_back(col.r); cols.push_back(col.g);
        cols.push_back(col.b); cols.push_back(col.a);
    }
    Mesh m{};
    m.vertexCount   = static_cast<int>(verts.size() / 3);
    m.triangleCount = m.vertexCount / 3;
    m.vertices = static_cast<float*>(MemAlloc(
        static_cast<unsigned int>(verts.size() * sizeof(float))));
    std::memcpy(m.vertices, verts.data(), verts.size() * sizeof(float));
    m.colors = static_cast<unsigned char*>(
        MemAlloc(static_cast<unsigned int>(cols.size())));
    std::memcpy(m.colors, cols.data(), cols.size());
    UploadMesh(&m, false);
    return m;
}

// --- Missile model -----------------------------------------------------------
// A slender body + nose cone aligned to the velocity vector, with tail fins and
// an optional boost plume. Dimensions are passed in display metres (scaled up
// from real vehicle size so they read against the magnified skyline) and
// converted to world units.
void drawMissile(Vector3 pos, Vector3 velRay, float lengthM, float radiusM,
                 Color body, Color nose, bool boosting, Color plume) {
    Vector3 fwd = velRay;
    if (Vector3Length(fwd) < 1e-9f) fwd = Vector3{0.0f, 1.0f, 0.0f};
    fwd = Vector3Normalize(fwd);
    const float len = lengthM / kScale;
    const float rad = radiusM / kScale;

    const Vector3 ref = (fabsf(fwd.y) > 0.9f) ? Vector3{1, 0, 0} : Vector3{0, 1, 0};
    const Vector3 right = Vector3Normalize(Vector3CrossProduct(fwd, ref));
    const Vector3 up    = Vector3Normalize(Vector3CrossProduct(right, fwd));

    const Vector3 tail     = Vector3Add(pos, Vector3Scale(fwd, -len * 0.5f));
    const Vector3 shoulder = Vector3Add(pos, Vector3Scale(fwd,  len * 0.18f));
    const Vector3 tip      = Vector3Add(pos, Vector3Scale(fwd,  len * 0.5f));

    DrawCylinderEx(tail, shoulder, rad, rad, 8, body); // airframe
    DrawCylinderEx(shoulder, tip, rad, 0.0f, 8, nose); // ogive nose

    // Four tail fins.
    const Vector3 finRoot = Vector3Add(pos, Vector3Scale(fwd, -len * 0.32f));
    const Vector3 dirs[4] = {right, Vector3Scale(right, -1.0f), up,
                             Vector3Scale(up, -1.0f)};
    for (const Vector3& d : dirs) {
        const Vector3 a = Vector3Add(finRoot, Vector3Scale(d, rad));
        const Vector3 b = Vector3Add(tail, Vector3Scale(d, rad));
        const Vector3 c = Vector3Add(tail, Vector3Scale(d, rad * 3.2f));
        tri2(a, b, c, body);
    }

    // Boost plume: a hot tapering flame off the tail.
    if (boosting) {
        const Vector3 pEnd = Vector3Add(tail, Vector3Scale(fwd, -len * 0.9f));
        DrawCylinderEx(tail, pEnd, rad * 0.9f, 0.0f, 8, plume);
        DrawSphere(Vector3Add(tail, Vector3Scale(fwd, -len * 0.25f)), rad * 1.1f,
                   ColorAlpha(plume, 0.6f));
    }
}

// --- Orbit / arcball camera state --------------------------------------------
struct OrbitCamera {
    Vector3 target{0.0f, 1.5f, 0.0f}; // airspace center maps to origin
    float   distance{30.0f};          // frames the UN skyline and the raid
    float   yaw{0.7f};
    float   pitch{0.62f};             // low enough to see the skyline in profile

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

void updateCamera(OrbitCamera& c, bool trackingActive, bool blockMouse = false) {
    // Zoom (wheel up = zoom in). Suppressed while the pointer is over the panel.
    const float wheel = blockMouse ? 0.0f : GetMouseWheelMove();
    if (wheel != 0.0f) {
        c.distance *= (1.0f - wheel * 0.1f);
        c.distance = Clamp(c.distance, 1.0f, 400.0f);
    }
    // Orbit: left/right mouse drag, or arrow keys. Mouse drag is disabled while a
    // slider is being manipulated so the two don't fight over the drag.
    if (!blockMouse && (IsMouseButtonDown(MOUSE_BUTTON_LEFT) ||
        IsMouseButtonDown(MOUSE_BUTTON_RIGHT))) {
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

// The representative color of a track (matches its rendered silhouette), used
// for the hover ring.
Color trackColor(const TelemetryRecord& r) {
    if (isHostile(r)) return threatColor(r.threatLevel);
    const bool engaged = r.targetId != kNoTargetId;
    if (isNeutral(r)) return engaged ? Color{30, 130, 50, 255} : Color{22, 95, 38, 255};
    return engaged ? GREEN : SKYBLUE; // friendly
}

// The hover-ring allegiance label: our own battery rounds are DOMESTIC, allied
// airborne interceptors are ALLY, inbound threats are ENEMY.
const char* trackLabel(const TelemetryRecord& r) {
    if (isHostile(r))  return "ENEMY";
    if (isNeutral(r))  return "ALLY";
    return "DOMESTIC";
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
    // Resizable window; the framebuffer (and thus the render resolution) tracks the
    // window size, and screenW/screenH are refreshed every frame from it. F11
    // toggles borderless fullscreen.
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    int screenW = 1280, screenH = 720;
    InitWindow(screenW, screenH, "Defense Simulation - C2 Tactical Display");
    SetWindowMinSize(800, 480);
    SetTargetFPS(60);
    // ESC is repurposed to end camera-follow, so disable raylib's default
    // "ESC closes the window" behaviour (the window's close button still quits).
    SetExitKey(KEY_NULL);

    // All UI text uses Roboto (found next to the other assets); DrawTextEx via the
    // DT() helper below. Falls back to raylib's built-in font if the file is absent.
    auto resolveAsset = [](const char* name) -> std::string {
        std::string p = std::string("assets/") + name;
        for (int i = 0; i < 6; ++i) {
            if (FILE* f = std::fopen(p.c_str(), "rb")) { std::fclose(f); return p; }
            p = "../" + p;
        }
        return std::string("assets/") + name;
    };
    Font uiFont = LoadFontEx(resolveAsset("Roboto-Regular.ttf").c_str(), 32, nullptr, 0);
    const bool haveFont = uiFont.texture.id != 0;
    if (haveFont) SetTextureFilter(uiFont.texture, TEXTURE_FILTER_BILINEAR);
    else          uiFont = GetFontDefault();
    auto DT = [&](const char* t, int x, int y, int sz, Color c) {
        DrawTextEx(uiFont, t, Vector2{static_cast<float>(x), static_cast<float>(y)},
                   static_cast<float>(sz), 1.0f, c);
    };

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
        DT("Waiting for telemetry producer...", 40, 40, 20, RAYWHITE);
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
    if (opt.distance > 0.0f) cam.distance = Clamp(opt.distance, 1.0f, 400.0f);
    TelemetrySnapshot snap;
    std::unordered_map<std::uint32_t, std::deque<TrailPt>> trails;
    std::unordered_map<std::uint32_t, double> trailSeen; // wall-clock last update
    std::vector<Detonation> detonations;

    // Static defended city: the real UN HQ + Midtown Manhattan, loaded 1:1 from
    // the binary map asset and drawn as flyweight instances -- two shared meshes
    // (box + cylinder) with a per-instance transform buffer. Group the instances
    // by (kind, prim) so each group is one DrawMeshInstanced with a class color.
    sim::MapData map;
    const std::string mapPath = sim::resolveMapPath();
    if (!sim::loadMapData(mapPath, map))
        TraceLog(LOG_WARNING, "map: failed to load %s", mapPath.c_str());

    const Mesh baseMesh[2] = {makeBoxMesh(), makeCylinderMesh(16)};
    Shader instShader = LoadShaderFromMemory(kInstVS, kInstFS);
    instShader.locs[SHADER_LOC_MATRIX_MVP] = GetShaderLocation(instShader, "mvp");
    instShader.locs[SHADER_LOC_MATRIX_MODEL] =
        GetShaderLocationAttrib(instShader, "instanceTransform");
    instShader.locs[SHADER_LOC_COLOR_DIFFUSE] =
        GetShaderLocation(instShader, "colDiffuse");
    Material cityMat = LoadMaterialDefault();
    cityMat.shader = instShader;

    // Per-(kind, prim) flyweight groups (ray space; local metres/kScale). Each
    // instance also gets a world bounding sphere for Stage-3 frustum culling: the
    // centre is the box mid-height, the radius the half-diagonal of the scaled
    // extents -- invariant to the Y-rotation, so it stays a tight, exact bound.
    InstanceGroup groups[5][2];
    for (const sim::MapInstance& m : map.instances) {
        const float sx = 2.0f * m.hx / kScale, sy = m.height / kScale,
                    sz = 2.0f * m.hy / kScale;
        const Matrix S = MatrixScale(sx, sy, sz);
        const Matrix R = MatrixRotateY(-m.angle);
        const Matrix T = MatrixTranslate(m.cx / kScale, 0.0f, m.cy / kScale);
        InstanceGroup& g =
            groups[static_cast<int>(m.kind) & 7][static_cast<int>(m.prim) & 1];
        g.xform.push_back(MatrixMultiply(MatrixMultiply(S, R), T));
        g.center.push_back(Vector3{m.cx / kScale, 0.5f * sy, m.cy / kScale});
        g.radius.push_back(0.5f * std::sqrt(sx * sx + sy * sy + sz * sz));
    }

    // --- Static-geometry fast path: persistent instanced buffers --------------
    // The city never moves, yet raylib's DrawMeshInstanced re-creates and re-uploads
    // the ENTIRE per-instance transform VBO every frame; at ~470k instances that
    // per-frame upload -- not the vertex work -- is what bounds the frame. Instead we
    // upload each group's transforms to the GPU exactly ONCE and give the group its
    // own VAO wired to the shared base-mesh position/normal/index buffers plus that
    // static instance buffer. Each frame then becomes a single glDrawElementsInstanced
    // with zero streaming, while every building is still drawn -- full detail, no cull
    // needed. (The dynamic DrawMeshInstanced path stays for the frustum-cull [F] and
    // LOD [L] modes, which need per-frame visibility rebuilds.)
    const int locMat = instShader.locs[SHADER_LOC_MATRIX_MODEL];
    auto buildStatic = [&](InstanceGroup& g, const Mesh& base) {
        if (g.xform.empty()) return;
        g.vaoId = rlLoadVertexArray();
        rlEnableVertexArray(g.vaoId);
        // Re-reference the shared base-mesh attributes into this VAO: position -> loc
        // 0, normal -> loc 2 (raylib's default attribute bindings), plus the element
        // (index) buffer.
        rlEnableVertexBuffer(base.vboId[0]);
        rlSetVertexAttribute(0, 3, RL_FLOAT, false, 0, 0);
        rlEnableVertexAttribute(0);
        rlEnableVertexBuffer(base.vboId[2]);
        rlSetVertexAttribute(2, 3, RL_FLOAT, false, 0, 0);
        rlEnableVertexAttribute(2);
        rlEnableVertexBufferElement(base.vboId[6]);
        // The mat4 instance transform arrives as 4 consecutive vec4 attributes with a
        // divisor of 1 (advance once per instance). raylib's Matrix stores its floats
        // in row-first memory order (m0,m4,m8,m12,...), but the shader reads column
        // vectors, so we must pack each matrix through MatrixToFloatV (m0,m1,m2,...)
        // exactly as DrawMeshInstanced does -- uploading the raw struct would transpose
        // every transform. Uploaded a single time, here.
        std::vector<float> flat;
        flat.reserve(g.xform.size() * 16);
        for (const Matrix& mtx : g.xform) {
            const float16 f = MatrixToFloatV(mtx);
            flat.insert(flat.end(), f.v, f.v + 16);
        }
        g.instVbo = rlLoadVertexBuffer(
            flat.data(), static_cast<int>(flat.size() * sizeof(float)), false);
        for (int i = 0; i < 4; ++i) {
            rlEnableVertexAttribute(locMat + i);
            rlSetVertexAttribute(locMat + i, 4, RL_FLOAT, false, sizeof(Matrix),
                                 i * static_cast<int>(sizeof(Vector4)));
            rlSetVertexAttributeDivisor(locMat + i, 1);
        }
        rlDisableVertexArray();
    };
    for (int k = 0; k < 5; ++k)
        for (int p = 0; p < 2; ++p) buildStatic(groups[k][p], baseMesh[p]);

    // --- Discrete geometric LOD: an 8-level "combined-buildings" gradient -----
    // Rather than culling distant detail, we SIMPLIFY it across many resolutions.
    // Full detail (individual buildings) is kept out to lodNear so near views stay
    // crisp; then kMaxTiers massing tiers carry the gradient out to lodFar. A grid
    // heightfield reads as a voxel carpet, so instead each tier COMBINES the real
    // buildings in each cell into ONE oriented block: oriented to the dominant local
    // street angle, sized to the actual cluster extent (streets/gaps survive), and
    // raised to a mostly-mean height (smooth, not the spiky per-cell maximum). This
    // looks like merged city blocks. Cell size and hand-off distance are
    // interpolated INDEPENDENTLY (geometric) so cells grow slower than distance and
    // distant blocks subtend a smaller screen size -- smoother, not chunkier.
    constexpr int kMaxTiers = 8;
    float pLodNear = 11.0f, pLodFar = 24.0f;   // hand-off distance span (km)
    float pCellNear = 28.0f, pCellFar = 64.0f; // combine-cell size span (m)
    float pSkyH = 80.0f;                        // skyline color-cutoff height (m)
    float pTiers = 7.0f;                        // massing tier count (rounded)
    float fadeKm = 1.5f;                        // half-width of the dither fade zone (km)

    int   nTiers = 7;
    float lodCell[kMaxTiers], lodT[kMaxTiers]; // per-tier cell size + near dist
    float tierDist[kMaxTiers];                 // editable hand-off distance per tier
    float lodMinE = 0.0f, lodMinN = 0.0f;      // grid origin (m), for cell hashing
    float lodExtE = 1.0f, lodExtN = 1.0f;      // world span (m)
    InstanceGroup massing[kMaxTiers][5];       // [tier][kind]; oriented boxes (prim 0)

    if (!map.instances.empty()) {
        float minE = 1e30f, minN = 1e30f, maxE = -1e30f, maxN = -1e30f;
        for (const sim::MapInstance& m : map.instances) {
            minE = std::min(minE, m.cx - m.hx); maxE = std::max(maxE, m.cx + m.hx);
            minN = std::min(minN, m.cy - m.hy); maxN = std::max(maxN, m.cy + m.hy);
        }
        lodMinE = minE; lodMinN = minN;
        lodExtE = maxE - minE; lodExtN = maxN - minN;
    }

    // Geometric defaults for the per-tier hand-off distances and grid sizes; both
    // are then individually editable via the tuner sliders (distances live; grid
    // sizes trigger a rebuild of that tier on release).
    nTiers = std::max(1, std::min(kMaxTiers, static_cast<int>(pTiers + 0.5f)));
    for (int m = 0; m < nTiers; ++m) {
        const float u = (nTiers > 1) ? static_cast<float>(m) / (nTiers - 1) : 0.0f;
        lodT[m]    = pLodNear * std::pow(pLodFar / pLodNear, u);
        lodCell[m] = pCellNear * std::pow(pCellFar / pCellNear, u);
    }

    // (Re)build all massing tiers by COMBINING buildings, tier by tier. Stamp each
    // tier's grid (lodCell[m]) with the real building footprints, so cells fill
    // where buildings actually are and streets/gaps stay empty; one combined,
    // cell-sized block per occupied cell. Uses the CURRENT lodCell[] so a grid-size
    // slider change just re-runs this.
    auto rebuildMassing = [&]() {
        for (int m = 0; m < nTiers; ++m) {
            for (int k = 0; k < 5; ++k) {
                massing[m][k].xform.clear();
                massing[m][k].center.clear();
                massing[m][k].radius.clear();
            }
            if (map.instances.empty()) continue;
            const float cell = lodCell[m];
            const int tnx = std::max(1, static_cast<int>(lodExtE / cell) + 1);
            const int tny = std::max(1, static_cast<int>(lodExtN / cell) + 1);
            auto clampX = [&](int x) { return std::min(std::max(x, 0), tnx - 1); };
            auto clampY = [&](int y) { return std::min(std::max(y, 0), tny - 1); };
            // Max building height per cell (0 = empty), stamped from footprint AABBs.
            std::vector<float> hc(static_cast<std::size_t>(tnx) * tny, 0.0f);
            for (const sim::MapInstance& b : map.instances) {
                const float ax = std::fabs(b.hx * std::cos(b.angle)) +
                                 std::fabs(b.hy * std::sin(b.angle));
                const float ay = std::fabs(b.hx * std::sin(b.angle)) +
                                 std::fabs(b.hy * std::cos(b.angle));
                const int x0 = clampX(static_cast<int>((b.cx - ax - lodMinE) / cell));
                const int x1 = clampX(static_cast<int>((b.cx + ax - lodMinE) / cell));
                const int y0 = clampY(static_cast<int>((b.cy - ay - lodMinN) / cell));
                const int y1 = clampY(static_cast<int>((b.cy + ay - lodMinN) / cell));
                for (int y = y0; y <= y1; ++y)
                    for (int x = x0; x <= x1; ++x) {
                        float& h = hc[static_cast<std::size_t>(y) * tnx + x];
                        if (b.height > h) h = b.height;
                    }
            }
            // One combined, cell-sized block per occupied cell: buildings sharing a
            // cell merge into it, adjacent occupied cells abut into a continuous
            // mass, and empty cells (streets/water) stay as gaps. Axis-aligned and
            // geography-neutral -- the only per-tier difference is the grid size.
            for (int ty = 0; ty < tny; ++ty)
                for (int tx = 0; tx < tnx; ++tx) {
                    const float h = hc[static_cast<std::size_t>(ty) * tnx + tx];
                    if (h <= 0.0f) continue;
                    const float cx = lodMinE + (tx + 0.5f) * cell;
                    const float cy = lodMinN + (ty + 0.5f) * cell;
                    const float sx = cell / kScale, sy = h / kScale, sz = cell / kScale;
                    const Matrix S = MatrixScale(sx, sy, sz);
                    const Matrix T = MatrixTranslate(cx / kScale, 0.0f, cy / kScale);
                    const int kind = (h >= pSkyH) ? static_cast<int>(sim::MapKind::Skyscraper)
                                                  : static_cast<int>(sim::MapKind::Block);
                    InstanceGroup& g = massing[m][kind];
                    g.xform.push_back(MatrixMultiply(S, T));
                    g.center.push_back(Vector3{cx / kScale, 0.5f * sy, cy / kScale});
                    g.radius.push_back(0.5f * std::sqrt(sx * sx + sy * sy + sz * sz));
                }
        }
    };
    rebuildMassing();
    // Seed the editable hand-off distances from the geometric defaults; the tuner
    // sliders adjust these directly (cell sizes stay baked, so no rebuild needed).
    for (int m = 0; m < kMaxTiers; ++m) tierDist[m] = lodT[m];

    const Mesh waterMesh = buildFlatMesh(map.waterTris, -0.006f, Color{24, 60, 104, 255});
    const Material waterMat = LoadMaterialDefault();

    std::uint64_t lastFrameId = UINT64_MAX;
    std::uint64_t lastTsNs = 0;
    double telemetryFps = 0.0;
    double throughputKBs = 0.0;

    // Camera follow: the entity id the camera is chasing (-1 = free camera).
    // Set by Tab-cycling the trackables list or by left-clicking a missile;
    // cleared by [C]/[Esc] or when the followed track leaves the scene.
    long long followId = -1;
    // Rigid-lock bookkeeping: once the camera has acquired the followed track we
    // pin its look-at point exactly to the track, so the track holds a fixed screen
    // position and its residual interpolation jitter moves the camera with it
    // (canceling on screen). followLocked latches after the smooth acquisition and
    // resets whenever the followed id changes.
    long long lockedFollowId = -1;
    bool      followLocked   = false;
    // Left-button click-vs-drag disambiguation: a press that releases without
    // moving far is a pick (select a missile); a moving press orbits the camera.
    Vector2 pressPos{0.0f, 0.0f};
    float    pressDrag = 0.0f;
    int neutralizedPeak = 0;
    int assetLosses = 0; // leaked threats that struck the city (running total)
    int frameCounter = 0;

    // Stage-3 render toggles + per-frame instrumentation so the culling win is
    // measurable on the HUD. [F] frustum cull, [P] depth pre-pass. The pre-pass is
    // OFF by default: at ~470k instances the renderer is bound by the per-frame
    // instance-buffer upload, and the pre-pass doubles that submission for a
    // fragment-stage saving our cheap shader doesn't need -- net-negative here.
    // It stays a toggle for fragment-bound scenes (expensive shading / heavy
    // overdraw), where laying depth down first pays off.
    bool frustumCull = true;
    bool depthPrepass = false;
    bool  staticInst = true;  // [B] persistent-buffer fast path (no per-frame upload)
    bool  lod = false;        // [L] discrete LOD; off by default (artifacts at range)
    int   cityTotal = 0, cityDrawn = 0, cityLod = 0;
    for (int k = 0; k < 5; ++k)
        for (int p = 0; p < 2; ++p) cityTotal += static_cast<int>(groups[k][p].xform.size());

    // --- Snapshot interpolation (entity interpolation) -----------------------
    // Telemetry arrives at the engine's fixed tick rate (60 Hz), well below the
    // render rate, so drawing raw snapshot positions makes fast tracks step
    // visibly. Extrapolating along velocity fills the gaps but overshoots and
    // snaps at each tick. Instead we render a fixed delay in the PAST and slide
    // between buffered snapshots, so every drawn position lies between two
    // known-good samples (never overshoots, never snaps).
    //
    // The subtlety is the TIME BASE. We interpolate in FRAME-INDEX space, not by
    // producer timestamp. The engine advances a FIXED physics step per telemetry
    // frame, so equal frame-id increments are equal motion; the timestamps, by
    // contrast, carry the producer's wall-clock scheduling jitter (a late frame
    // stamps ~2 ticks after the previous one while still advancing a single step),
    // and interpolating against them makes the speed lurch. Playing frame ids back
    // at the smoothed real production rate keeps motion uniform and immune to that
    // jitter, and driving the slide off raylib's smooth clock (not the per-frame
    // arrival detection) avoids folding render-frame jitter in. A short history is
    // kept so the target is always bracketed even when arrivals are uneven; frame
    // gaps from the render outrunning telemetry are absorbed by the bracket span.
    struct Snap { double frame; std::unordered_map<std::uint32_t, Vector3> pos; };
    std::deque<Snap> hist;             // oldest..newest, keyed by frame id
    double rateEma       = 60.0;       // telemetry frames/sec in real time
    double newestArrival = 0.0;        // GetTime() when the newest snapshot landed
    double playF         = 0.0;        // playback position in frame-id space
    bool   havePlay      = false;
    bool   haveArrival   = false;
    const Snap* loSnap = nullptr;      // interpolation bracket, refreshed per frame
    const Snap* hiSnap = nullptr;
    float  interpAlpha = 0.0f;         // position within [loSnap, hiSnap]
    auto livePos = [&](const TelemetryRecord& r) -> Vector3 {
        const Vector3 cur = recPos(r); // fall back to the raw latest sample
        if (loSnap == nullptr || hiSnap == nullptr) return cur;
        const auto lo = loSnap->pos.find(r.entityId);
        const auto hi = hiSnap->pos.find(r.entityId);
        if (lo == loSnap->pos.end() || hi == hiSnap->pos.end()) return cur;
        return Vector3Lerp(lo->second, hi->second, interpAlpha);
    };

    // Return the index into snap.records of the missile whose projected screen
    // position is nearest `sp` and within `maxPx` pixels, or -1. Points behind
    // the camera are rejected (a world point behind the eye projects to a bogus
    // on-screen location).
    auto pickNearestMissile = [&](Vector2 sp, float maxPx) -> int {
        const Camera3D c3 = cam.toCamera();
        const Vector3 fwd = Vector3Normalize(Vector3Subtract(c3.target, c3.position));
        int best = -1;
        float bestD2 = maxPx * maxPx;
        for (int i = 0; i < static_cast<int>(snap.records.size()); ++i) {
            const TelemetryRecord& r = snap.records[i];
            if (r.flags & FLAG_DESTROYED) continue;
            if (!isHostile(r) && !isInterceptor(r)) continue;
            const Vector3 wp = livePos(r);
            if (Vector3DotProduct(Vector3Subtract(wp, c3.position), fwd) <= 0.0f)
                continue; // behind the camera
            const Vector2 s = GetWorldToScreen(wp, c3);
            const float dx = s.x - sp.x, dy = s.y - sp.y, d2 = dx * dx + dy * dy;
            if (d2 < bestD2) { bestD2 = d2; best = i; }
        }
        return best;
    };

    // --- Render-rate control -------------------------------------------------
    // The visualizer is a separate process from the engine, so its refresh rate
    // is fully decoupled from the physics tick: capping (or uncapping) it here
    // never speeds up or slows down the simulation, it only changes how often the
    // latest telemetry snapshot is redrawn. A HUD slider (tied to an integer text
    // box) sets the cap live between 0 and 1000; 0 means uncapped. setFps() is the
    // single writer, so the two widgets never disagree.
    int  targetFps      = 60;      // current render cap; 0 = uncapped
    bool fpsSliderDrag  = false;   // the slider knob is being dragged
    bool fpsBoxEditing  = false;   // the integer text box has keyboard focus
    std::string fpsBoxText = "60"; // live text mirror of targetFps
    auto setFps = [&](int f) {
        f = (f < 0) ? 0 : (f > 1000 ? 1000 : f);
        if (f != targetFps) {
            targetFps = f;
            SetTargetFPS(targetFps); // raylib treats 0 as "no wait" (uncapped)
        }
        if (!fpsBoxEditing) fpsBoxText = std::to_string(targetFps);
    };
    auto commitFpsBox = [&]() {
        // An empty box reverts to the current value (so a stray click-away never
        // jumps to uncapped); to actually set 0 the user types the digit '0'.
        if (!fpsBoxText.empty()) setFps(std::stoi(fpsBoxText));
        fpsBoxText = std::to_string(targetFps);
    };

    while (!WindowShouldClose()) {
        // Sample the wall clock once per frame so the snapshot arrival time and the
        // production-rate estimate share a single consistent "now".
        const double loopTime = GetTime();

        // Track the live window size so the HUD/panel and 3D aspect follow resizes;
        // F11 toggles borderless fullscreen.
        if (IsKeyPressed(KEY_F11)) ToggleBorderlessWindowed();
        screenW = GetScreenWidth();
        screenH = GetScreenHeight();

        // Display/FPS control panel (top-right). Computed up front because the
        // pointer being over it (or actively dragging the slider) must suppress
        // camera orbit/zoom and the click-to-follow pick for this whole frame.
        const int panelW = 252, panelH = 92;
        const Rectangle panelRect{ static_cast<float>(screenW - panelW - 12),
                                   12.0f, static_cast<float>(panelW),
                                   static_cast<float>(panelH) };
        const Vector2 mousePos = GetMousePosition();
        const bool overPanel = CheckCollisionPointRec(mousePos, panelRect);
        const bool uiActive = overPanel || fpsSliderDrag;

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

            // Update trails and spawn detonation FX from this frame, collecting
            // the ids still live this frame so we can prune orphaned trails below.
            std::unordered_set<std::uint32_t> present;
            present.reserve(snap.records.size());
            const double now = loopTime;

            // Push this snapshot (producer time base) into the interpolation
            // history, caching only live tracks (destroyed ones burst as
            // detonations). A few frames of history are enough to always bracket
            // the delayed playback time.
            const double frameId = static_cast<double>(snap.header.frameId);
            // Smoothed real-time production rate (frames advanced per second of
            // local time), outlier-guarded. A scalar low-pass, not a feedback loop
            // into the motion, so it cannot ring.
            if (haveArrival && !hist.empty()) {
                const double dF = frameId - hist.back().frame;
                const double dL = now - newestArrival;
                if (dL > 1e-4 && dF > 0.0) {
                    const double instRate = dF / dL;
                    if (instRate > 10.0 && instRate < 1000.0)
                        rateEma += (instRate - rateEma) * 0.05;
                }
            }
            Snap s;
            s.frame = frameId;
            s.pos.reserve(snap.records.size());
            for (const auto& rr : snap.records)
                if (!(rr.flags & FLAG_DESTROYED))
                    s.pos[rr.entityId] = recPos(rr);
            hist.push_back(std::move(s));
            while (hist.size() > 8) hist.pop_front();
            newestArrival = now;
            haveArrival = true;
            for (const auto& r : snap.records) {
                if (r.flags & FLAG_DESTROYED) {
                    detonations.push_back(Detonation{recPos(r), 0.0f});
                    trails.erase(r.entityId);
                    trailSeen.erase(r.entityId);
                    if (r.flags & FLAG_ASSET_HIT) ++assetLosses;
                    continue;
                }
                present.insert(r.entityId);
                auto& tr = trails[r.entityId];
                tr.push_back(TrailPt{recPos(r), isBooster(r)});
                if (tr.size() > 60) tr.pop_front();
                trailSeen[r.entityId] = now;
            }

            // Prune trails for any entity that is no longer in the frame. Relying
            // on catching a track's one-frame FLAG_DESTROYED record leaks its trail
            // whenever that frame is dropped -- PollLatestFrame only ever returns
            // the newest snapshot, and under load (e.g. two rounds detonating on the
            // same frame) the destroy frame is easily skipped, stranding the plume
            // forever. An entity absent from the latest frame is gone, so drop it.
            for (auto it = trails.begin(); it != trails.end();) {
                if (present.count(it->first) == 0) {
                    trailSeen.erase(it->first);
                    it = trails.erase(it);
                } else {
                    ++it;
                }
            }
        }

        // Hard "forever" guard, independent of fresh frames: drop any trail not
        // refreshed within a short wall-clock window. Catches the case the
        // per-frame prune cannot -- the producer stalling or the run ending, which
        // otherwise freezes every in-flight plume on screen indefinitely.
        {
            constexpr double kTrailTtl = 1.5; // seconds without an update -> drop
            const double now = GetTime();
            for (auto it = trails.begin(); it != trails.end();) {
                const auto s = trailSeen.find(it->first);
                if (s == trailSeen.end() || now - s->second > kTrailTtl) {
                    if (s != trailSeen.end()) trailSeen.erase(s);
                    it = trails.erase(it);
                } else {
                    ++it;
                }
            }
        }

        // Advance the playback position in FRAME-INDEX space and pick this frame's
        // interpolation bracket. playF advances by real frame time times the
        // smoothed production rate, so on-screen motion is uniform regardless of
        // render rate; because the axis is frame id (a fixed motion step per frame)
        // it is immune to the producer's timestamp jitter. A GENTLE first-order
        // pull keeps playF a fixed delay behind the newest frame to cancel slow
        // drift; being first order it settles without overshoot, so it cannot ring
        // (unlike a hard re-anchor, which snaps every frame the rate estimate is
        // slightly off). Clamping to the buffer ends holds the newest sample on a
        // stall rather than extrapolating.
        loSnap = hiSnap = nullptr;
        if (hist.size() >= 2 && haveArrival) {
            const std::size_t n = hist.size();
            const double target = hist.back().frame - 2.5; // frames behind newest
            if (!havePlay) {
                playF = target;
                havePlay = true;
            } else {
                // Carry motion at the smoothed production rate, then apply a gentle
                // first-order drift trim with a FRAME-RATE-INDEPENDENT gain (dt over
                // a fixed time constant): a per-frame constant would correct far
                // harder at high render rates and re-inject the staircase ripple.
                const double dt = GetFrameTime();
                playF += dt * rateEma;
                const double tau = 0.2; // seconds; well above the 60 Hz frame step
                playF += (target - playF) * std::min(1.0, dt / tau);
            }
            if (playF < hist.front().frame) playF = hist.front().frame;
            if (playF > hist.back().frame)  playF = hist.back().frame;
            for (std::size_t i = 0; i + 1 < n; ++i) {
                if (playF >= hist[i].frame && playF <= hist[i + 1].frame) {
                    const double span =
                        std::max(1e-6, hist[i + 1].frame - hist[i].frame);
                    loSnap = &hist[i];
                    hiSnap = &hist[i + 1];
                    interpAlpha = static_cast<float>((playF - hist[i].frame) / span);
                    break;
                }
            }
            if (loSnap == nullptr) { loSnap = hiSnap = &hist.back(); interpAlpha = 1.0f; }
        }

        // --- Trackable set (hostiles then interceptors) for Tab-cycling -------
        std::vector<std::uint32_t> trackables;
        for (const auto& r : snap.records)
            if (isHostile(r) && !(r.flags & FLAG_DESTROYED)) trackables.push_back(r.entityId);
        for (const auto& r : snap.records)
            if (isInterceptor(r) && r.targetId != kNoTargetId) trackables.push_back(r.entityId);

        // Left-click to follow the missile under the cursor; a press that drags is
        // an orbit, not a pick, so only release-without-drag selects. Clicking a
        // missile zooms the camera in on it; empty space leaves the view alone.
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            pressPos = GetMousePosition();
            pressDrag = 0.0f;
        }
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
            pressDrag = std::max(pressDrag,
                                 Vector2Distance(GetMousePosition(), pressPos));
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && pressDrag < 6.0f &&
            !uiActive) {
            const int hit = pickNearestMissile(pressPos, 32.0f);
            if (hit >= 0) {
                followId = static_cast<long long>(snap.records[hit].entityId);
                cam.distance = 3.0f; // zoom in on the selected missile
            }
        }

        if (IsKeyPressed(KEY_TAB)) {
            if (trackables.empty()) followId = -1;
            else {
                int cur = -1;
                for (int i = 0; i < static_cast<int>(trackables.size()); ++i)
                    if (static_cast<long long>(trackables[i]) == followId) { cur = i; break; }
                followId = static_cast<long long>(
                    trackables[(cur + 1) % static_cast<int>(trackables.size())]);
            }
        }
        if (IsKeyPressed(KEY_C) || IsKeyPressed(KEY_ESCAPE)) followId = -1; // free camera
        if (IsKeyPressed(KEY_F)) frustumCull = !frustumCull;
        if (IsKeyPressed(KEY_P)) depthPrepass = !depthPrepass;
        if (IsKeyPressed(KEY_L)) lod = !lod;
        if (IsKeyPressed(KEY_B)) staticInst = !staticInst;

        // Resolve the followed entity; if it has left the scene, drop back to free
        // camera. While following, smoothly chase it (orbit then rotates about it).
        const TelemetryRecord* followed = nullptr;
        if (followId >= 0) {
            for (const auto& r : snap.records)
                if (static_cast<long long>(r.entityId) == followId &&
                    !(r.flags & FLAG_DESTROYED)) { followed = &r; break; }
            if (!followed) followId = -1;
        }
        const bool tracking = followed != nullptr;
        if (tracking) {
            const Vector3 mp = livePos(*followed);
            if (followId != lockedFollowId) { // new target: re-acquire smoothly
                followLocked = false;
                lockedFollowId = followId;
            }
            if (followLocked) {
                cam.target = mp; // rigid: camera is a child of the track's position
            } else {
                cam.target = Vector3Lerp(cam.target, mp, 0.2f); // smooth acquire
                if (Vector3Distance(cam.target, mp) < 0.05f) followLocked = true;
            }
        }
        updateCamera(cam, tracking, uiActive);

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

            // East River: only the real river surface is painted (rasterised
            // from the OSM coastline), so the land is the deck and streets never
            // flood. Then the skyline as flyweight instances -- two shared meshes
            // drawn once per (kind, prim) group with a class color; backface
            // culling stays on (the primitives are wound outward).
            rlDisableBackfaceCulling();
            DrawMesh(waterMesh, waterMat, MatrixIdentity());
            rlEnableBackfaceCulling();

            // The static fast path (default) draws the whole city from persistent
            // GPU buffers with no per-frame upload or CPU cull. The dynamic path
            // below is used only when a mode needs per-frame visibility rebuilds:
            // frustum culling toggled via [B], or the discrete LOD system via [L].
            const bool useStatic = staticInst && !lod;

            // Stage 3: frustum-cull each group into its `visible` list, then draw
            // instanced. An optional depth pre-pass (color writes off) lays down
            // the skyline's depth first so the color pass -- which reruns the
            // shaded fragment stage -- early-Z rejects the heavy overdraw of a
            // dense city viewed edge-on (raylib's default glDepthFunc is LEQUAL,
            // so the equal-depth second pass draws correctly).
            if (!useStatic) {
            const Frustum fr = extractFrustum();
            const Camera3D cam3 = cam.toCamera();
            const Vector3 camPos = cam3.position;
            // Switch tiers by distance with a DITHERED fade zone of half-width
            // `fade` km around each boundary: within the band a cell shows the finer
            // or coarser tier based on a hash of a fixed ~R m ground patch vs the
            // fade fraction, so the transition dissolves in a randomised zone rather
            // than a hard ring. Both sides of a boundary use the SAME patch hash, so
            // they are complementary (fade=0 collapses back to a hard switch).
            const float fade = std::max(fadeKm, 0.0f);
            const float kPatch = 70.0f; // randomised-zone patch size (m)
            auto d2 = [&](const Vector3& c) {
                const float dx = c.x - camPos.x, dy = c.y - camPos.y,
                            dz = c.z - camPos.z;
                return dx * dx + dy * dy + dz * dz;
            };
            auto patchHash = [&](const Vector3& c) {
                const long long x = static_cast<long long>(
                    std::floor((c.x * kScale - lodMinE) / kPatch));
                const long long y = static_cast<long long>(
                    std::floor((c.z * kScale - lodMinN) / kPatch));
                unsigned long long h = static_cast<unsigned long long>(x) * 0x9E3779B97F4A7C15ull ^
                                       static_cast<unsigned long long>(y) * 0xC2B2AE3D27D4EB4Full;
                h ^= h >> 29; h *= 0xBF58476D1CE4E5B9ull; h ^= h >> 32;
                return static_cast<float>((h >> 40) & 0xFFFFFF) / 16777216.0f;
            };
            // frac at a boundary distance t: 0 at the near edge, 1 at the far edge.
            auto frac = [&](float dd, float t) {
                return (fade <= 0.0f) ? (dd >= t ? 1.0f : 0.0f)
                                      : Clamp((dd - (t - fade)) / (2.0f * fade), 0.0f, 1.0f);
            };
            cityDrawn = 0;
            cityLod = 0;

            // --- Full-detail tier: fades out into massing tier 0 at TT[0] -------
            {
                const float t0 = tierDist[0], lo0 = (t0 - fade) * (t0 - fade),
                            hi0 = (t0 + fade) * (t0 + fade);
                for (int k = 0; k < 5; ++k) {
                    for (int p = 0; p < 2; ++p) {
                        InstanceGroup& g = groups[k][p];
                        g.visible.clear();
                        for (std::size_t i = 0; i < g.xform.size(); ++i) {
                            const Vector3& c = g.center[i];
                            if (lod) {
                                const float ds = d2(c);
                                if (ds >= hi0) continue;                // fully massed
                                if (frustumCull && !sphereVisible(fr, c, g.radius[i])) continue;
                                if (ds > lo0 &&                         // in fade zone
                                    patchHash(c) < frac(std::sqrt(ds), t0)) continue;
                            } else if (frustumCull && !sphereVisible(fr, c, g.radius[i])) {
                                continue;
                            }
                            g.visible.push_back(g.xform[i]);
                        }
                        cityDrawn += static_cast<int>(g.visible.size());
                    }
                }
            }

            // --- Massing tiers: fade IN at TT[m], fade OUT at TT[m+1] -----------
            for (int m = 0; m < nTiers; ++m) {
                const bool  hasOuter = (m + 1 < nTiers);
                const float ti = tierDist[m];
                const float loI = (ti - fade) * (ti - fade), hiI = (ti + fade) * (ti + fade);
                const float to = hasOuter ? tierDist[m + 1] : 0.0f;
                const float loO = hasOuter ? (to - fade) * (to - fade) : 0.0f;
                const float hiO = hasOuter ? (to + fade) * (to + fade) : 3.4e38f;
                for (int k = 0; k < 5; ++k) {
                    InstanceGroup& g = massing[m][k];
                    g.visible.clear();
                    if (lod) {
                        for (std::size_t i = 0; i < g.xform.size(); ++i) {
                            const Vector3& c = g.center[i];
                            const float ds = d2(c);
                            if (ds < loI) continue;                     // finer still owns
                            if (hasOuter && ds >= hiO) continue;        // coarser took over
                            if (frustumCull && !sphereVisible(fr, c, g.radius[i])) continue;
                            const float h = patchHash(c);
                            if (ds < hiI &&                             // inner fade zone
                                h >= frac(std::sqrt(ds), ti)) continue; // finer still owns
                            if (hasOuter && ds > loO &&                 // outer fade zone
                                h < frac(std::sqrt(ds), to)) continue;  // coarser took over
                            g.visible.push_back(g.xform[i]);
                        }
                    }
                    cityLod += static_cast<int>(g.visible.size());
                }
            }

            auto drawGroups = [&]() {
                for (int k = 0; k < 5; ++k) {
                    cityMat.maps[MATERIAL_MAP_DIFFUSE].color =
                        kindColor(static_cast<sim::MapKind>(k));
                    for (int p = 0; p < 2; ++p) {
                        InstanceGroup& g = groups[k][p];
                        if (!g.visible.empty())
                            DrawMeshInstanced(baseMesh[p], cityMat, g.visible.data(),
                                              static_cast<int>(g.visible.size()));
                    }
                    // Massing tiers share the box mesh and the class color.
                    for (int m = 0; m < nTiers; ++m) {
                        InstanceGroup& lg = massing[m][k];
                        if (!lg.visible.empty())
                            DrawMeshInstanced(baseMesh[0], cityMat, lg.visible.data(),
                                              static_cast<int>(lg.visible.size()));
                    }
                }
            };
            if (depthPrepass) {
                rlColorMask(false, false, false, false); // depth-only pass
                drawGroups();
                rlColorMask(true, true, true, true);
            }
            drawGroups(); // shaded color pass (early-Z against the pre-pass depth)
            } else {
                // --- Static fast path: draw persistent instance buffers ----------
                // One bound VAO + one glDrawElementsInstanced per group, no upload
                // and no CPU cull -- the transforms already live on the GPU. mvp is
                // model-view-projection with an identity model (the per-instance
                // transform is the model), read from rlgl's current matrices.
                cityDrawn = cityTotal;
                cityLod = 0;
                const Matrix mvp =
                    MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());
                rlEnableShader(instShader.id);
                rlSetUniformMatrix(instShader.locs[SHADER_LOC_MATRIX_MVP], mvp);
                auto drawStatic = [&]() {
                    for (int k = 0; k < 5; ++k) {
                        const Color col = kindColor(static_cast<sim::MapKind>(k));
                        const float c4[4] = {col.r / 255.0f, col.g / 255.0f,
                                             col.b / 255.0f, col.a / 255.0f};
                        rlSetUniform(instShader.locs[SHADER_LOC_COLOR_DIFFUSE], c4,
                                     RL_SHADER_UNIFORM_VEC4, 1);
                        for (int p = 0; p < 2; ++p) {
                            InstanceGroup& g = groups[k][p];
                            if (g.vaoId == 0) continue;
                            rlEnableVertexArray(g.vaoId);
                            rlDrawVertexArrayElementsInstanced(
                                0, baseMesh[p].triangleCount * 3, nullptr,
                                static_cast<int>(g.xform.size()));
                            rlDisableVertexArray();
                        }
                    }
                };
                if (depthPrepass) {
                    rlColorMask(false, false, false, false);
                    drawStatic();
                    rlColorMask(true, true, true, true);
                }
                drawStatic();
                rlDisableShader();
            }

            // Battery / defended-footprint ring around the UN Secretariat (the
            // protected radius, ~6 km, matching the engine default).
            const Vector3 asset = worldToRay(50000.0f, 50000.0f, 0.0f);
            DrawCircle3D(asset, 6.0f, Vector3{1, 0, 0}, 90.0f,
                         Color{0, 120, 160, 150});

            int activeHostiles = 0;

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

            // Entities. Missiles keep real proportions but are drawn enlarged so
            // they read against the magnified skyline (same spirit as kCityScale);
            // a short heading dart is kept as a sensor annotation.
            for (const auto& r : snap.records) {
                if (r.flags & FLAG_DESTROYED) continue;
                const Vector3 p = livePos(r);
                const Vector3 v = recVel(r);
                const bool boosting = isBooster(r);

                if (isHostile(r)) {
                    ++activeHostiles;
                    const Color tc = threatColor(r.threatLevel);
                    // Ballistic missile silhouette (enlarged for legibility).
                    drawMissile(p, v, 300.0f, 14.0f, tc,
                                Color{40, 40, 44, 255}, boosting,
                                Color{255, 150, 40, 255});
                    drawHeading(p, v, 0.5f, ColorAlpha(RED, 0.7f));
                } else if (isInterceptor(r)) {
                    const bool engaged = r.targetId != kNoTargetId;
                    const bool neutral = isNeutral(r);
                    // Friendly = bright green/skyblue; allied-neutral = dark green.
                    const Color body =
                        neutral ? (engaged ? Color{30, 130, 50, 255}
                                           : Color{22, 95, 38, 255})
                                : (engaged ? GREEN : SKYBLUE);
                    // Slender THAAD-class interceptor (enlarged for legibility).
                    drawMissile(p, v, 210.0f, 9.0f, body,
                                Color{230, 240, 255, 255}, boosting,
                                Color{120, 200, 255, 255});
                    drawHeading(p, v, 0.42f,
                                neutral ? ColorAlpha(Color{40, 150, 60, 255}, 0.7f)
                                        : ColorAlpha(Color{120, 200, 255, 255}, 0.7f));
                    // ProNav LOS line to assigned hostile.
                    if (engaged) {
                        const Color los = neutral ? Color{40, 150, 60, 140}
                                                  : Color{0, 255, 120, 140};
                        for (const auto& t : snap.records) {
                            if (t.entityId == r.targetId &&
                                !(t.flags & FLAG_DESTROYED)) {
                                DrawLine3D(p, livePos(t), los);
                                break;
                            }
                        }
                    }
                } else {
                    DrawSphere(p, 0.02f, Color{90, 100, 110, 255}); // other traffic
                }
            }

            // Detonation FX: a bright flash + expanding fading fireball shell.
            for (const auto& d : detonations) {
                const float radius = 0.15f + d.age * 0.9f;
                const unsigned char a = static_cast<unsigned char>(
                    Clamp(255.0f * (1.0f - d.age / 1.2f), 0.0f, 255.0f));
                DrawSphere(d.pos, radius * 0.45f,
                           ColorAlpha(Color{255, 220, 140, 255}, a / 300.0f));
                DrawSphereWires(d.pos, radius, 10, 10,
                                ColorAlpha(Color{255, 160, 40, 255}, a / 255.0f));
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

        // Size the HUD panel to the widest line (the instances readout) so nothing
        // spills past the dark background.
        char instLine[128];
        std::snprintf(instLine, sizeof(instLine),
                      "instances : %d / %d drawn   [B]buf:%s [F]cull:%s [L]lod:%s",
                      cityDrawn + cityLod, cityTotal, staticInst ? "on" : "off",
                      frustumCull ? "on" : "off", lod ? "on" : "off");
        const float hudTextW = MeasureTextEx(uiFont, instLine, 18.0f, 1.0f).x;
        const int hudW = std::max(360, static_cast<int>(hudTextW) + 32);
        DrawRectangle(0, 0, hudW, 240, Color{0, 0, 0, 160});
        DT("C2 TACTICAL DISPLAY", 16, 12, 20, RAYWHITE);
        char line[128];
        std::snprintf(line, sizeof(line), "source        : %s", consumer->SourceName());
        DT(line, 16, 40, 18, LIGHTGRAY);
        std::snprintf(line, sizeof(line), "active hostiles : %d", activeHostiles);
        DT(line, 16, 62, 18, activeHostiles > 0 ? ORANGE : GREEN);
        std::snprintf(line, sizeof(line), "neutralized     : %d", neutralized);
        DT(line, 16, 84, 18, GREEN);
        std::snprintf(line, sizeof(line), "asset losses    : %d", assetLosses);
        DT(line, 16, 106, 18, assetLosses > 0 ? RED : GRAY);
        std::snprintf(line, sizeof(line), "success rate    : %.0f%%", successRate);
        DT(line, 16, 128, 18, RAYWHITE);
        std::snprintf(line, sizeof(line), "telemetry       : %.1f Hz  (%.0f KB/s)",
                      telemetryFps, throughputKBs);
        DT(line, 16, 150, 18, SKYBLUE);
        std::snprintf(line, sizeof(line), "frame           : %llu",
                      static_cast<unsigned long long>(snap.header.frameId));
        DT(line, 16, 172, 18, GRAY);
        DT(instLine, 16, 194, 18, Color{150, 200, 170, 255});

        DT("[Click] follow  [Tab] cycle  [Esc/C] free-cam  [drag] orbit  [wheel] zoom  [B]uf [F]cull [L]od",
                 16, screenH - 26, 16, Color{160, 170, 180, 255});
        {
            char fps[24];
            std::snprintf(fps, sizeof(fps), "%d FPS", GetFPS());
            const int f = GetFPS();
            DT(fps, 16, screenH - 48, 18, f >= 55 ? GREEN : (f >= 30 ? YELLOW : RED));
        }

        if (tracking) {
            std::snprintf(line, sizeof(line), "FOLLOWING #%lld", followId);
            DT(line, screenW - 240, 112, 20, GREEN);
        }

        // --- Render-FPS control: slider + integer text box (0..1000, 0=uncapped).
        // Both widgets are views of targetFps; setFps() is the only writer, so a
        // drag and a typed value can never disagree.
        {
            const Rectangle track{ panelRect.x + 14, panelRect.y + 54, 150, 6 };
            const Rectangle box{ track.x + track.width + 16, track.y - 10, 58, 26 };
            const float frac = targetFps / 1000.0f;
            const float knobX = track.x + frac * track.width;
            const Rectangle knob{ knobX - 6.0f, track.y - 7.0f, 12.0f, 20.0f };

            // Slider: grab on the knob or anywhere along the track, then follow the
            // pointer (even if it slides off the track) until the button releases.
            const Rectangle grab{ track.x - 6.0f, track.y - 8.0f,
                                  track.width + 12.0f, 22.0f };
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&
                (CheckCollisionPointRec(mousePos, knob) ||
                 CheckCollisionPointRec(mousePos, grab))) {
                fpsSliderDrag = true;
                fpsBoxEditing = false;
            }
            if (fpsSliderDrag) {
                if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                    const float f = Clamp((mousePos.x - track.x) / track.width,
                                          0.0f, 1.0f);
                    setFps(static_cast<int>(std::lround(f * 1000.0f)));
                } else {
                    fpsSliderDrag = false;
                }
            }

            // Text box: click to focus, type digits, Enter/click-away commits.
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                const bool hitBox = CheckCollisionPointRec(mousePos, box);
                if (fpsBoxEditing && !hitBox) commitFpsBox(); // click-away commits
                fpsBoxEditing = hitBox;
                if (hitBox) fpsBoxText.clear(); // start fresh on focus
            }
            if (fpsBoxEditing) {
                for (int key = GetCharPressed(); key > 0; key = GetCharPressed())
                    if (key >= '0' && key <= '9' && fpsBoxText.size() < 4)
                        fpsBoxText.push_back(static_cast<char>(key));
                if (IsKeyPressed(KEY_BACKSPACE) && !fpsBoxText.empty())
                    fpsBoxText.pop_back();
                if (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER)) {
                    commitFpsBox();
                    fpsBoxEditing = false;
                }
            }

            // Draw the panel.
            DrawRectangleRec(panelRect, Color{0, 0, 0, 160});
            DrawRectangleLinesEx(panelRect, 1.0f, Color{40, 60, 90, 255});
            DT("DISPLAY", static_cast<int>(panelRect.x) + 14,
               static_cast<int>(panelRect.y) + 8, 18, RAYWHITE);
            DT(targetFps == 0 ? "render cap : uncapped" : "render cap",
               static_cast<int>(panelRect.x) + 14,
               static_cast<int>(panelRect.y) + 30, 14, LIGHTGRAY);
            DrawRectangleRec(track, Color{60, 70, 90, 255});
            DrawRectangle(static_cast<int>(track.x), static_cast<int>(track.y),
                          static_cast<int>(frac * track.width),
                          static_cast<int>(track.height), Color{0, 150, 190, 255});
            DrawRectangleRec(knob, fpsSliderDrag ? SKYBLUE : RAYWHITE);
            DrawRectangleRec(box, Color{20, 26, 36, 255});
            DrawRectangleLinesEx(box, 1.0f,
                                 fpsBoxEditing ? SKYBLUE : Color{70, 80, 100, 255});
            const std::string shown = fpsBoxEditing ? fpsBoxText + "_" : fpsBoxText;
            DT(shown.c_str(), static_cast<int>(box.x) + 6,
               static_cast<int>(box.y) + 5, 16, RAYWHITE);
        }

        // --- Hover ring: encircle the missile under the cursor and label its
        // allegiance (ENEMY / ALLY / DOMESTIC). One missile at a time, and only
        // while not orbiting (a drag would make the pointer position meaningless).
        if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT) &&
            !IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && !overPanel) {
            const int hov = pickNearestMissile(GetMousePosition(), 45.0f);
            if (hov >= 0) {
                const TelemetryRecord& r = snap.records[hov];
                const Vector2 s = GetWorldToScreen(livePos(r), cam.toCamera());
                const Color col = trackColor(r);
                DrawRing(s, 18.0f, 21.0f, 0.0f, 360.0f, 48, col);
                DrawRing(s, 21.0f, 22.5f, 0.0f, 360.0f, 48, ColorAlpha(BLACK, 0.5f));
                const char* lbl = trackLabel(r);
                const float lw = MeasureTextEx(uiFont, lbl, 18.0f, 1.0f).x;
                DrawRectangle(static_cast<int>(s.x) + 26, static_cast<int>(s.y) - 11,
                              static_cast<int>(lw) + 8, 22, ColorAlpha(BLACK, 0.55f));
                DT(lbl, static_cast<int>(s.x) + 30, static_cast<int>(s.y) - 9, 18, col);
            }
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

    for (int k = 0; k < 5; ++k)
        for (int p = 0; p < 2; ++p) {
            if (groups[k][p].instVbo) rlUnloadVertexBuffer(groups[k][p].instVbo);
            if (groups[k][p].vaoId) rlUnloadVertexArray(groups[k][p].vaoId);
        }
    UnloadMesh(baseMesh[0]);
    UnloadMesh(baseMesh[1]);
    UnloadMesh(waterMesh);
    UnloadShader(instShader);
    if (haveFont) UnloadFont(uiFont);
    CloseWindow();
    return 0;
}
