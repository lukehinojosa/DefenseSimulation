// In-browser C2 viewer (WebAssembly + raylib on WebGL2).
//
// Unlike the desktop c2_visualizer, which is a read-only client of a separate
// simulation process, this build runs the engagement scenario itself, in the
// page: EngagementScenario is stepped on a fixed 60 Hz tick and rendered with
// interpolation, so motion stays smooth at any display refresh rate. The page
// (web/shell.html) owns the controls (start/pause/restart, speed, camera
// gestures) and the HUD, and talks to this module through the web_* exports.
//
// The 1:1 New York skyline (~485k buildings) is the expensive part, so it gets
// its own renderer (CityRenderer below):
//   * one shared unit box mesh, instanced, with a compact 32-byte per-instance
//     record (centre, heading, extents, class) uploaded to the GPU once;
//   * instances grouped into 1 km tiles, so each tile is a single instanced
//     draw that is frustum-culled as a whole;
//   * screen-space-size LOD: within a tile, instances are sorted largest first,
//     so a tile draws only the *prefix* of buildings that would cover at least
//     pxMin pixels from the camera (a binary search per tile per frame). The
//     vertex shader grows the marginal ones in over the next pxMin pixels, so
//     there is no popping;
//   * an adaptive quality governor raises pxMin while frames run long (phones)
//     and lowers it back as headroom returns.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <GLES3/gl3.h>
#include <emscripten/emscripten.h>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "sim/CityLayout.hpp"
#include "sim/EngagementScenario.hpp"
#include "sim/MapData.hpp"
#include "sim/SimConfig.hpp"
#include "sim/Telemetry.hpp"

namespace {

using namespace sim::telemetry;

// --- World scale (matches the desktop visualizer) ---------------------------
// 1 raylib unit = 1 km; the 100 x 100 km airspace is recentered on the origin,
// and the sim's Z-up frame maps to raylib's Y-up.
constexpr float kScale = 1000.0f;
constexpr float kHalfExtent = 50000.0f;
constexpr double kTickSeconds = 1.0 / 60.0; // one scenario tick of playback time

Vector3 worldToRay(float x, float y, float z) {
    return Vector3{(x - kHalfExtent) / kScale, z / kScale, (y - kHalfExtent) / kScale};
}
Vector3 recPos(const TelemetryRecord& r) { return worldToRay(r.posX, r.posY, r.posZ); }
Vector3 recVel(const TelemetryRecord& r) {
    return Vector3{r.velX / kScale, r.velZ / kScale, r.velY / kScale};
}

bool isHostile(const TelemetryRecord& r) {
    return r.entityType == static_cast<std::uint8_t>(sim::EntityType::Hostile);
}
bool isNeutral(const TelemetryRecord& r) {
    return r.entityType == static_cast<std::uint8_t>(sim::EntityType::Neutral);
}
bool isInterceptor(const TelemetryRecord& r) {
    return r.entityType == static_cast<std::uint8_t>(sim::EntityType::Friendly) ||
           isNeutral(r);
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

// A track's representative color (matches its silhouette) and its allegiance
// label for the hover ring: our battery rounds are DOMESTIC, allied airborne
// interceptors ALLY, inbound threats ENEMY (same as the desktop visualizer).
Color trackColor(const TelemetryRecord& r) {
    if (isHostile(r)) return threatColor(r.threatLevel);
    const bool engaged = r.targetId != kNoTargetId;
    if (isNeutral(r)) return engaged ? Color{30, 130, 50, 255} : Color{22, 95, 38, 255};
    return engaged ? GREEN : SKYBLUE;
}
const char* trackLabel(const TelemetryRecord& r) {
    if (isHostile(r)) return "ENEMY";
    if (isNeutral(r)) return "ALLY";
    return "DOMESTIC";
}

// Per-building-class colors, indexed by sim::MapKind (same palette as desktop).
constexpr Color kKindColor[5] = {
    { 42,  96, 145, 255}, // UnPlaza
    { 82,  94, 114, 255}, // Block (Midtown)
    {100, 145, 185, 255}, // UnComplex
    {110, 128, 160, 255}, // Skyscraper
    { 70, 175, 185, 255}, // Secretariat
};
constexpr Color kDeckColor{16, 22, 30, 255};
constexpr Color kSkyColor{8, 10, 16, 255};

// ============================================================================
// Compact web map ("UNMW", baked by tools/gen_web_map.py)
// ============================================================================
struct WebMap {
    sim::MapData map;                 // instances in file order (tile-major)
    float tileSize{1000.0f}, originE{0.0f}, originN{0.0f};
    int tilesX{0}, tilesY{0};
    std::vector<std::uint32_t> tileCount;
};

bool loadWebMap(const char* path, WebMap& out) {
    int compSize = 0;
    unsigned char* comp = LoadFileData(path, &compSize);
    if (comp == nullptr) return false;
    int size = 0;
    unsigned char* raw = DecompressData(comp, compSize, &size);
    UnloadFileData(comp);
    if (raw == nullptr) return false;

    std::size_t at = 0;
    bool ok = true;
    auto take = [&](void* dst, std::size_t n) {
        if (!ok || at + n > static_cast<std::size_t>(size)) { ok = false; return; }
        std::memcpy(dst, raw + at, n);
        at += n;
    };
    char magic[4];
    std::uint32_t version = 0, tx = 0, ty = 0, nInst = 0, nWater = 0;
    take(magic, 4);
    take(&version, 4);
    take(&out.tileSize, 4);
    take(&out.originE, 4);
    take(&out.originN, 4);
    take(&tx, 4);
    take(&ty, 4);
    take(&nInst, 4);
    take(&nWater, 4);
    if (!ok || std::memcmp(magic, "UNMW", 4) != 0 || version != 1u) {
        MemFree(raw);
        return false;
    }
    out.tilesX = static_cast<int>(tx);
    out.tilesY = static_cast<int>(ty);
    out.tileCount.resize(static_cast<std::size_t>(tx) * ty);
    take(out.tileCount.data(), out.tileCount.size() * 4);

    // Column-packed fields (see the tool for the layout and quantization).
    std::vector<std::uint16_t> qx(nInst), qy(nInst), qa(nInst), qhx(nInst),
        qhy(nInst), qh(nInst);
    std::vector<std::uint8_t> kind(nInst), prim(nInst);
    for (auto* col : {&qx, &qy, &qa, &qhx, &qhy, &qh}) take(col->data(), nInst * 2u);
    take(kind.data(), nInst);
    take(prim.data(), nInst);
    std::vector<std::int16_t> water(static_cast<std::size_t>(nWater) * 2);
    take(water.data(), water.size() * 2);
    MemFree(raw);
    if (!ok) return false;

    const float posStep = out.tileSize / 65535.0f;
    out.map.instances.clear();
    out.map.instances.reserve(nInst);
    std::size_t i = 0;
    for (int t = 0; t < out.tilesX * out.tilesY; ++t) {
        const float e0 = out.originE + static_cast<float>(t % out.tilesX) * out.tileSize;
        const float n0 = out.originN + static_cast<float>(t / out.tilesX) * out.tileSize;
        for (std::uint32_t k = 0; k < out.tileCount[t]; ++k, ++i) {
            sim::MapInstance m{};
            m.cx = e0 + qx[i] * posStep;
            m.cy = n0 + qy[i] * posStep;
            m.angle = qa[i] * (PI / 65536.0f);
            m.hx = qhx[i] * 0.01f;
            m.hy = qhy[i] * 0.01f;
            m.height = qh[i] * 0.01f;
            m.kind = static_cast<sim::MapKind>(kind[i] < 5 ? kind[i] : 1);
            m.prim = static_cast<sim::MapPrim>(prim[i] & 1);
            out.map.instances.push_back(m);
        }
    }
    out.map.waterTris.clear();
    out.map.waterTris.reserve(nWater);
    for (std::uint32_t v = 0; v < nWater; ++v) {
        out.map.waterTris.push_back(
            sim::Vector3{water[2 * v] * 0.5, water[2 * v + 1] * 0.5, 0.0});
    }
    return i == nInst;
}

// ============================================================================
// Frustum (Gribb-Hartmann, from raylib's current modelview * projection)
// ============================================================================
struct Frustum { Vector4 plane[6]; };

Frustum extractFrustum() {
    const Matrix m = MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());
    Frustum f;
    f.plane[0] = {m.m3 - m.m0, m.m7 - m.m4, m.m11 - m.m8,  m.m15 - m.m12};
    f.plane[1] = {m.m3 + m.m0, m.m7 + m.m4, m.m11 + m.m8,  m.m15 + m.m12};
    f.plane[2] = {m.m3 + m.m1, m.m7 + m.m5, m.m11 + m.m9,  m.m15 + m.m13};
    f.plane[3] = {m.m3 - m.m1, m.m7 - m.m5, m.m11 - m.m9,  m.m15 - m.m13};
    f.plane[4] = {m.m3 + m.m2, m.m7 + m.m6, m.m11 + m.m10, m.m15 + m.m14};
    f.plane[5] = {m.m3 - m.m2, m.m7 - m.m6, m.m11 - m.m10, m.m15 - m.m14};
    for (Vector4& p : f.plane) {
        const float len = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        if (len > 0.0f) { p.x /= len; p.y /= len; p.z /= len; p.w /= len; }
    }
    return f;
}

// An AABB is visible unless its most-positive corner lies behind some plane.
bool boxVisible(const Frustum& f, Vector3 lo, Vector3 hi) {
    for (const Vector4& p : f.plane) {
        const float x = p.x >= 0.0f ? hi.x : lo.x;
        const float y = p.y >= 0.0f ? hi.y : lo.y;
        const float z = p.z >= 0.0f ? hi.z : lo.z;
        if (p.x * x + p.y * y + p.z * z + p.w < 0.0f) return false;
    }
    return true;
}

// ============================================================================
// City renderer
// ============================================================================
const char* kCityVS = R"(#version 300 es
precision highp float;
layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexNormal;
layout(location = 2) in vec4 instA;   // centre x, centre z (km), cos, sin of heading
layout(location = 3) in vec4 instB;   // size x, height, size z (km), class (+8 = massing)
uniform mat4 mvp;
uniform vec3 camPos;
uniform float lodK;                   // pixels per (km of size / km of distance)
uniform float pxMin;                  // LOD threshold, pixels
uniform vec4 palette[5];
out vec3 vNormal;
out vec3 vColor;
void main() {
    vec3 s = instB.xyz;
    vec3 centre = vec3(instA.x, 0.5 * s.y, instA.y);
    float d = max(distance(camPos, centre), 1e-3);
    // Screen-space LOD: buildings near the cut-off grow in from nothing over one
    // more pxMin of apparent size instead of popping. Massing blocks are the
    // far-field stand-in for exactly those buildings, so they never fade.
    bool massing = instB.w > 7.5;
    float px = max(s.y, max(s.x, s.z)) * lodK / d;
    float grow = massing ? 1.0 : smoothstep(pxMin, 2.0 * pxMin, px);
    vec3 p = vertexPosition * s * grow;
    float c = instA.z, sn = instA.w;
    vec3 w = vec3(instA.x + c * p.x - sn * p.z, p.y, instA.y + sn * p.x + c * p.z);
    vec3 n = vertexNormal / max(s, vec3(1e-4));
    vNormal = vec3(c * n.x - sn * n.z, n.y, sn * n.x + c * n.z);
    vColor = palette[int(mod(instB.w + 0.5, 8.0))].rgb;
    gl_Position = mvp * vec4(w, 1.0);
}
)";

const char* kCityFS = R"(#version 300 es
precision mediump float;
in vec3 vNormal;
in vec3 vColor;
out vec4 finalColor;
void main() {
    vec3 L = normalize(vec3(0.35, 0.85, 0.30));
    float d = 0.55 + 0.45 * abs(dot(normalize(vNormal), L));
    finalColor = vec4(vColor * d, 1.0);
}
)";

GLuint compileShader(GLenum type, const char* src) {
    const GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(s, sizeof(log), nullptr, log);
        std::fprintf(stderr, "city shader: %s\n", log);
    }
    return s;
}

struct MeshData {
    std::vector<float> vtx;            // interleaved position(3) + normal(3)
    std::vector<std::uint16_t> idx;
};

// Unit box: x,z in [-0.5, 0.5], y in [0, 1]. No floor face: the ground deck
// always hides it, so dropping it saves a sixth of the city's vertex work.
MeshData makeBox() {
    MeshData m;
    auto quad = [&](Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector3 n) {
        const Vector3 fn = Vector3CrossProduct(Vector3Subtract(b, a), Vector3Subtract(c, a));
        if (Vector3DotProduct(fn, n) < 0.0f) std::swap(b, d); // CCW-outward winding
        const auto base = static_cast<std::uint16_t>(m.vtx.size() / 6);
        for (const Vector3& p : {a, b, c, d})
            m.vtx.insert(m.vtx.end(), {p.x, p.y, p.z, n.x, n.y, n.z});
        for (int e : {0, 1, 2, 0, 2, 3}) m.idx.push_back(static_cast<std::uint16_t>(base + e));
    };
    const float lo = -0.5f, hi = 0.5f;
    const Vector3 A{lo, 0, lo}, B{hi, 0, lo}, C{hi, 0, hi}, D{lo, 0, hi};
    const Vector3 E{lo, 1, lo}, F{hi, 1, lo}, G{hi, 1, hi}, H{lo, 1, hi};
    quad(E, F, G, H, {0, 1, 0});
    quad(D, C, G, H, {0, 0, 1});
    quad(A, B, F, E, {0, 0, -1});
    quad(B, C, G, F, {1, 0, 0});
    quad(A, D, H, E, {-1, 0, 0});
    return m;
}

// Unit cylinder: radius 0.5, y in [0, 1], smooth sides plus a flat cap.
MeshData makeCylinder(int sides) {
    MeshData m;
    auto add = [&](float x, float y, float z, float nx, float ny, float nz) {
        m.vtx.insert(m.vtx.end(), {x, y, z, nx, ny, nz});
        return static_cast<std::uint16_t>(m.vtx.size() / 6 - 1);
    };
    std::vector<std::uint16_t> b(sides), t(sides), cap(sides);
    for (int i = 0; i < sides; ++i) {
        const float a = 2.0f * PI * i / sides, c = cosf(a), s = sinf(a);
        b[i] = add(0.5f * c, 0.0f, 0.5f * s, c, 0.0f, s);
        t[i] = add(0.5f * c, 1.0f, 0.5f * s, c, 0.0f, s);
    }
    for (int i = 0; i < sides; ++i) {
        const int j = (i + 1) % sides;
        m.idx.insert(m.idx.end(), {b[i], t[j], b[j], b[i], t[i], t[j]});
    }
    const std::uint16_t mid = add(0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f);
    for (int i = 0; i < sides; ++i) {
        const float a = 2.0f * PI * i / sides;
        cap[i] = add(0.5f * cosf(a), 1.0f, 0.5f * sinf(a), 0.0f, 1.0f, 0.0f);
    }
    for (int i = 0; i < sides; ++i) m.idx.insert(m.idx.end(), {mid, cap[(i + 1) % sides], cap[i]});
    return m;
}

class CityRenderer {
public:
    struct Stats { int drawn{0}; int massing{0}; int total{0}; int draws{0}; };

    void init(const WebMap& wm) {
        // Shader (attribute slots fixed in the source via layout qualifiers).
        prog_ = glCreateProgram();
        const GLuint vs = compileShader(GL_VERTEX_SHADER, kCityVS);
        const GLuint fs = compileShader(GL_FRAGMENT_SHADER, kCityFS);
        glAttachShader(prog_, vs);
        glAttachShader(prog_, fs);
        glLinkProgram(prog_);
        glDeleteShader(vs);
        glDeleteShader(fs);
        uMvp_ = glGetUniformLocation(prog_, "mvp");
        uCam_ = glGetUniformLocation(prog_, "camPos");
        uLodK_ = glGetUniformLocation(prog_, "lodK");
        uPxMin_ = glGetUniformLocation(prog_, "pxMin");
        uPalette_ = glGetUniformLocation(prog_, "palette");

        // Per-instance records: boxes tile by tile (each tile's run sorted by
        // size, largest first, for prefix LOD), then every cylinder in one run.
        std::vector<float> inst;
        inst.reserve(wm.map.instances.size() * 8);
        std::vector<sim::MapInstance> cylinders;
        auto pushTo = [](std::vector<float>& out, const sim::MapInstance& m) {
            out.insert(out.end(),
                        {m.cx / kScale, m.cy / kScale, cosf(m.angle), sinf(m.angle),
                         2.0f * m.hx / kScale, m.height / kScale, 2.0f * m.hy / kScale,
                         static_cast<float>(m.kind)});
        };
        auto push = [&](const sim::MapInstance& m) { pushTo(inst, m); };
        auto sizeOf = [](const sim::MapInstance& m) {
            return std::max(m.height, 2.0f * std::max(m.hx, m.hy)) / kScale;
        };
        std::size_t src = 0;
        std::vector<sim::MapInstance> run;
        // Far-field segments, one per massing level, appended after the rest.
        // Each tile's segment is its blocks followed by the buildings at least
        // a cell wide (the towers that still read individually at that range),
        // and tiles are laid out in tile order, so neighbouring far tiles form
        // one contiguous range that draws in a single call.
        std::vector<float> massing[2];
        for (std::size_t ti = 0; ti < wm.tileCount.size(); ++ti) {
            const std::uint32_t count = wm.tileCount[ti];
            run.clear();
            for (std::uint32_t k = 0; k < count; ++k, ++src) {
                const sim::MapInstance& m = wm.map.instances[src];
                if (m.prim == sim::MapPrim::Cylinder) cylinders.push_back(m);
                else run.push_back(m);
            }
            if (run.empty()) continue;
            std::stable_sort(run.begin(), run.end(),
                             [&](const sim::MapInstance& a, const sim::MapInstance& b) {
                                 return sizeOf(a) > sizeOf(b);
                             });
            Tile t;
            t.start = static_cast<int>(inst.size() / 8);
            t.count = static_cast<int>(run.size());
            t.lo = Vector3{1e30f, 0.0f, 1e30f};
            t.hi = Vector3{-1e30f, 0.0f, -1e30f};
            for (const sim::MapInstance& m : run) {
                const float r = std::sqrt(m.hx * m.hx + m.hy * m.hy) / kScale;
                const float x = m.cx / kScale, z = m.cy / kScale;
                t.lo.x = std::min(t.lo.x, x - r); t.hi.x = std::max(t.hi.x, x + r);
                t.lo.z = std::min(t.lo.z, z - r); t.hi.z = std::max(t.hi.z, z + r);
                t.hi.y = std::max(t.hi.y, m.height / kScale);
                sizes_.push_back(sizeOf(m));
                push(m);
            }
            const float e0 = wm.originE + static_cast<float>(ti % wm.tilesX) * wm.tileSize;
            const float n0 = wm.originN + static_cast<float>(ti / wm.tilesX) * wm.tileSize;
            for (int lv = 0; lv < 2; ++lv) {
                t.farStart[lv] = static_cast<int>(massing[lv].size() / 8);
                buildMassing(run, e0, n0, wm.tileSize, kMassCell[lv], massing[lv]);
                t.farBlocks[lv] = static_cast<int>(massing[lv].size() / 8) - t.farStart[lv];
                for (const sim::MapInstance& m : run) { // sorted largest first
                    if (sizeOf(m) < kMassCell[lv] / kScale) break;
                    pushTo(massing[lv], m);
                }
                t.farCount[lv] = static_cast<int>(massing[lv].size() / 8) - t.farStart[lv];
            }
            tiles_.push_back(t);
        }
        total_ = static_cast<int>(inst.size() / 8);
        cylStart_ = static_cast<int>(inst.size() / 8);
        cylCount_ = static_cast<int>(cylinders.size());
        for (const sim::MapInstance& m : cylinders) push(m);
        total_ += cylCount_;
        for (int lv = 0; lv < 2; ++lv) {
            const int base = static_cast<int>(inst.size() / 8);
            for (Tile& t : tiles_) t.farStart[lv] += base;
            inst.insert(inst.end(), massing[lv].begin(), massing[lv].end());
        }

        glGenBuffers(1, &instVbo_);
        glBindBuffer(GL_ARRAY_BUFFER, instVbo_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(inst.size() * sizeof(float)),
                     inst.data(), GL_STATIC_DRAW);

        box_ = buildMeshVao(makeBox());
        cyl_ = buildMeshVao(makeCylinder(16));
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    // Draw inside BeginMode3D. lodK = pixels per unit angular size; pxMin = the
    // LOD threshold in pixels.
    Stats draw(Vector3 camPos, float lodK, float pxMin) {
        Stats st;
        st.total = total_;
        const float switchPx = kMassSwitchPx * pxMin;
        rlDrawRenderBatchActive(); // flush raylib's immediate-mode batch first
        const Frustum fr = extractFrustum();
        const Matrix mvp = MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());

        glUseProgram(prog_);
        const float16 m = MatrixToFloatV(mvp);
        glUniformMatrix4fv(uMvp_, 1, GL_FALSE, m.v);
        glUniform3f(uCam_, camPos.x, camPos.y, camPos.z);
        glUniform1f(uLodK_, lodK);
        glUniform1f(uPxMin_, pxMin);
        float pal[20];
        for (int k = 0; k < 5; ++k) {
            pal[4 * k + 0] = kKindColor[k].r / 255.0f;
            pal[4 * k + 1] = kKindColor[k].g / 255.0f;
            pal[4 * k + 2] = kKindColor[k].b / 255.0f;
            pal[4 * k + 3] = 1.0f;
        }
        glUniform4fv(uPalette_, 5, pal);

        glBindBuffer(GL_ARRAY_BUFFER, instVbo_);
        glBindVertexArray(box_.vao);
        auto drawRange = [&](int first, int count) {
            if (count <= 0) return;
            setInstanceOffset(first);
            glDrawElementsInstanced(GL_TRIANGLES, box_.indexCount, GL_UNSIGNED_SHORT,
                                    nullptr, count);
            ++st.draws;
        };
        // An open run of far tiles at one level: [runStart, runEnd) in records.
        int runLevel = -1, runStart = 0, runEnd = 0;
        auto flushRun = [&]() {
            if (runLevel >= 0) drawRange(runStart, runEnd - runStart);
            runLevel = -1;
        };
        for (const Tile& t : tiles_) {
            if (!boxVisible(fr, t.lo, t.hi)) { flushRun(); continue; }
            // Nearest point of the tile's box to the eye: every building in it is
            // at least this far, so this distance bounds their on-screen size.
            const float dx = std::max({t.lo.x - camPos.x, 0.0f, camPos.x - t.hi.x});
            const float dy = std::max({t.lo.y - camPos.y, 0.0f, camPos.y - t.hi.y});
            const float dz = std::max({t.lo.z - camPos.z, 0.0f, camPos.z - t.hi.z});
            const float d = std::sqrt(dx * dx + dy * dy + dz * dz);
            // Far tiles swap their small buildings for massing blocks once a
            // block would cover fewer than switchPx pixels. Their content is then
            // fixed (blocks + towers; the shader still fades towers below pxMin),
            // so consecutive far tiles at the same level extend one draw.
            const auto cellPx = [&](int lv) { return kMassCell[lv] / kScale * lodK / d; };
            if (d > 0.0f && cellPx(0) < switchPx) {
                const int lv = cellPx(1) >= switchPx ? 0 : 1;
                if (runLevel != lv || runEnd != t.farStart[lv]) {
                    flushRun();
                    runLevel = lv;
                    runStart = t.farStart[lv];
                }
                runEnd = t.farStart[lv] + t.farCount[lv];
                st.massing += t.farBlocks[lv];
                st.drawn += t.farCount[lv] - t.farBlocks[lv];
                continue;
            }
            // Near tile: sizes are descending, so draw the prefix of buildings
            // big enough to see.
            flushRun();
            const float minSize = pxMin * d / lodK;
            const float* first = sizes_.data() + t.start;
            const int n = static_cast<int>(
                std::partition_point(first, first + t.count,
                                     [&](float s) { return s >= minSize; }) - first);
            drawRange(t.start, n);
            st.drawn += n;
        }
        flushRun();
        if (cylCount_ > 0) {
            glBindVertexArray(cyl_.vao);
            setInstanceOffset(cylStart_);
            glDrawElementsInstanced(GL_TRIANGLES, cyl_.indexCount, GL_UNSIGNED_SHORT,
                                    nullptr, cylCount_);
            st.drawn += cylCount_;
            ++st.draws;
        }
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glUseProgram(0);
        return st;
    }

private:
    struct Tile {
        int start{0}, count{0};
        int farStart[2]{0, 0}, farCount[2]{0, 0}, farBlocks[2]{0, 0};
        Vector3 lo{}, hi{};
    };

    // Massing levels (cell size, m) and the on-screen cell size (px, scaled by
    // pxMin) below which a tile switches from individual buildings to blocks.
    static constexpr float kMassCell[2] = {50.0f, 125.0f};
    static constexpr float kMassSwitchPx = 6.0f;

    // Merge a tile's small buildings (narrower than one cell) into one block per
    // occupied cell: placed at their footprint centroid, covering their combined
    // footprint, at their area-weighted mean height, so the block keeps the
    // cell's built volume. Records go to @p out with class + 8 (massing flag).
    static void buildMassing(const std::vector<sim::MapInstance>& run, float e0, float n0,
                             float tileSize, float cell, std::vector<float>& out) {
        const int n = static_cast<int>(std::ceil(tileSize / cell));
        struct Acc { float a{0}, x{0}, y{0}, h{0}; };
        std::vector<Acc> acc(static_cast<std::size_t>(n) * n);
        for (const sim::MapInstance& m : run) {
            if (std::max(m.height, 2.0f * std::max(m.hx, m.hy)) >= cell) continue;
            const int ix = std::clamp(static_cast<int>((m.cx - e0) / cell), 0, n - 1);
            const int iy = std::clamp(static_cast<int>((m.cy - n0) / cell), 0, n - 1);
            Acc& c = acc[static_cast<std::size_t>(iy) * n + ix];
            const float area = 4.0f * m.hx * m.hy;
            c.a += area;
            c.x += area * m.cx;
            c.y += area * m.cy;
            c.h += area * m.height;
        }
        for (const Acc& c : acc) {
            if (c.a <= 0.0f) continue;
            const float side = std::sqrt(std::min(c.a, cell * cell)) / kScale;
            const float h = c.h / c.a;
            const float kind = static_cast<float>(
                h >= 80.0f ? sim::MapKind::Skyscraper : sim::MapKind::Block);
            out.insert(out.end(), {c.x / c.a / kScale, c.y / c.a / kScale, 1.0f, 0.0f,
                                   side, h / kScale, side, kind + 8.0f});
        }
    }
    struct MeshVao { GLuint vao{0}, vbo{0}, ibo{0}; GLsizei indexCount{0}; };

    MeshVao buildMeshVao(const MeshData& md) {
        MeshVao mv;
        mv.indexCount = static_cast<GLsizei>(md.idx.size());
        glGenVertexArrays(1, &mv.vao);
        glBindVertexArray(mv.vao);
        glGenBuffers(1, &mv.vbo);
        glBindBuffer(GL_ARRAY_BUFFER, mv.vbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(md.vtx.size() * sizeof(float)),
                     md.vtx.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                              reinterpret_cast<void*>(3 * sizeof(float)));
        glGenBuffers(1, &mv.ibo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mv.ibo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(md.idx.size() * sizeof(std::uint16_t)),
                     md.idx.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, instVbo_);
        for (GLuint a : {2u, 3u}) {
            glEnableVertexAttribArray(a);
            glVertexAttribDivisor(a, 1);
        }
        setInstanceOffset(0);
        return mv;
    }

    // WebGL2 has no base-instance draw, so a tile's run is selected by pointing
    // the per-instance attributes at its first record (bound VAO + instVbo_).
    void setInstanceOffset(int first) {
        const auto base = static_cast<std::uintptr_t>(first) * 8u * sizeof(float);
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float),
                              reinterpret_cast<void*>(base));
        glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float),
                              reinterpret_cast<void*>(base + 4 * sizeof(float)));
    }

    GLuint prog_{0}, instVbo_{0};
    GLint uMvp_{-1}, uCam_{-1}, uLodK_{-1}, uPxMin_{-1}, uPalette_{-1};
    MeshVao box_, cyl_;
    std::vector<Tile> tiles_;
    std::vector<float> sizes_; // per box instance size (km), parallel to the buffer
    int cylStart_{0}, cylCount_{0}, total_{0};
};

// Flat East River surface: local-metre triangles baked into one static mesh.
Mesh buildFlatMesh(const std::vector<sim::Vector3>& tris, float y, Color col) {
    Mesh m{};
    m.vertexCount = static_cast<int>(tris.size());
    m.triangleCount = m.vertexCount / 3;
    m.vertices = static_cast<float*>(MemAlloc(static_cast<unsigned int>(tris.size() * 3 * sizeof(float))));
    m.colors = static_cast<unsigned char*>(MemAlloc(static_cast<unsigned int>(tris.size() * 4)));
    for (std::size_t i = 0; i < tris.size(); ++i) {
        m.vertices[3 * i + 0] = static_cast<float>(tris[i].x) / kScale;
        m.vertices[3 * i + 1] = y;
        m.vertices[3 * i + 2] = static_cast<float>(tris[i].y) / kScale;
        m.colors[4 * i + 0] = col.r;
        m.colors[4 * i + 1] = col.g;
        m.colors[4 * i + 2] = col.b;
        m.colors[4 * i + 3] = col.a;
    }
    UploadMesh(&m, false);
    return m;
}

// --- Missiles: flyweight instancing --------------------------------------------
// Every missile shares one unit mesh (same proportions as the desktop
// visualizer's silhouette: airframe, ogive nose, four tail fins, boost plume)
// and differs only by a 44-byte instance record: position, heading, length,
// radius, and three colors. The vertex shader builds each missile's frame from
// its heading, so the whole raid is a single instanced draw per frame instead of
// a few hundred immediate-mode calls. Unit mesh axes: y runs tail (-0.5) to tip
// (+0.5) in units of length; x/z are radial in units of radius.
const char* kMissileVS = R"(#version 300 es
precision highp float;
layout(location = 0) in vec3 vtx;
layout(location = 1) in float part;     // 0 airframe + fins, 1 nose, 2 plume
layout(location = 2) in vec4 posLen;    // position (km), length (km)
layout(location = 3) in vec4 dirRad;    // unit heading, radius (km)
layout(location = 4) in vec4 bodyCol;
layout(location = 5) in vec4 noseCol;
layout(location = 6) in vec4 plumeCol;  // alpha 0 = motor out (plume collapsed)
uniform mat4 mvp;
out vec4 vColor;
void main() {
    vec3 f = dirRad.xyz;
    vec3 ref = abs(f.y) > 0.9 ? vec3(1.0, 0.0, 0.0) : vec3(0.0, 1.0, 0.0);
    vec3 r = normalize(cross(f, ref));
    vec3 u = normalize(cross(r, f));
    vColor = part < 0.5 ? bodyCol : (part < 1.5 ? noseCol : plumeCol);
    float keep = (part > 1.5 && plumeCol.a == 0.0) ? 0.0 : 1.0;
    vec3 w = posLen.xyz + keep * (f * (vtx.y * posLen.w) + (r * vtx.x + u * vtx.z) * dirRad.w);
    gl_Position = mvp * vec4(w, 1.0);
}
)";

// Shared by the missiles and the line batch: unlit vertex color, as raylib's
// own immediate-mode shapes are.
const char* kFlatFS = R"(#version 300 es
precision mediump float;
in vec4 vColor;
out vec4 finalColor;
void main() { finalColor = vColor; }
)";

GLuint linkProgram(const char* vsSrc, const char* fsSrc) {
    const GLuint prog = glCreateProgram();
    const GLuint vs = compileShader(GL_VERTEX_SHADER, vsSrc);
    const GLuint fs = compileShader(GL_FRAGMENT_SHADER, fsSrc);
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
}

void uploadMvp(GLint loc) {
    const Matrix mvp = MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());
    const float16 m = MatrixToFloatV(mvp);
    glUniformMatrix4fv(loc, 1, GL_FALSE, m.v);
}

class MissileRenderer {
public:
    struct Inst {
        float px, py, pz, len;      // km
        float dx, dy, dz, rad;      // unit heading, km
        std::uint8_t body[4], nose[4], plume[4];
    };
    static_assert(sizeof(Inst) == 44, "missile instance record is 44 bytes");

    void init() {
        prog_ = linkProgram(kMissileVS, kFlatFS);
        uMvp_ = glGetUniformLocation(prog_, "mvp");

        // Unit mesh as (x, y, z, part) triangles.
        std::vector<float> v;
        auto tri = [&](Vector3 a, Vector3 b, Vector3 c, float part) {
            for (const Vector3& p : {a, b, c}) v.insert(v.end(), {p.x, p.y, p.z, part});
        };
        constexpr int kSides = 8;
        auto ring = [&](int i, float y, float r) {
            const float a = 2.0f * PI * static_cast<float>(i) / kSides;
            return Vector3{r * cosf(a), y, r * sinf(a)};
        };
        // Airframe tube (tail -> shoulder) and its tail cap.
        for (int i = 0; i < kSides; ++i) {
            tri(ring(i, -0.5f, 1.0f), ring(i + 1, -0.5f, 1.0f), ring(i + 1, 0.18f, 1.0f), 0.0f);
            tri(ring(i, -0.5f, 1.0f), ring(i + 1, 0.18f, 1.0f), ring(i, 0.18f, 1.0f), 0.0f);
            tri(Vector3{0.0f, -0.5f, 0.0f}, ring(i + 1, -0.5f, 1.0f), ring(i, -0.5f, 1.0f), 0.0f);
        }
        // Ogive nose (shoulder -> tip).
        for (int i = 0; i < kSides; ++i)
            tri(ring(i, 0.18f, 1.0f), ring(i + 1, 0.18f, 1.0f), Vector3{0.0f, 0.5f, 0.0f}, 1.0f);
        // Four tail fins, root at the airframe, swept back to the tail.
        const Vector3 dirs[4] = {{1, 0, 0}, {-1, 0, 0}, {0, 0, 1}, {0, 0, -1}};
        for (const Vector3& d : dirs) {
            tri(Vector3{d.x, -0.32f, d.z}, Vector3{d.x, -0.5f, d.z},
                Vector3{3.2f * d.x, -0.5f, 3.2f * d.z}, 0.0f);
        }
        // Boost plume (tail -> 0.9 lengths behind it), collapsed when unlit.
        for (int i = 0; i < kSides; ++i)
            tri(ring(i, -0.5f, 0.9f), ring(i + 1, -0.5f, 0.9f), Vector3{0.0f, -1.4f, 0.0f}, 2.0f);
        vertexCount_ = static_cast<GLsizei>(v.size() / 4);

        glGenVertexArrays(1, &vao_);
        glBindVertexArray(vao_);
        glGenBuffers(1, &meshVbo_);
        glBindBuffer(GL_ARRAY_BUFFER, meshVbo_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(v.size() * sizeof(float)),
                     v.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                              reinterpret_cast<void*>(3 * sizeof(float)));
        glGenBuffers(1, &instVbo_);
        glBindBuffer(GL_ARRAY_BUFFER, instVbo_);
        const GLsizei stride = sizeof(Inst);
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, stride, nullptr);
        glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, stride,
                              reinterpret_cast<void*>(offsetof(Inst, dx)));
        glVertexAttribPointer(4, 4, GL_UNSIGNED_BYTE, GL_TRUE, stride,
                              reinterpret_cast<void*>(offsetof(Inst, body)));
        glVertexAttribPointer(5, 4, GL_UNSIGNED_BYTE, GL_TRUE, stride,
                              reinterpret_cast<void*>(offsetof(Inst, nose)));
        glVertexAttribPointer(6, 4, GL_UNSIGNED_BYTE, GL_TRUE, stride,
                              reinterpret_cast<void*>(offsetof(Inst, plume)));
        for (GLuint a = 2; a <= 6; ++a) {
            glEnableVertexAttribArray(a);
            glVertexAttribDivisor(a, 1);
        }
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void clear() { inst_.clear(); }

    // Queue one missile (display metres, scaled up from the real airframe so it
    // reads against the skyline, as on the desktop).
    void add(Vector3 pos, Vector3 velRay, float lengthM, float radiusM, Color body,
             Color nose, bool boosting, Color plume) {
        Vector3 f = velRay;
        f = (Vector3Length(f) < 1e-9f) ? Vector3{0.0f, 1.0f, 0.0f} : Vector3Normalize(f);
        Inst m{pos.x, pos.y, pos.z, lengthM / kScale, f.x, f.y, f.z, radiusM / kScale,
               {body.r, body.g, body.b, body.a}, {nose.r, nose.g, nose.b, nose.a},
               {plume.r, plume.g, plume.b, static_cast<std::uint8_t>(boosting ? plume.a : 0)}};
        inst_.push_back(m);
    }

    // One instanced draw for every queued missile. Call inside BeginMode3D.
    void draw() {
        if (inst_.empty()) return;
        rlDrawRenderBatchActive(); // keep raylib's queued shapes in draw order
        glUseProgram(prog_);
        uploadMvp(uMvp_);
        glBindBuffer(GL_ARRAY_BUFFER, instVbo_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(inst_.size() * sizeof(Inst)),
                     inst_.data(), GL_STREAM_DRAW);
        glBindVertexArray(vao_);
        glDisable(GL_CULL_FACE); // fins are single sheets, seen from both sides
        glDrawArraysInstanced(GL_TRIANGLES, 0, vertexCount_,
                              static_cast<GLsizei>(inst_.size()));
        glEnable(GL_CULL_FACE);
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glUseProgram(0);
    }

private:
    GLuint prog_{0}, vao_{0}, meshVbo_{0}, instVbo_{0};
    GLint uMvp_{-1};
    GLsizei vertexCount_{0};
    std::vector<Inst> inst_;
};

// --- Detonations: flyweight spheres --------------------------------------------
// Each burst is a solid core plus a wire shell. Both shapes are shared unit
// meshes, drawn instanced from a 20-byte record (centre, radius, color), so any
// number of simultaneous bursts costs two draws.
const char* kBurstVS = R"(#version 300 es
precision highp float;
layout(location = 0) in vec3 vtx;      // unit sphere
layout(location = 1) in vec4 posRad;   // centre (km), radius (km)
layout(location = 2) in vec4 col;
uniform mat4 mvp;
out vec4 vColor;
void main() { vColor = col; gl_Position = mvp * vec4(posRad.xyz + vtx * posRad.w, 1.0); }
)";

class BurstRenderer {
public:
    void init() {
        prog_ = linkProgram(kBurstVS, kFlatFS);
        uMvp_ = glGetUniformLocation(prog_, "mvp");
        constexpr int kRings = 8, kSlices = 8;
        auto at = [](int ring, int slice) {
            const float th = PI * static_cast<float>(ring) / kRings;       // 0 = top pole
            const float ph = 2.0f * PI * static_cast<float>(slice) / kSlices;
            return Vector3{sinf(th) * cosf(ph), cosf(th), sinf(th) * sinf(ph)};
        };
        std::vector<Vector3> tris, wires;
        for (int r = 0; r < kRings; ++r) {
            for (int sl = 0; sl < kSlices; ++sl) {
                const Vector3 a = at(r, sl), b = at(r, sl + 1), c = at(r + 1, sl + 1),
                              d = at(r + 1, sl);
                tris.insert(tris.end(), {a, b, c, a, c, d});
                wires.insert(wires.end(), {a, b, a, d}); // latitude + longitude edges
            }
        }
        solid_ = buildMesh(tris);
        wire_ = buildMesh(wires);
    }

    void clear() { core_.clear(); shell_.clear(); }
    void addCore(Vector3 c, float r, Color col) { core_.push_back(make(c, r, col)); }
    void addShell(Vector3 c, float r, Color col) { shell_.push_back(make(c, r, col)); }

    void draw() {
        if (core_.empty() && shell_.empty()) return;
        rlDrawRenderBatchActive();
        glUseProgram(prog_);
        uploadMvp(uMvp_);
        drawSet(solid_, core_, GL_TRIANGLES);
        drawSet(wire_, shell_, GL_LINES);
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glUseProgram(0);
    }

private:
    struct Inst { float x, y, z, r; std::uint8_t c[4]; };
    struct Shape { GLuint vao{0}, vbo{0}, inst{0}; GLsizei count{0}; };

    static Inst make(Vector3 c, float r, Color col) {
        return Inst{c.x, c.y, c.z, r, {col.r, col.g, col.b, col.a}};
    }

    static Shape buildMesh(const std::vector<Vector3>& v) {
        Shape s;
        s.count = static_cast<GLsizei>(v.size());
        glGenVertexArrays(1, &s.vao);
        glBindVertexArray(s.vao);
        glGenBuffers(1, &s.vbo);
        glBindBuffer(GL_ARRAY_BUFFER, s.vbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(v.size() * sizeof(Vector3)),
                     v.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vector3), nullptr);
        glGenBuffers(1, &s.inst);
        glBindBuffer(GL_ARRAY_BUFFER, s.inst);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Inst), nullptr);
        glVertexAttribPointer(2, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(Inst),
                              reinterpret_cast<void*>(offsetof(Inst, c)));
        for (GLuint a : {1u, 2u}) {
            glEnableVertexAttribArray(a);
            glVertexAttribDivisor(a, 1);
        }
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        return s;
    }

    static void drawSet(const Shape& s, const std::vector<Inst>& inst, GLenum mode) {
        if (inst.empty()) return;
        glBindBuffer(GL_ARRAY_BUFFER, s.inst);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(inst.size() * sizeof(Inst)),
                     inst.data(), GL_STREAM_DRAW);
        glBindVertexArray(s.vao);
        glDrawArraysInstanced(mode, 0, s.count, static_cast<GLsizei>(inst.size()));
    }

    GLuint prog_{0};
    GLint uMvp_{-1};
    Shape solid_, wire_;
    std::vector<Inst> core_, shell_;
};

// --- Lines: one streamed batch per frame ---------------------------------------
// Trails and ProNav line-of-sight links all go into one vertex array, uploaded
// and drawn with a single GL_LINES call.
const char* kLineVS = R"(#version 300 es
precision highp float;
layout(location = 0) in vec3 pos;
layout(location = 1) in vec4 col;
uniform mat4 mvp;
out vec4 vColor;
void main() { vColor = col; gl_Position = mvp * vec4(pos, 1.0); }
)";

class LineBatch {
public:
    void init() {
        prog_ = linkProgram(kLineVS, kFlatFS);
        uMvp_ = glGetUniformLocation(prog_, "mvp");
        glGenVertexArrays(1, &vao_);
        glBindVertexArray(vao_);
        glGenBuffers(1, &vbo_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vert), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(Vert),
                              reinterpret_cast<void*>(offsetof(Vert, c)));
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void clear() { v_.clear(); }

    void add(Vector3 a, Vector3 b, Color c) {
        v_.push_back(Vert{a.x, a.y, a.z, {c.r, c.g, c.b, c.a}});
        v_.push_back(Vert{b.x, b.y, b.z, {c.r, c.g, c.b, c.a}});
    }

    void draw() {
        if (v_.empty()) return;
        rlDrawRenderBatchActive();
        glUseProgram(prog_);
        uploadMvp(uMvp_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(v_.size() * sizeof(Vert)),
                     v_.data(), GL_STREAM_DRAW);
        glBindVertexArray(vao_);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(v_.size()));
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glUseProgram(0);
    }

private:
    struct Vert { float x, y, z; std::uint8_t c[4]; };
    GLuint prog_{0}, vao_{0}, vbo_{0};
    GLint uMvp_{-1};
    std::vector<Vert> v_;
};

// --- Orbit camera -------------------------------------------------------------
struct OrbitCamera {
    Vector3 target{0.0f, 1.5f, 0.0f};
    float distance{30.0f};
    float yaw{0.7f};
    float pitch{0.62f};
    float fovy{55.0f};

    Camera3D toCamera() const {
        Camera3D c{};
        const float cp = cosf(pitch), sp = sinf(pitch);
        c.position = Vector3{target.x + distance * cp * sinf(yaw), target.y + distance * sp,
                             target.z + distance * cp * cosf(yaw)};
        c.target = target;
        c.up = Vector3{0.0f, 1.0f, 0.0f};
        c.fovy = fovy;
        c.projection = CAMERA_PERSPECTIVE;
        return c;
    }
};

// ============================================================================
// Application state
// ============================================================================
enum class RunState { Idle = 0, Running = 1, Paused = 2, Done = 3 };

struct Track {
    Vector3 prev{}, cur{};
    TelemetryRecord rec{};
    bool alive{false};
    std::deque<Vector3> trail;      // last second of positions
    std::deque<bool> trailHot;      // motor lit when each point was laid
};

struct Detonation { Vector3 pos; float age; bool assetHit; };

struct App {
    sim::SimConfig scfg;
    std::vector<sim::CityStructure> city; // master collision skyline (copied per run)
    std::unique_ptr<sim::EngagementScenario> scenario;
    std::vector<TelemetryRecord> records;
    std::vector<Track> tracks;            // indexed by entity id (ids are dense)
    std::vector<Detonation> detonations;
    int frame{0}, totalFrames{0}, assetLosses{0};
    double acc{0.0};
    float speed{1.0f};
    RunState state{RunState::Idle};

    CityRenderer city3d;
    MissileRenderer missiles;
    BurstRenderer bursts;
    LineBatch lines;
    CityRenderer::Stats cityStats;
    Mesh water{};
    Material waterMat{};

    OrbitCamera cam;
    long long followId{-1};
    float lift{0.0f};        // held rise (+1) / sink (-1) from the page buttons
    float alpha{1.0f};       // interpolation fraction of the frame on screen
    Vector2 hover{-1.0f, -1.0f}; // mouse position (canvas px), x < 0 = none
    float uiScale{1.0f};     // canvas pixels per CSS pixel
    Font font{};
    float pxMin{1.0f};       // LOD threshold (pixels), adapted to frame time
    double frameEma{1.0 / 60.0};
    double simMsEma{0.0};    // wall time spent stepping the scenario per frame
    double governorClock{0.0};
    double hudClock{0.0};
} app;

constexpr float kPxMinFloor = 1.0f, kPxMinCeil = 8.0f;

void ingestRecords() {
    for (Track& t : app.tracks) t.alive = false;
    for (const TelemetryRecord& r : app.records) {
        const Vector3 p = recPos(r);
        if (r.flags & FLAG_DESTROYED) {
            const bool hit = (r.flags & FLAG_ASSET_HIT) != 0;
            if (hit) ++app.assetLosses;
            app.detonations.push_back(Detonation{p, 0.0f, hit});
            continue;
        }
        if (r.entityId >= app.tracks.size()) app.tracks.resize(r.entityId + 1);
        Track& t = app.tracks[r.entityId];
        const bool wasAlive = !t.trail.empty();
        t.prev = wasAlive ? t.cur : p;
        t.cur = p;
        t.rec = r;
        t.alive = true;
        t.trail.push_back(p);
        t.trailHot.push_back((r.flags & FLAG_BOOSTER) != 0);
        if (t.trail.size() > 60) { t.trail.pop_front(); t.trailHot.pop_front(); }
    }
    for (Track& t : app.tracks) {
        if (!t.alive && !t.trail.empty()) { t.trail.clear(); t.trailHot.clear(); }
    }
}

void resetScenario() {
    app.scenario.reset(); // free the old run's copy of the skyline first
    app.scenario = std::make_unique<sim::EngagementScenario>(app.scfg, app.city);
    app.tracks.clear();
    app.detonations.clear();
    app.frame = 0;
    app.assetLosses = 0;
    app.acc = 0.0;
    app.followId = -1;
    app.scenario->buildRecords(app.records);
    ingestRecords();
}

Vector3 trackPos(const Track& t, float alpha) { return Vector3Lerp(t.prev, t.cur, alpha); }

void pushHud(double fps) {
    int hostiles = 0, interceptors = 0;
    for (const Track& t : app.tracks) {
        if (!t.alive) continue;
        if (isHostile(t.rec)) ++hostiles;
        else if (isInterceptor(t.rec)) ++interceptors;
    }
    const auto& mgr = app.scenario->manager();
    // The values travel as one array in wasm memory: EM_ASM passes at most 16
    // scalar arguments, and the HUD has outgrown that.
    const double v[] = {
        static_cast<double>(app.state), app.frame / 60.0, app.totalFrames / 60.0,
        static_cast<double>(hostiles), static_cast<double>(app.scfg.threatCount),
        static_cast<double>(mgr.interceptCount()), static_cast<double>(app.assetLosses),
        static_cast<double>(interceptors), static_cast<double>(app.scenario->friendlyLaunched()),
        fps, static_cast<double>(app.cityStats.drawn), static_cast<double>(app.cityStats.total),
        app.pxMin, app.followId >= 0 ? 1.0 : 0.0, app.simMsEma,
        static_cast<double>(app.cityStats.massing), app.cam.target.y};
    EM_ASM({
        if (!window.hudUpdate) return;
        const h = HEAPF64.subarray($0 >> 3, ($0 >> 3) + 17);
        window.hudUpdate({
            state: h[0], t: h[1], total: h[2], hostiles: h[3], threatsTotal: h[4],
            intercepts: h[5], losses: h[6], interceptors: h[7], launched: h[8],
            fps: h[9], drawn: h[10], buildings: h[11], pxMin: h[12], following: h[13],
            simMs: h[14], massing: h[15], alt: h[16]
        });
    }, v);
}

void setState(RunState s) {
    app.state = s;
    pushHud(1.0 / app.frameEma);
}

// One scenario tick: step, publish the frame, update tracks and effects.
void tick(double simDt) {
    app.scenario->step(simDt);
    app.scenario->buildRecords(app.records);
    ingestRecords();
    if (++app.frame >= app.totalFrames) {
        app.acc = 0.0;
        setState(RunState::Done);
    }
}

// Advance the scenario on a fixed tick. Playback time accumulates at the chosen
// speed; under load we cap both the tick count and the wall time spent per
// frame, dropping time (slow motion) rather than spiralling.
void stepSimulation(double frameDt) {
    if (app.state != RunState::Running) return;
    app.acc += frameDt * app.speed;
    const double simDt = kTickSeconds * app.scfg.timeScale;
    const double t0 = emscripten_get_now();
    int steps = 0;
    while (app.acc >= kTickSeconds) {
        if (steps >= 16 || emscripten_get_now() - t0 > 12.0) {
            app.acc = std::min(app.acc, kTickSeconds);
            break;
        }
        tick(simDt);
        app.acc -= kTickSeconds;
        ++steps;
        if (app.state == RunState::Done) break;
    }
    app.simMsEma += (emscripten_get_now() - t0 - app.simMsEma) * 0.1;
    for (Detonation& d : app.detonations) d.age += static_cast<float>(frameDt * app.speed);
    app.detonations.erase(
        std::remove_if(app.detonations.begin(), app.detonations.end(),
                       [](const Detonation& d) { return d.age > 1.2f; }),
        app.detonations.end());
}

// Raise the LOD threshold while frames run long, relax it as headroom returns.
// A gap of more than 100 ms is a stall (tab in the background, a GC pause, the
// page scrolled away), not a measure of rendering cost, so it is ignored.
void governQuality(double frameDt) {
    if (frameDt > 0.1) return;
    app.frameEma += (frameDt - app.frameEma) * 0.1;
    app.governorClock += frameDt;
    if (app.governorClock < 0.25) return;
    app.governorClock = 0.0;
    if (app.frameEma > 1.0 / 45.0) app.pxMin = std::min(app.pxMin * 1.25f, kPxMinCeil);
    else if (app.frameEma < 1.0 / 55.0) app.pxMin = std::max(app.pxMin / 1.1f, kPxMinFloor);
}

void handleKeys(float dt) {
    OrbitCamera& c = app.cam;
    const float ks = dt * 1.5f;
    if (IsKeyDown(KEY_LEFT)) c.yaw += ks;
    if (IsKeyDown(KEY_RIGHT)) c.yaw -= ks;
    if (IsKeyDown(KEY_UP)) c.pitch += ks;
    if (IsKeyDown(KEY_DOWN)) c.pitch -= ks;
    c.pitch = Clamp(c.pitch, -1.55f, 1.55f);
    if (IsKeyDown(KEY_EQUAL) || IsKeyDown(KEY_KP_ADD)) c.distance *= 1.0f - dt;
    if (IsKeyDown(KEY_MINUS) || IsKeyDown(KEY_KP_SUBTRACT)) c.distance *= 1.0f + dt;
    c.distance = Clamp(c.distance, 0.001f, 400.0f);
    if (app.followId >= 0) return;
    const float pan = dt * c.distance * 0.5f;
    // Rise / sink: the look-at point moves vertically (E / Page Up raise it,
    // Q / Page Down lower it, or the page's hold buttons), so the orbit can be
    // centred on missiles at altitude rather than only on the ground.
    float lift = app.lift;
    if (IsKeyDown(KEY_E) || IsKeyDown(KEY_PAGE_UP)) lift += 1.0f;
    if (IsKeyDown(KEY_Q) || IsKeyDown(KEY_PAGE_DOWN)) lift -= 1.0f;
    c.target.y = std::min(c.target.y + lift * pan, 25.0f);
    const Vector3 fwd{sinf(c.yaw), 0.0f, cosf(c.yaw)};
    const Vector3 right{fwd.z, 0.0f, -fwd.x};
    if (IsKeyDown(KEY_W)) c.target = Vector3Subtract(c.target, Vector3Scale(fwd, pan));
    if (IsKeyDown(KEY_S)) c.target = Vector3Add(c.target, Vector3Scale(fwd, pan));
    if (IsKeyDown(KEY_D)) c.target = Vector3Add(c.target, Vector3Scale(right, pan));
    if (IsKeyDown(KEY_A)) c.target = Vector3Subtract(c.target, Vector3Scale(right, pan));
}

void drawScene(float alpha) {
    const Camera3D cam3 = app.cam.toCamera();

    // Scale the near plane with the orbit distance so the 24-bit depth buffer
    // keeps the river and the deck apart at long range (see the desktop viewer).
    rlSetClipPlanes(Clamp(app.cam.distance * 0.01f, 1e-5f, 2.0f), 1000.0);
    BeginMode3D(cam3);

    // Ground layers (deck, river, grid) are painted in order as underlays that
    // never write depth while the eye is above ground: nothing above ground can
    // be behind them, and leaving them out of the depth buffer means the river
    // can never z-fight or show through building bases, whatever depth precision
    // the device's WebGL context provides. Below ground they are ordinary
    // depth-tested surfaces again, so they hide what is above them.
    const bool aboveGround = cam3.position.y > 0.0f;
    if (aboveGround) rlDisableDepthMask();
    DrawPlane(Vector3{0.0f, -0.02f, 0.0f}, Vector2{100.0f, 100.0f}, kDeckColor);
    rlDrawRenderBatchActive(); // deck first: DrawMesh below bypasses the batch
    rlDisableBackfaceCulling();
    DrawMesh(app.water, app.waterMat, MatrixIdentity());
    rlEnableBackfaceCulling();
    const Color gcol{160, 160, 160, 90};
    for (int i = -50; i <= 50; ++i) {
        const float f = static_cast<float>(i);
        DrawLine3D(Vector3{f, 0.0f, -50.0f}, Vector3{f, 0.0f, 50.0f}, gcol);
        DrawLine3D(Vector3{-50.0f, 0.0f, f}, Vector3{50.0f, 0.0f, f}, gcol);
    }
    rlDrawRenderBatchActive();
    if (aboveGround) rlEnableDepthMask();
    DrawCubeWires(Vector3{0.0f, 10.0f, 0.0f}, 100.0f, 20.0f, 100.0f, Color{40, 60, 90, 255});

    // lodK: pixels subtended per unit of (size / distance) at screen centre.
    const float lodK = 0.5f * static_cast<float>(GetRenderHeight()) /
                       tanf(0.5f * app.cam.fovy * DEG2RAD);
    app.cityStats = app.city3d.draw(cam3.position, lodK, app.pxMin);

    DrawCircle3D(Vector3{0.0f, 0.0f, 0.0f}, 6.0f, Vector3{1, 0, 0}, 90.0f,
                 Color{0, 120, 160, 150});

    // Trails: hot orange in boost, cool blue in cruise, fading toward the tail.
    // Every other sample is plenty for a one-second ribbon and halves the
    // per-frame upload; the last segment always reaches the drawn missile.
    app.lines.clear();
    app.missiles.clear();
    for (const Track& t : app.tracks) {
        if (!t.alive || t.trail.size() < 2) continue;
        const std::size_t n = t.trail.size();
        std::size_t prev = 0;
        for (std::size_t i = 2; i < n + 1; i += 2) {
            const std::size_t k = std::min(i, n - 1);
            const auto a = static_cast<unsigned char>(60.0f + 140.0f * k / n);
            const Color c = t.trailHot[k] ? Color{255, 150, 40, a} : Color{90, 150, 190, a};
            const Vector3 head = (k == n - 1) ? trackPos(t, alpha) : t.trail[k];
            app.lines.add(t.trail[prev], head, c);
            prev = k;
            if (k == n - 1) break;
        }
    }

    for (const Track& t : app.tracks) {
        if (!t.alive) continue;
        const TelemetryRecord& r = t.rec;
        const Vector3 p = trackPos(t, alpha);
        const Vector3 v = recVel(r);
        const bool boosting = (r.flags & FLAG_BOOSTER) != 0;
        if (isHostile(r)) {
            app.missiles.add(p, v, 300.0f, 14.0f, threatColor(r.threatLevel),
                             Color{40, 40, 44, 255}, boosting, Color{255, 150, 40, 255});
        } else if (isInterceptor(r)) {
            const bool engaged = r.targetId != kNoTargetId;
            const bool neutral = isNeutral(r);
            const Color body = neutral ? (engaged ? Color{30, 130, 50, 255} : Color{22, 95, 38, 255})
                                       : (engaged ? GREEN : SKYBLUE);
            app.missiles.add(p, v, 210.0f, 9.0f, body, Color{230, 240, 255, 255}, boosting,
                             Color{120, 200, 255, 255});
            if (engaged && r.targetId < app.tracks.size() && app.tracks[r.targetId].alive) {
                app.lines.add(p, trackPos(app.tracks[r.targetId], alpha),
                              neutral ? Color{40, 150, 60, 140} : Color{0, 255, 120, 140});
            }
        }
    }
    app.missiles.draw();
    app.lines.draw();

    // Detonations: a bright core inside an expanding, fading wire shell.
    app.bursts.clear();
    for (const Detonation& d : app.detonations) {
        const float radius = 0.15f + d.age * 0.9f;
        const float a = Clamp(1.0f - d.age / 1.2f, 0.0f, 1.0f);
        const Color core = d.assetHit ? Color{255, 90, 60, 255} : Color{255, 220, 140, 255};
        app.bursts.addCore(d.pos, radius * 0.45f, ColorAlpha(core, a * 0.85f));
        app.bursts.addShell(d.pos, radius, ColorAlpha(Color{255, 160, 40, 255}, a));
    }
    app.bursts.draw();
    EndMode3D();
}

// The missile whose on-screen position is nearest canvas pixel (x, y) and
// within radiusPx, or -1. Points behind the eye are skipped (they would project
// to bogus screen positions).
long long pickTrack(float x, float y, float radiusPx) {
    const Camera3D c3 = app.cam.toCamera();
    const Vector3 fwd = Vector3Normalize(Vector3Subtract(c3.target, c3.position));
    long long best = -1;
    float bestD2 = radiusPx * radiusPx;
    for (std::size_t id = 0; id < app.tracks.size(); ++id) {
        const Track& t = app.tracks[id];
        if (!t.alive || !(isHostile(t.rec) || isInterceptor(t.rec))) continue;
        const Vector3 p = trackPos(t, app.alpha);
        if (Vector3DotProduct(Vector3Subtract(p, c3.position), fwd) <= 0.0f) continue;
        const Vector2 s = GetWorldToScreen(p, c3);
        const float d2 = (s.x - x) * (s.x - x) + (s.y - y) * (s.y - y);
        if (d2 < bestD2) { bestD2 = d2; best = static_cast<long long>(id); }
    }
    return best;
}

// Hover ring: encircle the missile under the mouse and tag its allegiance
// (ENEMY / ALLY / DOMESTIC), sized in CSS pixels so it reads the same at any
// render resolution.
void drawHoverRing() {
    if (app.hover.x < 0.0f) return;
    const float k = app.uiScale;
    const long long id = pickTrack(app.hover.x, app.hover.y, 45.0f * k);
    if (id < 0) return;
    const Track& t = app.tracks[static_cast<std::size_t>(id)];
    const Vector2 s = GetWorldToScreen(trackPos(t, app.alpha), app.cam.toCamera());
    const Color col = trackColor(t.rec);
    DrawRing(s, 18.0f * k, 21.0f * k, 0.0f, 360.0f, 48, col);
    DrawRing(s, 21.0f * k, 22.5f * k, 0.0f, 360.0f, 48, ColorAlpha(BLACK, 0.5f));
    const char* lbl = trackLabel(t.rec);
    const float size = 18.0f * k;
    const float lw = MeasureTextEx(app.font, lbl, size, k).x;
    DrawRectangleRec(Rectangle{s.x + 26.0f * k, s.y - 11.0f * k, lw + 8.0f * k, 22.0f * k},
                     ColorAlpha(BLACK, 0.55f));
    DrawTextEx(app.font, lbl, Vector2{s.x + 30.0f * k, s.y - 10.0f * k}, size, k, col);
}

// Camera follow: chase the tracked missile; drop back to free camera once it is
// gone (intercepted or impacted).
void updateFollow(float alpha) {
    if (app.followId < 0) return;
    const auto id = static_cast<std::size_t>(app.followId);
    if (id < app.tracks.size() && app.tracks[id].alive) {
        app.cam.target = Vector3Lerp(app.cam.target, trackPos(app.tracks[id], alpha), 0.25f);
    } else {
        app.followId = -1;
    }
}

void frameLoop() {
    const float dt = std::min(GetFrameTime(), 0.25f);
    handleKeys(dt);
    stepSimulation(dt);
    governQuality(dt);

    const bool midRun = app.state == RunState::Running || app.state == RunState::Paused;
    const float alpha =
        midRun ? static_cast<float>(Clamp(app.acc / kTickSeconds, 0.0, 1.0)) : 1.0f;

    updateFollow(alpha);

    BeginDrawing();
    ClearBackground(kSkyColor);
    app.alpha = alpha;
    drawScene(alpha);
    drawHoverRing();
    EndDrawing();

    app.hudClock += dt;
    if (app.hudClock >= 0.25) {
        app.hudClock = 0.0;
        pushHud(1.0 / app.frameEma);
    }
}

} // namespace

// ============================================================================
// Page interface (called from shell.html)
// ============================================================================
extern "C" {

EMSCRIPTEN_KEEPALIVE void web_start() {
    resetScenario();
    setState(RunState::Running);
}

EMSCRIPTEN_KEEPALIVE void web_set_paused(int paused) {
    if (paused && app.state == RunState::Running) setState(RunState::Paused);
    else if (!paused && app.state == RunState::Paused) setState(RunState::Running);
}

// Hold-to-move vertical control: +1 rises, -1 sinks, 0 stops.
EMSCRIPTEN_KEEPALIVE void web_set_lift(float dir) { app.lift = Clamp(dir, -1.0f, 1.0f); }

EMSCRIPTEN_KEEPALIVE void web_set_speed(float s) { app.speed = Clamp(s, 0.1f, 8.0f); }

EMSCRIPTEN_KEEPALIVE void web_resize(int w, int h) {
    if (w > 0 && h > 0 && (w != GetScreenWidth() || h != GetScreenHeight())) SetWindowSize(w, h);
}

// Orbit by a drag of (dx, dy) CSS pixels.
EMSCRIPTEN_KEEPALIVE void web_orbit(float dx, float dy) {
    app.cam.yaw -= dx * 0.006f;
    app.cam.pitch = Clamp(app.cam.pitch + dy * 0.006f, -1.55f, 1.55f);
}

// Pan by a drag expressed as fractions of the viewport height, map style: the
// ground follows the finger. Panning releases a camera follow.
EMSCRIPTEN_KEEPALIVE void web_pan(float fx, float fy) {
    OrbitCamera& c = app.cam;
    app.followId = -1;
    const float worldPerScreen = 2.0f * c.distance * tanf(0.5f * c.fovy * DEG2RAD);
    const Vector3 back{sinf(c.yaw), 0.0f, cosf(c.yaw)}; // ground direction toward the eye
    const Vector3 right{back.z, 0.0f, -back.x};
    // Dragging "down" on a tilted view covers more ground than across it.
    const float depth = 1.0f / std::max(sinf(c.pitch), 0.35f);
    c.target = Vector3Subtract(c.target, Vector3Scale(right, fx * worldPerScreen));
    c.target = Vector3Subtract(c.target, Vector3Scale(back, fy * worldPerScreen * depth));
    c.target.x = Clamp(c.target.x, -60.0f, 60.0f);
    c.target.z = Clamp(c.target.z, -60.0f, 60.0f);
}

// Multiply the orbit distance (<1 zooms in).
EMSCRIPTEN_KEEPALIVE void web_zoom(float factor) {
    app.cam.distance = Clamp(app.cam.distance * factor, 0.001f, 400.0f);
}

// Tap/click at canvas pixel (x, y): follow the nearest missile on screen, if
// any lies within the touch radius. Returns 1 when a missile was picked.
EMSCRIPTEN_KEEPALIVE int web_tap(float x, float y, float radiusPx) {
    const long long best = pickTrack(x, y, radiusPx);
    if (best < 0) return 0;
    app.followId = best;
    app.cam.distance = std::min(app.cam.distance, 3.0f);
    pushHud(1.0 / app.frameEma);
    return 1;
}

// Mouse hover at canvas pixel (x, y), with @p scale canvas pixels per CSS pixel;
// a negative x clears it (pointer left, pressed, or not a mouse).
EMSCRIPTEN_KEEPALIVE void web_hover(float x, float y, float scale) {
    app.hover = Vector2{x, y};
    if (scale > 0.0f) app.uiScale = scale;
}

EMSCRIPTEN_KEEPALIVE void web_free_camera() {
    app.followId = -1;
    pushHud(1.0 / app.frameEma);
}

// Diagnostics: render @p frames frames back to back (no simulation stepping) at
// LOD threshold @p pxMin and return the mean wall time per frame in ms. With
// @p sync the GPU work is included (pixel read-back sync); without it, only the
// CPU cost of building and submitting the frame. Independent of
// requestAnimationFrame, so it is meaningful in a hidden tab. Reports the
// buildings drawn via the HUD.
EMSCRIPTEN_KEEPALIVE double web_bench(int frames, float pxMin, int sync) {
    const float keep = app.pxMin;
    app.pxMin = pxMin;
    const double t0 = emscripten_get_now();
    for (int i = 0; i < frames; ++i) {
        updateFollow(1.0f);
        BeginDrawing();
        ClearBackground(kSkyColor);
        drawScene(1.0f);
        drawHoverRing();
        EndDrawing();
        // Browsers treat glFinish as a flush; reading a pixel back is a real
        // GPU sync, so the timing includes the frame's GPU work.
        unsigned char px[4];
        if (sync) glReadPixels(0, 0, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, px);
    }
    const double ms = (emscripten_get_now() - t0) / std::max(frames, 1);
    pushHud(1.0 / app.frameEma);
    app.pxMin = keep;
    return ms;
}

// Diagnostics: advance the running scenario up to @p ticks ticks as fast as
// possible (no rendering) and return the mean wall time per tick in ms.
EMSCRIPTEN_KEEPALIVE double web_bench_sim(int ticks) {
    const double simDt = kTickSeconds * app.scfg.timeScale;
    const double t0 = emscripten_get_now();
    int n = 0;
    for (; n < ticks && app.state == RunState::Running; ++n) tick(simDt);
    pushHud(1.0 / app.frameEma);
    return n > 0 ? (emscripten_get_now() - t0) / n : 0.0;
}

EMSCRIPTEN_KEEPALIVE void web_reset_view() {
    app.followId = -1;
    app.cam = OrbitCamera{};
}

} // extern "C"

int main() {
    SetTraceLogLevel(LOG_WARNING);
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    const int w = EM_ASM_INT({ return window.desiredCanvasWidth ? window.desiredCanvasWidth() : 960; });
    const int h = EM_ASM_INT({ return window.desiredCanvasHeight ? window.desiredCanvasHeight() : 540; });
    InitWindow(w, h, "Defense Simulation");
    // Hover-tag font (same Roboto as the desktop HUD), rasterized large enough to
    // stay crisp at 2x render scale.
    app.font = LoadFontEx("/assets/Roboto-Regular.ttf", 48, nullptr, 0);
    if (app.font.texture.id != 0) SetTextureFilter(app.font.texture, TEXTURE_FILTER_BILINEAR);
    else app.font = GetFontDefault();

    app.scfg = sim::loadSimConfig("/config/simulation.yaml");
    app.totalFrames = static_cast<int>(app.scfg.durationSeconds * 60.0);

    WebMap wm;
    if (!loadWebMap("/assets/un_map.webbin", wm)) {
        EM_ASM({ if (window.simFailed) window.simFailed("Could not load the city map."); });
        return 1;
    }
    app.city3d.init(wm);
    app.missiles.init();
    app.bursts.init();
    app.lines.init();
    app.water = buildFlatMesh(wm.map.waterTris, -0.006f, Color{24, 60, 104, 255});
    app.waterMat = LoadMaterialDefault();
    app.city = sim::cityStructuresFromInstances(wm.map.instances, app.scfg.defendedAsset, 1.0);
    wm = WebMap{}; // the GPU and the collision skyline hold everything now

    resetScenario();
    setState(RunState::Idle);
    EM_ASM({ if (window.simReady) window.simReady(); });
    emscripten_set_main_loop(frameLoop, 0, 1);
    return 0;
}
