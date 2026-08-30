// Loader for the "UNMB" binary map asset produced by tools/gen_map_bin.py.
// Little-endian POD layout (x86/ARM little-endian assumed):
//   char[4] 'UNMB' | u32 version | f64 lat0,lon0,mlat,mlon
//   u32 instanceCount | u32 waterVertCount
//   instanceCount * { f32 cx,cy,angle,hx,hy,height ; u8 kind, prim }   (26 bytes)
//   waterVertCount * { f32 x,y }                                        (8 bytes)

#include "sim/MapData.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

namespace sim {
namespace {

// Read a fixed-size little-endian POD from the stream. Returns false on short
// read. (The producer writes little-endian; both build targets are LE.)
template <typename T>
bool readPod(std::istream& in, T& v) {
    in.read(reinterpret_cast<char*>(&v), sizeof(T));
    return static_cast<bool>(in);
}

bool fileExists(const std::string& p) {
    std::ifstream f(p, std::ios::binary);
    return f.good();
}

} // namespace

std::string resolveMapPath() {
    if (const char* env = std::getenv("SIM_MAP")) {
        if (env[0] != '\0') return env;
    }
    // Search the working directory and a few parents for assets/un_map.bin so
    // the tools run from either the build dir or the project root.
    std::string prefix;
    for (int depth = 0; depth < 6; ++depth) {
        const std::string cand = prefix + "assets/un_map.bin";
        if (fileExists(cand)) return cand;
        prefix += "../";
    }
    return "assets/un_map.bin"; // best-effort default (load will report failure)
}

bool loadMapData(const std::string& path, MapData& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        std::fprintf(stderr, "map: cannot open '%s'\n", path.c_str());
        return false;
    }

    char magic[4];
    in.read(magic, 4);
    if (!in || magic[0] != 'U' || magic[1] != 'N' || magic[2] != 'M' || magic[3] != 'B') {
        std::fprintf(stderr, "map: bad magic in '%s'\n", path.c_str());
        return false;
    }
    std::uint32_t version = 0;
    if (!readPod(in, version) || version != 2u) {
        std::fprintf(stderr, "map: unsupported version %u (need 2)\n", version);
        return false;
    }
    if (!readPod(in, out.lat0) || !readPod(in, out.lon0) ||
        !readPod(in, out.mlat) || !readPod(in, out.mlon)) {
        std::fprintf(stderr, "map: truncated header\n");
        return false;
    }
    std::uint32_t nInst = 0, nWaterVert = 0;
    if (!readPod(in, nInst) || !readPod(in, nWaterVert)) {
        std::fprintf(stderr, "map: truncated counts\n");
        return false;
    }

    out.instances.clear();
    out.instances.reserve(nInst);
    for (std::uint32_t i = 0; i < nInst; ++i) {
        MapInstance m{};
        std::uint8_t kind = 0, prim = 0;
        if (!readPod(in, m.cx) || !readPod(in, m.cy) || !readPod(in, m.angle) ||
            !readPod(in, m.hx) || !readPod(in, m.hy) || !readPod(in, m.height) ||
            !readPod(in, kind) || !readPod(in, prim)) {
            std::fprintf(stderr, "map: truncated instance %u/%u\n", i, nInst);
            return false;
        }
        m.kind = static_cast<MapKind>(kind);
        m.prim = static_cast<MapPrim>(prim);
        out.instances.push_back(m);
    }

    out.waterTris.clear();
    out.waterTris.reserve(nWaterVert);
    for (std::uint32_t i = 0; i < nWaterVert; ++i) {
        float x = 0.0f, y = 0.0f;
        if (!readPod(in, x) || !readPod(in, y)) {
            std::fprintf(stderr, "map: truncated water vert %u/%u\n", i, nWaterVert);
            return false;
        }
        out.waterTris.push_back(Vector3{x, y, 0.0});
    }
    return true;
}

} // namespace sim
