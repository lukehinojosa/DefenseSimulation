#ifndef SIM_MAPDATA_HPP
#define SIM_MAPDATA_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "sim/Vector3.hpp"

namespace sim {

/// Class of a building (drives display color; ignored by the engine).
enum class MapKind : std::uint8_t {
    UnPlaza = 0, Block = 1, UnComplex = 2, Skyscraper = 3, Secretariat = 4
};

/// Which shared flyweight primitive an instance draws.
enum class MapPrim : std::uint8_t {
    Box = 0,       ///< Unit cube, scaled to (2*hx, height, 2*hy), rotated by angle.
    Cylinder = 1   ///< Unit N-gon prism; hx/hy give a circular or elliptical radius.
};

/// One flyweight instance: a placed, oriented, scaled primitive. A building is
/// one (rectangle/round) or a few (T/L/notched) of these. Coordinates are local
/// east(cx)/north(cy) metres about the map origin (the UN Secretariat), 1:1.
struct MapInstance {
    float   cx, cy;    ///< center (local metres)
    float   angle;     ///< rotation of the local X axis from east (radians)
    float   hx, hy;    ///< half-extents along the local axes (metres)
    float   height;    ///< height (metres)
    MapKind kind;
    MapPrim prim;
};

/// A loaded map: flyweight building instances + the East River water surface.
struct MapData {
    double lat0{0.0}, lon0{0.0}, mlat{0.0}, mlon{0.0}; ///< projection reference
    std::vector<MapInstance> instances;
    std::vector<Vector3>     waterTris; ///< local metres, z = 0 (vertices in triples)
};

/// Load a "UNMB" binary map asset (see tools/gen_map_bin.py) into @p out.
/// Returns false (and logs) on any read/format error.
bool loadMapData(const std::string& path, MapData& out);

/// Resolve the map asset path: $SIM_MAP if set, else search for
/// assets/un_map.bin in the working dir and a few parent directories.
std::string resolveMapPath();

} // namespace sim

#endif // SIM_MAPDATA_HPP
