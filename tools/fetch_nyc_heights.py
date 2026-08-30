"""Fetch authoritative building heights from NYC Open Data (DoITT Building
Footprints, resource 5zhs-2jue) over the same 10 km bbox as the OSM fetch.

Most OSM building outlines in this area carry no height and no building:part, so
after the OSM/part backfill a large low-rise remainder still falls back to a flat
default. DoITT publishes a `height_roof` (in FEET) for ~100% of NYC buildings; we
tile the bbox, pull `height_roof` + footprint geometry per tile (Socrata caps a
response at 50k rows, so heavy tiles subdivide), and cache each tile to
tools/tiles/nyc_<key>.geojson. gen_map_bin.py then fills any still-height-less OSM
building from the DoITT footprint that contains it.

Output: tools/tiles/nyc_<key>.geojson  (one per leaf tile; {height_roof, geometry})

Data: NYC Open Data, DoITT Building Footprints (public domain / open data).
"""
import json, math, os, subprocess, sys, time

CENTER_LAT, CENTER_LON = 40.7489, -73.9680
RADIUS_M = 10000.0
MLAT = 111132.0
MLON = 111320.0 * math.cos(math.radians(CENTER_LAT))
DLAT = RADIUS_M / MLAT
DLON = RADIUS_M / MLON
S, N = CENTER_LAT - DLAT, CENTER_LAT + DLAT
W, E = CENTER_LON - DLON, CENTER_LON + DLON

TILES_LAT = 10
TILES_LON = 10
MAX_SUBDIVIDE = 3
ROW_CAP = 50000           # Socrata hard response cap; >= means the tile is clipped
ENDPOINT = "https://data.cityofnewyork.us/resource/5zhs-2jue.geojson"
CACHE = os.path.join("tools", "tiles")
os.makedirs(CACHE, exist_ok=True)


def socrata(n, w, s, e, timeout_s=60):
    """GET one bbox of {height_roof, the_geom}. Returns the parsed feature list,
    or None on failure. within_box(col, NW_lat, NW_lon, SE_lat, SE_lon)."""
    where = "within_box(the_geom, %f, %f, %f, %f)" % (n, w, s, e)
    try:
        p = subprocess.run(
            ["curl", "-s", "--max-time", str(timeout_s + 15), "-G", ENDPOINT,
             "--data-urlencode", "$select=height_roof,the_geom",
             "--data-urlencode", "$limit=%d" % ROW_CAP,
             "--data-urlencode", "$where=" + where],
            capture_output=True, text=True)
        out = p.stdout
        if out and out.lstrip().startswith("{"):
            data = json.loads(out)
            return data.get("features", [])
    except Exception as ex:
        sys.stderr.write("  socrata error: %s\n" % ex)
    return None


def tile_key(s, w, n, e):
    return "%.5f_%.5f_%.5f_%.5f" % (s, w, n, e)


def fetch_tile(s, w, n, e, depth=0):
    key = tile_key(s, w, n, e)
    path = os.path.join(CACHE, "nyc_%s.geojson" % key)
    if os.path.exists(path) and os.path.getsize(path) > 0:
        try:
            with open(path) as f:
                return len(json.load(f).get("features", []))
        except Exception:
            pass  # corrupt cache -> refetch

    feats = socrata(n, w, s, e)
    # Subdivide on failure OR when the response is clipped at the row cap.
    if feats is None or (len(feats) >= ROW_CAP and depth < MAX_SUBDIVIDE):
        if depth < MAX_SUBDIVIDE:
            print("    subdividing heavy/failed tile (depth %d)" % (depth + 1),
                  flush=True)
            mlat, mlon = (s + n) / 2, (w + e) / 2
            total = 0
            for (qs, qw, qn, qe) in ((s, w, mlat, mlon), (s, mlon, mlat, e),
                                     (mlat, w, n, mlon), (mlat, mlon, n, e)):
                total += fetch_tile(qs, qw, qn, qe, depth + 1)
            return total
        if feats is None:
            sys.stderr.write("  FAILED tile %s (giving up)\n" % key)
            return 0

    with open(path, "w") as f:
        json.dump({"features": feats}, f)
    return len(feats)


def main():
    print("bbox: S=%.5f W=%.5f N=%.5f E=%.5f  (~%.0f km radius)"
          % (S, W, N, E, RADIUS_M / 1000))
    tiles = []
    for i in range(TILES_LAT):
        for j in range(TILES_LON):
            ts = S + (N - S) * i / TILES_LAT
            tn = S + (N - S) * (i + 1) / TILES_LAT
            tw = W + (E - W) * j / TILES_LON
            te = W + (E - W) * (j + 1) / TILES_LON
            tiles.append((ts, tw, tn, te))

    grand = 0
    for idx, (ts, tw, tn, te) in enumerate(tiles):
        cnt = fetch_tile(ts, tw, tn, te)
        grand += cnt
        print("[%3d/%3d] nyc footprints=%-6d  running=%d"
              % (idx + 1, len(tiles), cnt, grand), flush=True)
        time.sleep(0.5)  # be polite to the public API
    print("done. cached nyc height tiles in %s (running total ~%d)"
          % (CACHE, grand))


if __name__ == "__main__":
    main()
