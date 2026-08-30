"""Tiled OpenStreetMap fetch for the 1:1 defended city (UN HQ, New York).

A single Overpass query over a 10 km-radius bbox (~390k building ways) would time
out, so we grid the bbox into tiles and fetch each one, subdividing adaptively
when a tile is too heavy. Every tile response is cached to tools/tiles/, so the
fetch is fully resumable: rerun after an interruption and only the missing tiles
are requested. Building ways that straddle a tile edge appear in both tiles and
are de-duplicated by OSM id downstream (in gen_map_bin.py).

Outputs (in the working directory):
  tools/tiles/bld_<key>.json   one per building tile (raw Overpass "out geom")
  un_coast_10k.json            coastline + water polygons over the whole bbox

Data © OpenStreetMap contributors, ODbL.
"""
import json, math, os, subprocess, sys, time

# UN Secretariat (approx). The exact projection origin is recomputed from the
# Secretariat way in gen_map_bin.py; this only needs to centre the fetch bbox.
CENTER_LAT, CENTER_LON = 40.7489, -73.9680
RADIUS_M = 10000.0
MLAT = 111132.0
MLON = 111320.0 * math.cos(math.radians(CENTER_LAT))

DLAT = RADIUS_M / MLAT
DLON = RADIUS_M / MLON
S, N = CENTER_LAT - DLAT, CENTER_LAT + DLAT
W, E = CENTER_LON - DLON, CENTER_LON + DLON

TILES_LAT = 10            # ~2 km tiles at this latitude
TILES_LON = 10
MAX_SUBDIVIDE = 3         # extra levels when a tile is too big / errors
# Primary endpoint first; the mirrors are fallbacks only (many are often down).
# We stay on whichever endpoint last worked and rotate ONLY after a failure, so a
# dead mirror never taxes the common path.
ENDPOINTS = [
    "https://overpass-api.de/api/interpreter",
    "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
]
# Optional override: SIM_OVERPASS=<url> forces a single endpoint (used to fill
# the fringe from the mail.ru mirror while overpass-api.de was rate-limiting us).
if os.environ.get("SIM_OVERPASS"):
    ENDPOINTS = [os.environ["SIM_OVERPASS"]]
CACHE = os.path.join("tools", "tiles")
os.makedirs(CACHE, exist_ok=True)

_ep = 0

def overpass(query, timeout_s):
    """POST a query via curl. Returns parsed JSON, or None after all retries.
    Sticks to the working endpoint; only advances to the next on failure."""
    global _ep
    for attempt in range(len(ENDPOINTS) + 2):
        url = ENDPOINTS[_ep % len(ENDPOINTS)]
        try:
            p = subprocess.run(
                ["curl", "-s", "--max-time", str(timeout_s + 15),
                 "-X", "POST", url, "--data-urlencode", "data=" + query],
                capture_output=True, text=True)
            out = p.stdout
            if out and out.lstrip().startswith("{"):
                return json.loads(out)
        except Exception as ex:
            sys.stderr.write("  endpoint error (%s): %s\n" % (url, ex))
        # Failure: rotate to the next endpoint and back off.
        _ep += 1
        time.sleep(3 + attempt * 2)
    return None

def tile_key(s, w, n, e):
    return "%.5f_%.5f_%.5f_%.5f" % (s, w, n, e)

def fetch_tile(s, w, n, e, selector, prefix, depth=0):
    """Fetch one bbox of ways matching `selector` (e.g. "building" or
    "building:part"), subdividing on failure. Writes a cache file per leaf tile
    named "<prefix>_<key>.json". Returns the number of ways written."""
    key = tile_key(s, w, n, e)
    path = os.path.join(CACHE, "%s_%s.json" % (prefix, key))
    if os.path.exists(path) and os.path.getsize(path) > 0:
        try:
            with open(path) as f:
                return len(json.load(f).get("elements", []))
        except Exception:
            pass  # corrupt cache -> refetch

    to = 90 if depth == 0 else 120
    q = ('[out:json][timeout:%d];(way["%s"](%f,%f,%f,%f););out geom;'
         % (to, selector, s, w, n, e))
    data = overpass(q, to)
    if data is None or "elements" not in data:
        if depth < MAX_SUBDIVIDE:
            print("    subdividing heavy/failed tile (depth %d)" % (depth + 1),
                  flush=True)
            mlat, mlon = (s + n) / 2, (w + e) / 2
            total = 0
            for (qs, qw, qn, qe) in ((s, w, mlat, mlon), (s, mlon, mlat, e),
                                     (mlat, w, n, mlon), (mlat, mlon, n, e)):
                total += fetch_tile(qs, qw, qn, qe, selector, prefix, depth + 1)
            return total
        sys.stderr.write("  FAILED tile %s (giving up)\n" % key)
        return 0

    with open(path, "w") as f:
        json.dump(data, f)
    return len(data["elements"])

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

    # Two passes over the same tile grid: the building outlines (bld_) and the
    # building:part polygons (part_). NYC's 3-D building import puts the real
    # per-tower heights on the PARTS; the plain outline is often height-less, so
    # gen_map_bin.py backfills a building's height from its tallest part. Parts
    # are cached separately, so re-running only fetches whatever is missing.
    # SIM_FETCH=building,building:part (default both). Use SIM_FETCH=building:part
    # to fetch ONLY the parts when the building tiles are already cached (a re-run
    # of the building pass would needlessly re-issue the heavy top-level queries
    # that were previously subdivided, since only their child tiles are cached).
    want = os.environ.get("SIM_FETCH", "building,building:part").split(",")
    grand = 0
    for (selector, prefix, label) in (("building", "bld", "buildings"),
                                       ("building:part", "part", "parts")):
        if selector not in want:
            continue
        grand = 0
        print("--- fetching %s ---" % label, flush=True)
        for idx, (ts, tw, tn, te) in enumerate(tiles):
            cnt = fetch_tile(ts, tw, tn, te, selector, prefix)
            grand += cnt
            print("[%3d/%3d] %s ways=%-6d  running=%d"
                  % (idx + 1, len(tiles), label, cnt, grand), flush=True)
            time.sleep(1.0)  # be polite to the public API

    # Coastline + water polygons over the whole bbox (few, long ways -> one query).
    if os.path.exists("un_coast_10k.json") and os.path.getsize("un_coast_10k.json") > 0:
        print("coastline already cached (un_coast_10k.json) -> skipping", flush=True)
        print("done. cached tiles in %s" % CACHE)
        return
    print("fetching coastline + water ...", flush=True)
    cq = ('[out:json][timeout:120];('
          'way["natural"="coastline"](%f,%f,%f,%f);'
          'way["natural"="water"](%f,%f,%f,%f);'
          'way["waterway"="riverbank"](%f,%f,%f,%f);'
          ');out geom;' % (S, W, N, E, S, W, N, E, S, W, N, E))
    coast = overpass(cq, 120)
    if coast is not None:
        with open("un_coast_10k.json", "w") as f:
            json.dump(coast, f)
        print("coastline/water ways:", len(coast.get("elements", [])))
    else:
        print("WARNING: coastline fetch failed")

    print("done. cached building tiles in %s (running total ways ~%d, "
          "pre-dedupe)" % (CACHE, grand))

if __name__ == "__main__":
    main()
