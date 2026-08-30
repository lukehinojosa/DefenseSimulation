"""Pre-bake the fetched OSM tiles into flyweight box/cylinder instances + water,
written to the compact binary asset assets/un_map.bin.

Input : tools/tiles/bld_*.json  (from fetch_osm.py; de-duped by OSM way id here)
        un_coast_10k.json        (coastline + water ways over the bbox)
Output: assets/un_map.bin

Binary format (little-endian):
  char[4]  magic 'UNMB'
  u32      version (=2)
  f64      lat0, lon0, mlat, mlon        (projection reference)
  u32      instanceCount
  u32      waterVertCount                (= 3 * waterTriCount)
  instanceCount * { f32 cx,cy,angle,hx,hy,height ; u8 kind, prim }  ('<6f2B')
  waterVertCount * { f32 x,y }                                       ('<2f')
prim: 0 = box, 1 = cylinder. Coordinates are local east/north metres about the
UN Secretariat centroid, 1:1. A building expands to one (rectangle/round) or a
few (T/L/notched) instances via a min-area OBB + rectilinear decomposition.

Data © OpenStreetMap contributors, ODbL.
"""
PRIM_BOX, PRIM_CYL = 0, 1
import glob, json, math, os, struct, sys, time
from collections import Counter
import numpy as np
from scipy.spatial import cKDTree

TILE_GLOB = os.path.join("tools", "tiles", "bld_*.json")
COAST = "un_coast_10k.json"
OUT = os.path.join("assets", "un_map.bin")
KINDVAL = {"UnPlaza": 0, "Block": 1, "UnComplex": 2, "Skyscraper": 3, "Secretariat": 4}
WATER_CELL = 25.0          # water raster resolution (m); coarse enough for 20 km

# --- Projection origin: the UN Secretariat centroid (falls back to a fixed pt) --
def centroid(g):
    return (sum(p["lat"] for p in g) / len(g), sum(p["lon"] for p in g) / len(g))

def find_secretariat_origin():
    for path in glob.glob(TILE_GLOB):
        try:
            data = json.load(open(path))
        except Exception:
            continue
        for w in data.get("elements", []):
            if (w.get("type") == "way" and "geometry" in w and
                    w.get("tags", {}).get("name") ==
                    "United Nations Secretariat Building"):
                return centroid(w["geometry"])
    return (40.7489, -73.9680)

lat0, lon0 = find_secretariat_origin()
mlat = 111132.0
mlon = 111320.0 * math.cos(math.radians(lat0))

def enup(p):
    return ((p["lon"] - lon0) * mlon, (p["lat"] - lat0) * mlat)

# --- Building tags -> height / kind ------------------------------------------
DEFAULT_H = 24.0  # fallback when neither the outline nor any part gives a height

def height_from_tags(t):
    """Real height from OSM tags (metres), or None if untagged. NYC outlines are
    frequently height-less -- those are backfilled from building:part below."""
    h = t.get("height")
    if h:
        try:
            return float(str(h).replace("m", "").strip())
        except Exception:
            pass
    lv = t.get("building:levels")
    if lv:
        try:
            return float(lv) * 3.7
        except Exception:
            pass
    return None

def kind_of(name, h):
    n = name or ""
    if n == "United Nations Secretariat Building":
        return "Secretariat"
    if n == "United Nations Headquarters":
        return "UnPlaza"
    if h >= 120:
        return "Skyscraper"
    if "United Nations" in n or "Hammarsk" in n:
        return "UnComplex"
    return "Block"

# --- Footprint geometry helpers ----------------------------------------------
def poly_area(p):
    s = 0.0
    for i in range(len(p)):
        x1, y1 = p[i]
        x2, y2 = p[(i + 1) % len(p)]
        s += x1 * y2 - x2 * y1
    return abs(s) * 0.5

def convex_hull(pts):
    pts = sorted(set(pts))
    if len(pts) <= 2:
        return pts
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo = []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    hi = []
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return lo[:-1] + hi[:-1]

def min_area_obb(pts):
    """Return (cx, cy, angle, hx, hy) of the minimum-area oriented box."""
    hull = convex_hull(pts)
    if len(hull) < 3:
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        cx = (min(xs) + max(xs)) / 2; cy = (min(ys) + max(ys)) / 2
        return (cx, cy, 0.0, max(1.0, (max(xs) - min(xs)) / 2),
                max(1.0, (max(ys) - min(ys)) / 2))
    best = None
    n = len(hull)
    for i in range(n):
        a = hull[i]; b = hull[(i + 1) % n]
        ex, ey = b[0] - a[0], b[1] - a[1]
        L = math.hypot(ex, ey)
        if L == 0:
            continue
        ux, uy = ex / L, ey / L
        vx, vy = -uy, ux
        minu = minv = 1e18; maxu = maxv = -1e18
        for p in hull:
            du = (p[0] - a[0]) * ux + (p[1] - a[1]) * uy
            dv = (p[0] - a[0]) * vx + (p[1] - a[1]) * vy
            minu = min(minu, du); maxu = max(maxu, du)
            minv = min(minv, dv); maxv = max(maxv, dv)
        area = (maxu - minu) * (maxv - minv)
        if best is None or area < best[0]:
            cu = (minu + maxu) / 2; cv = (minv + maxv) / 2
            cx = a[0] + cu * ux + cv * vx
            cy = a[1] + cu * uy + cv * vy
            best = (area, cx, cy, math.atan2(uy, ux),
                    (maxu - minu) / 2, (maxv - minv) / 2)
    return best[1:]

def perimeter(p):
    return sum(math.hypot(p[(i + 1) % len(p)][0] - p[i][0],
                          p[(i + 1) % len(p)][1] - p[i][1]) for i in range(len(p)))

def circularity(p):
    per = perimeter(p)
    return (4 * math.pi * poly_area(p) / (per * per)) if per > 0 else 0.0

def point_in_poly(x, y, poly):
    inside = False
    n = len(poly); j = n - 1
    for i in range(n):
        xi, yi = poly[i]; xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside

def decompose_boxes(poly, obb, max_cells=22, cell_min=4.0):
    """Rectilinear cover of a concave footprint by axis-aligned boxes in the OBB
    local frame, returned as world-space (cx,cy,angle,hx,hy) boxes."""
    cx, cy, ang, hx, hy = obb
    ux, uy = math.cos(ang), math.sin(ang)
    vx, vy = -uy, ux
    local = [(((px - cx) * ux + (py - cy) * uy),
              ((px - cx) * vx + (py - cy) * vy)) for px, py in poly]
    nu = max(1, min(max_cells, int(2 * hx / cell_min)))
    nv = max(1, min(max_cells, int(2 * hy / cell_min)))
    du = (2 * hx) / nu; dv = (2 * hy) / nv
    grid = [[False] * nu for _ in range(nv)]
    for r in range(nv):
        vc = -hy + (r + 0.5) * dv
        for c in range(nu):
            uc = -hx + (c + 0.5) * du
            grid[r][c] = point_in_poly(uc, vc, local)
    rects = []
    open_runs = {}
    def flush(c0, c1, r_end):
        r0 = open_runs.pop((c0, c1))
        rects.append((-hx + c0 * du, -hx + c1 * du, -hy + r0 * dv, -hy + r_end * dv))
    for r in range(nv):
        row = grid[r]; runs = []; c = 0
        while c < nu:
            if not row[c]:
                c += 1; continue
            c0 = c
            while c < nu and row[c]:
                c += 1
            runs.append((c0, c))
        cur = set(runs)
        for key in list(open_runs.keys()):
            if key not in cur:
                flush(key[0], key[1], r)
        for key in runs:
            if key not in open_runs:
                open_runs[key] = r
    for key in list(open_runs.keys()):
        flush(key[0], key[1], nv)
    boxes = []
    for (u0, u1, v0, v1) in rects:
        lu = (u0 + u1) / 2; lv = (v0 + v1) / 2
        wx = cx + lu * ux + lv * vx
        wy = cy + lu * uy + lv * vy
        boxes.append((wx, wy, ang, (u1 - u0) / 2, (v1 - v0) / 2))
    return boxes if boxes else [obb]

def classify(poly, h, kv, out):
    obb = min_area_obb(poly)
    cx, cy, ang, hx, hy = obb
    if hx < 0.5 or hy < 0.5:
        return None
    fill = poly_area(poly) / (4 * hx * hy) if hx * hy > 0 else 1.0
    circ = circularity(poly)
    if circ >= 0.85 and len(poly) >= 8:
        out.append((cx, cy, ang, hx, hy, h, kv, PRIM_CYL)); return "cylinder"
    if fill >= 0.90:
        out.append((cx, cy, ang, hx, hy, h, kv, PRIM_BOX)); return "one-box"
    for (wx, wy, wa, whx, why) in decompose_boxes(poly, obb):
        if whx >= 0.5 and why >= 0.5:
            out.append((wx, wy, wa, whx, why, h, kv, PRIM_BOX))
    return "multi-box"

# --- Pass 1a: collect building outlines (stream tiles, de-dupe by way id) -----
# Each building keeps its polygon, its tag-height (or None), and its name; the
# final height is resolved AFTER parts are loaded so height-less outlines can be
# backfilled from their tallest building:part.
builds = []          # (poly, h_tag_or_None, name)
seen = set()
t0 = time.time()
tiles = sorted(glob.glob(TILE_GLOB))
print("reading %d building tiles ..." % len(tiles), flush=True)
for ti, path in enumerate(tiles):
    try:
        data = json.load(open(path))
    except Exception as ex:
        sys.stderr.write("  skip %s: %s\n" % (path, ex)); continue
    for w in data.get("elements", []):
        if w.get("type") != "way" or "geometry" not in w:
            continue
        wid = w.get("id")
        if wid in seen:
            continue
        seen.add(wid)
        g = w["geometry"]
        if len(g) < 4:
            continue
        closed = (g[0]["lat"] == g[-1]["lat"] and g[0]["lon"] == g[-1]["lon"])
        pts = g[:-1] if closed else g
        poly = [enup(p) for p in pts]
        if len(poly) < 3:
            continue
        t = w.get("tags", {})
        builds.append((poly, height_from_tags(t), t.get("name", "") or ""))
    if (ti + 1) % 40 == 0 or ti + 1 == len(tiles):
        print("  building tiles %d/%d  unique=%d  (%.0fs)"
              % (ti + 1, len(tiles), len(builds), time.time() - t0), flush=True)
print("unique building outlines:", len(builds))

# --- Pass 1b: load building:part heights (de-dupe by way id) -----------------
# NYC's 3-D building import carries real per-tower heights on building:part
# polygons while the enclosing outline is often height-less. We take each part's
# centroid + height; the tallest part inside an outline becomes that building's
# height when it has no tag of its own.
part_pts = []        # (cx, cy, height)
seenp = set()
part_tiles = sorted(glob.glob(os.path.join("tools", "tiles", "part_*.json")))
print("reading %d building:part tiles ..." % len(part_tiles), flush=True)
for path in part_tiles:
    try:
        data = json.load(open(path))
    except Exception as ex:
        sys.stderr.write("  skip %s: %s\n" % (path, ex)); continue
    for w in data.get("elements", []):
        if w.get("type") != "way" or "geometry" not in w:
            continue
        wid = w.get("id")
        if wid in seenp:
            continue
        seenp.add(wid)
        hp = height_from_tags(w.get("tags", {}))
        if hp is None:
            continue
        g = w["geometry"]
        cx = sum((p["lon"] - lon0) * mlon for p in g) / len(g)
        cy = sum((p["lat"] - lat0) * mlat for p in g) / len(g)
        part_pts.append((cx, cy, hp))
print("building:part with height:", len(part_pts))

# --- Pass 1c: match parts to outlines via a uniform grid, take the max height -
GRID_CELL = 120.0
grid = {}
bboxes = []
for i, (poly, _, _) in enumerate(builds):
    xs = [p[0] for p in poly]; ys = [p[1] for p in poly]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    bboxes.append((x0, y0, x1, y1))
    for gx in range(int(math.floor(x0 / GRID_CELL)), int(math.floor(x1 / GRID_CELL)) + 1):
        for gy in range(int(math.floor(y0 / GRID_CELL)), int(math.floor(y1 / GRID_CELL)) + 1):
            grid.setdefault((gx, gy), []).append(i)
part_max = [None] * len(builds)
for (px, py, hp) in part_pts:
    cell = (int(math.floor(px / GRID_CELL)), int(math.floor(py / GRID_CELL)))
    for i in grid.get(cell, ()):
        x0, y0, x1, y1 = bboxes[i]
        if px < x0 or px > x1 or py < y0 or py > y1:
            continue
        if point_in_poly(px, py, builds[i][0]):
            if part_max[i] is None or hp > part_max[i]:
                part_max[i] = hp
            break  # a part belongs to exactly one outline
print("outlines matched to >=1 part:", sum(1 for h in part_max if h is not None))

# --- Pass 1c2: authoritative heights from NYC DoITT footprints ----------------
# Any outline STILL without a height (no tag, no part) is filled from the NYC
# DoITT footprint that contains its centroid (height_roof, FEET -> metres). DoITT
# covers ~100% of NYC buildings, so this resolves the low-rise remainder to real
# heights instead of a flat default. A tiny nearest-centroid fallback catches
# outlines whose centroid lands just outside the DoITT polygon (OSM vs DoITT
# footprints differ slightly).
FT_M = 0.3048

def outer_rings_m(geom):
    if not geom:
        return []
    gt = geom.get("type"); co = geom.get("coordinates")
    polys = co if gt == "MultiPolygon" else ([co] if gt == "Polygon" else [])
    rings = []
    for poly in polys:
        if not poly:
            continue
        pm = [((lon - lon0) * mlon, (lat - lat0) * mlat) for lon, lat in poly[0]]
        if len(pm) >= 3:
            rings.append(pm)
    return rings

doitt = []   # (poly_m, x0, y0, x1, y1, height_m)
nyc_tiles = sorted(glob.glob(os.path.join("tools", "tiles", "nyc_*.geojson")))
print("reading %d NYC height tiles ..." % len(nyc_tiles), flush=True)
for path in nyc_tiles:
    try:
        data = json.load(open(path))
    except Exception as ex:
        sys.stderr.write("  skip %s: %s\n" % (path, ex)); continue
    for f in data.get("features", []):
        hr = f.get("properties", {}).get("height_roof")
        if hr is None:
            continue
        try:
            hm = float(hr) * FT_M
        except Exception:
            continue
        if hm <= 0.0:
            continue
        for pm in outer_rings_m(f.get("geometry")):
            xs = [p[0] for p in pm]; ys = [p[1] for p in pm]
            doitt.append((pm, min(xs), min(ys), max(xs), max(ys), hm))
print("DoITT footprints with height:", len(doitt))

DCELL = 120.0
dgrid = {}
for di, (pm, x0, y0, x1, y1, hm) in enumerate(doitt):
    for gx in range(int(math.floor(x0 / DCELL)), int(math.floor(x1 / DCELL)) + 1):
        for gy in range(int(math.floor(y0 / DCELL)), int(math.floor(y1 / DCELL)) + 1):
            dgrid.setdefault((gx, gy), []).append(di)
# KD-tree of DoITT centroids for the nearest fallback (scipy already imported).
dcent = np.array([((r[1] + r[3]) * 0.5, (r[2] + r[4]) * 0.5) for r in doitt],
                 dtype=np.float64) if doitt else np.empty((0, 2))
dtree = cKDTree(dcent) if len(dcent) else None
NEAR_M = 10.0  # nearest-centroid fallback radius (m)

def centroid_xy(poly):
    return (sum(p[0] for p in poly) / len(poly), sum(p[1] for p in poly) / len(poly))

doitt_h = [None] * len(builds)
need = pip_hit = near_hit = 0
for i, (poly, h_tag, name) in enumerate(builds):
    if h_tag is not None or part_max[i] is not None:
        continue
    need += 1
    cxb, cyb = centroid_xy(poly)
    cell = (int(math.floor(cxb / DCELL)), int(math.floor(cyb / DCELL)))
    for di in dgrid.get(cell, ()):
        pm, x0, y0, x1, y1, hm = doitt[di]
        if cxb < x0 or cxb > x1 or cyb < y0 or cyb > y1:
            continue
        if point_in_poly(cxb, cyb, pm):
            doitt_h[i] = hm; pip_hit += 1; break
    if doitt_h[i] is None and dtree is not None:
        dist, di = dtree.query([cxb, cyb], k=1)
        if dist <= NEAR_M:
            doitt_h[i] = doitt[di][5]; near_hit += 1
print("outlines needing DoITT: %d ; filled (contains=%d, nearest=%d) = %d"
      % (need, pip_hit, near_hit, pip_hit + near_hit))

# --- Pass 1c3: fill the remainder (mostly New Jersey + non-DoITT NYC) from -----
# Overture. tools/overture_heights.csv holds building centroids + height/num_floors
# for the whole bbox; Overture reuses OSM footprints, so an OSM outline still
# without a height matches its Overture twin by nearest centroid almost exactly.
OVERTURE = os.path.join("tools", "overture_heights.csv")
ov_h = [None] * len(builds)
ov_fill = 0
if os.path.exists(OVERTURE):
    ovc = []  # (x, y)
    ovh = []  # height (m)
    with open(OVERTURE) as f:
        header = f.readline().strip().split(",")  # lon,lat,height,num_floors
        for line in f:
            parts = line.rstrip("\n").split(",")
            if len(parts) < 4:
                continue
            try:
                lon = float(parts[0]); lat = float(parts[1])
            except ValueError:
                continue
            h = None
            if parts[2] != "":
                try: h = float(parts[2])
                except ValueError: h = None
            if h is None and parts[3] != "":
                try: h = float(parts[3]) * 3.5   # num_floors -> metres
                except ValueError: h = None
            if h is None or h <= 0.0:
                continue
            ovc.append(((lon - lon0) * mlon, (lat - lat0) * mlat)); ovh.append(h)
    print("Overture heights loaded:", len(ovc))
    if ovc:
        ovh = np.array(ovh)
        otree = cKDTree(np.array(ovc, dtype=np.float64))
        OV_NEAR = 25.0  # nearest-centroid match radius (m); Overture reuses OSM
                        # footprints, so the same building is usually <a few m off,
                        # and 25 m recovers the ML-footprint twins too
        for i, (poly, h_tag, name) in enumerate(builds):
            if h_tag is not None or part_max[i] is not None or doitt_h[i] is not None:
                continue
            cxb, cyb = centroid_xy(poly)
            dist, oi = otree.query([cxb, cyb], k=1)
            if dist <= OV_NEAR:
                ov_h[i] = float(ovh[oi]); ov_fill += 1
    print("remainder filled from Overture:", ov_fill)
else:
    print("NOTE: %s missing -> Overture backfill skipped" % OVERTURE)

# --- Pass 1d: resolve each height from its best source ------------------------
# Priority: OSM tag (often a nice tip/spire height) > tallest building:part >
# DoITT height_roof (authoritative NYC) > Overture (NJ + the rest). Anything still
# unresolved has NO published height in any source (tiny/obscure structures); those
# borrow the nearest resolved building's height so they blend into their block
# instead of standing out as a uniform default stub.
resolved = [None] * len(builds)   # source height (m) or None
src_part = src_doitt = src_ov = 0
for i in range(len(builds)):
    h_tag = builds[i][1]
    if h_tag is not None:
        resolved[i] = h_tag
    elif part_max[i] is not None:
        resolved[i] = part_max[i]; src_part += 1
    elif doitt_h[i] is not None:
        resolved[i] = doitt_h[i]; src_doitt += 1
    elif ov_h[i] is not None:
        resolved[i] = ov_h[i]; src_ov += 1

# Nearest-resolved fallback for the remainder (local estimate, not a flat default).
rc = [centroid_xy(builds[i][0]) for i in range(len(builds)) if resolved[i] is not None]
rh = [resolved[i] for i in range(len(builds)) if resolved[i] is not None]
rtree = cKDTree(np.array(rc, dtype=np.float64)) if rc else None
neighbor_fill = 0
for i in range(len(builds)):
    if resolved[i] is None:
        if rtree is not None:
            _, ri = rtree.query(centroid_xy(builds[i][0]), k=1)
            resolved[i] = float(rh[ri]); neighbor_fill += 1
        else:
            resolved[i] = DEFAULT_H

# Classify with the resolved heights.
instances = []
class_counts = Counter()
nbuild = 0
for i, (poly, h_tag, name) in enumerate(builds):
    h = resolved[i]
    k = kind_of(name, h)
    if k == "Secretariat":
        h = 154.9
    elif k == "UnPlaza":
        h = 7.0
    nbuild += 1
    c = classify(poly, h, KINDVAL[k], instances)
    if c:
        class_counts[c] += 1
print("heights: part=%d  DoITT=%d  Overture=%d  nearest-neighbor=%d"
      % (src_part, src_doitt, src_ov, neighbor_fill))

print("buildings:", nbuild, "-> instances:", len(instances),
      "(avg %.2f prims/building)" % (len(instances) / max(1, nbuild)))
print("representation:", dict(class_counts))

# --- Pass 2: water via coastline right-side rasterisation --------------------
# OSM coastline is oriented land-on-left, so a grid cell is water iff it lies on
# the RIGHT of its nearest coastline segment. The nearest-over-all-segments test
# classifies open water correctly too, not just the near-shore strip.
water_tris = []
if os.path.exists(COAST):
    coast = json.load(open(COAST))
    segs = []
    for w in coast.get("elements", []):
        if w.get("type") != "way" or "geometry" not in w:
            continue
        if w.get("tags", {}).get("natural") != "coastline":
            continue  # side test is only valid for oriented coastline ways
        p = [enup(n) for n in w["geometry"]]
        for i in range(len(p) - 1):
            segs.append((p[i][0], p[i][1], p[i + 1][0], p[i + 1][1]))
    print("coastline segments:", len(segs), flush=True)

    if segs and instances:
        segs = np.array(segs, dtype=np.float64)
        minE = min(min(i[0] - i[3] for i in instances), segs[:, [0, 2]].min())
        maxE = max(max(i[0] + i[3] for i in instances), segs[:, [0, 2]].max())
        minN = min(min(i[1] - i[4] for i in instances), segs[:, [1, 3]].min())
        maxN = max(max(i[1] + i[4] for i in instances), segs[:, [1, 3]].max())
        ncols = max(2, int((maxE - minE) / WATER_CELL))
        nrows = max(2, int((maxN - minN) / WATER_CELL))
        csx = (maxE - minE) / ncols; csy = (maxN - minN) / nrows
        print("water grid: %d x %d cells (%.0f m)" % (ncols, nrows, WATER_CELL),
              flush=True)
        tw0 = time.time()
        # Sample the coastline into oriented points (~12 m apart) and index them
        # in a KD-tree. Each grid cell takes the side (left=land / right=water) of
        # its nearest sample's segment: O((cells+samples) log) instead of the
        # O(cells * segments) full scan, which is essential at 20 km shoreline.
        ax = segs[:, 0]; ay = segs[:, 1]; bx = segs[:, 2]; by = segs[:, 3]
        dx = bx - ax; dy = by - ay
        seglen = np.hypot(dx, dy)
        px_list, py_list, ux_list, uy_list = [], [], [], []
        STEP = 12.0
        for s in range(len(segs)):
            L = seglen[s]
            if L == 0:
                continue
            k = max(1, int(L / STEP))
            ts = (np.arange(k + 1) / k)
            px_list.append(ax[s] + ts * dx[s]); py_list.append(ay[s] + ts * dy[s])
            ux_list.append(np.full(k + 1, dx[s] / L)); uy_list.append(np.full(k + 1, dy[s] / L))
        SPx = np.concatenate(px_list); SPy = np.concatenate(py_list)
        SUx = np.concatenate(ux_list); SUy = np.concatenate(uy_list)
        tree = cKDTree(np.column_stack([SPx, SPy]))
        gx = minE + (np.arange(ncols) + 0.5) * csx
        gy = minN + (np.arange(nrows) + 0.5) * csy
        GX, GY = np.meshgrid(gx, gy)
        flat = np.column_stack([GX.ravel(), GY.ravel()])
        _, nn = tree.query(flat, k=1, workers=-1)
        # Right of the directed nearest segment (cross_z < 0) => water.
        cross = SUx[nn] * (flat[:, 1] - SPy[nn]) - SUy[nn] * (flat[:, 0] - SPx[nn])
        water = (cross < 0.0).reshape(GX.shape)
        print("  raster: %d samples, %d cells, done in %.0fs"
              % (len(SPx), flat.shape[0], time.time() - tw0), flush=True)
        for r in range(nrows):
            row = water[r]; c = 0
            while c < ncols:
                if not row[c]:
                    c += 1; continue
                c0 = c
                while c < ncols and row[c]:
                    c += 1
                x0 = minE + c0 * csx; x1 = minE + c * csx
                y0 = minN + r * csy; y1 = minN + (r + 1) * csy
                water_tris.append(((x0, y0), (x1, y0), (x1, y1)))
                water_tris.append(((x0, y0), (x1, y1), (x0, y1)))
else:
    print("WARNING: %s missing -> no water" % COAST)
print("water triangles:", len(water_tris))

# --- Write binary ------------------------------------------------------------
os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "wb") as f:
    f.write(b"UNMB")
    f.write(struct.pack("<I", 2))
    f.write(struct.pack("<4d", lat0, lon0, mlat, mlon))
    f.write(struct.pack("<II", len(instances), len(water_tris) * 3))
    for (cx, cy, ang, hx, hy, h, k, prim) in instances:
        f.write(struct.pack("<6f2B", cx, cy, ang, hx, hy, h, k, prim))
    for tri in water_tris:
        for (x, y) in tri:
            f.write(struct.pack("<2f", x, y))
print("wrote %s: %d bytes (%d instances, %d water verts)"
      % (OUT, os.path.getsize(OUT), len(instances), len(water_tris) * 3))

# --- Read-back verify --------------------------------------------------------
with open(OUT, "rb") as f:
    assert f.read(4) == b"UNMB"
    ver, = struct.unpack("<I", f.read(4))
    struct.unpack("<4d", f.read(32))
    ni, nw = struct.unpack("<II", f.read(8))
    first = struct.unpack("<6f2B", f.read(26))
    print("verify: version", ver, "instances", ni, "waterVerts", nw)
    print("verify: first cx=%.1f cy=%.1f ang=%.3f hx=%.1f hy=%.1f h=%.1f kind=%d prim=%d"
          % first)
