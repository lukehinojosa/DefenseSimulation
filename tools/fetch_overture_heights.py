"""Export building heights from Overture Maps over the 10 km UN bbox.

DoITT covers NYC only; ~49k buildings in the fetch bbox are across the Hudson in
New Jersey (plus small NYC structures DoITT omits), so after the DoITT backfill a
large remainder still has no height. Overture's buildings theme merges OSM +
Microsoft ML + Esri and carries a `height` (metres, ML-derived where a survey
height is absent) for most US buildings -- covering NJ and the non-DoITT NYC
structures in one source.

We run a single bbox query against the public Overture parquet on S3 (anonymous,
no credentials) via DuckDB and write a compact CSV of building CENTROIDS +
height/num_floors. gen_map_bin.py matches any still-height-less OSM outline to the
nearest Overture centroid. Overture reuses OSM footprints where OSM has them, so
those centroids line up almost exactly.

Output: tools/overture_heights.csv  (lon,lat,height,num_floors)
"""
import math, os, sys, time

RELEASE = "2026-08-19.0"   # latest Overture release (docs.overturemaps.org)
CENTER_LAT, CENTER_LON = 40.7489, -73.9680
RADIUS_M = 10000.0
MLAT = 111132.0
MLON = 111320.0 * math.cos(math.radians(CENTER_LAT))
S, N = CENTER_LAT - RADIUS_M / MLAT, CENTER_LAT + RADIUS_M / MLAT
W, E = CENTER_LON - RADIUS_M / MLON, CENTER_LON + RADIUS_M / MLON
OUT = os.path.join("tools", "overture_heights.csv")


def main():
    import duckdb
    con = duckdb.connect()
    con.execute("INSTALL httpfs; LOAD httpfs; INSTALL spatial; LOAD spatial; "
                "SET s3_region='us-west-2';")
    src = ("read_parquet('s3://overturemaps-us-west-2/release/%s/"
           "theme=buildings/type=building/*', hive_partitioning=1)" % RELEASE)
    # bbox-overlap predicate (uses the per-row bbox struct for row-group pruning);
    # keep only rows that actually carry a height signal.
    q = """COPY (
        SELECT ST_X(ST_Centroid(geometry)) AS lon,
               ST_Y(ST_Centroid(geometry)) AS lat,
               height, num_floors
        FROM {src}
        WHERE bbox.xmin < {E} AND bbox.xmax > {W}
          AND bbox.ymin < {N} AND bbox.ymax > {S}
          AND (height IS NOT NULL OR num_floors IS NOT NULL)
    ) TO '{out}' (FORMAT csv, HEADER)""".format(
        src=src, W=W, E=E, S=S, N=N, out=OUT.replace("\\", "/"))
    print("bbox S=%.5f W=%.5f N=%.5f E=%.5f  release=%s" % (S, W, N, E, RELEASE),
          flush=True)
    print("querying Overture buildings (one full-bbox scan) ...", flush=True)
    t = time.time()
    con.execute(q)
    n = con.execute("SELECT count(*) FROM read_csv('%s')"
                    % OUT.replace("\\", "/")).fetchone()[0]
    print("wrote %s: %d rows in %.0fs" % (OUT, n, time.time() - t), flush=True)


if __name__ == "__main__":
    main()
