#!/usr/bin/env python3
"""Bake assets/un_map.bin into the compact, tiled map the web viewer streams.

The desktop asset stores every building as six float32s (26 bytes), which
deflates poorly (13 MB -> 9 MB). The browser build instead downloads a
quantized, tile-sorted, column-packed copy, raw-DEFLATE compressed so the wasm
side can inflate it with raylib's DecompressData (sinfl):

    char[4] 'UNMW' | u32 version (1)
    f32 tileSize | f32 originE, originN | u32 tilesX, tilesY      (metres)
    u32 instanceCount | u32 waterVertCount
    u32 tileCount[tilesY * tilesX]          instances per tile, row-major
    instanceCount * u16 qx, then u16 qy     tile-local position, tileSize/65535 m
    instanceCount * u16 qa                  angle mod pi, pi/65536 rad
    instanceCount * u16 hx, hy, h (3 cols)  half-extents and height, centimetres
    instanceCount * u8 kind, then u8 prim
    waterVertCount * i16 x, y               local metres * 2 (0.5 m steps)

Instances are grouped by tile, and within a tile sorted by screen-relevant size
(largest first). That ordering is what lets the viewer's LOD draw a *prefix*
of each tile: the smallest buildings drop out first as the camera pulls away.
Boxes and cylinders are symmetric under a 180-degree turn, so angle mod pi
loses nothing. Quantization error is at most 1.5 cm (position) and 0.5 cm
(size), far below anything visible or relevant to collision.

Usage: gen_web_map.py <in un_map.bin> <out un_map.webbin> [tileSize=1000]
"""

import math
import struct
import sys
import zlib
from array import array

INST = struct.Struct('<6fBB')


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    src, dst = sys.argv[1], sys.argv[2]
    tile = float(sys.argv[3]) if len(sys.argv) > 3 else 1000.0

    data = open(src, 'rb').read()
    if data[:4] != b'UNMB' or struct.unpack_from('<I', data, 4)[0] != 2:
        sys.exit(f'{src}: not a v2 UNMB map')
    n_inst, n_water = struct.unpack_from('<II', data, 40)
    off = 48
    inst = list(INST.iter_unpack(data[off:off + n_inst * INST.size]))
    off += n_inst * INST.size
    water = struct.unpack_from(f'<{n_water * 2}f', data, off)

    # Tile grid over the instance centres.
    min_e = math.floor(min(r[0] for r in inst))
    min_n = math.floor(min(r[1] for r in inst))
    tiles_x = int((max(r[0] for r in inst) - min_e) // tile) + 1
    tiles_y = int((max(r[1] for r in inst) - min_n) // tile) + 1

    def tile_of(r):
        tx = min(int((r[0] - min_e) // tile), tiles_x - 1)
        ty = min(int((r[1] - min_n) // tile), tiles_y - 1)
        return ty * tiles_x + tx, tx, ty

    def size(r):  # largest projected dimension: footprint span or height
        return max(r[5], 2.0 * max(r[3], r[4]))

    keyed = [(tile_of(r), -size(r), r) for r in inst]
    keyed.sort(key=lambda k: (k[0][0], k[1]))

    counts = [0] * (tiles_x * tiles_y)
    qx, qy, qa = array('H'), array('H'), array('H')
    qhx, qhy, qh = array('H'), array('H'), array('H')
    kind, prim = array('B'), array('B')

    def q16(v, step):
        i = int(round(v / step))
        if not 0 <= i <= 65535:
            sys.exit(f'value {v} out of u16 range at step {step}')
        return i

    for (t, tx, ty), _, r in keyed:
        cx, cy, ang, hx, hy, h, k, p = r
        counts[t] += 1
        qx.append(q16(cx - (min_e + tx * tile), tile / 65535.0))
        qy.append(q16(cy - (min_n + ty * tile), tile / 65535.0))
        qa.append(int(round((ang % math.pi) / math.pi * 65536.0)) & 0xFFFF)
        qhx.append(q16(hx, 0.01))
        qhy.append(q16(hy, 0.01))
        qh.append(q16(h, 0.01))
        kind.append(k)
        prim.append(p)

    wq = array('h', (int(round(v * 2.0)) for v in water))

    out = bytearray(b'UNMW')
    out += struct.pack('<IfffII', 1, tile, float(min_e), float(min_n), tiles_x, tiles_y)
    out += struct.pack('<II', n_inst, n_water)
    out += array('I', counts).tobytes()
    for col in (qx, qy, qa, qhx, qhy, qh, kind, prim, wq):
        if sys.byteorder != 'little':
            col.byteswap()
        out += col.tobytes()

    comp = zlib.compressobj(9, zlib.DEFLATED, -15)  # raw DEFLATE (no zlib header)
    blob = comp.compress(bytes(out)) + comp.flush()
    open(dst, 'wb').write(blob)
    print(f'gen_web_map: {n_inst} instances in {tiles_x}x{tiles_y} tiles of '
          f'{tile:.0f} m; {len(data)} -> {len(out)} packed -> {len(blob)} bytes')


if __name__ == '__main__':
    main()
