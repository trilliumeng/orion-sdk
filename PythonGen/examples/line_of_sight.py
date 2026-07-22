#!/usr/bin/env python3
"""Calculate ground position from gimbal line of sight using terrain data.

Requirements:
    pip install requests

Usage:
    python line_of_sight.py <gimbal_ip> [tile_level]
"""

import sys
import math
import struct
import gzip
import time
from pathlib import Path

# Add generated code to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'Communications' / 'python'))

from orion_sdk.packets import GeolocateTelemetryCore, OrionRangeData
from orion_sdk.enums import RangeDataSrc_t
from orion_sdk.connection import OrionConnection

try:
    import requests
except ImportError:
    print("Error: pip install requests")
    sys.exit(1)

# WGS-84 parameters
WGS84_A = 6378137.0
WGS84_E2 = 0.00669437999014
PI = math.pi


def ecef_to_lla(x, y, z):
    """Convert ECEF to LLA (radians)."""
    lon = math.atan2(y, x)
    p = math.sqrt(x*x + y*y)
    lat = math.atan2(z, p * (1 - WGS84_E2))
    for _ in range(5):
        sin_lat = math.sin(lat)
        N = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat * sin_lat)
        lat = math.atan2(z + WGS84_E2 * N * sin_lat, p)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    N = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat * sin_lat)
    alt = p / cos_lat - N if abs(cos_lat) > 1e-10 else abs(z) / abs(sin_lat) - N * (1 - WGS84_E2)
    return (lat, lon, alt)


class TerrainProvider:
    """Provides terrain elevation from Cesium Ion tiles."""

    def __init__(self, level=12, cache_dir="cache"):
        self.level = level
        self.cache_dir = Path(cache_dir)
        self.tiles = {}

    def get_elevation(self, lat, lon):
        """Get terrain elevation at lat/lon (radians). Returns -10000 if not found."""
        scale = PI / (1 << self.level)
        tx, ty = int((lon + PI) / scale), int((lat + PI * 0.5) / scale)
        key = (tx, ty)

        if key not in self.tiles:
            self.tiles[key] = self._load_tile(tx, ty)

        tile = self.tiles[key]
        if not tile:
            return -10000.0

        # Search triangles
        for tri in tile['triangles']:
            a, b, c = tile['verts'][tri[0]], tile['verts'][tri[1]], tile['verts'][tri[2]]
            # Barycentric test
            d00 = (c[0]-a[0])*(c[0]-a[0]) + (c[1]-a[1])*(c[1]-a[1])
            d01 = (c[0]-a[0])*(b[0]-a[0]) + (c[1]-a[1])*(b[1]-a[1])
            d02 = (c[0]-a[0])*(lat-a[0]) + (c[1]-a[1])*(lon-a[1])
            d11 = (b[0]-a[0])*(b[0]-a[0]) + (b[1]-a[1])*(b[1]-a[1])
            d12 = (b[0]-a[0])*(lat-a[0]) + (b[1]-a[1])*(lon-a[1])
            d = d00*d11 - d01*d01
            if abs(d) < 1e-10:
                continue
            u, v = d11*d02 - d01*d12, d00*d12 - d01*d02
            if u >= 0 and v >= 0 and u + v < d:
                return (u*c[2] + v*b[2] + (d-u-v)*a[2]) / d
        return -10000.0

    def _load_tile(self, tx, ty):
        """Load or download a terrain tile."""
        tile_dir = self.cache_dir / str(self.level) / str(tx)
        tile_path = tile_dir / f"{ty}.terrain"

        if not tile_path.exists():
            tile_dir.mkdir(parents=True, exist_ok=True)
            url = f"https://assets.agi.com/stk-terrain/v1/tilesets/world/tiles/{self.level}/{tx}/{ty}.terrain"
            try:
                r = requests.get(url, headers={"Accept": "application/vnd.quantized-mesh"}, timeout=10)
                if r.status_code != 200:
                    return None
                try:
                    data = gzip.decompress(r.content)
                except:
                    data = r.content
                tile_path.write_bytes(data)
            except:
                return None

        return self._parse_tile(tile_path, tx, ty)

    def _parse_tile(self, path, tx, ty):
        """Parse quantized-mesh terrain tile."""
        try:
            data = path.read_bytes()
            header = struct.unpack_from('<3d2f4d3d', data, 0)
            center_lla = ecef_to_lla(header[0], header[1], header[2])
            min_h, max_h = header[3], header[4]
            scale = PI / (1 << self.level)

            offset = 88
            vc = struct.unpack_from('<I', data, offset)[0]
            offset += 4
            u = list(struct.unpack_from(f'<{vc}H', data, offset)); offset += vc*2
            v = list(struct.unpack_from(f'<{vc}H', data, offset)); offset += vc*2
            h = list(struct.unpack_from(f'<{vc}H', data, offset)); offset += vc*2

            verts = []
            ua, va, ha = 0, 0, 0
            for i in range(vc):
                ua += (u[i] >> 1) ^ (-(u[i] & 1))
                va += (v[i] >> 1) ^ (-(v[i] & 1))
                ha += (h[i] >> 1) ^ (-(h[i] & 1))
                lat = center_lla[0] + (va/32767 - 0.5) * scale
                lon = center_lla[1] + (ua/32767 - 0.5) * scale
                alt = ha * (max_h - min_h) / 32767 + min_h
                verts.append([lat, lon, alt])

            triangles = []
            if offset + 4 <= len(data):
                tc = struct.unpack_from('<I', data, offset)[0]
                offset += 4
                idx = list(struct.unpack_from(f'<{tc*3}H', data, offset))
                mx = 0
                for i in range(len(idx)):
                    if idx[i] == 0:
                        idx[i] = mx; mx += 1
                    else:
                        idx[i] = mx - idx[i]
                for i in range(0, len(idx), 3):
                    triangles.append([idx[i], idx[i+1], idx[i+2]])

            return {'verts': verts, 'triangles': triangles}
        except:
            return None


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <gimbal_ip> [tile_level]")
        return 1

    level = int(sys.argv[2]) if len(sys.argv) >= 3 else 12
    terrain = TerrainProvider(level)
    conn = OrionConnection.open_network(sys.argv[1])

    try:
        while True:
            packet = conn.receive(timeout=0.1)
            if not isinstance(packet, GeolocateTelemetryCore):
                continue

            pos = packet.posECEF
            los = packet.losECEF
            mag = math.sqrt(los[0]**2 + los[1]**2 + los[2]**2)
            if mag < 1e-10:
                continue
            los = [l/mag for l in los]

            # Ray march
            target_lla = None
            slant = 0
            prev_above = True
            for dist in range(0, 50000, 50):
                pt = [pos[0] + los[0]*dist, pos[1] + los[1]*dist, pos[2] + los[2]*dist]
                lla = ecef_to_lla(*pt)
                elev = terrain.get_elevation(lla[0], lla[1])
                if elev < -9000:
                    continue
                above = lla[2] > elev
                if not above and prev_above:
                    target_lla = (lla[0], lla[1], elev)
                    slant = dist
                    break
                prev_above = above

            if target_lla:
                msl = target_lla[2] - packet.geoidUndulation
                print(f"\rTARGET: {math.degrees(target_lla[0]):10.6f} {math.degrees(target_lla[1]):11.6f} {msl:6.1f}m, "
                      f"RANGE: {slant:6.0f}m   ", end='')
                range_pkt = OrionRangeData()
                range_pkt.range = slant
                range_pkt.maxAgeMs = 1000
                range_pkt.source = RangeDataSrc_t.RANGE_SRC_OTHER
                conn.send(range_pkt)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print()
    finally:
        conn.close()

    return 0


if __name__ == '__main__':
    sys.exit(main())
