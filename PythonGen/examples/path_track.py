#!/usr/bin/env python3
"""Path tracking - gimbal follows waypoints from path.csv.

Usage:
    python path_track.py <gimbal_ip> [step_angle] [cross_steps] [cross_ratio]
"""

import sys
import math
import time
from pathlib import Path

# Add generated code to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'Communications' / 'python'))

from orion_sdk.packets import OrionPath, GeolocateTelemetryCore
from orion_sdk.structs import Point
from orion_sdk.connection import OrionConnection

# WGS-84 ellipsoid
WGS84_A = 6378137.0
WGS84_E2 = 0.00669437999014


def lla_to_ecef(lat, lon, alt):
    """Convert LLA (radians) to ECEF (meters)."""
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    N = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return ((N + alt) * cos_lat * math.cos(lon),
            (N + alt) * cos_lat * math.sin(lon),
            (N * (1.0 - WGS84_E2) + alt) * sin_lat)


def get_path_data(path):
    """Read path points from path.csv."""
    csv_path = Path(__file__).parent / "path.csv"
    try:
        with open(csv_path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split(',')
                if len(parts) >= 3 and path.numPoints < 15:
                    lat, lon, alt = float(parts[0]), float(parts[1]), float(parts[2])
                    ecef = lla_to_ecef(math.radians(lat), math.radians(lon), alt)
                    point = Point()
                    point.posEcef = list(ecef)
                    path.Point.append(point)
                    path.numPoints += 1
                    print(f"POINT {path.numPoints:2d}: {lat:10.6f}, {lon:11.6f}, {alt:.0f}")
    except FileNotFoundError:
        pass
    return path.numPoints


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <gimbal_ip> [step_angle] [cross_steps] [cross_ratio]")
        return 1

    path = OrionPath()
    path.numPoints = 0
    path.Point = []

    # Parse optional arguments
    if len(sys.argv) >= 3: path.alongTrackStepAngle = math.radians(float(sys.argv[2]))
    if len(sys.argv) >= 4: path.numCrossTrackSteps = int(sys.argv[3])
    if len(sys.argv) >= 5: path.crossTrackStepRatio = float(sys.argv[4])

    # Grab the path points from path.csv
    if not get_path_data(path):
        print("Failed to read path data from path.csv")
        return 1

    conn = OrionConnection.open_network(sys.argv[1])

    # Send path to gimbal
    conn.send(path)

    # Monitor path progress
    try:
        while True:
            packet = conn.receive(timeout=0.1)
            if isinstance(packet, GeolocateTelemetryCore):
                print(f"\rPath segment: from point {packet.pathFrom:2d} to point {packet.pathTo:2d} "
                      f"({packet.pathProgress * 100:3.0f}%), stare time = {packet.stareTime:.1f}   ", end='')
            time.sleep(0.02)
    except KeyboardInterrupt:
        print()
    finally:
        conn.close()

    return 0


if __name__ == '__main__':
    sys.exit(main())
