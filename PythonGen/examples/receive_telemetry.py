#!/usr/bin/env python3
"""Receive and display telemetry from an Orion gimbal.

Usage:
    python receive_telemetry.py <gimbal_ip>
"""

import sys
import math
from pathlib import Path

# Add generated code to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'Communications' / 'python'))

from orion_sdk.packets import GeolocateTelemetryCore, GpsData, OrionCmd
from orion_sdk.connection import OrionConnection


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <gimbal_ip>")
        return 1

    conn = OrionConnection.open_network(sys.argv[1])

    try:
        while True:
            packet = conn.receive(timeout=0.5)
            if packet is None:
                continue

            if isinstance(packet, GeolocateTelemetryCore):
                print(f"Pan={math.degrees(packet.pan):7.2f}deg, "
                      f"Tilt={math.degrees(packet.tilt):7.2f}deg, "
                      f"Lat={math.degrees(packet.posLat):10.5f}, "
                      f"Lon={math.degrees(packet.posLon):10.5f}, "
                      f"Alt={packet.posAlt:7.1f}m")
            elif isinstance(packet, GpsData):
                print(f"GPS: Lat={math.degrees(packet.Latitude):9.5f}, "
                      f"Lon={math.degrees(packet.Longitude):10.5f}, "
                      f"Alt={packet.Altitude:7.1f}m")
            elif isinstance(packet, OrionCmd):
                print(f"Cmd: Mode={packet.Cmd.Mode}, "
                      f"Target={[math.degrees(t) for t in packet.Cmd.Target]}")

    except KeyboardInterrupt:
        pass
    finally:
        conn.close()

    return 0


if __name__ == '__main__':
    sys.exit(main())
