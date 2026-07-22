#!/usr/bin/env python3
"""Send a geopoint command to an Orion gimbal.

Usage:
    python geo_point.py <gimbal_ip> [LAT LON ALT] [VN VE VD]
"""

import sys
import math
from pathlib import Path

# Add generated code to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'Communications' / 'python'))

from orion_sdk.packets import GeopointCmd
from orion_sdk.enums import geopointOptions
from orion_sdk.connection import OrionConnection


def main():
    # Default geopoint
    pos = [math.radians(45.7), math.radians(-121.5), 30.0]  # lat, lon, alt
    vel = [0.0, 0.0, 0.0]  # NED

    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <gimbal_ip> [LAT LON ALT] [VN VE VD]")
        return 1

    # Parse optional arguments
    if len(sys.argv) >= 3: pos[0] = math.radians(float(sys.argv[2]))
    if len(sys.argv) >= 4: pos[1] = math.radians(float(sys.argv[3]))
    if len(sys.argv) >= 5: pos[2] = float(sys.argv[4])
    if len(sys.argv) >= 6: vel[0] = float(sys.argv[5])
    if len(sys.argv) >= 7: vel[1] = float(sys.argv[6])
    if len(sys.argv) >= 8: vel[2] = float(sys.argv[7])

    print(f"GEOPOINT: Pos = {{ {math.degrees(pos[0]):.5f}, {math.degrees(pos[1]):.5f}, {pos[2]:.1f} }}, "
          f"Vel = {{ {vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f} }}")

    conn = OrionConnection.open_network(sys.argv[1])

    # Form the packet
    cmd = GeopointCmd()
    cmd.targetLat = pos[0]
    cmd.targetLon = pos[1]
    cmd.targetAlt = pos[2]
    cmd.targetVelNED = vel
    cmd.joystickRange = 0
    cmd.options = geopointOptions.geopointNone

    # Send the packet
    conn.send(cmd)

    # Wait for confirmation
    for _ in range(50):
        packet = conn.receive(timeout=0.1)
        if isinstance(packet, GeopointCmd):
            conn.close()
            return 0

    print("Gimbal failed to respond")
    conn.close()
    return 1


if __name__ == '__main__':
    sys.exit(main())
