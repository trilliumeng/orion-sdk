#!/usr/bin/env python3
"""Send external GPS and heading data to an Orion gimbal.

Usage:
    python gps_and_heading.py <gimbal_ip> [LAT LON ALT] [VN VE VD] [HDG]
"""

import sys
import math
from pathlib import Path

# Add generated code to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'Communications' / 'python'))

from orion_sdk.packets import GpsData, OrionExtHeadingData
from orion_sdk.connection import OrionConnection


def main():
    # Defaults
    pos = [math.radians(45.7), math.radians(-121.5), 300.0]  # lat, lon, alt
    vel = [3.0, 22.0, -4.0]  # NED
    heading = math.radians(270.0)
    heading_noise = math.radians(3.0)

    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <gimbal_ip> [LAT LON ALT] [VN VE VD] [HDG]")
        return 1

    # Parse optional arguments
    if len(sys.argv) >= 3: pos[0] = math.radians(float(sys.argv[2]))
    if len(sys.argv) >= 4: pos[1] = math.radians(float(sys.argv[3]))
    if len(sys.argv) >= 5: pos[2] = float(sys.argv[4])
    if len(sys.argv) >= 6: vel[0] = float(sys.argv[5])
    if len(sys.argv) >= 7: vel[1] = float(sys.argv[6])
    if len(sys.argv) >= 8: vel[2] = float(sys.argv[7])
    if len(sys.argv) >= 9: heading = math.radians(float(sys.argv[8]))

    print(f"LLA: {{ {math.degrees(pos[0]):.5f}, {math.degrees(pos[1]):.5f}, {pos[2]:.1f} }}")
    print(f"VEL: {{ {vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f} }}")
    print(f"HDG: {math.degrees(heading):.0f}")

    conn = OrionConnection.open_network(sys.argv[1])

    # Form a GPS data packet
    gps = GpsData()
    gps.Latitude = pos[0]
    gps.Longitude = pos[1]
    gps.Altitude = pos[2]
    gps.VelNED = vel
    gps.PDOP = 2.2
    gps.TrackedSats = 8
    gps.Hacc = 10.0
    gps.Vacc = 20.0
    gps.FixType = 3
    gps.FixState = 1

    # Send the packet
    conn.send(gps)

    # Wait for confirmation
    for _ in range(50):
        if isinstance(conn.receive(timeout=0.1), GpsData):
            break
    else:
        print("Gimbal failed to respond")
        conn.close()
        return 1

    # Now form an external heading packet
    hdg = OrionExtHeadingData()
    hdg.extHeading = heading
    hdg.noise = heading_noise
    hdg.headingInGimbalAxis = 0
    hdg.headingFromAlign = 0
    hdg.pitch = 0.0

    # Send the packet
    conn.send(hdg)

    # Wait for confirmation
    for _ in range(50):
        if isinstance(conn.receive(timeout=0.1), OrionExtHeadingData):
            conn.close()
            return 0

    print("Gimbal failed to respond")
    conn.close()
    return 1


if __name__ == '__main__':
    sys.exit(main())
