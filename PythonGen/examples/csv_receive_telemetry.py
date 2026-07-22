#!/usr/bin/env python3
"""Log telemetry to CSV file.

Usage:
    python csv_receive_telemetry.py <gimbal_ip> [output.csv]
"""

import sys
import math
import csv
from pathlib import Path
from datetime import datetime

# Add generated code to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'Communications' / 'python'))

from orion_sdk.packets import GeolocateTelemetryCore, GpsData, OrionDiagnostics
from orion_sdk.connection import OrionConnection


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <gimbal_ip> [output.csv]")
        return 1

    output_file = sys.argv[2] if len(sys.argv) >= 3 else datetime.now().strftime("%H_%M_%S") + ".csv"
    conn = OrionConnection.open_network(sys.argv[1])

    csv_file = open(output_file, 'w', newline='')
    writer = csv.writer(csv_file)
    writer.writerow(['systemTime', 'pan', 'tilt', 'lat', 'lon', 'alt', 'hfov', 'vfov', 'mode'])

    latest_gps = None
    count = 0

    try:
        while True:
            packet = conn.receive(timeout=0.5)
            if packet is None:
                continue

            if isinstance(packet, GeolocateTelemetryCore):
                count += 1
                writer.writerow([
                    packet.systemTime,
                    math.degrees(packet.pan),
                    math.degrees(packet.tilt),
                    math.degrees(packet.posLat),
                    math.degrees(packet.posLon),
                    packet.posAlt,
                    math.degrees(packet.hfov),
                    math.degrees(packet.vfov),
                    packet.mode
                ])
                csv_file.flush()
                if count % 10 == 0:
                    print(f"Logged {count} packets - Pan={math.degrees(packet.pan):7.2f}deg, "
                          f"Tilt={math.degrees(packet.tilt):7.2f}deg")
            elif isinstance(packet, GpsData):
                latest_gps = packet

    except KeyboardInterrupt:
        print(f"\nLogged {count} packets to {output_file}")
    finally:
        csv_file.close()
        conn.close()

    return 0


if __name__ == '__main__':
    sys.exit(main())
