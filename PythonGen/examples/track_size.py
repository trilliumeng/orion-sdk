#!/usr/bin/env python3
"""Increase the object tracking box size on an Orion gimbal.

Usage:
    python track_size.py <gimbal_ip>
"""

import sys
from pathlib import Path

# Add generated code to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'Communications' / 'python'))

from orion_sdk.packets import TrackOptions, GeolocateTelemetryCore
from orion_sdk.connection import OrionConnection


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <gimbal_ip>")
        return 1

    conn = OrionConnection.open_network(sys.argv[1])

    # Request the track options packet
    conn.send(TrackOptions())

    track_options = None
    geo_telem = None

    # Wait for TrackOptions and GeolocateTelemetryCore
    for _ in range(100):
        packet = conn.receive(timeout=0.1)
        if isinstance(packet, TrackOptions):
            track_options = packet
        elif isinstance(packet, GeolocateTelemetryCore):
            geo_telem = packet
        if track_options and geo_telem:
            break

    if not track_options or not geo_telem:
        print("Could not increase track size without receiving necessary settings.")
        conn.close()
        return 1

    # Increase track size by 1%
    if geo_telem.hasTrackData and geo_telem.primaryTrackData.Active:
        track_options.TrackSize = geo_telem.primaryTrackData.Size + 0.01
    else:
        track_options.TrackSize += 0.01

    # Send the Track Options packet
    conn.send(track_options)
    print("Increased object track size.")

    conn.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
