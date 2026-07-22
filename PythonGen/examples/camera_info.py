#!/usr/bin/env python3
"""Request and display camera configuration from an Orion gimbal.

Usage:
    python camera_info.py <gimbal_ip>
"""

import sys
import math
from pathlib import Path

# Add generated code to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'Communications' / 'python'))

from orion_sdk.packets import OrionCameras
from orion_sdk.enums import OrionCameraType_t
from orion_sdk.connection import OrionConnection


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <gimbal_ip>")
        return 1

    conn = OrionConnection.open_network(sys.argv[1])

    # Request the camera settings packet
    conn.send(OrionCameras())

    # Wait for response
    for _ in range(50):
        packet = conn.receive(timeout=0.1)
        if not isinstance(packet, OrionCameras):
            continue

        # Print a header row
        print(" Index  Type     Zoom  WFOV  NFOV")
        print("-" * 34)

        # Loop through each camera
        for i, cam in enumerate(packet.OrionCamSettings):
            if cam.Type == OrionCameraType_t.CAMERA_TYPE_NONE:
                continue

            # Build type string
            type_names = {
                OrionCameraType_t.CAMERA_TYPE_VISIBLE: "Visible",
                OrionCameraType_t.CAMERA_TYPE_SWIR: "SWIR",
                OrionCameraType_t.CAMERA_TYPE_MWIR: "MWIR",
                OrionCameraType_t.CAMERA_TYPE_LWIR: "LWIR",
            }
            type_str = type_names.get(cam.Type, "Unknown")

            # Calculate max zoom ratio
            zoom = cam.MaxFocalLength / cam.MinFocalLength if cam.MinFocalLength > 0 else 1.0

            # Compute wide and narrow horizontal FOV
            array_size = cam.PixelPitch * cam.ArrayWidth
            wfov = 2.0 * math.atan2(0.5 * array_size, cam.MinFocalLength) if cam.MinFocalLength > 0 else 0.0
            nfov = 2.0 * math.atan2(0.5 * array_size, cam.MaxFocalLength) if cam.MaxFocalLength > 0 else 0.0

            print(f" {i:5d}  {type_str:<7s} {zoom:5.1f} {math.degrees(wfov):5.1f} {math.degrees(nfov):5.1f}")

        conn.close()
        return 0

    print("Gimbal failed to respond")
    conn.close()
    return 1


if __name__ == '__main__':
    sys.exit(main())
