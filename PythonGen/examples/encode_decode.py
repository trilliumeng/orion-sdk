#!/usr/bin/env python3
"""Demonstrates packet encoding and decoding without a gimbal connection.

Usage:
    python encode_decode.py
"""

import sys
import math
from pathlib import Path

# Add generated code to path
_examples_dir = Path(__file__).parent
_generated_path = _examples_dir.parent.parent / 'Communications' / 'python'
sys.path.insert(0, str(_generated_path))

try:
    from orion_sdk.packets import OrionCmd
    from orion_sdk.enums import OrionMode_t
except ImportError as e:
    print(f"Error: Generated protocol code not found: {e}")
    print("Run GenerateOrionPublicPacket.sh from Public directory first.")
    sys.exit(1)


def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def print_cmd(cmd):
    """Print command data to stdout."""
    print(f"Pan: {rad2deg(cmd.Cmd.Target[0]):.1f} deg/s, "
          f"Tilt: {rad2deg(cmd.Cmd.Target[1]):.1f} deg/s, "
          f"ImpulseTime: {cmd.Cmd.ImpulseTime:.1f}, "
          f"Mode: {cmd.Cmd.Mode}, "
          f"Stabilized: {cmd.Cmd.Stabilized}")


def main():
    # Form a command that will tell the gimbal to move at 10 deg/s in pan for one second
    cmd = OrionCmd()
    cmd.Cmd.Target = [deg2rad(10.0), deg2rad(0.0)]
    cmd.Cmd.Mode = OrionMode_t.ORION_MODE_RATE
    cmd.Cmd.ImpulseTime = 1.0
    cmd.Cmd.Stabilized = 0

    # This is how you form a packet
    encoded_bytes = cmd.encode()

    # Decode the packet
    decoded_cmd, _ = OrionCmd.decode(encoded_bytes)

    # Print the command data
    print_cmd(decoded_cmd)

    return 0


if __name__ == '__main__':
    sys.exit(main())
