#!/usr/bin/env python3
"""Basic gimbal connection and rate commands.

Usage:
    python basic_connection.py <gimbal_ip>
"""

import sys
import math
import time
from pathlib import Path

# Add generated code to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'Communications' / 'python'))

from orion_sdk.packets import OrionCmd
from orion_sdk.enums import OrionMode_t
from orion_sdk.connection import OrionConnection


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <gimbal_ip>")
        return 1

    conn = OrionConnection.open_network(sys.argv[1])

    # Send rate commands (10 deg/s pan for 5 seconds)
    for i in range(50):
        cmd = OrionCmd()
        cmd.Cmd.Target = [math.radians(10.0), 0.0]  # Pan right at 10 deg/s
        cmd.Cmd.Mode = OrionMode_t.ORION_MODE_RATE
        cmd.Cmd.Stabilized = 0
        cmd.Cmd.ImpulseTime = 1.0
        conn.send(cmd)
        time.sleep(0.1)

    # Stop
    cmd = OrionCmd()
    cmd.Cmd.Target = [0.0, 0.0]
    cmd.Cmd.Mode = OrionMode_t.ORION_MODE_RATE
    cmd.Cmd.Stabilized = 1
    cmd.Cmd.ImpulseTime = 0
    conn.send(cmd)

    conn.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
