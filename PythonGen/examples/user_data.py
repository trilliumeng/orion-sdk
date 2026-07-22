#!/usr/bin/env python3
"""User data passthrough - send keyboard input through the gimbal.

Usage:
    python user_data.py <gimbal_ip> [port]

    port: 0=Ethernet, 2=Primary RS-232, 3=FP2, 4=FP1
"""

import sys
import time
from pathlib import Path

# Add generated code to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'Communications' / 'python'))

from orion_sdk.packets import OrionUserData
from orion_sdk.enums import UserDataPort_t
from orion_sdk.connection import OrionConnection

# Cross-platform keyboard handling
if sys.platform == 'win32':
    import msvcrt
    def get_key():
        return msvcrt.getch().decode('utf-8', errors='ignore') if msvcrt.kbhit() else ''
else:
    import select, termios, tty
    _old = None
    def _setup():
        global _old
        if not _old:
            _old = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
    def _restore():
        global _old
        if _old:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, _old)
            _old = None
    def get_key():
        return sys.stdin.read(1) if select.select([sys.stdin], [], [], 0)[0] else ''


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <gimbal_ip> [port]")
        return 1

    dest_port = int(sys.argv[2]) if len(sys.argv) >= 3 else UserDataPort_t.USER_DATA_PORT_PRIMARY
    print(f"Sending User Data to Gimbal COM{dest_port}")

    conn = OrionConnection.open_network(sys.argv[1])

    user_out = OrionUserData()
    user_out.port = dest_port
    user_out.id = 0
    buffer = []

    if sys.platform != 'win32':
        _setup()

    try:
        while True:
            # Get keyboard input
            initial_size = len(buffer)
            while True:
                c = get_key()
                if not c or len(buffer) >= 250:
                    break
                buffer.append(ord(c))
                sys.stdout.write(c)
                sys.stdout.flush()

            # Send if we have data and didn't add anything
            if buffer and len(buffer) == initial_size:
                user_out.data = buffer.copy()
                user_out.size = len(buffer)
                conn.send(user_out)
                print(f"\nSending packet {user_out.id}: {user_out.size} byte{'s' if user_out.size != 1 else ''}...")
                user_out.id += 1
                buffer.clear()

            # Check for incoming packets
            packet = conn.receive(timeout=0.05)
            if isinstance(packet, OrionUserData):
                text = bytes(packet.data[:packet.size]).decode('utf-8', errors='replace')
                print(f"Received Packet {packet.id}: {text}")

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        if sys.platform != 'win32':
            _restore()
        conn.close()

    return 0


if __name__ == '__main__':
    sys.exit(main())
