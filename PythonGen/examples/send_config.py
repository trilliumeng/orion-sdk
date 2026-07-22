#!/usr/bin/env python3
"""Upload gimbal configuration from an OrionUi config file.

Usage:
    python send_config.py <gimbal_ip> <config_file.orionconfig>
"""

import sys
import time
from pathlib import Path

# Add generated code to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'Communications' / 'python'))

from orion_sdk.protocol import SYNC_0, SYNC_1, verify_checksum
from orion_sdk.connection import OrionConnection

MAX_RETRIES = 5
ORION_PKT_PRIVATE_20 = 0x20


class RawPacket:
    def __init__(self, packet_id, data):
        self.packet_id = packet_id
        self.data = data
        self.retries = 0
        self.acked = False


def load_packets_from_file(filepath):
    """Load all valid Orion packets from a config file."""
    packets = []
    state = 0
    buffer = bytearray()
    packet_id = 0
    payload_len = 0

    try:
        with open(filepath, 'rb') as f:
            while True:
                byte = f.read(1)
                if not byte:
                    break
                b = byte[0]

                if state == 0:
                    if b == SYNC_0:
                        buffer = bytearray([b])
                        state = 1
                elif state == 1:
                    if b == SYNC_1:
                        buffer.append(b)
                        state = 2
                    else:
                        state = 1 if b == SYNC_0 else 0
                        buffer = bytearray([b]) if b == SYNC_0 else bytearray()
                elif state == 2:
                    buffer.append(b)
                    packet_id = b
                    state = 3
                elif state == 3:
                    buffer.append(b)
                    payload_len = b
                    state = 4
                elif state == 4:
                    buffer.append(b)
                    if len(buffer) >= 4 + payload_len + 2:
                        packet = bytes(buffer)
                        state = 0
                        buffer = bytearray()
                        if verify_checksum(packet):
                            packets.append(RawPacket(packet[2], packet))
                            print(f"Found OrionPacket ID 0x{packet[2]:02x} in file")
    except FileNotFoundError:
        print(f"Failed to open file {filepath}")
        return []

    return packets


def print_progress(done, total):
    if total == 0:
        return
    progress = (done * 60 + (total // 2)) // total
    bar = '=' * progress + ' ' * (60 - progress)
    print(f"\r[{bar}] ({done:2d}/{total:2d})", end='')
    sys.stdout.flush()


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <gimbal_ip> <config_file.orionconfig>")
        return 1

    packets = load_packets_from_file(sys.argv[2])
    if not packets:
        print(f"No Orion packets found in {sys.argv[2]}")
        return 1

    conn = OrionConnection.open_network(sys.argv[1])

    total = len(packets)
    done = 0
    failed = []
    idx = 0

    while idx < len(packets):
        pkt = packets[idx]

        if pkt.acked:
            idx += 1
            continue

        # Don't send 0x20 unless it's the only one left
        remaining = sum(1 for p in packets if not p.acked)
        if pkt.packet_id == ORION_PKT_PRIVATE_20 and remaining > 1:
            idx = (idx + 1) % len(packets)
            continue

        if pkt.retries >= MAX_RETRIES:
            failed.append(pkt)
            pkt.acked = True
            idx += 1
            continue

        conn.send_raw(pkt.data)
        pkt.retries += 1

        # Look for ack
        start = time.time()
        while time.time() - start < 0.1:
            response = conn.receive(timeout=0.05)
            if response and hasattr(response, 'PACKET_ID') and response.PACKET_ID == pkt.packet_id:
                pkt.acked = True
                done += 1
                break

        print_progress(done, total)

        idx += 1
        if idx >= len(packets) and any(not p.acked for p in packets):
            idx = 0

        time.sleep(0.05)

    print()
    conn.close()

    if not failed:
        print("All packets sent successfully!")
    else:
        print("The following packet IDs failed to send:")
        for pkt in failed:
            print(f"  0x{pkt.packet_id:02x}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
