#!/usr/bin/env python3
"""Capture video frames from an Orion gimbal with KLV metadata.

Requirements:
    pip install av pillow

Usage:
    python video_player.py <gimbal_ip> <video_dest_ip> [port] [record_file.ts]

Controls:
    S - Save snapshot
    Q - Quit
"""

import sys
import socket
import struct
import time
import threading
from pathlib import Path

# Add generated code to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'Communications' / 'python'))

from orion_sdk.packets import OrionNetworkVideo
from orion_sdk.enums import StreamType_t
from orion_sdk.connection import OrionConnection

try:
    import av
    from PIL import Image
except ImportError:
    print("Error: pip install av pillow")
    sys.exit(1)

# KLV parsing
KLV_KEY = bytes([0x06, 0x0E, 0x2B, 0x34, 0x02, 0x0B, 0x01, 0x01, 0x0E, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00])


def parse_klv(data):
    """Parse KLV metadata, returns (lat, lon, alt) in radians/meters or None."""
    idx = data.find(KLV_KEY)
    if idx < 0:
        return None
    offset = idx + 16
    if offset >= len(data):
        return None
    first = data[offset]
    if first < 128:
        length, skip = first, 1
    else:
        nb = first & 0x7F
        if offset + 1 + nb > len(data):
            return None
        length = int.from_bytes(data[offset+1:offset+1+nb], 'big')
        skip = 1 + nb
    offset += skip
    if offset + length > len(data):
        return None

    lat, lon, alt = 0.0, 0.0, 0.0
    pos = offset
    end = offset + length
    while pos < end - 2:
        tag, pos = data[pos], pos + 1
        first = data[pos]
        if first < 128:
            tlen, pos = first, pos + 1
        else:
            nb = first & 0x7F
            tlen = int.from_bytes(data[pos+1:pos+1+nb], 'big')
            pos += 1 + nb
        if pos + tlen > end:
            break
        val = data[pos:pos+tlen]
        pos += tlen
        if tag == 13 and tlen == 4:
            lat = struct.unpack('>i', val)[0] * (90.0 / (2**31-1)) * 0.0174532925
        elif tag == 14 and tlen == 4:
            lon = struct.unpack('>i', val)[0] * (180.0 / (2**31-1)) * 0.0174532925
        elif tag == 15 and tlen == 2:
            alt = struct.unpack('>H', val)[0] / 65535.0 * 19900.0 - 900.0
    return (lat, lon, alt) if lat != 0 or lon != 0 else None


class VideoReceiver:
    def __init__(self, port, record_path=None):
        self.port = port
        self.record_file = open(record_path, 'wb') if record_path else None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', port))
        self.sock.settimeout(0.1)
        self.running = True
        self.metadata = None
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        buf = bytearray()
        while self.running:
            try:
                data, _ = self.sock.recvfrom(65536)
                if self.record_file:
                    self.record_file.write(data)
                buf.extend(data)
                klv = parse_klv(bytes(buf[-4096:]))
                if klv:
                    with self._lock:
                        self.metadata = klv
                if len(buf) > 1024*1024:
                    buf = buf[-512*1024:]
            except socket.timeout:
                pass

    def get_metadata(self):
        with self._lock:
            return self.metadata

    def capture_frame(self):
        try:
            container = av.open(f"udp://@:{self.port}", options={'buffer_size': '1048576'}, timeout=3)
            for frame in container.decode(video=0):
                rgb = frame.to_ndarray(format='rgb24')
                container.close()
                return (rgb, frame.width, frame.height)
            container.close()
        except:
            pass
        return None

    def stop(self):
        self.running = False
        self._thread.join(timeout=1)
        self.sock.close()
        if self.record_file:
            self.record_file.close()


# Keyboard input
if sys.platform == 'win32':
    import msvcrt
    def get_key():
        return msvcrt.getch().decode('utf-8', errors='ignore').lower() if msvcrt.kbhit() else ''
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
    def get_key():
        return sys.stdin.read(1).lower() if select.select([sys.stdin], [], [], 0)[0] else ''


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <gimbal_ip> <video_dest_ip> [port] [record.ts]")
        return 1

    video_port = int(sys.argv[3]) if len(sys.argv) >= 4 else 15004
    record_path = sys.argv[4] if len(sys.argv) >= 5 else None

    conn = OrionConnection.open_network(sys.argv[1])

    # Configure video stream
    settings = OrionNetworkVideo()
    parts = sys.argv[2].split('.')
    settings.DestIp = (int(parts[0]) << 24) | (int(parts[1]) << 16) | (int(parts[2]) << 8) | int(parts[3])
    settings.Port = video_port
    settings.StreamType = StreamType_t.STREAM_TYPE_H264
    conn.send(settings)

    receiver = VideoReceiver(video_port, record_path)
    print(f"Video: udp://@:{video_port}")
    print("Press S to save, Q to quit")

    if sys.platform != 'win32':
        _setup()

    frame_count = 0
    try:
        while True:
            key = get_key()
            if key == 's':
                result = receiver.capture_frame()
                if result:
                    frame_count += 1
                    rgb, w, h = result
                    img = Image.fromarray(rgb)
                    filename = f"{frame_count:05d}.jpg"
                    img.save(filename, 'JPEG', quality=75)
                    meta = receiver.get_metadata()
                    if meta:
                        print(f"\nSaved {filename}: {meta[0]*57.3:.6f}, {meta[1]*57.3:.6f}, {meta[2]:.1f}m")
                    else:
                        print(f"\nSaved {filename}")
            elif key == 'q':
                break

            meta = receiver.get_metadata()
            if meta:
                print(f"\rSensor: {meta[0]*57.3:.4f}, {meta[1]*57.3:.4f}, {meta[2]:.0f}m   ", end='')
            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        if sys.platform != 'win32':
            _restore()
        receiver.stop()
        conn.close()

    return 0


if __name__ == '__main__':
    sys.exit(main())
