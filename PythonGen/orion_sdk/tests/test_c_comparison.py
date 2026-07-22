"""Byte-for-byte comparison tests against C implementation.

These tests verify that the Python encoding produces identical bytes
to the C implementation (OrionPublicPacket.c).

Test vectors are derived from the C encoding formulas:
- float32ScaledTo2SignedBeBytes(value, scaler) -> round(value * scaler) as big-endian int16
- float32ScaledTo1UnsignedBytes(value, min, scaler) -> round((value - min) * scaler) as uint8

Run GenerateOrionPublicPacket.sh first to generate the bindings.
"""

import pytest
import struct
import math
import sys
from pathlib import Path


@pytest.fixture(scope='module')
def generated():
    """Import generated code from Communications/python/orion_sdk."""
    test_dir = Path(__file__).parent
    generated_path = test_dir.parent.parent.parent / 'Communications' / 'python'

    if not generated_path.exists():
        pytest.skip("Generated code not found. Run GenerateOrionPublicPacket.sh first.")

    old_path = sys.path.copy()
    sys.path.insert(0, str(generated_path))

    mods_to_remove = [k for k in sys.modules if k.startswith('orion_sdk')]
    for mod in mods_to_remove:
        del sys.modules[mod]

    try:
        from orion_sdk.structs import OrionCmd, Date
        from orion_sdk.enums import OrionMode_t
        from orion_sdk.protocol import compute_checksum, build_packet, verify_checksum

        return {
            'OrionCmd': OrionCmd,
            'Date': Date,
            'OrionMode_t': OrionMode_t,
            'compute_checksum': compute_checksum,
            'build_packet': build_packet,
            'verify_checksum': verify_checksum,
        }
    except ImportError as e:
        pytest.skip(f"Generated code import failed: {e}")
    finally:
        sys.path = old_path


def c_float32_scaled_to_signed16_be(value: float, scaler: float) -> bytes:
    """Simulate C's float32ScaledTo2SignedBeBytes function."""
    encoded = int(round(value * scaler))
    encoded = max(-32768, min(32767, encoded))
    return struct.pack('>h', encoded)


def c_float32_scaled_to_unsigned8(value: float, min_val: float, scaler: float) -> bytes:
    """Simulate C's float32ScaledTo1UnsignedBytes function."""
    encoded = int(round((value - min_val) * scaler))
    encoded = max(0, min(255, encoded))
    return struct.pack('>B', encoded)


class TestOrionCmdByteComparison:
    """Compare Python OrionCmd encoding against C implementation byte output."""

    def test_encode_zero_target(self, generated):
        """Test encoding with zero target values."""
        OrionCmd = generated['OrionCmd']

        cmd = OrionCmd()
        cmd.Target = [0.0, 0.0]
        cmd.Mode = 0
        cmd.Stabilized = 0
        cmd.ImpulseTime = 0.0

        encoded = cmd.encode()
        expected = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        assert encoded == expected

    def test_encode_positive_target(self, generated):
        """Test encoding positive target values."""
        OrionCmd = generated['OrionCmd']

        cmd = OrionCmd()
        cmd.Target = [0.5, 0.3]
        cmd.Mode = 16
        cmd.Stabilized = 1
        cmd.ImpulseTime = 0.5

        encoded = cmd.encode()

        expected = bytearray()
        expected += c_float32_scaled_to_signed16_be(0.5, 1000.0)
        expected += c_float32_scaled_to_signed16_be(0.3, 1000.0)
        expected += struct.pack('>B', 16)
        expected += struct.pack('>B', 1)
        expected += c_float32_scaled_to_unsigned8(0.5, 0.0, 10.0)

        assert encoded == bytes(expected)

    def test_encode_negative_target(self, generated):
        """Test encoding negative target values."""
        OrionCmd = generated['OrionCmd']

        cmd = OrionCmd()
        cmd.Target = [-0.5, -1.0]
        cmd.Mode = 80
        cmd.Stabilized = 0
        cmd.ImpulseTime = 2.5

        encoded = cmd.encode()

        expected = bytearray()
        expected += c_float32_scaled_to_signed16_be(-0.5, 1000.0)
        expected += c_float32_scaled_to_signed16_be(-1.0, 1000.0)
        expected += struct.pack('>B', 80)
        expected += struct.pack('>B', 0)
        expected += c_float32_scaled_to_unsigned8(2.5, 0.0, 10.0)

        assert encoded == bytes(expected)

    def test_encode_pi_radians(self, generated):
        """Test encoding pi radians (common angle value)."""
        OrionCmd = generated['OrionCmd']

        cmd = OrionCmd()
        cmd.Target = [math.pi, -math.pi/2]
        cmd.Mode = 0
        cmd.Stabilized = 0
        cmd.ImpulseTime = 0.0

        encoded = cmd.encode()

        expected = bytearray()
        expected += c_float32_scaled_to_signed16_be(math.pi, 1000.0)
        expected += c_float32_scaled_to_signed16_be(-math.pi/2, 1000.0)
        expected += struct.pack('>B', 0)
        expected += struct.pack('>B', 0)
        expected += struct.pack('>B', 0)

        assert encoded == bytes(expected)


class TestDateBitfieldComparison:
    """Compare Date struct bitfield encoding."""

    def test_encode_date_2025_01_01(self, generated):
        """Test encoding Jan 1, 2025."""
        Date = generated['Date']

        d = Date()
        d.year = 25
        d.month = 1
        d.day = 1

        encoded = d.encode()
        decoded, _ = Date.decode(encoded)
        assert decoded.year == 25
        assert decoded.month == 1
        assert decoded.day == 1

    def test_encode_date_2025_12_31(self, generated):
        """Test encoding Dec 31, 2025."""
        Date = generated['Date']

        d = Date()
        d.year = 25
        d.month = 12
        d.day = 31

        encoded = d.encode()
        decoded, _ = Date.decode(encoded)
        assert decoded.year == 25
        assert decoded.month == 12
        assert decoded.day == 31

    def test_encode_date_max_values(self, generated):
        """Test encoding maximum valid values."""
        Date = generated['Date']

        d = Date()
        d.year = 127
        d.month = 12
        d.day = 31

        encoded = d.encode()
        decoded, _ = Date.decode(encoded)
        assert decoded.year == 127
        assert decoded.month == 12
        assert decoded.day == 31


class TestScalingPrecision:
    """Test that scaling precision matches C implementation."""

    def test_scaling_precision_small(self, generated):
        """Test precision for small values."""
        OrionCmd = generated['OrionCmd']

        cmd = OrionCmd()
        cmd.Target = [0.001, 0.001]
        cmd.Mode = 0
        cmd.Stabilized = 0
        cmd.ImpulseTime = 0.0

        encoded = cmd.encode()
        expected_target = struct.pack('>h', 1)
        assert encoded[0:2] == expected_target
        assert encoded[2:4] == expected_target

    def test_scaling_precision_rounding(self, generated):
        """Test rounding behavior."""
        OrionCmd = generated['OrionCmd']

        cmd = OrionCmd()
        cmd.Target = [0.0005, 0.0015]
        cmd.Mode = 0
        cmd.Stabilized = 0
        cmd.ImpulseTime = 0.0

        encoded = cmd.encode()
        decoded, _ = OrionCmd.decode(encoded)
        assert abs(decoded.Target[0] - 0.0005) < 0.001
        assert abs(decoded.Target[1] - 0.0015) < 0.001


class TestChecksum:
    """Verify checksum matches C implementation."""

    def test_checksum_known_values(self, generated):
        """Test checksum with known input/output."""
        compute_checksum = generated['compute_checksum']

        data = bytes([0xD0, 0x0D])
        a, b = compute_checksum(data)
        assert a == 222
        assert b == 180

    def test_checksum_packet(self, generated):
        """Test checksum on a complete packet."""
        build_packet = generated['build_packet']
        verify_checksum = generated['verify_checksum']

        packet = build_packet(packet_id=1, payload=bytes([0x00] * 7))
        assert verify_checksum(packet) is True
        assert packet[0] == 0xD0
        assert packet[1] == 0x0D
        assert packet[2] == 1
        assert packet[3] == 7
