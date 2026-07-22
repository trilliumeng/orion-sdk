"""Integration tests for generated Python bindings.

These tests verify that the generated packet classes work correctly
for encoding and decoding.

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
    # Add generated code path
    test_dir = Path(__file__).parent
    generated_path = test_dir.parent.parent.parent / 'Communications' / 'python'

    if not generated_path.exists():
        pytest.skip("Generated code not found. Run GenerateOrionPublicPacket.sh first.")

    # Temporarily modify sys.path
    old_path = sys.path.copy()
    sys.path.insert(0, str(generated_path))

    # Clear any cached imports of orion_sdk
    mods_to_remove = [k for k in sys.modules if k.startswith('orion_sdk')]
    for mod in mods_to_remove:
        del sys.modules[mod]

    try:
        from orion_sdk.enums import OrionPktType_t, OrionMode_t
        from orion_sdk.structs import OrionCmd, Date, PrimaryTrackData
        from orion_sdk.protocol import (
            PacketRegistry, compute_checksum, verify_checksum,
            build_packet, encode_packet, decode_packet,
            SYNC_0, SYNC_1
        )
        from orion_sdk.packets import OrionCmd as OrionCmdPacket

        return {
            'OrionPktType_t': OrionPktType_t,
            'OrionMode_t': OrionMode_t,
            'OrionCmd': OrionCmd,
            'Date': Date,
            'PrimaryTrackData': PrimaryTrackData,
            'PacketRegistry': PacketRegistry,
            'compute_checksum': compute_checksum,
            'verify_checksum': verify_checksum,
            'build_packet': build_packet,
            'encode_packet': encode_packet,
            'decode_packet': decode_packet,
            'SYNC_0': SYNC_0,
            'SYNC_1': SYNC_1,
            'OrionCmdPacket': OrionCmdPacket,
        }
    except ImportError as e:
        pytest.skip(f"Generated code import failed: {e}")
    finally:
        sys.path = old_path


class TestOrionCmdStruct:
    """Test OrionCmd structure encoding/decoding."""

    def test_encode_basic(self, generated):
        """Encode a basic OrionCmd."""
        OrionCmd = generated['OrionCmd']
        OrionMode_t = generated['OrionMode_t']

        cmd = OrionCmd()
        cmd.Target = [0.5, 0.3]
        cmd.Mode = OrionMode_t.ORION_MODE_RATE
        cmd.Stabilized = 1
        cmd.ImpulseTime = 0.5

        encoded = cmd.encode()
        assert len(encoded) == 7

        target0 = struct.unpack('>h', encoded[0:2])[0]
        assert target0 == 500

        target1 = struct.unpack('>h', encoded[2:4])[0]
        assert target1 == 300

    def test_decode_basic(self, generated):
        """Decode a basic OrionCmd."""
        OrionCmd = generated['OrionCmd']
        OrionMode_t = generated['OrionMode_t']

        encoded = bytes([0x01, 0xF4, 0x01, 0x2C, 0x10, 0x01, 0x05])
        decoded, consumed = OrionCmd.decode(encoded)

        assert consumed == 7
        assert decoded.Target[0] == pytest.approx(0.5, rel=1e-3)
        assert decoded.Target[1] == pytest.approx(0.3, rel=1e-3)
        assert decoded.Mode == OrionMode_t.ORION_MODE_RATE
        assert decoded.Stabilized == 1
        assert decoded.ImpulseTime == pytest.approx(0.5, rel=0.1)

    def test_roundtrip(self, generated):
        """Test encode-decode roundtrip."""
        OrionCmd = generated['OrionCmd']
        OrionMode_t = generated['OrionMode_t']

        original = OrionCmd()
        original.Target = [1.234, -0.567]
        original.Mode = OrionMode_t.ORION_MODE_POSITION
        original.Stabilized = 0
        original.ImpulseTime = 2.5

        encoded = original.encode()
        decoded, _ = OrionCmd.decode(encoded)

        assert decoded.Target[0] == pytest.approx(original.Target[0], rel=1e-3)
        assert decoded.Target[1] == pytest.approx(original.Target[1], rel=1e-3)
        assert decoded.Mode == original.Mode
        assert decoded.Stabilized == original.Stabilized
        assert decoded.ImpulseTime == pytest.approx(original.ImpulseTime, rel=0.1)

    def test_negative_target(self, generated):
        """Test negative target values (signed16)."""
        OrionCmd = generated['OrionCmd']

        cmd = OrionCmd()
        cmd.Target = [-0.5, -1.0]
        cmd.Mode = 0
        cmd.Stabilized = 0
        cmd.ImpulseTime = 0

        encoded = cmd.encode()
        decoded, _ = OrionCmd.decode(encoded)

        assert decoded.Target[0] == pytest.approx(-0.5, rel=1e-3)
        assert decoded.Target[1] == pytest.approx(-1.0, rel=1e-3)


class TestDateStruct:
    """Test Date structure with bitfields."""

    def test_encode_date(self, generated):
        """Encode a date with bitfields."""
        Date = generated['Date']

        d = Date()
        d.year = 25
        d.month = 12
        d.day = 16

        encoded = d.encode()
        assert len(encoded) == 2

    def test_decode_date(self, generated):
        """Decode a date from bitfields."""
        Date = generated['Date']

        d = Date()
        d.year = 25
        d.month = 12
        d.day = 16

        encoded = d.encode()
        decoded, consumed = Date.decode(encoded)

        assert consumed == 2
        assert decoded.year == 25
        assert decoded.month == 12
        assert decoded.day == 16

    def test_roundtrip_various_dates(self, generated):
        """Test roundtrip for various dates."""
        Date = generated['Date']

        test_cases = [
            (0, 1, 1),
            (127, 12, 31),
            (24, 6, 15),
        ]

        for year, month, day in test_cases:
            d = Date()
            d.year = year
            d.month = month
            d.day = day

            encoded = d.encode()
            decoded, _ = Date.decode(encoded)

            assert decoded.year == year
            assert decoded.month == month
            assert decoded.day == day


class TestPrimaryTrackData:
    """Test PrimaryTrackData with mixed fields."""

    def test_encode_track_data(self, generated):
        """Encode track data with scaled fields and bitfields."""
        PrimaryTrackData = generated['PrimaryTrackData']

        track = PrimaryTrackData()
        track.Pos = [0.01, -0.02]
        track.Size = 0.05
        track.Confidence = 0.9
        track.Reserved0 = 0
        track.Coasting = 0
        track.Active = 1

        encoded = track.encode()
        assert len(encoded) == 7

    def test_roundtrip_track_data(self, generated):
        """Test track data roundtrip."""
        PrimaryTrackData = generated['PrimaryTrackData']

        original = PrimaryTrackData()
        original.Pos = [0.05, -0.03]
        original.Size = 0.1
        original.Confidence = 0.75
        original.Reserved0 = 0
        original.Coasting = 1
        original.Active = 1

        encoded = original.encode()
        decoded, _ = PrimaryTrackData.decode(encoded)

        assert decoded.Pos[0] == pytest.approx(original.Pos[0], rel=1e-2)
        assert decoded.Pos[1] == pytest.approx(original.Pos[1], rel=1e-2)
        assert decoded.Size == pytest.approx(original.Size, rel=0.01)
        assert decoded.Confidence == pytest.approx(original.Confidence, rel=0.01)
        assert decoded.Coasting == original.Coasting
        assert decoded.Active == original.Active


class TestChecksumCalculation:
    """Test Fletcher's checksum calculation."""

    def test_checksum_simple(self, generated):
        """Test checksum on simple data."""
        compute_checksum = generated['compute_checksum']

        data = bytes([0x00])
        a, b = compute_checksum(data)
        assert a == 1
        assert b == 1

    def test_checksum_sync_bytes(self, generated):
        """Test checksum on sync bytes."""
        compute_checksum = generated['compute_checksum']
        SYNC_0 = generated['SYNC_0']
        SYNC_1 = generated['SYNC_1']

        data = bytes([SYNC_0, SYNC_1])
        a, b = compute_checksum(data)

        expected_a = (((0xD0 + 1) % 251) + 0x0D) % 251
        expected_b = (((0xD0 + 1) % 251) + expected_a) % 251
        assert a == expected_a
        assert b == expected_b

    def test_verify_valid_packet(self, generated):
        """Verify checksum on a valid packet."""
        build_packet = generated['build_packet']
        verify_checksum = generated['verify_checksum']

        payload = b'\x00\x00\x00\x00'
        packet = build_packet(1, payload)
        assert verify_checksum(packet) is True

    def test_verify_corrupted_packet(self, generated):
        """Verify checksum fails on corrupted packet."""
        build_packet = generated['build_packet']
        verify_checksum = generated['verify_checksum']

        packet = build_packet(1, b'\x00\x00')
        corrupted = bytearray(packet)
        corrupted[4] ^= 0xFF
        corrupted = bytes(corrupted)
        assert verify_checksum(corrupted) is False


class TestPacketBuild:
    """Test packet building with header and checksum."""

    def test_build_packet_structure(self, generated):
        """Verify packet structure."""
        build_packet = generated['build_packet']
        SYNC_0 = generated['SYNC_0']
        SYNC_1 = generated['SYNC_1']

        payload = b'\x01\x02\x03'
        packet = build_packet(packet_id=5, payload=payload)

        assert packet[0] == SYNC_0
        assert packet[1] == SYNC_1
        assert packet[2] == 5
        assert packet[3] == 3
        assert packet[4:7] == payload
        assert len(packet) == 9

    def test_build_empty_packet(self, generated):
        """Build request packet (zero length)."""
        build_packet = generated['build_packet']
        verify_checksum = generated['verify_checksum']

        packet = build_packet(packet_id=10, payload=b'')
        assert packet[2] == 10
        assert packet[3] == 0
        assert len(packet) == 6
        assert verify_checksum(packet) is True


class TestPacketRegistry:
    """Test packet type registry."""

    def test_registry_has_packets(self, generated):
        """Registry should have packet types."""
        PacketRegistry = generated['PacketRegistry']

        ids = PacketRegistry.all_ids()
        assert len(ids) > 0

    def test_get_orion_cmd(self, generated):
        """Get OrionCmd packet class."""
        PacketRegistry = generated['PacketRegistry']
        OrionPktType_t = generated['OrionPktType_t']

        pkt_class = PacketRegistry.get_packet_class(OrionPktType_t.ORION_PKT_CMD)
        assert pkt_class is not None

    def test_get_unknown_packet(self, generated):
        """Unknown packet ID returns None."""
        PacketRegistry = generated['PacketRegistry']

        pkt_class = PacketRegistry.get_packet_class(999)
        assert pkt_class is None

    def test_get_packet_name(self, generated):
        """Get packet name from ID."""
        PacketRegistry = generated['PacketRegistry']
        OrionPktType_t = generated['OrionPktType_t']

        name = PacketRegistry.get_packet_name(OrionPktType_t.ORION_PKT_CMD)
        assert 'Cmd' in name or 'CMD' in name


class TestEncodeDecodePacket:
    """Test full packet encode/decode cycle."""

    def test_encode_orion_cmd_packet(self, generated):
        """Encode OrionCmd to full wire format."""
        OrionCmdPacket = generated['OrionCmdPacket']
        OrionMode_t = generated['OrionMode_t']
        OrionPktType_t = generated['OrionPktType_t']
        encode_packet = generated['encode_packet']
        verify_checksum = generated['verify_checksum']
        SYNC_0 = generated['SYNC_0']
        SYNC_1 = generated['SYNC_1']

        pkt = OrionCmdPacket()
        pkt.Cmd.Target = [0.5, 0.3]
        pkt.Cmd.Mode = OrionMode_t.ORION_MODE_RATE
        pkt.Cmd.Stabilized = 1
        pkt.Cmd.ImpulseTime = 0.5

        wire_data = encode_packet(pkt)

        assert wire_data[0] == SYNC_0
        assert wire_data[1] == SYNC_1
        assert wire_data[2] == OrionPktType_t.ORION_PKT_CMD
        assert verify_checksum(wire_data) is True

    def test_decode_orion_cmd_packet(self, generated):
        """Decode OrionCmd from wire format."""
        OrionCmdPacket = generated['OrionCmdPacket']
        OrionMode_t = generated['OrionMode_t']
        encode_packet = generated['encode_packet']
        decode_packet = generated['decode_packet']

        original = OrionCmdPacket()
        original.Cmd.Target = [1.0, -0.5]
        original.Cmd.Mode = OrionMode_t.ORION_MODE_POSITION
        original.Cmd.Stabilized = 1
        original.Cmd.ImpulseTime = 1.0

        wire_data = encode_packet(original)
        decoded, consumed = decode_packet(wire_data)

        assert decoded is not None
        assert decoded.Cmd.Target[0] == pytest.approx(1.0, rel=1e-3)
        assert decoded.Cmd.Target[1] == pytest.approx(-0.5, rel=1e-3)
        assert decoded.Cmd.Mode == OrionMode_t.ORION_MODE_POSITION
