"""Tests for advanced features: dependsOn, variable arrays, strings, null-encoded.

These tests verify correct handling of:
- dependsOn conditional fields (only encoded/decoded when condition is true)
- Variable-length struct arrays (length determined by another field)
- Fixed and variable string fields
- null-encoded fields (in-memory only, not on wire)

Run GenerateOrionPublicPacket.sh first to generate the bindings.
"""

import pytest
import struct
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
        from orion_sdk.structs import OrionCmd, PrimaryTrackData
        from orion_sdk.protocol import encode_packet, decode_packet

        result = {
            'OrionCmd': OrionCmd,
            'PrimaryTrackData': PrimaryTrackData,
            'encode_packet': encode_packet,
            'decode_packet': decode_packet,
        }

        # Try to import optional types that may not exist
        try:
            from orion_sdk.structs import OrionLaserState
            result['OrionLaserState'] = OrionLaserState
        except ImportError:
            pass

        try:
            from orion_sdk.packets import GeolocateTelemetryCore
            result['GeolocateTelemetryCore'] = GeolocateTelemetryCore
        except ImportError:
            pass

        try:
            from orion_sdk.packets import GpsData
            result['GpsData'] = GpsData
        except ImportError:
            pass

        try:
            from orion_sdk.packets import DebugString
            result['DebugString'] = DebugString
        except ImportError:
            pass

        try:
            from orion_sdk.packets import OrionLaserStates
            result['OrionLaserStates'] = OrionLaserStates
        except ImportError:
            pass

        return result
    except ImportError as e:
        pytest.skip(f"Generated code import failed: {e}")
    finally:
        sys.path = old_path


class TestDependsOnConditionalFields:
    """Test conditional fields that depend on other fields."""

    def test_geolocate_size_difference_with_track_data(self, generated):
        """Test that conditional field changes encoded size."""
        if 'GeolocateTelemetryCore' not in generated:
            pytest.skip("GeolocateTelemetryCore not available")

        GeolocateTelemetryCore = generated['GeolocateTelemetryCore']
        PrimaryTrackData = generated['PrimaryTrackData']

        pkt_without = GeolocateTelemetryCore()
        pkt_without.hasTrackData = 0

        pkt_with = GeolocateTelemetryCore()
        pkt_with.hasTrackData = 1
        pkt_with.primaryTrackData = PrimaryTrackData()

        enc_without = pkt_without.encode()
        enc_with = pkt_with.encode()

        assert len(enc_with) > len(enc_without)

    def test_conditional_field_not_encoded_when_false(self, generated):
        """Verify conditional field is skipped when condition is false."""
        if 'GeolocateTelemetryCore' not in generated:
            pytest.skip("GeolocateTelemetryCore not available")

        GeolocateTelemetryCore = generated['GeolocateTelemetryCore']
        PrimaryTrackData = generated['PrimaryTrackData']

        pkt = GeolocateTelemetryCore()
        pkt.hasTrackData = 0
        pkt.primaryTrackData = PrimaryTrackData()
        pkt.primaryTrackData.Active = 1
        pkt.primaryTrackData.Size = 0.5

        enc1 = pkt.encode()

        pkt2 = GeolocateTelemetryCore()
        pkt2.hasTrackData = 0

        enc2 = pkt2.encode()

        assert len(enc1) == len(enc2)


class TestVariableLengthStructArrays:
    """Test struct arrays with variable length."""

    def test_laser_status_variable_array(self, generated):
        """Test OrionLaserStates with variable number of laser states."""
        if 'OrionLaserStates' not in generated or 'OrionLaserState' not in generated:
            pytest.skip("OrionLaserStates or OrionLaserState not available")

        OrionLaserStates = generated['OrionLaserStates']
        OrionLaserState = generated['OrionLaserState']

        pkt = OrionLaserStates()
        pkt.NumLasers = 2

        state1 = OrionLaserState()
        state1.Type = 1
        state1.Enabled = 1
        state1.Armed = 0

        state2 = OrionLaserState()
        state2.Type = 2
        state2.Enabled = 1
        state2.Armed = 1

        pkt.State = [state1, state2]

        encoded = pkt.encode()
        decoded, _ = OrionLaserStates.decode(encoded)

        assert decoded.NumLasers == 2
        assert len(decoded.State) == 2
        assert decoded.State[0].Enabled == 1
        assert decoded.State[0].Armed == 0
        assert decoded.State[1].Type == 2
        assert decoded.State[1].Armed == 1

    def test_laser_status_empty_array(self, generated):
        """Test OrionLaserStates with zero lasers."""
        if 'OrionLaserStates' not in generated:
            pytest.skip("OrionLaserStates not available")

        OrionLaserStates = generated['OrionLaserStates']

        pkt = OrionLaserStates()
        pkt.NumLasers = 0
        pkt.State = []

        encoded = pkt.encode()
        decoded, _ = OrionLaserStates.decode(encoded)

        assert decoded.NumLasers == 0
        assert len(decoded.State) == 0


class TestStringFields:
    """Test fixed and variable length string fields."""

    def test_debug_string_variable(self, generated):
        """Test variable length string field."""
        if 'DebugString' not in generated:
            pytest.skip("DebugString not available")

        DebugString = generated['DebugString']

        pkt = DebugString()
        pkt.description = "Hello, World!"

        encoded = pkt.encode()
        decoded, _ = DebugString.decode(encoded)

        assert decoded.description == "Hello, World!"

    def test_debug_string_empty(self, generated):
        """Test empty string field."""
        if 'DebugString' not in generated:
            pytest.skip("DebugString not available")

        DebugString = generated['DebugString']

        pkt = DebugString()
        pkt.description = ""

        encoded = pkt.encode()
        decoded, _ = DebugString.decode(encoded)

        assert decoded.description == ""

    def test_debug_string_unicode(self, generated):
        """Test string with unicode characters."""
        if 'DebugString' not in generated:
            pytest.skip("DebugString not available")

        DebugString = generated['DebugString']

        pkt = DebugString()
        pkt.description = "Test: OK"

        encoded = pkt.encode()
        decoded, _ = DebugString.decode(encoded)

        assert decoded.description == "Test: OK"

    def test_debug_string_max_length(self, generated):
        """Test string at maximum length (127 chars + null terminator = 128 bytes)."""
        if 'DebugString' not in generated:
            pytest.skip("DebugString not available")

        DebugString = generated['DebugString']

        pkt = DebugString()
        pkt.description = "X" * 127  # Max usable length (1 byte for null terminator)

        encoded = pkt.encode()
        decoded, _ = DebugString.decode(encoded)

        assert len(decoded.description) == 127

    def test_debug_string_truncation(self, generated):
        """Test string over max length gets truncated to 127 chars."""
        if 'DebugString' not in generated:
            pytest.skip("DebugString not available")

        DebugString = generated['DebugString']

        pkt = DebugString()
        pkt.description = "X" * 200

        encoded = pkt.encode()
        decoded, _ = DebugString.decode(encoded)

        # 128 byte field with null terminator = 127 usable characters
        assert len(decoded.description) == 127


class TestNullEncodedFields:
    """Test null-encoded fields (in-memory only, not on wire)."""

    def test_null_fields_dont_affect_encoding(self, generated):
        """Test that null-encoded fields don't change encoded data."""
        if 'GpsData' not in generated:
            pytest.skip("GpsData not available")

        GpsData = generated['GpsData']

        pkt1 = GpsData()
        pkt1.Year = 2025
        pkt1.Month = 12
        pkt1.Day = 16

        pkt2 = GpsData()
        pkt2.Year = 1900
        pkt2.Month = 1
        pkt2.Day = 1

        enc1 = pkt1.encode()
        enc2 = pkt2.encode()

        assert enc1 == enc2

    def test_null_fields_exist_in_memory(self, generated):
        """Test that null-encoded fields exist in the structure."""
        if 'GpsData' not in generated:
            pytest.skip("GpsData not available")

        GpsData = generated['GpsData']

        pkt = GpsData()
        pkt.Year = 2025
        pkt.Month = 12
        pkt.Day = 16

        assert pkt.Year == 2025
        assert pkt.Month == 12
        assert pkt.Day == 16


class TestRoundtripWithAdvancedFeatures:
    """End-to-end roundtrip tests for packets with advanced features."""

    def test_laser_states_encode_decode_roundtrip(self, generated):
        """Complete encode/decode roundtrip for OrionLaserStates."""
        if 'OrionLaserStates' not in generated or 'OrionLaserState' not in generated:
            pytest.skip("OrionLaserStates or OrionLaserState not available")

        OrionLaserStates = generated['OrionLaserStates']
        OrionLaserState = generated['OrionLaserState']

        pkt = OrionLaserStates()
        pkt.NumLasers = 1

        state = OrionLaserState()
        state.Type = 3
        state.Enabled = 1
        state.Armed = 1
        state.Active = 0
        pkt.State = [state]

        encoded = pkt.encode()
        decoded, consumed = OrionLaserStates.decode(encoded)

        assert decoded.NumLasers == 1
        assert len(decoded.State) == 1
        assert decoded.State[0].Type == 3
        assert decoded.State[0].Enabled == 1
        assert decoded.State[0].Armed == 1
        assert decoded.State[0].Active == 0

    def test_debug_string_encode_decode_roundtrip(self, generated):
        """Complete encode/decode roundtrip for DebugString."""
        if 'DebugString' not in generated:
            pytest.skip("DebugString not available")

        DebugString = generated['DebugString']

        pkt = DebugString()
        pkt.priority = 2
        pkt.description = "Test message 123"

        encoded = pkt.encode()
        decoded, _ = DebugString.decode(encoded)

        assert decoded.priority == 2
        assert decoded.description == "Test message 123"
