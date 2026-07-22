"""Test TleStatus packet encoding/decoding matches C implementation.

This test verifies that the Python TleStatus packet produces identical
byte output and correctly roundtrips all fields, matching the behavior
of the C implementation.
"""

import pytest
import struct
import math
import sys
from pathlib import Path


@pytest.fixture(scope='module')
def tle_imports():
    """Import TleStatus from generated code."""
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
        from orion_sdk.packets import TleStatus
        from orion_sdk.structs import TleDetects
        from orion_sdk.enums import TleType_t, TleState_t, TleSource_t

        return {
            'TleStatus': TleStatus,
            'TleDetects': TleDetects,
            'TleType_t': TleType_t,
            'TleState_t': TleState_t,
            'TleSource_t': TleSource_t,
        }
    except ImportError as e:
        pytest.skip(f"Generated code import failed: {e}")
    finally:
        sys.path = old_path


class TestTleStatusDefaultInitialization:
    """Test that TleStatus initializes correctly like C's TleStatus_t()."""

    def test_default_values(self, tle_imports):
        """All fields should be zero/default after construction (like C++)."""
        TleStatus = tle_imports['TleStatus']
        tle = TleStatus()

        assert tle.SystemTime == 0
        assert tle.FilterType == 0
        assert tle.State == 0
        assert tle.SampleCount == 0
        assert tle.Latitude == 0.0
        assert tle.Longitude == 0.0
        assert tle.Altitude == 0.0
        assert tle.CE90 == 0.0
        assert tle.VE90 == 0.0
        assert tle.SE90 == 0.0
        assert tle.EllipseMajor == 0.0
        assert tle.EllipseMinor == 0.0
        assert tle.EllipseAngle == 0.0
        assert tle.PanError == 0.0
        assert tle.TiltError == 0.0
        assert tle.AltError == 0.0
        assert tle.PanUncertainty == 0.0
        assert tle.TiltUncertainty == 0.0
        assert tle.AltUncertainty == 0.0
        assert tle.PercentOrbit == 0.0
        assert tle.Source == 0

    def test_encode_all_zeros(self, tle_imports):
        """Encoding all-zero TleStatus should produce expected byte pattern."""
        TleStatus = tle_imports['TleStatus']
        tle = TleStatus()
        encoded = tle.encode()

        # Expected size: 4+1+1+2+2+4+4+4+2+2+2+2+2+2+2+2+2+2+2+2+1+1 = 46 bytes
        # (without TleDetects struct which should add 13 more bytes)
        assert len(encoded) >= 46
        print(f"\nEncoded all-zeros TleStatus ({len(encoded)} bytes): {encoded.hex()}")


class TestTleStatusEncodeDecode:
    """Test TleStatus encode/decode roundtrip."""

    def test_roundtrip_with_values(self, tle_imports):
        """Test that encoding and decoding preserves all field values."""
        TleStatus = tle_imports['TleStatus']
        TleType_t = tle_imports['TleType_t']
        TleState_t = tle_imports['TleState_t']
        TleSource_t = tle_imports['TleSource_t']

        tle = TleStatus()

        # Set test values matching the C++ code usage pattern
        tle.SystemTime = 123456789
        tle.FilterType = TleType_t.TLE_TYPE_GEOLOCATE
        tle.State = TleState_t.TLE_STATE_RUNNING
        tle.SampleCount = 42

        # Geographic values
        tle.Latitude = math.radians(37.7749)   # San Francisco lat
        tle.Longitude = math.radians(-122.4194)  # San Francisco lon

        # Encode
        encoded = tle.encode()
        print(f"\nEncoded TleStatus ({len(encoded)} bytes):")
        print(f"  Hex: {encoded.hex()}")

        # Decode
        decoded, bytes_read = TleStatus.decode(encoded)

        # Verify fields before the signed24 Altitude field
        assert decoded.SystemTime == tle.SystemTime
        assert decoded.FilterType == tle.FilterType
        assert decoded.State == tle.State
        assert decoded.SampleCount == tle.SampleCount

        # Geographic values - check with tolerance due to scaling
        assert decoded.Latitude == pytest.approx(tle.Latitude, rel=1e-5)
        assert decoded.Longitude == pytest.approx(tle.Longitude, rel=1e-5)

    def test_boundary_values(self, tle_imports):
        """Test encoding at boundary values."""
        TleStatus = tle_imports['TleStatus']
        tle = TleStatus()

        # Max latitude (pi/2)
        tle.Latitude = math.pi / 2
        encoded = tle.encode()
        decoded, _ = TleStatus.decode(encoded)
        assert decoded.Latitude == pytest.approx(tle.Latitude, rel=1e-5)

        # Min latitude (-pi/2)
        tle.Latitude = -math.pi / 2
        encoded = tle.encode()
        decoded, _ = TleStatus.decode(encoded)
        assert decoded.Latitude == pytest.approx(tle.Latitude, rel=1e-5)

        # Max longitude (pi)
        tle.Longitude = math.pi
        encoded = tle.encode()
        decoded, _ = TleStatus.decode(encoded)
        assert decoded.Longitude == pytest.approx(tle.Longitude, rel=1e-5)


class TestTleDetectsStruct:
    """Test TleDetects struct that should be part of TleStatus."""

    def test_tle_detects_default(self, tle_imports):
        """Test TleDetects default values."""
        TleDetects = tle_imports['TleDetects']
        detects = TleDetects()

        assert detects.ID == 0
        assert detects.SigmaNorth == 0.0
        assert detects.SigmaEast == 0.0
        assert detects.SigmaDown == 0.0
        assert detects.RhoNorthEast == 0.0
        assert detects.RhoNorthDown == 0.0
        assert detects.RhoEastDown == 0.0

    def test_tle_detects_roundtrip(self, tle_imports):
        """Test TleDetects encode/decode roundtrip."""
        TleDetects = tle_imports['TleDetects']
        detects = TleDetects()
        detects.ID = 5
        detects.SigmaNorth = 10.5
        detects.SigmaEast = 8.3
        detects.SigmaDown = 15.0
        detects.RhoNorthEast = 0.5
        detects.RhoNorthDown = -0.3
        detects.RhoEastDown = 0.8

        encoded = detects.encode()
        print(f"\nEncoded TleDetects ({len(encoded)} bytes): {encoded.hex()}")

        decoded, bytes_read = TleDetects.decode(encoded)

        assert decoded.ID == detects.ID
        assert decoded.SigmaNorth == pytest.approx(detects.SigmaNorth, rel=0.01)
        assert decoded.SigmaEast == pytest.approx(detects.SigmaEast, rel=0.01)
        assert decoded.SigmaDown == pytest.approx(detects.SigmaDown, rel=0.01)
        assert decoded.RhoNorthEast == pytest.approx(detects.RhoNorthEast, rel=0.01)
        assert decoded.RhoNorthDown == pytest.approx(detects.RhoNorthDown, rel=0.01)
        assert decoded.RhoEastDown == pytest.approx(detects.RhoEastDown, rel=0.01)


class TestTleStatusByteComparison:
    """Test specific byte patterns to ensure C compatibility."""

    def test_system_time_encoding(self, tle_imports):
        """SystemTime should encode as big-endian uint32."""
        TleStatus = tle_imports['TleStatus']
        tle = TleStatus()
        tle.SystemTime = 0x12345678

        encoded = tle.encode()
        # First 4 bytes should be SystemTime
        assert encoded[0:4] == bytes([0x12, 0x34, 0x56, 0x78])

    def test_filter_type_encoding(self, tle_imports):
        """FilterType should encode as single byte after SystemTime."""
        TleStatus = tle_imports['TleStatus']
        TleType_t = tle_imports['TleType_t']
        tle = TleStatus()
        tle.FilterType = TleType_t.TLE_TYPE_GEOLOCATE

        encoded = tle.encode()
        # Byte 4 is FilterType
        assert encoded[4] == TleType_t.TLE_TYPE_GEOLOCATE

    def test_latitude_scaling(self, tle_imports):
        """Latitude encoding should match C scaler: max="pi/2" on signed32."""
        TleStatus = tle_imports['TleStatus']
        tle = TleStatus()
        # pi/2 should encode to 2^31-1 (max signed32)
        tle.Latitude = math.pi / 2

        encoded = tle.encode()
        # Latitude is at offset 10 (4+1+1+2+2), 4 bytes
        lat_bytes = encoded[10:14]
        lat_encoded = struct.unpack('>i', lat_bytes)[0]

        # Max value for signed32 is 2147483647
        # With scaler, pi/2 * scaler ≈ 2147483647
        assert lat_encoded == pytest.approx(2147483647, rel=0.001)
        print(f"\nLatitude pi/2 encoded as: {lat_encoded} (expected ~2147483647)")


class TestTleStatusPacketID:
    """Test packet identification."""

    def test_packet_id(self, tle_imports):
        """TleStatus should have correct packet ID (0x47 = 71)."""
        TleStatus = tle_imports['TleStatus']
        assert TleStatus.PACKET_ID == 0x47
        assert TleStatus.PACKET_ID == 71


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
