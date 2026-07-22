"""Unit tests for scaling.py - encode/decode scaling calculations.

CRITICAL: This tests the signed vs unsigned scaling formulas.
Getting this wrong is a common source of bugs.

ProtoGen uses DIFFERENT formulas for signed vs unsigned:
  UNSIGNED: encoded = (value - min) * scaler
  SIGNED:   encoded = value * scaler  (NO min offset!)
"""

import pytest
import math

from orion_sdk.runtime.scaling import (
    encode_scaled_unsigned, decode_scaled_unsigned,
    encode_scaled_signed, decode_scaled_signed,
    encode_scaled, decode_scaled,
    compute_scaler, parse_scaling_from_xml, parse_encoded_type
)


class TestSignedVsUnsigned:
    """Critical tests for signed vs unsigned scaling difference."""

    def test_unsigned_uses_min_offset(self):
        """UNSIGNED encoding subtracts min before scaling."""
        # value=10, min=5, scaler=2
        # encoded = (10 - 5) * 2 = 10
        result = encode_scaled_unsigned(value=10.0, min_val=5.0, scaler=2.0)
        assert result == 10

    def test_signed_no_min_offset(self):
        """SIGNED encoding does NOT subtract min."""
        # value=10, scaler=2
        # encoded = 10 * 2 = 20
        result = encode_scaled_signed(value=10.0, scaler=2.0)
        assert result == 20

    def test_signed_negative_value(self):
        """SIGNED can encode negative values directly."""
        result = encode_scaled_signed(value=-5.0, scaler=100.0)
        assert result == -500

    def test_unsigned_decode_adds_min(self):
        """UNSIGNED decoding adds min back."""
        # encoded=10, min=5, scaler=2
        # value = (10 / 2) + 5 = 10
        result = decode_scaled_unsigned(encoded=10, min_val=5.0, scaler=2.0)
        assert result == pytest.approx(10.0)

    def test_signed_decode_no_min(self):
        """SIGNED decoding does NOT add min."""
        # encoded=20, scaler=2
        # value = 20 / 2 = 10
        result = decode_scaled_signed(encoded=20, scaler=2.0)
        assert result == pytest.approx(10.0)


class TestOrionCmdTarget:
    """Test scaling for OrionCmd.Target field.

    From XML: inMemoryType="float32" encodedType="signed16" scaler="1000.0"
    This is a SIGNED field, so NO min offset!
    """

    def test_encode_positive_angle(self):
        """Encode positive angle (e.g., 0.5 radians pan)."""
        # signed16 with scaler=1000
        # encoded = 0.5 * 1000 = 500
        result = encode_scaled_signed(value=0.5, scaler=1000.0)
        assert result == 500

    def test_encode_negative_angle(self):
        """Encode negative angle (e.g., -0.3 radians tilt)."""
        result = encode_scaled_signed(value=-0.3, scaler=1000.0)
        assert result == -300

    def test_decode_positive(self):
        """Decode positive encoded value."""
        result = decode_scaled_signed(encoded=500, scaler=1000.0)
        assert result == pytest.approx(0.5)

    def test_decode_negative(self):
        """Decode negative encoded value."""
        result = decode_scaled_signed(encoded=-300, scaler=1000.0)
        assert result == pytest.approx(-0.3)

    def test_roundtrip(self):
        """Test encode-decode roundtrip."""
        original = 1.234
        encoded = encode_scaled_signed(original, scaler=1000.0)
        decoded = decode_scaled_signed(encoded, scaler=1000.0)
        assert decoded == pytest.approx(original, rel=1e-3)


class TestGeopointLatLon:
    """Test scaling for GeopointCmd lat/lon fields.

    From XML: inMemoryType="float64" encodedType="signed32" scaler="180*10000000/pi"
    SIGNED field with complex scaler expression.
    """

    @pytest.fixture
    def lat_lon_scaler(self):
        return 180 * 10000000 / math.pi  # ~572957795.13

    def test_encode_zero_latitude(self, lat_lon_scaler):
        """Encode 0 degrees latitude."""
        result = encode_scaled_signed(value=0.0, scaler=lat_lon_scaler)
        assert result == 0

    def test_encode_positive_latitude(self, lat_lon_scaler):
        """Encode 45 degrees N (pi/4 radians)."""
        lat_rad = math.pi / 4  # 45 degrees
        result = encode_scaled_signed(value=lat_rad, scaler=lat_lon_scaler)
        # Expected: (pi/4) * (180*10000000/pi) = 45 * 10000000 = 450000000
        assert result == pytest.approx(450000000, rel=1e-6)

    def test_encode_negative_latitude(self, lat_lon_scaler):
        """Encode 45 degrees S (-pi/4 radians)."""
        lat_rad = -math.pi / 4
        result = encode_scaled_signed(value=lat_rad, scaler=lat_lon_scaler)
        assert result == pytest.approx(-450000000, rel=1e-6)

    def test_decode_roundtrip(self, lat_lon_scaler):
        """Test latitude encode-decode roundtrip."""
        original = math.radians(37.7749)  # San Francisco
        encoded = encode_scaled_signed(original, scaler=lat_lon_scaler)
        decoded = decode_scaled_signed(encoded, scaler=lat_lon_scaler)
        assert decoded == pytest.approx(original, rel=1e-6)


class TestTiltAngle:
    """Test scaling for tilt angle with min/max range.

    From XML: inMemoryType="float32" encodedType="unsigned16" min="-3*pi/2" max="pi/2"
    UNSIGNED field with min offset!
    """

    @pytest.fixture
    def tilt_params(self):
        min_val = -3 * math.pi / 2
        max_val = math.pi / 2
        # scaler = (2^16 - 1) / (max - min)
        range_val = max_val - min_val  # 2*pi
        scaler = 65535 / range_val
        return min_val, max_val, scaler

    def test_encode_min_tilt(self, tilt_params):
        """Encode minimum tilt angle."""
        min_val, max_val, scaler = tilt_params
        result = encode_scaled_unsigned(value=min_val, min_val=min_val, scaler=scaler)
        assert result == 0  # Min value encodes to 0

    def test_encode_max_tilt(self, tilt_params):
        """Encode maximum tilt angle."""
        min_val, max_val, scaler = tilt_params
        result = encode_scaled_unsigned(value=max_val, min_val=min_val, scaler=scaler)
        assert result == pytest.approx(65535, rel=1e-3)

    def test_encode_zero_tilt(self, tilt_params):
        """Encode zero tilt (looking horizontal)."""
        min_val, max_val, scaler = tilt_params
        result = encode_scaled_unsigned(value=0.0, min_val=min_val, scaler=scaler)
        # 0 is 3*pi/2 above min, which is 3/4 of the range
        expected = (0.0 - min_val) * scaler
        assert result == pytest.approx(expected, rel=1e-3)

    def test_decode_roundtrip(self, tilt_params):
        """Test tilt encode-decode roundtrip."""
        min_val, max_val, scaler = tilt_params
        original = -0.5  # Slight downward tilt
        encoded = encode_scaled_unsigned(original, min_val=min_val, scaler=scaler)
        decoded = decode_scaled_unsigned(encoded, min_val=min_val, scaler=scaler)
        assert decoded == pytest.approx(original, rel=1e-3)


class TestComputeScaler:
    """Test automatic scaler computation from min/max."""

    def test_explicit_scaler(self):
        """Explicit scaler is used directly."""
        result = compute_scaler(min_val=0, max_val=100, scaler=50.0,
                               encoded_bits=8, is_signed=False)
        assert result == 50.0

    def test_compute_from_range_unsigned(self):
        """Compute scaler from min/max for unsigned."""
        # 8-bit unsigned: range 0-255
        # Physical range: 0-100
        # scaler = 255 / 100 = 2.55
        result = compute_scaler(min_val=0.0, max_val=100.0, scaler=None,
                               encoded_bits=8, is_signed=False)
        assert result == pytest.approx(2.55)

    def test_compute_from_range_signed(self):
        """Compute scaler from min/max for signed."""
        # 16-bit signed: uses positive range (2^15-1) = 32767 for scaling
        # This ensures symmetric encoding where +max encodes to +32767 and -max to -32767
        result = compute_scaler(min_val=-1.0, max_val=1.0, scaler=None,
                               encoded_bits=16, is_signed=True)
        expected = 32767 / 2.0  # 16383.5
        assert result == pytest.approx(expected)

    def test_zero_range(self):
        """Zero physical range returns scaler of 1."""
        result = compute_scaler(min_val=5.0, max_val=5.0, scaler=None,
                               encoded_bits=8, is_signed=False)
        assert result == 1.0


class TestParseEncodedType:
    """Test type parsing for signedness and bit width."""

    def test_signed16(self):
        is_signed, bits = parse_encoded_type("signed16")
        assert is_signed is True
        assert bits == 16

    def test_unsigned8(self):
        is_signed, bits = parse_encoded_type("unsigned8")
        assert is_signed is False
        assert bits == 8

    def test_signed32(self):
        is_signed, bits = parse_encoded_type("signed32")
        assert is_signed is True
        assert bits == 32

    def test_float32(self):
        is_signed, bits = parse_encoded_type("float32")
        assert is_signed is True
        assert bits == 32

    def test_float64(self):
        is_signed, bits = parse_encoded_type("float64")
        assert is_signed is True
        assert bits == 64


class TestEncodeScaledWithClamping:
    """Test the unified encode_scaled function with clamping."""

    def test_clamp_unsigned_max(self):
        """Values above max are clamped."""
        # 8-bit unsigned, max is 255
        result = encode_scaled(value=1000.0, min_val=0.0, scaler=1.0,
                              is_signed=False, encoded_bits=8)
        assert result == 255

    def test_clamp_unsigned_min(self):
        """Values below min are clamped to 0."""
        result = encode_scaled(value=-10.0, min_val=0.0, scaler=1.0,
                              is_signed=False, encoded_bits=8)
        assert result == 0

    def test_clamp_signed_max(self):
        """Signed values above max are clamped."""
        # 8-bit signed, max is 127
        result = encode_scaled(value=200.0, min_val=0.0, scaler=1.0,
                              is_signed=True, encoded_bits=8)
        assert result == 127

    def test_clamp_signed_min(self):
        """Signed values below min are clamped."""
        # 8-bit signed, min is -128
        result = encode_scaled(value=-200.0, min_val=0.0, scaler=1.0,
                              is_signed=True, encoded_bits=8)
        assert result == -128


class TestDecodeScaledWithSignExtension:
    """Test decode_scaled with sign extension for signed types."""

    def test_sign_extend_negative(self):
        """Test sign extension for negative 8-bit value."""
        # 0xFF as unsigned is 255, but as signed 8-bit is -1
        result = decode_scaled(encoded=255, min_val=0.0, scaler=1.0,
                              is_signed=True, encoded_bits=8)
        assert result == pytest.approx(-1.0)

    def test_no_sign_extend_unsigned(self):
        """Unsigned values are not sign extended."""
        result = decode_scaled(encoded=255, min_val=0.0, scaler=1.0,
                              is_signed=False, encoded_bits=8)
        assert result == pytest.approx(255.0)

    def test_sign_extend_16bit(self):
        """Test sign extension for 16-bit value."""
        # 0xFFFF as unsigned is 65535, as signed is -1
        result = decode_scaled(encoded=65535, min_val=0.0, scaler=1.0,
                              is_signed=True, encoded_bits=16)
        assert result == pytest.approx(-1.0)
