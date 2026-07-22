"""Unit tests for bitfields.py - MSB-first bitfield packing/unpacking.

ProtoGen uses MSB-first (big-endian) bit ordering:
- Bits accumulate from MSB toward LSB
- First field occupies most significant bits

Example: Three fields [A:3, B:3, C:2] pack as:
  Byte 0: [A2 A1 A0 B2 B1 B0 C1 C0]
"""

import pytest

from orion_sdk.runtime.bitfields import (
    BitfieldAccumulator, BitfieldReader,
    pack_bitfields, unpack_bitfields,
    BitfieldSpec
)


class TestBitfieldAccumulator:
    """Test MSB-first bit accumulation."""

    def test_single_byte_full(self):
        """Pack 8 bits into one byte."""
        acc = BitfieldAccumulator()
        acc.add_bits(0xFF, 8)
        assert acc.flush() == b'\xFF'

    def test_single_bit(self):
        """Pack single bit (MSB position)."""
        acc = BitfieldAccumulator()
        acc.add_bits(1, 1)
        # 1 bit in MSB position, rest zeros
        result = acc.flush()
        assert result == b'\x80'  # 10000000

    def test_two_bits(self):
        """Pack two separate bits."""
        acc = BitfieldAccumulator()
        acc.add_bits(1, 1)  # First bit
        acc.add_bits(0, 1)  # Second bit
        acc.add_bits(1, 1)  # Third bit
        # Should be: 10100000 = 0xA0
        result = acc.flush()
        assert result == b'\xA0'

    def test_nibbles(self):
        """Pack two 4-bit values."""
        acc = BitfieldAccumulator()
        acc.add_bits(0xA, 4)  # Upper nibble: 1010
        acc.add_bits(0x5, 4)  # Lower nibble: 0101
        result = acc.flush()
        assert result == b'\xA5'

    def test_cross_byte_boundary(self):
        """Pack field that crosses byte boundary."""
        acc = BitfieldAccumulator()
        acc.add_bits(0b111, 3)    # 3 bits: 111
        acc.add_bits(0b11111, 5)  # 5 bits: 11111 (completes first byte)
        acc.add_bits(0b1111, 4)   # 4 bits: 1111 (into second byte)
        result = acc.flush()
        # Byte 0: 11111111 = 0xFF
        # Byte 1: 11110000 = 0xF0
        assert result == b'\xFF\xF0'

    def test_date_struct(self):
        """Test Date struct bitfield layout from Orion protocol.

        Date has: year(7), month(4), day(5) = 16 bits = 2 bytes
        """
        acc = BitfieldAccumulator()
        acc.add_bits(25, 7)   # year = 25 (2025)
        acc.add_bits(12, 4)   # month = 12
        acc.add_bits(16, 5)   # day = 16

        result = acc.flush()
        assert len(result) == 2

        # Verify by unpacking
        reader = BitfieldReader(result)
        assert reader.read_bits(7) == 25
        assert reader.read_bits(4) == 12
        assert reader.read_bits(5) == 16

    def test_three_bit_fields(self):
        """Test three 3-bit fields followed by 7-bit padding."""
        acc = BitfieldAccumulator()
        acc.add_bits(5, 3)  # 101
        acc.add_bits(3, 3)  # 011
        acc.add_bits(7, 3)  # 111

        result = acc.flush()
        # 101 011 111 0 = 0xAF 0x80... but only need 9 bits
        # Byte 0: 10101111 = 0xAF
        # Byte 1: 10000000 = 0x80 (padded)
        assert result == b'\xAF\x80'


class TestBitfieldAccumulatorSigned:
    """Test signed bitfield encoding."""

    def test_positive_signed(self):
        """Positive value in signed field."""
        acc = BitfieldAccumulator()
        acc.add_signed_bits(5, 4)  # 5 in 4 bits = 0101
        result = acc.flush()
        assert result == b'\x50'  # 01010000

    def test_negative_signed(self):
        """Negative value in signed field (two's complement)."""
        acc = BitfieldAccumulator()
        acc.add_signed_bits(-1, 4)  # -1 in 4 bits = 1111
        result = acc.flush()
        assert result == b'\xF0'  # 11110000

    def test_negative_signed_larger(self):
        """Larger negative value."""
        acc = BitfieldAccumulator()
        acc.add_signed_bits(-5, 4)  # -5 in 4 bits = 1011
        result = acc.flush()
        assert result == b'\xB0'  # 10110000


class TestBitfieldReader:
    """Test MSB-first bit reading."""

    def test_read_full_byte(self):
        """Read 8 bits."""
        reader = BitfieldReader(b'\xAB')
        assert reader.read_bits(8) == 0xAB

    def test_read_nibbles(self):
        """Read two 4-bit values."""
        reader = BitfieldReader(b'\xA5')
        assert reader.read_bits(4) == 0xA
        assert reader.read_bits(4) == 0x5

    def test_read_single_bits(self):
        """Read individual bits."""
        reader = BitfieldReader(b'\xA0')  # 10100000
        assert reader.read_bits(1) == 1
        assert reader.read_bits(1) == 0
        assert reader.read_bits(1) == 1
        assert reader.read_bits(1) == 0

    def test_read_cross_byte(self):
        """Read field crossing byte boundary."""
        reader = BitfieldReader(b'\x0F\xF0')  # 00001111 11110000
        reader.read_bits(4)  # Skip first 4 bits (0000)
        result = reader.read_bits(8)  # Read 8 bits crossing boundary
        assert result == 0xFF  # 11111111

    def test_read_date_struct(self):
        """Test reading Date struct bitfields."""
        # Manually construct Date: year=25, month=12, day=16
        # year(7): 0011001 = 25
        # month(4): 1100 = 12
        # day(5): 10000 = 16
        # Combined: 0011001 1100 10000 = 0x33 0x90
        reader = BitfieldReader(b'\x33\x90')
        year = reader.read_bits(7)
        month = reader.read_bits(4)
        day = reader.read_bits(5)

        assert year == 25
        assert month == 12
        assert day == 16


class TestBitfieldReaderSigned:
    """Test signed bitfield reading with sign extension."""

    def test_read_positive_signed(self):
        """Read positive signed value."""
        reader = BitfieldReader(b'\x50')  # 01010000
        result = reader.read_signed_bits(4)
        assert result == 5

    def test_read_negative_signed(self):
        """Read negative signed value (two's complement)."""
        reader = BitfieldReader(b'\xF0')  # 11110000
        result = reader.read_signed_bits(4)
        assert result == -1

    def test_read_negative_larger(self):
        """Read larger negative value."""
        reader = BitfieldReader(b'\xB0')  # 10110000
        result = reader.read_signed_bits(4)
        assert result == -5


class TestPackUnpackRoundtrip:
    """Test pack/unpack helper functions."""

    def test_roundtrip_simple(self):
        """Simple roundtrip test."""
        fields = [(5, 4, False), (3, 4, False)]  # (value, bits, signed)
        packed = pack_bitfields(fields)

        unpacked = unpack_bitfields(packed, [(4, False), (4, False)])
        assert unpacked == [5, 3]

    def test_roundtrip_mixed_signedness(self):
        """Roundtrip with mixed signed/unsigned."""
        fields = [
            (7, 4, False),   # unsigned 7
            (-3, 4, True),   # signed -3
            (15, 4, False),  # unsigned 15
        ]
        packed = pack_bitfields(fields)

        specs = [(4, False), (4, True), (4, False)]
        unpacked = unpack_bitfields(packed, specs)
        assert unpacked == [7, -3, 15]

    def test_roundtrip_date(self):
        """Roundtrip Date struct fields."""
        fields = [
            (25, 7, False),  # year
            (12, 4, False),  # month
            (16, 5, False),  # day
        ]
        packed = pack_bitfields(fields)

        specs = [(7, False), (4, False), (5, False)]
        unpacked = unpack_bitfields(packed, specs)
        assert unpacked == [25, 12, 16]


class TestOrionBitfieldExamples:
    """Test actual bitfield patterns from Orion protocol."""

    def test_laser_state_flags(self):
        """Test OrionLaserState bitfield flags.

        From XML:
        - Type: enum (8 bits, not bitfield)
        - Enabled: bitfield1
        - Armed: bitfield1
        - Active: bitfield1
        - GroundSpeedLock: bitfield1
        - AltitudeLock: bitfield1
        - PasswordLock: bitfield1
        - ApCommLock: bitfield1
        - ApFlyingLock: bitfield1
        - BypassEnabled: bitfield1
        - Reserved: bitfield7
        """
        acc = BitfieldAccumulator()
        acc.add_bits(1, 1)  # Enabled = 1
        acc.add_bits(1, 1)  # Armed = 1
        acc.add_bits(0, 1)  # Active = 0
        acc.add_bits(0, 1)  # GroundSpeedLock = 0
        acc.add_bits(0, 1)  # AltitudeLock = 0
        acc.add_bits(0, 1)  # PasswordLock = 0
        acc.add_bits(0, 1)  # ApCommLock = 0
        acc.add_bits(0, 1)  # ApFlyingLock = 0
        acc.add_bits(0, 1)  # BypassEnabled = 0
        acc.add_bits(0, 7)  # Reserved = 0

        result = acc.flush()
        assert len(result) == 2

        # Verify
        reader = BitfieldReader(result)
        assert reader.read_bits(1) == 1  # Enabled
        assert reader.read_bits(1) == 1  # Armed
        assert reader.read_bits(1) == 0  # Active

    def test_track_data_flags(self):
        """Test PrimaryTrackData bitfield flags.

        From XML:
        - Reserved0: bitfield6
        - Coasting: bitfield1
        - Active: bitfield1
        """
        acc = BitfieldAccumulator()
        acc.add_bits(0, 6)   # Reserved0 = 0
        acc.add_bits(1, 1)   # Coasting = 1
        acc.add_bits(1, 1)   # Active = 1

        result = acc.flush()
        assert result == b'\x03'  # 00000011

        reader = BitfieldReader(result)
        assert reader.read_bits(6) == 0
        assert reader.read_bits(1) == 1
        assert reader.read_bits(1) == 1
