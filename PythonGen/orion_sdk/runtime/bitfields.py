"""Bitfield packing and unpacking for ProtoGen.

ProtoGen uses MSB-first (big-endian) bit ordering within bytes.
Bits accumulate from MSB toward LSB, flushing complete bytes.

Example: Three 3-bit fields [A, B, C] pack as:
  Byte 0: [A2 A1 A0 B2 B1 B0 C2 C1]
  Byte 1: [C0 x  x  x  x  x  x  x ]

The 'bitfieldGroup' attribute groups multiple fields to share
a single underlying storage, affecting how they're accessed but
not the wire format.
"""

from typing import List, Tuple, Optional
from dataclasses import dataclass, field


@dataclass
class BitfieldAccumulator:
    """Accumulates bits for encoding, MSB-first ordering.

    Use this class to pack multiple bitfields into bytes:
        acc = BitfieldAccumulator()
        acc.add_bits(value1, 3)  # 3-bit field
        acc.add_bits(value2, 5)  # 5-bit field
        acc.add_bits(value3, 4)  # 4-bit field
        data = acc.flush()       # Get packed bytes
    """
    _current_byte: int = 0
    _bits_in_byte: int = 0  # Number of bits accumulated (0-7)
    _output: bytearray = field(default_factory=bytearray)

    def add_bits(self, value: int, num_bits: int) -> None:
        """Add bits to the accumulator, MSB-first.

        Args:
            value: Integer value to pack (will be masked to num_bits)
            num_bits: Number of bits to pack (1-32)
        """
        if num_bits <= 0:
            return

        # Mask value to specified bits
        mask = (1 << num_bits) - 1
        value = value & mask

        # Process bits MSB-first
        remaining_bits = num_bits
        while remaining_bits > 0:
            # How many bits can we fit in current byte?
            space_in_byte = 8 - self._bits_in_byte
            bits_to_add = min(remaining_bits, space_in_byte)

            # Extract the topmost bits_to_add bits from value
            shift = remaining_bits - bits_to_add
            bits = (value >> shift) & ((1 << bits_to_add) - 1)

            # Shift bits into position and add to current byte
            self._current_byte |= bits << (space_in_byte - bits_to_add)
            self._bits_in_byte += bits_to_add
            remaining_bits -= bits_to_add

            # Flush byte if complete
            if self._bits_in_byte == 8:
                self._output.append(self._current_byte)
                self._current_byte = 0
                self._bits_in_byte = 0

    def add_signed_bits(self, value: int, num_bits: int) -> None:
        """Add signed bits to the accumulator.

        Handles two's complement representation for negative values.

        Args:
            value: Signed integer value
            num_bits: Number of bits (including sign bit)
        """
        if value < 0:
            # Convert to two's complement
            value = (1 << num_bits) + value
        self.add_bits(value, num_bits)

    def flush(self) -> bytes:
        """Flush remaining bits and return all accumulated bytes.

        Any partial byte is padded with zeros on the right (LSB side).

        Returns:
            Packed bytes
        """
        if self._bits_in_byte > 0:
            self._output.append(self._current_byte)
            self._current_byte = 0
            self._bits_in_byte = 0
        result = bytes(self._output)
        self._output = bytearray()
        return result

    def get_partial_byte_bits(self) -> int:
        """Return number of bits in partial byte (0-7)."""
        return self._bits_in_byte

    def reset(self) -> None:
        """Reset accumulator to empty state."""
        self._current_byte = 0
        self._bits_in_byte = 0
        self._output = bytearray()


@dataclass
class BitfieldReader:
    """Reads bits from a byte sequence, MSB-first ordering.

    Use this class to unpack bitfields from bytes:
        reader = BitfieldReader(data)
        value1 = reader.read_bits(3)   # 3-bit unsigned
        value2 = reader.read_bits(5)   # 5-bit unsigned
        value3 = reader.read_signed_bits(4)  # 4-bit signed
    """
    _data: bytes
    _byte_index: int = 0
    _bit_index: int = 0  # Bit position within current byte (0=MSB, 7=LSB)

    def __init__(self, data: bytes):
        self._data = data
        self._byte_index = 0
        self._bit_index = 0

    def read_bits(self, num_bits: int) -> int:
        """Read unsigned bits from the stream, MSB-first.

        Args:
            num_bits: Number of bits to read (1-32)

        Returns:
            Unsigned integer value
        """
        if num_bits <= 0:
            return 0

        result = 0
        remaining_bits = num_bits

        while remaining_bits > 0 and self._byte_index < len(self._data):
            # How many bits left in current byte?
            bits_left_in_byte = 8 - self._bit_index
            bits_to_read = min(remaining_bits, bits_left_in_byte)

            # Extract bits from current byte
            current_byte = self._data[self._byte_index]
            shift = bits_left_in_byte - bits_to_read
            mask = ((1 << bits_to_read) - 1)
            bits = (current_byte >> shift) & mask

            # Add to result
            result = (result << bits_to_read) | bits

            self._bit_index += bits_to_read
            remaining_bits -= bits_to_read

            # Move to next byte if needed
            if self._bit_index >= 8:
                self._byte_index += 1
                self._bit_index = 0

        return result

    def read_signed_bits(self, num_bits: int) -> int:
        """Read signed bits from the stream.

        Args:
            num_bits: Number of bits (including sign bit)

        Returns:
            Signed integer value
        """
        value = self.read_bits(num_bits)
        # Sign extend if MSB is set
        if value >= (1 << (num_bits - 1)):
            value -= (1 << num_bits)
        return value

    def bytes_consumed(self) -> int:
        """Return number of complete bytes consumed."""
        return self._byte_index + (1 if self._bit_index > 0 else 0)

    def skip_to_byte_boundary(self) -> None:
        """Skip remaining bits in current byte."""
        if self._bit_index > 0:
            self._byte_index += 1
            self._bit_index = 0

    def remaining_bytes(self) -> int:
        """Return number of remaining full bytes."""
        return len(self._data) - self._byte_index


@dataclass
class BitfieldSpec:
    """Specification for a single bitfield within a group."""
    name: str
    num_bits: int
    is_signed: bool = False
    default_value: int = 0


@dataclass
class BitfieldGroup:
    """A group of bitfields that share underlying storage.

    In ProtoGen, bitfieldGroup="name" groups fields together.
    This affects the generated struct layout but not wire format.
    """
    name: str
    fields: List[BitfieldSpec] = field(default_factory=list)

    def total_bits(self) -> int:
        """Return total bits in the group."""
        return sum(f.num_bits for f in self.fields)

    def total_bytes(self) -> int:
        """Return bytes needed (rounded up)."""
        return (self.total_bits() + 7) // 8

    def add_field(self, spec: BitfieldSpec) -> None:
        """Add a field to the group."""
        self.fields.append(spec)


def pack_bitfields(fields: List[Tuple[int, int, bool]]) -> bytes:
    """Pack a list of bitfield values into bytes.

    Args:
        fields: List of (value, num_bits, is_signed) tuples

    Returns:
        Packed bytes
    """
    acc = BitfieldAccumulator()
    for value, num_bits, is_signed in fields:
        if is_signed:
            acc.add_signed_bits(value, num_bits)
        else:
            acc.add_bits(value, num_bits)
    return acc.flush()


def unpack_bitfields(data: bytes, specs: List[Tuple[int, bool]]) -> List[int]:
    """Unpack bitfield values from bytes.

    Args:
        data: Packed bytes
        specs: List of (num_bits, is_signed) tuples

    Returns:
        List of unpacked values
    """
    reader = BitfieldReader(data)
    result = []
    for num_bits, is_signed in specs:
        if is_signed:
            result.append(reader.read_signed_bits(num_bits))
        else:
            result.append(reader.read_bits(num_bits))
    return result


def generate_bitfield_encoder(
    fields: List[BitfieldSpec],
    var_prefix: str = "self.",
    indent: str = "        "
) -> str:
    """Generate Python code for encoding bitfields.

    Args:
        fields: List of bitfield specifications
        var_prefix: Prefix for field variables (e.g., "self." or "")
        indent: Indentation string

    Returns:
        Python code string
    """
    lines = [f"{indent}acc = BitfieldAccumulator()"]
    for f in fields:
        if f.is_signed:
            lines.append(f"{indent}acc.add_signed_bits({var_prefix}{f.name}, {f.num_bits})")
        else:
            lines.append(f"{indent}acc.add_bits({var_prefix}{f.name}, {f.num_bits})")
    lines.append(f"{indent}data.extend(acc.flush())")
    return "\n".join(lines)


def generate_bitfield_decoder(
    fields: List[BitfieldSpec],
    var_prefix: str = "self.",
    indent: str = "        "
) -> str:
    """Generate Python code for decoding bitfields.

    Args:
        fields: List of bitfield specifications
        var_prefix: Prefix for field variables
        indent: Indentation string

    Returns:
        Python code string
    """
    total_bytes = (sum(f.num_bits for f in fields) + 7) // 8
    lines = [
        f"{indent}reader = BitfieldReader(data[offset:offset+{total_bytes}])",
    ]
    for f in fields:
        if f.is_signed:
            lines.append(f"{indent}{var_prefix}{f.name} = reader.read_signed_bits({f.num_bits})")
        else:
            lines.append(f"{indent}{var_prefix}{f.name} = reader.read_bits({f.num_bits})")
    lines.append(f"{indent}offset += {total_bytes}")
    return "\n".join(lines)
