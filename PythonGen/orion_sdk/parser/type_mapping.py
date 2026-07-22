"""Type mapping between ProtoGen XML types and Python types.

This module provides mappings for:
- ProtoGen primitive types (signed8, unsigned16, float32, etc.)
- Struct format codes for packing/unpacking
- Python type hints
- Size calculations

ProtoGen uses big-endian byte order throughout.
"""

from typing import NamedTuple, Optional, Dict
from dataclasses import dataclass
import struct


class TypeInfo(NamedTuple):
    """Information about a ProtoGen type."""
    python_type: str           # Python type hint (int, float, bytes, etc.)
    struct_format: str         # struct module format code (big-endian)
    size_bytes: int            # Size in bytes
    is_signed: bool            # True if signed type
    is_float: bool             # True if floating point
    bit_width: int             # Size in bits
    default_value: str         # Default value as Python literal
    is_custom_size: bool = False  # True if non-standard size (e.g., 24-bit)


# Mapping of ProtoGen type names to TypeInfo
# All struct formats use '>' prefix for big-endian
TYPE_MAP: Dict[str, TypeInfo] = {
    # Signed integers
    'signed8': TypeInfo('int', '>b', 1, True, False, 8, '0'),
    'int8': TypeInfo('int', '>b', 1, True, False, 8, '0'),
    'signed16': TypeInfo('int', '>h', 2, True, False, 16, '0'),
    'int16': TypeInfo('int', '>h', 2, True, False, 16, '0'),
    'signed32': TypeInfo('int', '>i', 4, True, False, 32, '0'),
    'int32': TypeInfo('int', '>i', 4, True, False, 32, '0'),
    'signed64': TypeInfo('int', '>q', 8, True, False, 64, '0'),
    'int64': TypeInfo('int', '>q', 8, True, False, 64, '0'),

    # Unsigned integers
    'unsigned8': TypeInfo('int', '>B', 1, False, False, 8, '0'),
    'uint8': TypeInfo('int', '>B', 1, False, False, 8, '0'),
    'unsigned16': TypeInfo('int', '>H', 2, False, False, 16, '0'),
    'uint16': TypeInfo('int', '>H', 2, False, False, 16, '0'),
    'unsigned32': TypeInfo('int', '>I', 4, False, False, 32, '0'),
    'uint32': TypeInfo('int', '>I', 4, False, False, 32, '0'),
    'unsigned64': TypeInfo('int', '>Q', 8, False, False, 64, '0'),
    'uint64': TypeInfo('int', '>Q', 8, False, False, 64, '0'),

    # Floating point
    'float16': TypeInfo('float', '>e', 2, True, True, 16, '0.0'),  # IEEE 754 half-precision
    'float32': TypeInfo('float', '>f', 4, True, True, 32, '0.0'),
    'float': TypeInfo('float', '>f', 4, True, True, 32, '0.0'),
    'float64': TypeInfo('float', '>d', 8, True, True, 64, '0.0'),
    'double': TypeInfo('float', '>d', 8, True, True, 64, '0.0'),

    # Special types
    'bool': TypeInfo('bool', '>B', 1, False, False, 8, 'False'),
}


def get_type_info(type_name: str) -> TypeInfo:
    """Get type information for a ProtoGen type name.

    Args:
        type_name: ProtoGen type like 'signed16', 'float32', etc.

    Returns:
        TypeInfo for the type

    Raises:
        ValueError: If type is unknown
    """
    normalized = type_name.lower().strip()
    if normalized in TYPE_MAP:
        return TYPE_MAP[normalized]

    # Try to parse custom bit widths like 'signed24'
    if normalized.startswith('signed'):
        try:
            bits = int(normalized[6:])
            return _make_custom_signed(bits)
        except ValueError:
            pass
    elif normalized.startswith('unsigned'):
        try:
            bits = int(normalized[8:])
            return _make_custom_unsigned(bits)
        except ValueError:
            pass

    raise ValueError(f"Unknown type: {type_name}")


def _make_custom_signed(bits: int) -> TypeInfo:
    """Create TypeInfo for a custom-width signed integer."""
    size_bytes = (bits + 7) // 8
    # Check if this is a non-standard size (not 8, 16, 32, or 64 bits)
    is_custom = bits not in (8, 16, 32, 64)

    # Use the smallest standard type that fits for struct format
    # Note: For custom sizes like 24-bit, struct will pack/unpack extra bytes
    # The templates must handle this by slicing appropriately
    if bits <= 8:
        fmt = '>b'
    elif bits <= 16:
        fmt = '>h'
    elif bits <= 32:
        fmt = '>i'
    else:
        fmt = '>q'
    return TypeInfo('int', fmt, size_bytes, True, False, bits, '0', is_custom)


def _make_custom_unsigned(bits: int) -> TypeInfo:
    """Create TypeInfo for a custom-width unsigned integer."""
    size_bytes = (bits + 7) // 8
    # Check if this is a non-standard size (not 8, 16, 32, or 64 bits)
    is_custom = bits not in (8, 16, 32, 64)

    if bits <= 8:
        fmt = '>B'
    elif bits <= 16:
        fmt = '>H'
    elif bits <= 32:
        fmt = '>I'
    else:
        fmt = '>Q'
    return TypeInfo('int', fmt, size_bytes, False, False, bits, '0', is_custom)


def is_primitive_type(type_name: str) -> bool:
    """Check if a type name is a primitive (not struct/enum)."""
    normalized = type_name.lower().strip()
    if normalized in TYPE_MAP:
        return True
    # Check for custom bit widths
    if normalized.startswith('signed') or normalized.startswith('unsigned'):
        try:
            int(normalized.replace('signed', '').replace('unsigned', ''))
            return True
        except ValueError:
            pass
    return False


def get_struct_format(types: list) -> str:
    """Build a struct format string for multiple types.

    Args:
        types: List of ProtoGen type names

    Returns:
        Combined struct format string (big-endian)
    """
    if not types:
        return '>'

    formats = []
    for t in types:
        info = get_type_info(t)
        # Strip the '>' prefix since we add it once at the start
        formats.append(info.struct_format[1:])

    return '>' + ''.join(formats)


def compute_size(types: list) -> int:
    """Compute total size in bytes for a list of types.

    Args:
        types: List of ProtoGen type names

    Returns:
        Total size in bytes
    """
    return sum(get_type_info(t).size_bytes for t in types)


@dataclass
class FieldType:
    """Complete type information for a field."""
    base_type: str              # The ProtoGen type name
    type_info: TypeInfo         # TypeInfo for the base type
    is_array: bool = False      # True if array type
    array_length: Optional[int] = None  # Fixed length, or None if variable
    is_variable_array: bool = False  # True if variable-length array
    length_field: Optional[str] = None  # Field that specifies array length
    is_string: bool = False     # True if string type
    string_length: Optional[int] = None  # Max string length
    is_fixed_string: bool = False  # True if fixedstring (null-padded)
    is_bitfield: bool = False   # True if bitfield
    bitfield_bits: int = 0      # Bits if bitfield
    is_null_encoded: bool = False  # True if encodedType="null"


def parse_field_type(
    in_memory_type: str,
    encoded_type: Optional[str] = None,
    array: Optional[str] = None,
    variable_array: Optional[str] = None,
    string: Optional[str] = None,
    fixedstring: Optional[str] = None,
    bitfield_bits: int = 0
) -> FieldType:
    """Parse field attributes into a FieldType.

    Args:
        in_memory_type: The inMemoryType attribute
        encoded_type: The encodedType attribute (may be 'null')
        array: Fixed array length attribute
        variable_array: Variable array length field name
        string: String max length
        fixedstring: Fixed string length
        bitfield_bits: Number of bits if bitfield

    Returns:
        FieldType with all attributes set
    """
    # Handle null-encoded fields
    is_null = encoded_type and encoded_type.lower() == 'null'

    # Determine the actual encoded type
    actual_type = in_memory_type if is_null or not encoded_type else encoded_type

    # Get base type info (may not be primitive for structs)
    try:
        type_info = get_type_info(actual_type)
    except ValueError:
        # Unknown type - likely a struct or enum
        type_info = TypeInfo('object', '', 0, False, False, 0, 'None')

    result = FieldType(
        base_type=actual_type,
        type_info=type_info,
        is_null_encoded=is_null
    )

    # Handle arrays
    if array:
        result.is_array = True
        result.array_length = int(array)
    elif variable_array:
        result.is_array = True
        result.is_variable_array = True
        result.length_field = variable_array

    # Handle strings
    if string:
        result.is_string = True
        result.string_length = int(string)
    elif fixedstring:
        result.is_string = True
        result.is_fixed_string = True
        result.string_length = int(fixedstring)

    # Handle bitfields
    if bitfield_bits > 0:
        result.is_bitfield = True
        result.bitfield_bits = bitfield_bits

    return result


def python_type_hint(field_type: FieldType) -> str:
    """Generate Python type hint for a field.

    Args:
        field_type: The FieldType to generate hint for

    Returns:
        Python type hint string
    """
    base = field_type.type_info.python_type

    if field_type.is_string:
        return 'str'
    elif field_type.is_array:
        return f'List[{base}]'
    else:
        return base


def python_default_value(field_type: FieldType) -> str:
    """Generate Python default value for a field.

    Args:
        field_type: The FieldType

    Returns:
        Python literal for default value
    """
    if field_type.is_string:
        return '""'
    elif field_type.is_array:
        return '[]'
    elif field_type.is_null_encoded:
        return field_type.type_info.default_value
    else:
        return field_type.type_info.default_value


def encode_value(value: any, type_info: TypeInfo) -> bytes:
    """Encode a single value to bytes.

    Args:
        value: Python value to encode
        type_info: Type information

    Returns:
        Encoded bytes
    """
    return struct.pack(type_info.struct_format, value)


def decode_value(data: bytes, type_info: TypeInfo, offset: int = 0) -> tuple:
    """Decode a single value from bytes.

    Args:
        data: Byte buffer
        type_info: Type information
        offset: Byte offset to read from

    Returns:
        Tuple of (decoded_value, bytes_consumed)
    """
    value = struct.unpack_from(type_info.struct_format, data, offset)[0]
    return (value, type_info.size_bytes)


def encode_int24_signed(value: int) -> bytes:
    """Encode a signed 24-bit integer to 3 bytes (big-endian).

    Args:
        value: Signed integer value (-8388608 to 8388607)

    Returns:
        3 bytes in big-endian order
    """
    # Clamp to 24-bit signed range
    value = max(-8388608, min(8388607, value))
    # Pack as 32-bit signed, take last 3 bytes (big-endian)
    packed = struct.pack('>i', value)
    return packed[1:4]  # Skip first byte, keep last 3


def decode_int24_signed(data: bytes, offset: int = 0) -> int:
    """Decode a signed 24-bit integer from 3 bytes (big-endian).

    Args:
        data: Byte buffer
        offset: Byte offset to read from

    Returns:
        Signed integer value
    """
    # Read 3 bytes
    b0 = data[offset]
    b1 = data[offset + 1]
    b2 = data[offset + 2]

    # Sign-extend: if high bit is set, prepend 0xFF, else 0x00
    if b0 & 0x80:
        padded = bytes([0xFF, b0, b1, b2])
    else:
        padded = bytes([0x00, b0, b1, b2])

    return struct.unpack('>i', padded)[0]


def encode_int24_unsigned(value: int) -> bytes:
    """Encode an unsigned 24-bit integer to 3 bytes (big-endian).

    Args:
        value: Unsigned integer value (0 to 16777215)

    Returns:
        3 bytes in big-endian order
    """
    # Clamp to 24-bit unsigned range
    value = max(0, min(16777215, value))
    # Pack as 32-bit unsigned, take last 3 bytes (big-endian)
    packed = struct.pack('>I', value)
    return packed[1:4]  # Skip first byte, keep last 3


def decode_int24_unsigned(data: bytes, offset: int = 0) -> int:
    """Decode an unsigned 24-bit integer from 3 bytes (big-endian).

    Args:
        data: Byte buffer
        offset: Byte offset to read from

    Returns:
        Unsigned integer value
    """
    # Read 3 bytes, prepend 0x00
    b0 = data[offset]
    b1 = data[offset + 1]
    b2 = data[offset + 2]
    padded = bytes([0x00, b0, b1, b2])
    return struct.unpack('>I', padded)[0]


def encode_float16(value: float, sigbits: int = 9) -> bytes:
    """Encode a float32 to ProtoGen's custom float16 format (big-endian).

    This matches C's float32ToFloat16ex() function. ProtoGen uses a non-standard
    float16 with a variable number of significand bits (default 9).

    Format with sigbits=9:
        - 1 sign bit
        - 6 exponent bits (bias = 31)
        - 9 significand bits

    Args:
        value: Float value to encode
        sigbits: Number of significand bits (default 9 to match ProtoGen)

    Returns:
        2 bytes in big-endian order
    """
    # Convert float to its IEEE 754 binary32 representation
    packed = struct.pack('>f', value)
    bits = struct.unpack('>I', packed)[0]

    # Extract components from binary32
    sign = (bits >> 31) & 1
    exponent = (bits >> 23) & 0xFF  # 8-bit biased exponent
    significand = bits & 0x007FFFFF  # 23-bit significand

    # Compute bias for the custom float16 format
    # exponent_bits = 16 - 1 - sigbits = 15 - sigbits
    # bias = 2^(exponent_bits - 1) - 1 = 2^(14 - sigbits) - 1
    bias = (1 << (14 - sigbits)) - 1

    # Reduce significand precision
    reduced_sig = significand >> (23 - sigbits)

    # Handle zero
    if reduced_sig == 0 and exponent == 0:
        return struct.pack('>H', 0x8000 if sign else 0)

    # Convert exponent: unbias from binary32 (127) and rebias for float16
    signed_exp = exponent - 127
    new_exp = signed_exp + bias

    # Handle overflow (exponent too large)
    max_exp = (1 << (15 - sigbits)) - 2  # Reserve all-1s for NaN/Inf
    if new_exp > max_exp:
        # Return max value with correct sign
        output = (sign << 15) | (max_exp << sigbits) | ((1 << sigbits) - 1)
        return struct.pack('>H', output)

    # Handle underflow (exponent too small or negative)
    if new_exp <= 0:
        return struct.pack('>H', 0x8000 if sign else 0)

    # Assemble float16: sign (1) | exponent (15-sigbits) | significand (sigbits)
    output = (sign << 15) | (new_exp << sigbits) | reduced_sig
    return struct.pack('>H', output)


def decode_float16(data: bytes, offset: int = 0, sigbits: int = 9) -> float:
    """Decode ProtoGen's custom float16 format to float32 (big-endian).

    This matches C's float16ToFloat32ex() function.

    Args:
        data: Byte buffer
        offset: Byte offset to read from
        sigbits: Number of significand bits (default 9 to match ProtoGen)

    Returns:
        Decoded float value
    """
    value = struct.unpack_from('>H', data, offset)[0]

    # Handle zero
    if (value & 0x7FFF) == 0:
        return 0.0

    # Extract sign
    sign = (value >> 15) & 1

    # Mask for significand bits
    sig_mask = (1 << sigbits) - 1

    # Extract significand and exponent
    significand = value & sig_mask
    exponent = (value & 0x7FFF) >> sigbits

    # Compute bias
    bias = (1 << (14 - sigbits)) - 1

    # Convert exponent: unbias from float16 and rebias for binary32 (127)
    new_exp = exponent - bias + 127

    # Expand significand from sigbits to 23 bits
    expanded_sig = significand << (23 - sigbits)

    # Assemble binary32: sign (1) | exponent (8) | significand (23)
    bits = (sign << 31) | (new_exp << 23) | expanded_sig

    # Convert to float
    packed = struct.pack('>I', bits)
    return struct.unpack('>f', packed)[0]
