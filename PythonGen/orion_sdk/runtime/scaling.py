"""Scaling calculations for ProtoGen encoded fields.

This module handles the critical distinction between signed and unsigned
scaling formulas used by ProtoGen:

UNSIGNED: encoded = (value - min) * scaler
SIGNED:   encoded = value * scaler  (NO min offset!)

This is a common source of bugs - getting this wrong causes incorrect
packet encoding/decoding.
"""

import math
import struct
from typing import Optional, NamedTuple
from ..parser.expressions import safe_eval, safe_eval_or_none


def to_float32(value: float) -> float:
    """Convert a float64 value to float32 precision.

    This ensures Python calculations match C's float32 behavior.
    Important for scaling calculations where rounding differences
    at midpoint values (like 127.5) can cause encoding mismatches.
    """
    return struct.unpack('f', struct.pack('f', value))[0]


def to_c_double_precision(value: float) -> float:
    """Match C's double literal precision.

    ProtoGen outputs scaler constants as string literals with limited precision
    (approximately 15-16 significant digits). When C parses these back, it gets
    a slightly different value than the mathematically exact result. This function
    round-trips through a string representation to match C's behavior.

    This is critical for exact encode/decode matching between C and Python.
    """
    # Format with enough precision to distinguish different doubles,
    # but match what C compilers typically use for double literals
    formatted = f"{value:.13g}"
    return float(formatted)


class ScalingParams(NamedTuple):
    """Parameters for scaling a field value."""
    min_value: float
    max_value: float
    scaler: float
    is_signed: bool
    encoded_bits: int


def compute_scaler(
    min_val: Optional[float],
    max_val: Optional[float],
    scaler: Optional[float],
    encoded_bits: int,
    is_signed: bool,
    use_float32: bool = True
) -> float:
    """Compute the scaling factor for a field.

    ProtoGen allows specifying either an explicit scaler OR min/max range.
    If scaler is provided, use it directly.
    If min/max provided, compute scaler from range and bit width.

    Args:
        min_val: Minimum value (physical units)
        max_val: Maximum value (physical units)
        scaler: Explicit scaler if provided
        encoded_bits: Number of bits in encoded representation
        is_signed: True if the encoded type is signed
        use_float32: If True, convert to float32 precision (for float32 in-memory types)

    Returns:
        The scaler value to use for encoding/decoding
    """
    # If explicit scaler provided, use it
    if scaler is not None:
        if use_float32:
            return to_float32(scaler)
        else:
            return to_c_double_precision(scaler)

    # Need min/max to compute scaler
    if min_val is None or max_val is None:
        return 1.0  # No scaling

    # Compute encoded range based on bit width and signedness
    if is_signed:
        # Signed range: -2^(n-1) to 2^(n-1)-1
        # Use the maximum positive value for scaling: 2^(n-1) - 1
        encoded_range = (1 << (encoded_bits - 1)) - 1
    else:
        # Unsigned range: 0 to 2^n - 1
        encoded_range = (1 << encoded_bits) - 1

    physical_range = max_val - min_val
    if physical_range == 0:
        return 1.0

    result = encoded_range / physical_range
    if use_float32:
        # Convert to float32 precision for float32 in-memory types
        return to_float32(result)
    else:
        # For float64, match C's string literal precision
        return to_c_double_precision(result)


def encode_scaled_unsigned(value: float, min_val: float, scaler: float) -> int:
    """Encode a value using unsigned scaling.

    Formula: encoded = round((value - min) * scaler)

    Args:
        value: Physical value to encode
        min_val: Minimum value in physical units
        scaler: Scaling factor

    Returns:
        Encoded integer value
    """
    return int(round((value - min_val) * scaler))


def encode_scaled_signed(value: float, scaler: float) -> int:
    """Encode a value using signed scaling.

    Formula: encoded = round(value * scaler)

    IMPORTANT: Signed encoding does NOT subtract min!

    Args:
        value: Physical value to encode
        scaler: Scaling factor

    Returns:
        Encoded integer value (may be negative)
    """
    return int(round(value * scaler))


def decode_scaled_unsigned(encoded: int, min_val: float, scaler: float) -> float:
    """Decode an unsigned scaled value.

    Formula: value = (encoded / scaler) + min

    Args:
        encoded: Encoded integer value
        min_val: Minimum value in physical units
        scaler: Scaling factor

    Returns:
        Decoded physical value
    """
    if scaler == 0:
        return min_val
    return (encoded / scaler) + min_val


def decode_scaled_signed(encoded: int, scaler: float) -> float:
    """Decode a signed scaled value.

    Formula: value = encoded / scaler

    IMPORTANT: Signed decoding does NOT add min!

    Args:
        encoded: Encoded integer value (may be negative)
        scaler: Scaling factor

    Returns:
        Decoded physical value
    """
    if scaler == 0:
        return 0.0
    return encoded / scaler


def encode_scaled(
    value: float,
    min_val: float,
    scaler: float,
    is_signed: bool,
    encoded_bits: int
) -> int:
    """Encode a value with proper signed/unsigned handling.

    Args:
        value: Physical value to encode
        min_val: Minimum value in physical units
        scaler: Scaling factor
        is_signed: True if encoded type is signed
        encoded_bits: Number of bits in encoded representation

    Returns:
        Encoded integer value, clamped to valid range
    """
    if is_signed:
        encoded = encode_scaled_signed(value, scaler)
        # Clamp to signed range
        min_encoded = -(1 << (encoded_bits - 1))
        max_encoded = (1 << (encoded_bits - 1)) - 1
    else:
        encoded = encode_scaled_unsigned(value, min_val, scaler)
        # Clamp to unsigned range
        min_encoded = 0
        max_encoded = (1 << encoded_bits) - 1

    return max(min_encoded, min(max_encoded, encoded))


def decode_scaled(
    encoded: int,
    min_val: float,
    scaler: float,
    is_signed: bool,
    encoded_bits: int
) -> float:
    """Decode a value with proper signed/unsigned handling.

    Args:
        encoded: Encoded integer value
        min_val: Minimum value in physical units
        scaler: Scaling factor
        is_signed: True if encoded type is signed
        encoded_bits: Number of bits in encoded representation

    Returns:
        Decoded physical value
    """
    # Sign-extend if necessary
    if is_signed and encoded >= (1 << (encoded_bits - 1)):
        encoded -= (1 << encoded_bits)

    if is_signed:
        return decode_scaled_signed(encoded, scaler)
    else:
        return decode_scaled_unsigned(encoded, min_val, scaler)


def parse_scaling_from_xml(
    min_attr: Optional[str],
    max_attr: Optional[str],
    scaler_attr: Optional[str],
    encoded_type: str
) -> ScalingParams:
    """Parse scaling parameters from ProtoGen XML attributes.

    Args:
        min_attr: 'min' attribute value (expression string)
        max_attr: 'max' attribute value (expression string)
        scaler_attr: 'scaler' attribute value (expression string)
        encoded_type: The inMemoryType or encodedType (e.g., 'signed16')

    Returns:
        ScalingParams with all computed values
    """
    # Evaluate expressions
    min_val = safe_eval_or_none(min_attr)
    max_val = safe_eval_or_none(max_attr)
    scaler_val = safe_eval_or_none(scaler_attr)

    # Determine signedness and bit width from type
    is_signed, encoded_bits = parse_encoded_type(encoded_type)

    # Default min/max if not specified
    if min_val is None:
        min_val = 0.0
    if max_val is None:
        if is_signed:
            max_val = float((1 << (encoded_bits - 1)) - 1)
        else:
            max_val = float((1 << encoded_bits) - 1)

    # Compute scaler at double precision - the template will handle
    # float32 conversion if needed based on in-memory type
    final_scaler = compute_scaler(min_val, max_val, scaler_val, encoded_bits, is_signed, use_float32=False)

    return ScalingParams(
        min_value=min_val,
        max_value=max_val,
        scaler=final_scaler,
        is_signed=is_signed,
        encoded_bits=encoded_bits
    )


def parse_encoded_type(type_str: str) -> tuple:
    """Parse an encoded type string into (is_signed, bit_width).

    Args:
        type_str: Type like 'signed16', 'unsigned8', 'float32', 'bitfield4', etc.

    Returns:
        Tuple of (is_signed: bool, bit_width: int)
    """
    type_str = type_str.lower().strip()

    # Handle special types
    if type_str in ('float32', 'float'):
        return (True, 32)
    if type_str in ('float64', 'double'):
        return (True, 64)

    # Handle bitfield types (bitfield1, bitfield4, etc.)
    if type_str.startswith('bitfield'):
        bits_str = type_str[8:]  # Remove 'bitfield'
        return (False, int(bits_str) if bits_str else 1)

    # Parse signed/unsigned + bit width
    if type_str.startswith('signed'):
        bits_str = type_str[6:]  # Remove 'signed'
        return (True, int(bits_str) if bits_str else 32)
    elif type_str.startswith('unsigned'):
        bits_str = type_str[8:]  # Remove 'unsigned'
        return (False, int(bits_str) if bits_str else 32)
    elif type_str.startswith('int'):
        bits_str = type_str[3:]  # Remove 'int'
        return (True, int(bits_str) if bits_str else 32)
    elif type_str.startswith('uint'):
        bits_str = type_str[4:]  # Remove 'uint'
        return (False, int(bits_str) if bits_str else 32)

    # Default to unsigned 8-bit
    return (False, 8)


def generate_encode_call(
    field_name: str,
    params: ScalingParams,
    indent: str = "    "
) -> str:
    """Generate Python code for encoding a scaled field.

    Args:
        field_name: Name of the field variable
        params: Scaling parameters
        indent: Indentation string

    Returns:
        Python code string for encoding
    """
    if params.is_signed:
        return f"{indent}encoded = int(round({field_name} * {params.scaler}))"
    else:
        return f"{indent}encoded = int(round(({field_name} - {params.min_value}) * {params.scaler}))"


def generate_decode_call(
    encoded_var: str,
    params: ScalingParams,
    indent: str = "    "
) -> str:
    """Generate Python code for decoding a scaled field.

    Args:
        encoded_var: Name of the encoded value variable
        params: Scaling parameters
        indent: Indentation string

    Returns:
        Python code string for decoding
    """
    if params.is_signed:
        return f"{indent}value = {encoded_var} / {params.scaler}"
    else:
        return f"{indent}value = ({encoded_var} / {params.scaler}) + {params.min_value}"
