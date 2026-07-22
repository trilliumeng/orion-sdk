"""Runtime scaling helpers for generated protocol code.

These functions implement C-compatible float32 and float64 rounding behavior
for encoding scaled values. They are used by the generated structs.py and
packets.py modules.

Auto-generated code imports this module rather than duplicating these functions.
"""

import struct


def _to_f32(x: float) -> float:
    """Convert a float to float32 precision.

    This matches C's implicit conversion when assigning a double to a float.
    """
    return struct.unpack('f', struct.pack('f', x))[0]


def _c_round_unsigned(value: float, min_val: float, scaler: float) -> int:
    """Encode using C's float32 rounding behavior for unsigned values.

    C computes: (uint16_t)((value - min) * scaler + 0.5f) using float32.
    We simulate this by converting all operands to float32 before operations.

    Args:
        value: The physical value to encode
        min_val: The minimum value of the range
        scaler: The scaling factor

    Returns:
        The encoded integer value
    """
    # Convert all operands to float32 precision to match C
    val_f32 = _to_f32(value)
    min_f32 = _to_f32(min_val)
    scaler_f32 = _to_f32(scaler)
    # Compute (value - min) * scaler in float32
    scaled = _to_f32(_to_f32(val_f32 - min_f32) * scaler_f32)
    # Round half up (C's behavior)
    return int(scaled + 0.5)


def _c_round_signed(value: float, scaler: float) -> int:
    """Encode using C's float32 rounding behavior for signed values.

    C computes using float32 and rounds half-away-from-zero.

    Args:
        value: The physical value to encode
        scaler: The scaling factor

    Returns:
        The encoded integer value
    """
    # Convert operands to float32 precision
    val_f32 = _to_f32(value)
    scaler_f32 = _to_f32(scaler)
    scaled = _to_f32(val_f32 * scaler_f32)
    # Round half away from zero (C's behavior)
    if scaled >= 0:
        return int(scaled + 0.5)
    else:
        return int(scaled - 0.5)


def _c_round_unsigned_f64(value: float, min_val: float, scaler: float) -> int:
    """Encode using double precision for float64 in-memory types.

    For float64 fields, C uses double precision throughout the calculation.

    Args:
        value: The physical value to encode
        min_val: The minimum value of the range
        scaler: The scaling factor

    Returns:
        The encoded integer value
    """
    scaled = (value - min_val) * scaler
    return int(scaled + 0.5)


def _c_round_signed_f64(value: float, scaler: float) -> int:
    """Encode using double precision for float64 in-memory types.

    For float64 fields, C uses double precision throughout the calculation.

    Args:
        value: The physical value to encode
        scaler: The scaling factor

    Returns:
        The encoded integer value
    """
    scaled = value * scaler
    if scaled >= 0:
        return int(scaled + 0.5)
    else:
        return int(scaled - 0.5)
