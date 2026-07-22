"""Type-safe field information for code generation.

This module provides dataclasses that replace the Dict[str, Any] previously
returned by CodeGenerator.get_field_info(). This provides:
- Type safety and IDE autocompletion
- Clear documentation of available fields
- Compile-time error detection for typos
"""

from dataclasses import dataclass, field
from typing import Optional, Any


@dataclass
class ScalingInfo:
    """Scaling parameters for encoded fields."""
    min: float
    max: float
    scaler: float
    is_signed: bool
    bits: int


@dataclass
class FieldInfo:
    """Complete field metadata for code generation.

    This dataclass contains all information needed by templates to generate
    encode/decode code for a protocol field.
    """
    # Basic info
    name: str
    comment: str
    type_hint: str
    default: str

    # Boolean flags
    is_array: bool = False
    is_string: bool = False
    is_bitfield: bool = False
    is_null_encoded: bool = False
    is_null_in_memory: bool = False
    is_float64_in_memory: bool = False
    is_struct: bool = False
    is_enum: bool = False
    has_scaling: bool = False

    # References
    struct_ref: Optional[str] = None
    enum_ref: Optional[str] = None

    # Dependencies
    depends_on: Optional[str] = None
    depends_on_value: Optional[Any] = None
    constant: Optional[str] = None

    # Array info
    array_length: Optional[int] = None
    array_length_expr: Optional[str] = None
    variable_array: Optional[str] = None

    # String info
    string_length: Optional[int] = None
    is_fixed_string: Optional[bool] = None

    # Bitfield info
    bitfield_bits: Optional[int] = None
    bitfield_group: Optional[str] = None

    # Scaling info
    scaling: Optional[ScalingInfo] = None

    # Type info for encoding
    struct_format: Optional[str] = None
    type_size: Optional[int] = None
    is_signed: Optional[bool] = None
    is_float: Optional[bool] = None
    is_custom_size: Optional[bool] = None
    bit_width: Optional[int] = None
    is_float16: bool = False

    def to_dict(self) -> dict:
        """Convert to dictionary for backward compatibility with templates.

        This allows gradual migration - templates can continue using
        dict-style access while we transition to attribute access.
        """
        result = {
            'name': self.name,
            'comment': self.comment,
            'type_hint': self.type_hint,
            'default': self.default,
            'is_array': self.is_array,
            'is_string': self.is_string,
            'is_bitfield': self.is_bitfield,
            'is_null_encoded': self.is_null_encoded,
            'is_null_in_memory': self.is_null_in_memory,
            'is_float64_in_memory': self.is_float64_in_memory,
            'is_struct': self.is_struct,
            'is_enum': self.is_enum,
            'has_scaling': self.has_scaling,
            'struct_ref': self.struct_ref,
            'enum_ref': self.enum_ref,
            'depends_on': self.depends_on,
            'depends_on_value': self.depends_on_value,
            'constant': self.constant,
        }

        # Add optional array info
        if self.array_length is not None:
            result['array_length'] = self.array_length
        if self.array_length_expr is not None:
            result['array_length_expr'] = self.array_length_expr
        if self.variable_array is not None:
            result['variable_array'] = self.variable_array

        # Add optional string info
        if self.string_length is not None:
            result['string_length'] = self.string_length
        if self.is_fixed_string is not None:
            result['is_fixed_string'] = self.is_fixed_string

        # Add optional bitfield info
        if self.bitfield_bits is not None:
            result['bitfield_bits'] = self.bitfield_bits
        if self.bitfield_group is not None:
            result['bitfield_group'] = self.bitfield_group

        # Add scaling as nested dict
        if self.scaling is not None:
            result['scaling'] = {
                'min': self.scaling.min,
                'max': self.scaling.max,
                'scaler': self.scaling.scaler,
                'is_signed': self.scaling.is_signed,
                'bits': self.scaling.bits,
            }

        # Add optional type info
        if self.struct_format is not None:
            result['struct_format'] = self.struct_format
        if self.type_size is not None:
            result['type_size'] = self.type_size
        if self.is_signed is not None:
            result['is_signed'] = self.is_signed
        if self.is_float is not None:
            result['is_float'] = self.is_float
        if self.is_custom_size is not None:
            result['is_custom_size'] = self.is_custom_size
        if self.bit_width is not None:
            result['bit_width'] = self.bit_width
        if self.is_float16:
            result['is_float16'] = self.is_float16

        return result
