"""XML parsing and type system modules."""

from .xml_parser import (
    Protocol, FieldDef, PacketDef, StructDef, EnumDef, EnumValue,
    parse_protocol, get_packet_id_value, resolve_array_length, group_bitfields
)
from .type_mapping import get_type_info, TypeInfo
from .field_info import FieldInfo, ScalingInfo
from .expressions import safe_eval

__all__ = [
    'Protocol', 'FieldDef', 'PacketDef', 'StructDef', 'EnumDef', 'EnumValue',
    'parse_protocol', 'get_packet_id_value', 'resolve_array_length', 'group_bitfields',
    'get_type_info', 'TypeInfo',
    'FieldInfo', 'ScalingInfo',
    'safe_eval',
]
