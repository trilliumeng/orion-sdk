"""ProtoGen XML parser.

Parses ProtoGen protocol XML files into Python data structures suitable
for code generation.
"""

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Any
from pathlib import Path
import re

from .expressions import safe_eval, safe_eval_or_none


@dataclass
class EnumValue:
    """A single enum value."""
    name: str
    value: Optional[int] = None  # None = auto-increment
    comment: str = ""
    hidden: bool = False


@dataclass
class EnumDef:
    """An enumeration definition."""
    name: str
    values: List[EnumValue] = field(default_factory=list)
    comment: str = ""
    is_global: bool = False  # True if defined at protocol level

    def get_value(self, name: str) -> Optional[int]:
        """Get numeric value for an enum value name."""
        current = 0
        for v in self.values:
            if v.value is not None:
                current = v.value
            if v.name == name:
                return current
            current += 1
        return None


@dataclass
class FieldDef:
    """A field (Data element) definition."""
    name: str
    in_memory_type: str
    encoded_type: Optional[str] = None
    comment: str = ""

    # Scaling attributes
    min_attr: Optional[str] = None
    max_attr: Optional[str] = None
    scaler_attr: Optional[str] = None

    # Array attributes
    array_length: Optional[str] = None  # Fixed length (may be expression)
    variable_array: Optional[str] = None  # Field name for length

    # String attributes
    string_length: Optional[str] = None
    fixedstring_length: Optional[str] = None

    # Bitfield attributes
    bitfield_bits: int = 0
    bitfield_group: Optional[str] = None

    # Struct/enum reference
    struct_ref: Optional[str] = None
    enum_ref: Optional[str] = None

    # Default and constant
    default_value: Optional[str] = None
    constant_value: Optional[str] = None

    # Conditional
    depends_on: Optional[str] = None
    depends_on_value: Optional[str] = None

    def is_null_encoded(self) -> bool:
        """Check if field has null encoding (in-memory only)."""
        return self.encoded_type and self.encoded_type.lower() == 'null'

    def is_bitfield(self) -> bool:
        """Check if this is a bitfield."""
        if self.bitfield_bits > 0:
            return True
        if self.encoded_type and self.encoded_type.startswith('bitfield'):
            return True
        if self.in_memory_type and self.in_memory_type.startswith('bitfield'):
            return True
        return False

    def get_bitfield_bits(self) -> int:
        """Get number of bits if bitfield."""
        if self.bitfield_bits > 0:
            return self.bitfield_bits
        # Check encodedType first
        if self.encoded_type and self.encoded_type.startswith('bitfield'):
            try:
                return int(self.encoded_type[8:])  # 'bitfieldN' -> N
            except ValueError:
                pass
        # Then check inMemoryType
        if self.in_memory_type and self.in_memory_type.startswith('bitfield'):
            try:
                return int(self.in_memory_type[8:])  # 'bitfieldN' -> N
            except ValueError:
                pass
        return 0

    def is_array(self) -> bool:
        """Check if field is an array."""
        return self.array_length is not None or self.variable_array is not None

    def is_string(self) -> bool:
        """Check if field is a string."""
        return self.string_length is not None or self.fixedstring_length is not None


@dataclass
class StructDef:
    """A structure definition."""
    name: str
    fields: List[FieldDef] = field(default_factory=list)
    comment: str = ""
    is_global: bool = False  # True if defined at protocol level
    enums: List[EnumDef] = field(default_factory=list)  # Inline enums

    # Array attributes (when struct is used as array in packet)
    array_length: Optional[str] = None
    variable_array: Optional[str] = None


@dataclass
class PacketDef:
    """A packet definition."""
    name: str
    packet_id: str  # Enum value name like 'ORION_PKT_CMD'
    comment: str = ""
    fields: List[FieldDef] = field(default_factory=list)
    structs: List[StructDef] = field(default_factory=list)  # Inline structs
    enums: List[EnumDef] = field(default_factory=list)  # Inline enums


@dataclass
class CodeBlock:
    """A Code element for custom code insertion."""
    name: str
    language: str
    content: str
    location: str = ""  # 'encode', 'decode', etc.


@dataclass
class Protocol:
    """Complete protocol definition."""
    name: str
    version: str
    endian: str = "big"
    file_prefix: str = ""
    comment: str = ""

    enums: Dict[str, EnumDef] = field(default_factory=dict)
    structs: Dict[str, StructDef] = field(default_factory=dict)
    packets: List[PacketDef] = field(default_factory=list)
    code_blocks: List[CodeBlock] = field(default_factory=list)

    # Resolved enum values (name -> int)
    enum_values: Dict[str, int] = field(default_factory=dict)

    def resolve_enum_value(self, name: str) -> Optional[int]:
        """Resolve an enum value name to its integer value."""
        if name in self.enum_values:
            return self.enum_values[name]

        # Search all enums
        for enum in self.enums.values():
            val = enum.get_value(name)
            if val is not None:
                self.enum_values[name] = val
                return val

        # Try to parse as integer
        try:
            return int(name, 0)  # Handles 0x prefix
        except ValueError:
            pass

        # Try expression evaluation
        try:
            return int(safe_eval(name))
        except (ValueError, TypeError):
            pass

        return None


def parse_field(elem: ET.Element) -> FieldDef:
    """Parse a Data element into a FieldDef."""
    in_memory_type = elem.get('inMemoryType', elem.get('struct', elem.get('enum', 'unsigned8')))
    array_attr = elem.get('array')
    string_attr = elem.get('string')
    fixedstring_attr = elem.get('fixedstring')

    # Handle inMemoryType="string" with array="N" - this means a string of length N
    if in_memory_type == 'string' and array_attr:
        string_attr = array_attr
        array_attr = None

    # Handle inMemoryType="fixedstring" with array="N" - fixed-length string of N chars
    if in_memory_type == 'fixedstring' and array_attr:
        fixedstring_attr = array_attr
        array_attr = None

    f = FieldDef(
        name=elem.get('name', ''),
        in_memory_type=in_memory_type,
        encoded_type=elem.get('encodedType'),
        comment=elem.get('comment', ''),
        min_attr=elem.get('min'),
        max_attr=elem.get('max'),
        scaler_attr=elem.get('scaler'),
        array_length=array_attr,
        variable_array=elem.get('variableArray'),
        string_length=string_attr,
        fixedstring_length=fixedstring_attr,
        struct_ref=elem.get('struct'),
        enum_ref=elem.get('enum'),
        default_value=elem.get('default'),
        constant_value=elem.get('constant'),
        depends_on=elem.get('dependsOn'),
        depends_on_value=elem.get('dependsOnValue'),
        bitfield_group=elem.get('bitfieldGroup'),
    )

    # Note: inMemoryType='null' means field exists on wire but not in C struct
    # Keep it as 'null' so downstream code can detect this and handle accordingly

    return f


def parse_enum(elem: ET.Element, is_global: bool = False) -> EnumDef:
    """Parse an Enum element into an EnumDef."""
    enum = EnumDef(
        name=elem.get('name', ''),
        comment=elem.get('comment', ''),
        is_global=is_global
    )

    for val_elem in elem.findall('Value'):
        value_str = val_elem.get('value')
        value = None
        if value_str:
            try:
                value = int(value_str, 0)  # Handles 0x prefix
            except ValueError:
                pass

        enum.values.append(EnumValue(
            name=val_elem.get('name', ''),
            value=value,
            comment=val_elem.get('comment', ''),
            hidden=val_elem.get('hidden', 'false').lower() == 'true'
        ))

    return enum


def parse_struct(elem: ET.Element, is_global: bool = False) -> StructDef:
    """Parse a Structure element into a StructDef."""
    struct = StructDef(
        name=elem.get('name', ''),
        comment=elem.get('comment', ''),
        is_global=is_global,
        array_length=elem.get('array'),
        variable_array=elem.get('variableArray'),
    )

    for child in elem:
        if child.tag in ('Data', 'data'):
            struct.fields.append(parse_field(child))
        elif child.tag == 'Enum':
            struct.enums.append(parse_enum(child, is_global=False))

    return struct


def parse_packet(elem: ET.Element) -> PacketDef:
    """Parse a Packet element into a PacketDef."""
    packet = PacketDef(
        name=elem.get('name', ''),
        packet_id=elem.get('ID', ''),
        comment=elem.get('comment', ''),
    )

    for child in elem:
        if child.tag in ('Data', 'data'):
            packet.fields.append(parse_field(child))
        elif child.tag == 'Structure':
            struct = parse_struct(child, is_global=False)
            packet.structs.append(struct)
            # Create a field that references this inline struct
            struct_field = FieldDef(
                name=struct.name,
                in_memory_type=struct.name,
                struct_ref=struct.name,
                comment=struct.comment,
                array_length=child.get('array'),
                variable_array=child.get('variableArray'),
            )
            packet.fields.append(struct_field)
        elif child.tag == 'Enum':
            packet.enums.append(parse_enum(child, is_global=False))

    return packet


def parse_protocol(xml_path: str) -> Protocol:
    """Parse a ProtoGen XML file into a Protocol object.

    Args:
        xml_path: Path to the XML file

    Returns:
        Protocol object with all definitions
    """
    tree = ET.parse(xml_path)
    root = tree.getroot()

    protocol = Protocol(
        name=root.get('name', 'Unknown'),
        version=root.get('version', '0.0.0'),
        endian=root.get('endian', 'big'),
        file_prefix=root.get('file', ''),
        comment=root.get('comment', ''),
    )

    # First pass: collect all global enums (needed for resolving values)
    for elem in root:
        if elem.tag == 'Enum':
            enum = parse_enum(elem, is_global=True)
            protocol.enums[enum.name] = enum
            # Pre-populate enum values
            current = 0
            for v in enum.values:
                if v.value is not None:
                    current = v.value
                protocol.enum_values[v.name] = current
                current += 1

    # Second pass: collect global structs
    for elem in root:
        if elem.tag == 'Structure':
            struct = parse_struct(elem, is_global=True)
            protocol.structs[struct.name] = struct

    # Third pass: collect packets
    for elem in root:
        if elem.tag == 'Packet':
            packet = parse_packet(elem)
            protocol.packets.append(packet)

            # Also collect inline enums and structs
            for enum in packet.enums:
                if enum.name not in protocol.enums:
                    protocol.enums[enum.name] = enum
                    current = 0
                    for v in enum.values:
                        if v.value is not None:
                            current = v.value
                        protocol.enum_values[v.name] = current
                        current += 1

            for struct in packet.structs:
                if struct.name not in protocol.structs:
                    protocol.structs[struct.name] = struct
                # Also collect enums from inline structs
                for enum in struct.enums:
                    if enum.name not in protocol.enums:
                        protocol.enums[enum.name] = enum
                        current = 0
                        for v in enum.values:
                            if v.value is not None:
                                current = v.value
                            protocol.enum_values[v.name] = current
                            current += 1

    # Fourth pass: collect Code blocks
    for elem in root:
        if elem.tag == 'Code':
            code_block = CodeBlock(
                name=elem.get('name', ''),
                language=elem.get('language', ''),
                content=elem.text or '',
                location=elem.get('location', ''),
            )
            protocol.code_blocks.append(code_block)

    return protocol


def get_packet_id_value(protocol: Protocol, packet: PacketDef) -> Optional[int]:
    """Get the numeric packet ID for a packet.

    Args:
        protocol: The protocol definition
        packet: The packet definition

    Returns:
        Integer packet ID or None if not resolvable
    """
    return protocol.resolve_enum_value(packet.packet_id)


def resolve_array_length(protocol: Protocol, length_expr: str) -> int:
    """Resolve an array length expression to an integer.

    Args:
        protocol: Protocol for enum resolution
        length_expr: Expression string (may be enum name or number)

    Returns:
        Integer length
    """
    # Try enum resolution first
    val = protocol.resolve_enum_value(length_expr)
    if val is not None:
        return val

    # Try direct integer
    try:
        return int(length_expr, 0)
    except ValueError:
        pass

    # Try expression evaluation
    try:
        return int(safe_eval(length_expr))
    except (ValueError, TypeError):
        pass

    return 1  # Default


def get_all_fields_flat(packet: PacketDef, protocol: Protocol) -> List[FieldDef]:
    """Get all fields in a packet, expanding struct references.

    Args:
        packet: The packet definition
        protocol: Protocol for struct lookup

    Returns:
        Flat list of all fields
    """
    result = []

    def add_fields(fields: List[FieldDef], prefix: str = ""):
        for f in fields:
            if f.struct_ref and f.struct_ref in protocol.structs:
                struct = protocol.structs[f.struct_ref]
                add_fields(struct.fields, f"{prefix}{f.name}.")
            else:
                result.append(f)

    add_fields(packet.fields)
    return result


def group_bitfields(fields: List[FieldDef]) -> List[List[FieldDef]]:
    """Group consecutive bitfields together.

    Args:
        fields: List of field definitions

    Returns:
        List of groups, where each group is either:
        - A single non-bitfield field
        - Multiple consecutive bitfield fields
    """
    groups = []
    current_group = []

    for f in fields:
        if f.is_bitfield():
            current_group.append(f)
        else:
            if current_group:
                groups.append(current_group)
                current_group = []
            groups.append([f])

    if current_group:
        groups.append(current_group)

    return groups
