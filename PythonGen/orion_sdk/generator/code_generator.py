"""Code generator for ProtoGen Python bindings.

Uses Jinja2 templates to generate Python code from parsed protocol definitions.
"""

from pathlib import Path
from typing import Dict, Any, List, Optional
from jinja2 import Environment, FileSystemLoader, select_autoescape

from ..parser.xml_parser import (
    Protocol, FieldDef, PacketDef, StructDef,
    get_packet_id_value, resolve_array_length, group_bitfields
)
from ..parser.type_mapping import get_type_info
from ..runtime.scaling import parse_scaling_from_xml
from ..parser.field_info import FieldInfo, ScalingInfo


class CodeGenerator:
    """Generates Python code from ProtoGen protocol definitions."""

    def __init__(self, protocol: Protocol, output_dir: str, template_dir: str = None):
        """Initialize the code generator.

        Args:
            protocol: Parsed protocol definition
            output_dir: Directory to write generated code
            template_dir: Directory containing Jinja2 templates (default: ./templates)
        """
        self.protocol = protocol
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        if template_dir is None:
            # Templates are at orion_sdk/templates (one level up from generator/)
            template_dir = Path(__file__).parent.parent / 'templates'

        self.env = Environment(
            loader=FileSystemLoader(str(template_dir)),
            autoescape=select_autoescape(['html', 'xml']),
            trim_blocks=True,
            lstrip_blocks=True,
        )

        # Add Python builtins to globals
        self.env.globals['zip'] = zip
        self.env.globals['enumerate'] = enumerate
        self.env.globals['len'] = len
        self.env.globals['range'] = range
        self.env.globals['int'] = int
        self.env.globals['float'] = float
        self.env.globals['str'] = str

        # Register custom filters
        self.env.filters['snake_case'] = self._snake_case
        self.env.filters['pascal_case'] = self._pascal_case
        self.env.filters['type_hint'] = self._type_hint
        self.env.filters['default_value'] = self._default_value
        self.env.filters['struct_format'] = self._struct_format

    def _snake_case(self, name: str) -> str:
        """Convert CamelCase to snake_case."""
        import re
        s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
        return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()

    def _pascal_case(self, name: str) -> str:
        """Convert snake_case to PascalCase."""
        return ''.join(word.capitalize() for word in name.split('_'))

    def _type_hint(self, field: FieldDef) -> str:
        """Get Python type hint for a field."""
        if field.enum_ref:
            return field.enum_ref
        if field.struct_ref:
            return field.struct_ref
        if field.is_string():
            return 'str'
        if field.is_array():
            base_type = self._get_base_type_hint(field)
            return f'List[{base_type}]'
        return self._get_base_type_hint(field)

    def _get_base_type_hint(self, field: FieldDef) -> str:
        """Get base Python type hint."""
        if field.struct_ref:
            return field.struct_ref
        if field.enum_ref:
            return 'int'

        # For scaled fields, use in_memory_type (the user-facing type) not encoded_type
        # This ensures type hints show 'float' for scaled values, not 'int'
        has_scaling = field.scaler_attr or field.min_attr or field.max_attr
        if has_scaling and field.in_memory_type:
            type_name = field.in_memory_type
        else:
            # Determine the type to use
            type_name = field.encoded_type if field.encoded_type else field.in_memory_type
            if type_name and type_name.lower() == 'null':
                type_name = field.in_memory_type

        try:
            info = get_type_info(type_name)
            return info.python_type
        except ValueError:
            return 'Any'

    def _default_value(self, field: FieldDef) -> str:
        """Get Python default value for a field."""
        if field.default_value:
            # Try to parse as enum value
            val = self.protocol.resolve_enum_value(field.default_value)
            if val is not None:
                return str(val)
            # Try expression
            try:
                return str(safe_eval(field.default_value))
            except ValueError:
                return repr(field.default_value)

        if field.constant_value:
            return field.constant_value

        if field.is_string():
            return '""'
        if field.is_array():
            return '[]'
        if field.struct_ref:
            return f'{field.struct_ref}()'
        if field.enum_ref:
            return '0'

        type_name = field.encoded_type if field.encoded_type else field.in_memory_type
        if type_name and type_name.lower() == 'null':
            type_name = field.in_memory_type

        try:
            info = get_type_info(type_name)
            return info.default_value
        except ValueError:
            return '0'

    def _struct_format(self, field: FieldDef) -> str:
        """Get struct format code for a field."""
        type_name = field.encoded_type if field.encoded_type else field.in_memory_type
        if type_name and type_name.lower() == 'null':
            return ''
        if type_name and type_name.startswith('bitfield'):
            return ''
        try:
            info = get_type_info(type_name)
            return info.struct_format
        except ValueError:
            return ''

    def get_field_info_typed(self, field: FieldDef) -> FieldInfo:
        """Get comprehensive information about a field for templates.

        Args:
            field: The field definition

        Returns:
            FieldInfo dataclass with all field info needed for code generation
        """
        # Check if field has null in-memory type (doesn't exist in C struct)
        is_null_in_memory = (field.in_memory_type and
                             field.in_memory_type.lower() == 'null')

        # Check if field uses float64/double in-memory type
        # (affects how scaling is computed - use double precision, not float32)
        in_mem = field.in_memory_type.lower() if field.in_memory_type else ''
        is_float64_in_memory = in_mem in ('float64', 'double')

        has_scaling = bool(field.scaler_attr or field.min_attr or field.max_attr)

        # Create base FieldInfo
        info = FieldInfo(
            name=field.name,
            comment=field.comment,
            type_hint=self._type_hint(field),
            default=self._default_value(field),
            is_array=field.is_array(),
            is_string=field.is_string(),
            is_bitfield=field.is_bitfield(),
            is_null_encoded=field.is_null_encoded(),
            is_null_in_memory=is_null_in_memory,
            is_float64_in_memory=is_float64_in_memory,
            is_struct=field.struct_ref is not None,
            is_enum=field.enum_ref is not None,
            struct_ref=field.struct_ref,
            enum_ref=field.enum_ref,
            has_scaling=has_scaling,
            depends_on=field.depends_on,
            depends_on_value=field.depends_on_value,
            constant=field.constant_value,
        )

        # Array info
        if field.array_length:
            info.array_length = resolve_array_length(self.protocol, field.array_length)
            info.array_length_expr = field.array_length
        if field.variable_array:
            info.variable_array = field.variable_array

        # String info
        if field.string_length:
            info.string_length = int(field.string_length)
            info.is_fixed_string = False
        if field.fixedstring_length:
            info.string_length = int(field.fixedstring_length)
            info.is_fixed_string = True

        # Bitfield info
        if field.is_bitfield():
            info.bitfield_bits = field.get_bitfield_bits()
            info.bitfield_group = field.bitfield_group

        # Scaling info
        if has_scaling:
            type_name = field.encoded_type if field.encoded_type else field.in_memory_type
            params = parse_scaling_from_xml(
                field.min_attr, field.max_attr, field.scaler_attr, type_name
            )
            info.scaling = ScalingInfo(
                min=params.min_value,
                max=params.max_value,
                scaler=params.scaler,
                is_signed=params.is_signed,
                bits=params.encoded_bits,
            )

        # Type info for encoding
        type_name = field.encoded_type if field.encoded_type else field.in_memory_type
        # For enum fields without explicit encodedType, default to unsigned8
        if field.enum_ref and not field.encoded_type:
            type_name = 'unsigned8'
        if type_name and type_name.lower() != 'null' and not type_name.startswith('bitfield'):
            try:
                type_info = get_type_info(type_name)
                info.struct_format = type_info.struct_format
                info.type_size = type_info.size_bytes
                info.is_signed = type_info.is_signed
                info.is_float = type_info.is_float
                info.is_custom_size = type_info.is_custom_size
                info.bit_width = type_info.bit_width
                # Flag for float16 types (need custom encoding, not IEEE 754)
                info.is_float16 = type_name.lower() == 'float16'
            except ValueError:
                pass

        return info

    def get_field_info(self, field: FieldDef) -> Dict[str, Any]:
        """Get field info as a dictionary for templates.

        This is a convenience wrapper around get_field_info_typed() that
        returns a dict for backward compatibility with Jinja2 templates.
        """
        return self.get_field_info_typed(field).to_dict()

    def generate_enums_file(self) -> str:
        """Generate complete enums.py file."""
        template = self.env.get_template('enums.py.j2')
        return template.render(
            enums=self.protocol.enums.values(),
            protocol=self.protocol,
        )

    def generate_structs_file(self) -> str:
        """Generate complete structs.py file."""
        template = self.env.get_template('structs.py.j2')
        structs_info = []
        for struct in self.protocol.structs.values():
            structs_info.append({
                'struct': struct,
                'fields_info': [self.get_field_info(f) for f in struct.fields],
                'field_groups': group_bitfields(struct.fields),
            })
        return template.render(
            structs=self.protocol.structs.values(),
            structs_info=structs_info,
            protocol=self.protocol,
            generator=self,
        )

    def generate_packets_file(self) -> str:
        """Generate complete packets.py file."""
        template = self.env.get_template('packets.py.j2')
        packets_info = []
        for packet in self.protocol.packets:
            packet_id = get_packet_id_value(self.protocol, packet)
            packets_info.append({
                'packet': packet,
                'packet_id': packet_id,
                'fields_info': [self.get_field_info(f) for f in packet.fields],
                'field_groups': group_bitfields(packet.fields),
            })
        return template.render(
            packets=self.protocol.packets,
            packets_info=packets_info,
            protocol=self.protocol,
            generator=self,
        )

    def generate_protocol_file(self) -> str:
        """Generate protocol.py with packet registry and utilities."""
        template = self.env.get_template('protocol.py.j2')
        packets_info = []
        for packet in self.protocol.packets:
            packet_id = get_packet_id_value(self.protocol, packet)
            packets_info.append({
                'name': packet.name,
                'id': packet_id,
                'id_name': packet.packet_id,
            })
        return template.render(
            packets_info=packets_info,
            protocol=self.protocol,
        )

    def generate_connection_file(self) -> str:
        """Generate connection.py with serial/network communication."""
        template = self.env.get_template('connection.py.j2')
        return template.render(protocol=self.protocol)

    # =========================================================================
    # Cross-language test generation (delegated to TestGenerator)
    # =========================================================================

    def generate_c_test_harness(self, c_header_path: str = None) -> str:
        """Generate C test harness source code.

        Delegates to TestGenerator for the actual generation.

        Args:
            c_header_path: Path to OrionPublicPacket.h to parse for available
                          packets and fields. If None, all packets/fields are included.
        """
        from .test_generator import TestGenerator
        test_gen = TestGenerator(self)
        return test_gen.generate_c_test_harness(c_header_path)

    def generate_cross_validation_test(self, c_header_path: str = None) -> str:
        """Generate Python cross-validation test script.

        Delegates to TestGenerator for the actual generation.

        Args:
            c_header_path: Path to OrionPublicPacket.h to determine which
                          packets have C implementations.
        """
        from .test_generator import TestGenerator
        test_gen = TestGenerator(self)
        return test_gen.generate_cross_validation_test(c_header_path)

    def generate_all(self, include_tests: bool = True, c_header_path: str = None) -> Dict[str, str]:
        """Generate all Python files.

        Args:
            include_tests: If True, also generate cross-language test files
            c_header_path: Path to OrionPublicPacket.h for C test generation

        Returns:
            Dictionary mapping filename to content
        """
        files = {}

        # Generate enums
        files['enums.py'] = self.generate_enums_file()

        # Generate structs
        files['structs.py'] = self.generate_structs_file()

        # Generate packets
        files['packets.py'] = self.generate_packets_file()

        # Generate protocol registry
        files['protocol.py'] = self.generate_protocol_file()

        # Generate connection utilities
        files['connection.py'] = self.generate_connection_file()

        # Generate __init__.py
        files['__init__.py'] = self._generate_init_file()

        # Generate cross-language test files
        if include_tests:
            files['tests/test_harness.c'] = self.generate_c_test_harness(c_header_path)
            files['tests/test_cross_validation.py'] = self.generate_cross_validation_test(c_header_path)

        return files

    def _generate_init_file(self) -> str:
        """Generate __init__.py for the package."""
        return f'''"""Generated Python bindings for {self.protocol.name} protocol.

Protocol version: {self.protocol.version}
Generated by ProtoGen Python.
"""

from .enums import *
from .structs import *
from .packets import *
from .protocol import PacketRegistry, decode_packet, encode_packet
from .connection import OrionConnection

__version__ = "{self.protocol.version}"
__protocol__ = "{self.protocol.name}"
'''

    def write_all(self, include_tests: bool = True, c_header_path: str = None) -> List[str]:
        """Generate and write all files to output directory.

        Args:
            include_tests: If True, also generate cross-language test files
            c_header_path: Path to OrionPublicPacket.h for C test generation

        Returns:
            List of written file paths
        """
        files = self.generate_all(include_tests=include_tests, c_header_path=c_header_path)
        written = []

        for filename, content in files.items():
            filepath = self.output_dir / filename
            # Create parent directories if needed (for tests/ subdirectory)
            filepath.parent.mkdir(parents=True, exist_ok=True)
            filepath.write_text(content, encoding='utf-8')
            written.append(str(filepath))

        return written
