"""Test generator for cross-language validation.

This module handles generation of C test harnesses and Python cross-validation
tests. It was extracted from code_generator.py to separate concerns.
"""

import re
from typing import Dict, Any, List, Tuple, Set, Optional

from ..parser.xml_parser import Protocol, FieldDef, PacketDef, get_packet_id_value


class EdgeCaseGenerator:
    """Generates edge case test values for comprehensive coverage."""

    @staticmethod
    def signed_integer_cases(bits: int) -> List[Tuple[str, int]]:
        """Generate edge cases for signed integers."""
        min_val = -(1 << (bits - 1))
        max_val = (1 << (bits - 1)) - 1
        return [
            ("zero", 0),
            ("one", 1),
            ("neg_one", -1),
            ("min", min_val),
            ("max", max_val),
            ("mid_pos", max_val // 2),
            ("mid_neg", min_val // 2),
        ]

    @staticmethod
    def unsigned_integer_cases(bits: int) -> List[Tuple[str, int]]:
        """Generate edge cases for unsigned integers."""
        max_val = (1 << bits) - 1
        return [
            ("zero", 0),
            ("one", 1),
            ("max", max_val),
            ("mid", max_val // 2),
            ("quarter", max_val // 4),
            ("three_quarter", (max_val * 3) // 4),
        ]

    @staticmethod
    def float_cases(min_val: float = None, max_val: float = None) -> List[Tuple[str, float]]:
        """Generate edge cases for floats."""
        cases = [
            ("zero", 0.0),
            ("one", 1.0),
            ("neg_one", -1.0),
            ("small_pos", 1e-6),
            ("small_neg", -1e-6),
        ]
        if min_val is not None:
            cases.append(("min", min_val))
        if max_val is not None:
            cases.append(("max", max_val))
        if min_val is not None and max_val is not None:
            cases.append(("mid", (min_val + max_val) / 2))
        return cases

    @staticmethod
    def bitfield_cases(bits: int, is_signed: bool = False) -> List[Tuple[str, int]]:
        """Generate edge cases for bitfields."""
        max_val = (1 << bits) - 1
        cases = [
            ("all_zeros", 0),
            ("all_ones", max_val),
            ("lsb_only", 1),
        ]
        if bits > 1:
            cases.append(("msb_only", 1 << (bits - 1)))
            cases.append(("alternating_01", 0x55555555 & max_val))
            cases.append(("alternating_10", 0xAAAAAAAA & max_val))

        if is_signed:
            # Two's complement boundaries
            signed_max = (1 << (bits - 1)) - 1
            signed_min = -(1 << (bits - 1))
            cases.extend([
                ("max_positive", signed_max),
                ("min_negative", signed_min),
            ])
        return cases

    @staticmethod
    def array_length_cases(max_len: int, is_variable: bool = False) -> List[Tuple[str, int]]:
        """Generate edge cases for array lengths."""
        if is_variable:
            cases = [
                ("empty", 0),
                ("single", 1),
            ]
            if max_len > 2:
                cases.append(("half", max_len // 2))
            cases.append(("full", max_len))
            return cases
        else:
            # Fixed arrays - just test full length
            return [("full", max_len)]


class TestGenerator:
    """Generates cross-language test harnesses and validation tests.

    This class handles generation of:
    - C test harness source code
    - Python cross-validation test scripts

    It requires a CodeGenerator instance to access the protocol,
    Jinja2 environment, and field info methods.
    """

    def __init__(self, code_generator):
        """Initialize the test generator.

        Args:
            code_generator: CodeGenerator instance with protocol and templates
        """
        self.code_gen = code_generator
        self.protocol = code_generator.protocol
        self.env = code_generator.env

    def get_field_info(self, field: FieldDef) -> Dict[str, Any]:
        """Get field info dict (delegates to code generator)."""
        return self.code_gen.get_field_info(field)

    def _parse_function_params(self, params_str: str) -> List[Dict[str, str]]:
        """Parse C function parameters into type/name pairs.

        Args:
            params_str: Parameter string like "void* pkt, float Undulation"

        Returns:
            List of dicts with 'type' and 'name' keys (excluding void* pkt)
        """
        params = []
        for param in params_str.split(','):
            param = param.strip()
            if not param or 'void*' in param or 'void *' in param:
                continue  # Skip the pkt parameter

            # Handle various C type formats
            # Examples: "float Undulation", "uint8_t Index", "crownModes mode", "OrionRetractCmd_t Cmd"
            # Pattern: everything before the last word is the type, last word is the name
            parts = param.rsplit(None, 1)
            if len(parts) == 2:
                c_type, name = parts
                params.append({
                    'type': c_type.strip(),
                    'name': name.strip(),
                })
        return params

    def _get_simple_packet_test_values(self, packet: PacketDef, params: List[Dict[str, str]]) -> List[str]:
        """Generate C test values for a simple packet's parameters.

        Uses the same logic as regular packets to ensure consistency with Python tests.

        Args:
            packet: The packet definition
            params: List of parameter dicts with 'type' and 'name'

        Returns:
            List of C literal strings for test values
        """
        test_values = []

        for param in params:
            param_name = param['name']
            c_type = param['type']

            # Find the matching field in the packet's XML definition
            matching_field = None
            for field in packet.fields:
                if field.name == param_name:
                    matching_field = field
                    break

            if matching_field:
                # Use field info to generate consistent test value
                info = self.get_field_info(matching_field)

                # Handle scaled fields
                if info.get('has_scaling'):
                    scaling = info.get('scaling', {})
                    phys_min, phys_max = self._get_physical_range(scaling)
                    val = (phys_min + phys_max) / 2

                    # Check in-memory type
                    in_mem_type = matching_field.in_memory_type.lower() if matching_field.in_memory_type else ''
                    is_int_type = ('int' in in_mem_type or 'unsigned' in in_mem_type or
                                   'signed' in in_mem_type) and 'float' not in in_mem_type

                    if is_int_type:
                        test_values.append(str(int(val)))
                    else:
                        is_double = 'double' in in_mem_type or 'float64' in in_mem_type
                        c_suffix = "" if is_double else "f"
                        test_values.append(f"{val:.6f}{c_suffix}")
                elif info.get('is_float'):
                    test_values.append('1.5f')
                elif info.get('is_enum'):
                    # Use 0 for enums (first value)
                    test_values.append('0')
                else:
                    # Integer
                    test_values.append('42')
            else:
                # Fallback based on C type
                c_type_lower = c_type.lower()
                if 'float' in c_type_lower:
                    test_values.append('1.5f')
                elif 'double' in c_type_lower:
                    test_values.append('1.5')
                elif c_type.endswith('_t'):
                    test_values.append('0')  # Enum or struct
                else:
                    test_values.append('42')

        return test_values

    def _get_c_printf_format_for_param(self, param: Dict[str, str], packet: PacketDef) -> str:
        """Get printf format specifier for a simple packet parameter.

        Uses the XML field info to determine the correct format.

        Args:
            param: Parameter dict with 'type' and 'name'
            packet: The packet definition

        Returns:
            Printf format string
        """
        param_name = param['name']
        c_type = param['type']

        # Find the matching field in the packet's XML definition
        for field in packet.fields:
            if field.name == param_name:
                info = self.get_field_info(field)
                return self._get_printf_format(info, field)

        # Fallback based on C type
        c_type_lower = c_type.lower()
        if 'float' in c_type_lower:
            return '%.9g'
        elif 'double' in c_type_lower:
            return '%.17g'
        elif 'uint64' in c_type_lower:
            return '%\" PRIu64 \"'
        elif 'int64' in c_type_lower:
            return '%\" PRId64 \"'
        elif 'uint' in c_type_lower or 'unsigned' in c_type_lower:
            return '%u'
        elif c_type.endswith('_t'):
            return '%d'
        else:
            return '%d'

    def parse_c_header(self, header_path: str) -> Tuple[Set[str], Dict[str, Set[str]], Dict[str, str]]:
        """Parse C header to find available packets and struct fields.

        Args:
            header_path: Path to OrionPublicPacket.h

        Returns:
            Tuple of:
            - Set of packet names that have PacketStructure encode/decode functions
            - Dict mapping struct names to sets of field names
            - Dict mapping packet names to their Packet function signatures (for wrappers)
        """
        try:
            with open(header_path, 'r') as f:
                content = f.read()
        except FileNotFoundError:
            return set(), {}, {}

        # Also try to read the shim header for macro definitions
        shim_content = ""
        shim_path = header_path.replace('OrionPublicPacket.h', '../Utils/OrionPublicPacketShim.h')
        try:
            with open(shim_path, 'r') as f:
                shim_content = f.read()
        except FileNotFoundError:
            pass

        # Find packets with PacketStructure functions
        # Pattern: encode<Name>PacketStructure(void* pkt, const <Name>_t* user)
        packet_pattern = r'void encode(\w+)PacketStructure\s*\(\s*void\s*\*\s*pkt\s*,\s*const\s+\1_t\s*\*'
        packets_with_struct = set(re.findall(packet_pattern, content))

        # Also check for macro definitions in shim: #define encode<Name>PacketStructure
        macro_pattern = r'#define\s+encode(\w+)PacketStructure\s+'
        packets_with_struct.update(re.findall(macro_pattern, shim_content))

        # Find packets with simple Packet functions (not PacketStructure)
        # Pattern: void encode<Name>Packet(void* pkt, <params>);
        # We need to capture the full signature for wrapper generation
        packet_fn_pattern = r'void encode(\w+)Packet\s*\(([^)]+)\)\s*;'
        packet_signatures = {}
        for match in re.finditer(packet_fn_pattern, content):
            name = match.group(1)
            # Skip if this packet already has PacketStructure function (or macro alias)
            if name in packets_with_struct:
                continue
            # Also skip PacketStructure matches (the pattern catches "PacketStructure" as "Packet" + "Structure")
            if name.endswith('Structure'):
                continue
            params = match.group(2)
            # Parse the parameters into (type, name) pairs
            parsed_params = self._parse_function_params(params)
            packet_signatures[name] = {
                'raw': params,
                'params': parsed_params,
            }

        # Parse struct definitions to get field names
        # Pattern: typedef struct { ... } <Name>_t;
        struct_fields = {}
        struct_pattern = r'typedef struct\s*\{([^}]+)\}(\w+_t);'
        for match in re.finditer(struct_pattern, content, re.DOTALL):
            body = match.group(1)
            struct_name = match.group(2)

            # Extract field names from struct body
            # Pattern: <type> <name>; or <type> <name>[<size>]; or <type> <name> : <bits>;
            field_pattern = r'^\s*[\w\s\*]+\s+(\w+)(?:\s*\[\s*\w+\s*\]|\s*:\s*\d+)?\s*;'
            fields = set()
            for line in body.split('\n'):
                field_match = re.match(field_pattern, line.strip())
                if field_match:
                    fields.add(field_match.group(1))

            # Remove _t suffix for lookup
            base_name = struct_name[:-2] if struct_name.endswith('_t') else struct_name
            struct_fields[base_name] = fields

        return packets_with_struct, struct_fields, packet_signatures

    def _get_physical_range(self, scaling: Dict[str, Any]) -> Tuple[float, float]:
        """Compute the actual physical value range from scaling parameters.

        The scaling dict contains encoded range values when explicit min/max
        weren't specified in XML. This method computes the true physical range.

        Args:
            scaling: Dictionary with 'min', 'max', 'scaler', 'is_signed', 'bits'

        Returns:
            Tuple of (physical_min, physical_max)
        """
        scaler = scaling.get('scaler', 1.0)
        is_signed = scaling.get('is_signed', False)
        bits = scaling.get('bits', 32)
        stored_min = scaling.get('min', 0)
        stored_max = scaling.get('max', 1)

        if scaler == 0:
            scaler = 1.0

        # Compute default encoded max for this type
        if is_signed:
            default_encoded_max = float((1 << (bits - 1)) - 1)
        else:
            default_encoded_max = float((1 << bits) - 1)

        # If stored_max equals default encoded max and scaler != 1.0, these are
        # default values (no explicit min/max in XML) and we need to compute
        # physical range from the scaler
        if stored_max == default_encoded_max and scaler != 1.0:
            if is_signed:
                # Signed: physical range spans negative to positive
                phys_max = default_encoded_max / scaler
                min_encoded = -(1 << (bits - 1))
                phys_min = min_encoded / scaler
            else:
                # Unsigned: physical min is stored_min (offset), max from encoded/scaler
                phys_min = stored_min
                phys_max = (default_encoded_max / scaler) + stored_min
            return (phys_min, phys_max)
        else:
            # These are actual physical range values (explicit min/max in XML)
            return (stored_min, stored_max)

    def _get_test_value(self, field: FieldDef, info: Dict[str, Any],
                        c_fields: Set[str] = None) -> Tuple[Any, str, str]:
        """Generate deterministic test value for a field.

        Args:
            field: Field definition
            info: Field info dict
            c_fields: Set of field names that exist in the C struct (if None, include all)

        Returns:
            Tuple of (python_value, python_assignment, c_assignment)
        """
        field_name = field.name

        # Skip null-encoded or null in-memory fields (don't exist in C struct)
        if info.get('is_null_encoded') or info.get('is_null_in_memory'):
            return None, None, None

        # Constant fields use constant value
        if info.get('constant') is not None:
            return None, None, None  # Constants are auto-set

        # Skip fields that don't exist in C struct
        if c_fields is not None and field_name not in c_fields:
            # Return Python assignment only, no C assignment
            py_val, py_assign, _ = self._get_test_value_impl(field, info)
            return py_val, py_assign, None

        return self._get_test_value_impl(field, info)

    def _get_edge_case_values(self, field: FieldDef, info: Dict[str, Any],
                              case_name: str = "default") -> List[Tuple[str, Any]]:
        """Get edge case test values for a field.

        Args:
            field: Field definition
            info: Field info dict
            case_name: Specific case to generate ("default" for all cases)

        Returns:
            List of (case_name, value) tuples
        """
        cases = []

        # Skip non-encodable fields
        if info.get('is_null_encoded') or info.get('is_null_in_memory') or info.get('constant'):
            return cases

        # Struct fields - defer to nested handling
        if info.get('is_struct'):
            return cases

        # Enums - use first few values
        if info.get('is_enum'):
            enum_ref = info.get('enum_ref')
            enum_def = self.protocol.enums.get(enum_ref)
            if enum_def and enum_def.values:
                for i, val in enumerate(enum_def.values[:3]):  # First 3 values
                    if not getattr(val, 'hidden', False):
                        cases.append((f"enum_{i}", val.name))
            return cases

        # Bitfields
        if info.get('is_bitfield'):
            bits = info.get('bitfield_bits', 1)
            is_signed = info.get('is_signed', False)
            return EdgeCaseGenerator.bitfield_cases(bits, is_signed)

        # Scaled fields
        if info.get('has_scaling'):
            scaling = info.get('scaling', {})
            phys_min, phys_max = self._get_physical_range(scaling)
            return EdgeCaseGenerator.float_cases(phys_min, phys_max)

        # Regular numeric fields
        if info.get('is_float'):
            return EdgeCaseGenerator.float_cases()

        # Integer fields
        bit_width = info.get('bit_width', 32)
        if info.get('is_signed'):
            return EdgeCaseGenerator.signed_integer_cases(bit_width)
        else:
            return EdgeCaseGenerator.unsigned_integer_cases(bit_width)

    def _generate_test_case_set(self, packet: PacketDef, fields_info: List[Dict[str, Any]],
                                c_fields: Set[str] = None,
                                case_type: str = "default") -> Dict[str, Any]:
        """Generate a complete test case set for a packet.

        Args:
            packet: Packet definition
            fields_info: List of field info dicts
            c_fields: Set of field names in C struct
            case_type: Type of test case ("default", "min", "max", "zero", "random")

        Returns:
            Dict with py_assignments, c_assignments, py_values, case_name
        """
        py_assignments = []
        c_assignments = []
        py_values = {}

        # Collect variable array specifiers
        variable_array_specifiers = set()
        for info in fields_info:
            if info.get('variable_array'):
                variable_array_specifiers.add(info['variable_array'])

        for field, info in zip(packet.fields, fields_info):
            field_name = field.name
            field_in_c = c_fields is None or field_name in c_fields

            # Variable array specifiers get 0
            if field_name in variable_array_specifiers:
                py_assignments.append(f"pkt.{field_name} = 0")
                py_values[field_name] = 0
                if field_in_c:
                    c_assignments.append(f"user.{field_name} = 0")
                continue

            # Get edge case values
            edge_cases = self._get_edge_case_values(field, info)
            if not edge_cases:
                # Use default test value
                _, py_assign, c_assign = self._get_test_value(field, info, c_fields)
                if py_assign:
                    py_assignments.append(py_assign)
                if c_assign and field_in_c:
                    c_assignments.append(c_assign)
                continue

            # Select value based on case_type
            if case_type == "default" and edge_cases:
                # Use first non-boundary case (usually "zero" or "one")
                selected = edge_cases[0] if len(edge_cases) == 1 else edge_cases[1]
            elif case_type == "min" and any(c[0] == "min" for c in edge_cases):
                selected = next(c for c in edge_cases if c[0] == "min")
            elif case_type == "max" and any(c[0] == "max" for c in edge_cases):
                selected = next(c for c in edge_cases if c[0] == "max")
            elif case_type == "zero" and any(c[0] in ("zero", "all_zeros") for c in edge_cases):
                selected = next(c for c in edge_cases if c[0] in ("zero", "all_zeros"))
            else:
                selected = edge_cases[0]

            case_name, value = selected
            py_values[field_name] = value

            # Generate assignments
            if info.get('is_enum'):
                enum_ref = info.get('enum_ref')
                py_assignments.append(f"pkt.{field_name} = {enum_ref}.{value}")
                if field_in_c:
                    c_assignments.append(f"user.{field_name} = {value}")
            elif info.get('is_float') or (info.get('has_scaling') and not info.get('is_bitfield')):
                # Check for double type
                in_mem_type = field.in_memory_type.lower() if field and field.in_memory_type else ''
                is_double = 'double' in in_mem_type or 'float64' in in_mem_type
                c_suffix = "" if is_double else "f"
                py_assignments.append(f"pkt.{field_name} = {value}")
                if field_in_c:
                    c_assignments.append(f"user.{field_name} = {value}{c_suffix}")
            else:
                py_assignments.append(f"pkt.{field_name} = {value}")
                if field_in_c:
                    c_assignments.append(f"user.{field_name} = {value}")

        return {
            'case_name': case_type,
            'py_assignments': py_assignments,
            'c_assignments': c_assignments,
            'py_values': py_values,
        }

    def _get_test_value_impl(self, field: FieldDef, info: Dict[str, Any]) -> Tuple[Any, str, str]:
        """Implementation of test value generation."""
        field_name = field.name

        # Handle structs (nested)
        if info.get('is_struct'):
            struct_ref = info.get('struct_ref')
            if info.get('is_array'):
                # Struct array - just use defaults
                return None, None, None
            else:
                # Single struct - use default constructor
                return None, None, None

        # Handle enums
        if info.get('is_enum'):
            enum_ref = info.get('enum_ref')
            enum_def = self.protocol.enums.get(enum_ref)
            if enum_def and enum_def.values:
                # Use first non-hidden value
                for val in enum_def.values:
                    if not getattr(val, 'hidden', False):
                        py_val = f"{enum_ref}.{val.name}"
                        c_val = val.name
                        py_assign = f"pkt.{field_name} = {py_val}"
                        c_assign = f"user.{field_name} = {c_val}"
                        return py_val, py_assign, c_assign
            return 0, f"pkt.{field_name} = 0", f"user.{field_name} = 0"

        # Handle strings
        if info.get('is_string'):
            test_str = field_name[:8]  # Truncate to 8 chars
            py_assign = f'pkt.{field_name} = "{test_str}"'
            c_assign = f'strncpy(user.{field_name}, "{test_str}", sizeof(user.{field_name})-1)'
            return f'"{test_str}"', py_assign, c_assign

        # Handle arrays (non-struct)
        if info.get('is_array'):
            array_len = info.get('array_length', 0)
            if array_len and not info.get('variable_array'):
                # Generate array with incrementing values
                if info.get('has_scaling'):
                    scaling = info.get('scaling', {})
                    # Compute physical range from scaler
                    phys_min, phys_max = self._get_physical_range(scaling)
                    # Check for double type (no 'f' suffix)
                    in_mem_type = field.in_memory_type.lower() if field and field.in_memory_type else ''
                    is_double = 'double' in in_mem_type or 'float64' in in_mem_type
                    c_suffix = "" if is_double else "f"
                    # Use values spread across the range
                    py_vals = []
                    c_assigns = []
                    for i in range(array_len):
                        val = phys_min + (phys_max - phys_min) * (i + 1) / (array_len + 1)
                        py_vals.append(f"{val:.6f}")
                        c_assigns.append(f"user.{field_name}[{i}] = {val:.6f}{c_suffix}")
                    py_assign = f"pkt.{field_name} = [{', '.join(py_vals)}]"
                    c_assign = "; ".join(c_assigns)
                    return py_vals, py_assign, c_assign
                else:
                    # Integer array
                    py_vals = [str(i % 256) for i in range(array_len)]
                    c_assigns = [f"user.{field_name}[{i}] = {i % 256}" for i in range(array_len)]
                    py_assign = f"pkt.{field_name} = [{', '.join(py_vals)}]"
                    c_assign = "; ".join(c_assigns)
                    return py_vals, py_assign, c_assign
            return None, None, None

        # Handle scaled fields (non-array)
        if info.get('has_scaling'):
            scaling = info.get('scaling', {})
            # Compute physical range from scaler
            phys_min, phys_max = self._get_physical_range(scaling)
            # Use midpoint
            val = (phys_min + phys_max) / 2

            # Check if in-memory type is an integer (not float)
            # For integer types with explicit scaler, use integer test values
            in_mem_type = field.in_memory_type.lower() if field and field.in_memory_type else ''
            is_int_type = ('int' in in_mem_type or 'unsigned' in in_mem_type or
                           'signed' in in_mem_type) and 'float' not in in_mem_type

            if is_int_type:
                val = int(val)
                py_assign = f"pkt.{field_name} = {val}"
                c_assign = f"user.{field_name} = {val}"
            else:
                # Use 'f' suffix only for float32, not for float64/double
                is_double = 'double' in in_mem_type or 'float64' in in_mem_type
                c_suffix = "" if is_double else "f"
                py_assign = f"pkt.{field_name} = {val:.6f}"
                c_assign = f"user.{field_name} = {val:.6f}{c_suffix}"
            return val, py_assign, c_assign

        # Handle bitfields
        if info.get('is_bitfield'):
            bits = info.get('bitfield_bits', 1)
            val = 1 if bits == 1 else (1 << (bits - 1)) - 1  # Use 1 for bool, or mid-range
            py_assign = f"pkt.{field_name} = {val}"
            c_assign = f"user.{field_name} = {val}"
            return val, py_assign, c_assign

        # Handle regular integers/floats
        is_float = info.get('is_float', False)
        if is_float:
            py_assign = f"pkt.{field_name} = 1.5"
            c_assign = f"user.{field_name} = 1.5f"
            return 1.5, py_assign, c_assign
        else:
            py_assign = f"pkt.{field_name} = 42"
            c_assign = f"user.{field_name} = 42"
            return 42, py_assign, c_assign

    def _get_printf_format(self, info: Dict[str, Any], field: FieldDef = None) -> str:
        """Get C printf format specifier for a field type.

        Note: Scaled fields with float inMemoryType are stored as float in C struct.
        Scaled fields with integer inMemoryType are stored as int.
        """
        # Check inMemoryType for the actual storage type
        if field and field.in_memory_type:
            mem_type = field.in_memory_type.lower()
            if 'float' in mem_type or 'double' in mem_type:
                return "%f"
            # Integer type with scaling - still stored as integer in C
            if info.get('has_scaling'):
                if 'int' in mem_type or 'unsigned' in mem_type or 'signed' in mem_type:
                    # Use integer format based on signedness
                    if 'signed' in mem_type and 'unsigned' not in mem_type:
                        return "%d"
                    return "%u"

        # Default: scaled fields with unknown type assumed to be float
        if info.get('has_scaling'):
            return "%f"

        if info.get('is_float'):
            return "%f"
        if info.get('is_signed'):
            type_size = info.get('type_size', 4)
            if type_size == 8:
                return '%" PRId64 "'  # Use PRId64 for int64_t portability
            return "%d"
        type_size = info.get('type_size', 4)
        if type_size == 8:
            return '%" PRIu64 "'  # Use PRIu64 for uint64_t portability
        return "%u"

    def _get_packet_test_info(self, packet: PacketDef,
                              c_fields: Set[str] = None,
                              has_c_impl: bool = True) -> Dict[str, Any]:
        """Get comprehensive test information for a packet.

        Args:
            packet: Packet definition
            c_fields: Set of field names that exist in C struct (None = all fields)
            has_c_impl: Whether this packet has C PacketStructure functions
        """
        fields_info = [self.get_field_info(f) for f in packet.fields]
        c_assignments = []
        py_assignments = []
        c_field_outputs = []

        # Collect field names that are variableArray length specifiers
        # These fields control the length of variable arrays, so we need to set them to 0
        # to ensure Python and C encode the same number of elements (0)
        variable_array_specifiers = set()
        for field, info in zip(packet.fields, fields_info):
            if info.get('variable_array'):
                variable_array_specifiers.add(info['variable_array'])

        # Collect field names that are dependsOn triggers (enable conditional fields)
        # For these fields, we need non-zero values to encode the conditional fields
        depends_on_triggers = set()
        for field, info in zip(packet.fields, fields_info):
            if info.get('depends_on'):
                depends_on_triggers.add(info['depends_on'])

        # Track conditional fields for conditional testing
        conditional_fields = []

        # Track field metadata for value comparison
        field_metadata = []

        for field, info in zip(packet.fields, fields_info):
            field_name = field.name

            # Check if field exists in C struct
            field_in_c = c_fields is None or field_name in c_fields

            # Track conditional fields
            if info.get('depends_on'):
                conditional_fields.append({
                    'name': field_name,
                    'depends_on': info['depends_on'],
                    'depends_on_value': info.get('depends_on_value'),
                })

            # Build field metadata for comparison
            if not info.get('is_null_encoded') and not info.get('is_null_in_memory') and not info.get('constant'):
                field_meta = {
                    'name': field_name,
                    'type_hint': info.get('type_hint', 'Any'),
                    'is_float': info.get('is_float', False) or info.get('has_scaling', False),
                    'is_array': info.get('is_array', False),
                    'is_struct': info.get('is_struct', False),
                    'is_enum': info.get('is_enum', False),
                    'is_string': info.get('is_string', False),
                    'is_bitfield': info.get('is_bitfield', False),
                    'struct_ref': info.get('struct_ref'),
                    'enum_ref': info.get('enum_ref'),
                    'array_length': info.get('array_length'),
                    'variable_array': info.get('variable_array'),
                    'in_c_struct': field_in_c,
                }
                if info.get('has_scaling'):
                    scaling = info.get('scaling', {})
                    # Calculate tolerance based on scaling precision
                    # Use 2 LSB tolerance, but also account for float32 precision limits
                    scaler = scaling.get('scaler', 1.0)
                    phys_min = scaling.get('min', 0)
                    phys_max = scaling.get('max', 1)
                    if scaler and scaler != 0:
                        lsb_tolerance = 2.0 / abs(scaler)  # 2 LSB tolerance
                        # Float32 has ~7 significant digits, so relative precision is ~1e-6
                        # For large values, need to increase absolute tolerance
                        max_abs_val = max(abs(phys_min), abs(phys_max), 1.0)
                        float32_tolerance = max_abs_val * 1e-5  # Slightly higher for safety
                        field_meta['tolerance'] = max(lsb_tolerance, float32_tolerance)
                    else:
                        field_meta['tolerance'] = 1e-6
                field_metadata.append(field_meta)

            # For variableArray length specifiers, use 0 so that both C and Python
            # encode 0 variable array elements (avoids mismatches from empty lists)
            if field_name in variable_array_specifiers:
                py_assign = f"pkt.{field_name} = 0"
                c_assign = f"user.{field_name} = 0"
                py_assignments.append(py_assign)
                if field_in_c:
                    c_assignments.append(c_assign)
                continue

            # For dependsOn triggers, use non-zero values to enable conditional fields
            if field_name in depends_on_triggers:
                if info.get('is_enum'):
                    # Use second enum value (non-zero) if available
                    enum_ref = info.get('enum_ref')
                    enum_def = self.protocol.enums.get(enum_ref)
                    if enum_def and len(enum_def.values) > 1:
                        # Use second value (index 1) to ensure non-zero
                        val = enum_def.values[1]
                        py_assign = f"pkt.{field_name} = {enum_ref}.{val.name}"
                        c_assign = f"user.{field_name} = {val.name}"
                    else:
                        # Only one enum value, use 1
                        py_assign = f"pkt.{field_name} = 1"
                        c_assign = f"user.{field_name} = 1"
                else:
                    # Integer/boolean trigger - use 1
                    py_assign = f"pkt.{field_name} = 1"
                    c_assign = f"user.{field_name} = 1"
                py_assignments.append(py_assign)
                if field_in_c:
                    c_assignments.append(c_assign)
                continue

            # Get test values and assignments
            _, py_assign, c_assign = self._get_test_value(field, info, c_fields)
            if py_assign:
                py_assignments.append(py_assign)
            if c_assign and field_in_c:
                c_assignments.append(c_assign)

            # Build C output statements for decode (only for fields in C struct)
            if not field_in_c:
                continue
            # Skip fields that don't exist in C struct (null in-memory or null-encoded)
            if info.get('is_null_encoded') or info.get('is_null_in_memory') or info.get('constant'):
                continue

            if info.get('is_struct'):
                # Output nested struct fields with prefix
                # Skip struct arrays (complex output, not supported yet)
                if info.get('is_array'):
                    continue
                struct_ref = info.get('struct_ref')
                struct_def = self.protocol.structs.get(struct_ref)
                if struct_def:
                    for sf in struct_def.fields:
                        sf_info = self.get_field_info(sf)
                        if sf_info.get('is_null_encoded') or sf_info.get('is_null_in_memory'):
                            continue
                        # Skip nested struct arrays within the struct
                        if sf_info.get('is_array'):
                            continue
                        fmt = self._get_printf_format(sf_info, sf)
                        c_field_outputs.append(
                            f'printf("{field_name}.{sf.name}={fmt}\\n", user.{field_name}.{sf.name});'
                        )
            elif info.get('is_string'):
                c_field_outputs.append(f'printf("{field_name}=%s\\n", user.{field_name});')
            elif info.get('is_array') and info.get('variable_array'):
                # Variable-length array - use length field to determine output
                length_field = info.get('variable_array')
                fmt = self._get_printf_format(info, field)
                c_field_outputs.append(f'printf("{field_name}=");')
                c_field_outputs.append(f'for (int i = 0; i < user.{length_field}; i++) {{')
                c_field_outputs.append(f'    printf("{fmt}%s", user.{field_name}[i], i < user.{length_field}-1 ? "," : "");')
                c_field_outputs.append(f'}}')
                c_field_outputs.append(f'printf("\\n");')
            elif info.get('is_array'):
                # Fixed-length array
                array_len = info.get('array_length', 0)
                fmt = self._get_printf_format(info, field)
                c_field_outputs.append(f'printf("{field_name}=");')
                c_field_outputs.append(f'for (int i = 0; i < {array_len}; i++) {{')
                c_field_outputs.append(f'    printf("{fmt}%s", user.{field_name}[i], i < {array_len}-1 ? "," : "");')
                c_field_outputs.append(f'}}')
                c_field_outputs.append(f'printf("\\n");')
            elif info.get('is_enum'):
                c_field_outputs.append(f'printf("{field_name}=%d\\n", (int)user.{field_name});')
            else:
                fmt = self._get_printf_format(info, field)
                c_field_outputs.append(f'printf("{field_name}={fmt}\\n", user.{field_name});')

        # Generate edge case test sets
        edge_case_sets = []
        for case_type in ["default", "zero", "min", "max"]:
            case_set = self._generate_test_case_set(packet, fields_info, c_fields, case_type)
            if case_set['py_assignments']:  # Only include if there are assignments
                edge_case_sets.append(case_set)

        return {
            'packet': packet,
            'packet_id': get_packet_id_value(self.protocol, packet),
            'fields_info': fields_info,
            'field_metadata': field_metadata,
            'c_assignments': c_assignments,
            'py_assignments': py_assignments,
            'c_field_outputs': c_field_outputs,
            'has_c_impl': has_c_impl,
            'conditional_fields': conditional_fields,
            'edge_case_sets': edge_case_sets,
            'has_variable_arrays': bool(variable_array_specifiers),
            'variable_array_specifiers': list(variable_array_specifiers),
        }

    def generate_c_test_harness(self, c_header_path: str = None) -> str:
        """Generate C test harness source code.

        Args:
            c_header_path: Path to OrionPublicPacket.h to parse for available
                          packets and fields. If None, all packets/fields are included.
        """
        template = self.env.get_template('test_harness.c.j2')

        # Parse C header to get available packets and struct fields
        if c_header_path:
            c_packets, c_struct_fields, packet_signatures = self.parse_c_header(c_header_path)
        else:
            c_packets, c_struct_fields, packet_signatures = set(), {}, {}

        packets_info = []
        for p in self.protocol.packets:
            # Check if packet has a C struct (required for struct-based test harness)
            has_struct = p.name in c_struct_fields if c_header_path else True

            # Packet has C impl if:
            # 1. Has PacketStructure function, OR
            # 2. Has simple Packet function (with or without _t struct)
            has_c_impl = not c_header_path or p.name in c_packets or (
                p.name in packet_signatures
            )
            c_fields = c_struct_fields.get(p.name) if c_header_path else None
            pkt_info = self._get_packet_test_info(p, c_fields, has_c_impl)

            # Determine the type of C implementation
            if c_header_path and p.name in packet_signatures:
                sig_info = packet_signatures[p.name]
                if has_struct:
                    # Has both Packet function and _t struct - needs wrapper
                    pkt_info['needs_wrapper'] = True
                    pkt_info['packet_fn_signature'] = sig_info['raw']
                    pkt_info['is_simple_packet'] = False
                else:
                    # Has Packet function but NO _t struct - simple packet
                    pkt_info['needs_wrapper'] = False
                    pkt_info['is_simple_packet'] = True
                    pkt_info['simple_params'] = sig_info['params']
                    # Generate test values consistent with Python tests
                    pkt_info['simple_test_values'] = self._get_simple_packet_test_values(
                        p, sig_info['params']
                    )
                    pkt_info['simple_printf_formats'] = [
                        self._get_c_printf_format_for_param(param, p)
                        for param in sig_info['params']
                    ]
            else:
                pkt_info['needs_wrapper'] = False
                pkt_info['is_simple_packet'] = False

            packets_info.append(pkt_info)

        return template.render(
            packets_info=packets_info,
            protocol=self.protocol,
            generator=self.code_gen,
        )

    def generate_cross_validation_test(self, c_header_path: str = None) -> str:
        """Generate Python cross-validation test script.

        Args:
            c_header_path: Path to OrionPublicPacket.h to determine which
                          packets have C implementations.
        """
        template = self.env.get_template('test_cross_validation.py.j2')

        # Parse C header to get available packets
        if c_header_path:
            c_packets, c_struct_fields, packet_signatures = self.parse_c_header(c_header_path)
        else:
            c_packets, c_struct_fields, packet_signatures = set(), {}, {}

        packets_info = []
        for p in self.protocol.packets:
            # Check if packet has a C struct (required for struct-based test harness)
            has_struct = p.name in c_struct_fields if c_header_path else True

            # Packet has C impl if:
            # 1. Has PacketStructure function, OR
            # 2. Has simple Packet function (with or without _t struct)
            has_c_impl = not c_header_path or p.name in c_packets or (
                p.name in packet_signatures
            )
            c_fields = c_struct_fields.get(p.name) if c_header_path else None
            pkt_info = self._get_packet_test_info(p, c_fields, has_c_impl)

            # Mark simple packets (no _t struct)
            if c_header_path and p.name in packet_signatures and not has_struct:
                pkt_info['is_simple_packet'] = True
                pkt_info['simple_params'] = packet_signatures[p.name]['params']
            else:
                pkt_info['is_simple_packet'] = False

            packets_info.append(pkt_info)

        return template.render(
            packets_info=packets_info,
            protocol=self.protocol,
            generator=self.code_gen,
        )
