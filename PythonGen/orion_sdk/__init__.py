"""Orion SDK - Python bindings for Trillium Orion gimbal protocol.

This package provides:
- Code generation from ProtoGen XML protocol definitions
- Runtime support for packet encoding/decoding
- Connection utilities for serial and network communication

Basic usage:
    # Generate Python bindings from XML
    from orion_sdk import generate_bindings
    generate_bindings('OrionPublicProtocol.xml', 'output/')

    # Or use the CLI:
    # orion-protogen OrionPublicProtocol.xml output/

Example with generated code:
    from output import OrionCmd, OrionConnection

    conn = OrionConnection()
    conn.open_network()  # Auto-discover gimbal

    cmd = OrionCmd()
    cmd.Cmd.Target = [0.5, 0.3]  # Pan, tilt in radians
    cmd.Cmd.Mode = 16  # Rate mode
    conn.send(cmd)
"""

__version__ = "1.0.0"

# Import from package modules
from .parser.xml_parser import parse_protocol, Protocol
from .generator.code_generator import CodeGenerator


def generate_bindings(xml_path: str, output_dir: str, template_dir: str = None) -> list:
    """Generate Python bindings from a ProtoGen XML file.

    Args:
        xml_path: Path to the ProtoGen XML protocol definition
        output_dir: Directory to write generated Python files
        template_dir: Optional custom template directory

    Returns:
        List of generated file paths

    Example:
        >>> from orion_sdk import generate_bindings
        >>> files = generate_bindings('OrionPublicProtocol.xml', 'orion/')
        >>> print(files)
        ['orion/enums.py', 'orion/structs.py', 'orion/packets.py', ...]
    """
    protocol = parse_protocol(xml_path)
    generator = CodeGenerator(protocol, output_dir, template_dir)
    return generator.write_all()


__all__ = [
    'generate_bindings',
    'CodeGenerator',
    'parse_protocol',
    'Protocol',
    '__version__',
]
