#!/usr/bin/env python3
"""CLI for generating Python bindings from ProtoGen XML.

Usage:
    orion-protogen <input.xml> <output_dir> [--c-header-path <path>]
    python3 -m orion_sdk.cli <input.xml> <output_dir> [--c-header-path <path>]

Examples:
    # Generate bindings only
    python3 -m orion_sdk.cli protocol.xml ./output

    # Generate bindings with test files
    python3 -m orion_sdk.cli protocol.xml ./output --c-header-path ../Protocol.h

    # Generate and compile test harness
    python3 -m orion_sdk.cli protocol.xml ./output --c-header-path ../Protocol.h \\
        --compile-tests --project-root /path/to/project
"""

import sys
import argparse
import subprocess
from pathlib import Path


def check_and_install_deps(requirements_path: Path, verbose: bool = False) -> bool:
    """Check for missing dependencies and install them.

    Args:
        requirements_path: Path to requirements.txt
        verbose: Print installation output

    Returns:
        True if all deps are available (or were installed), False on failure
    """
    if not requirements_path.exists():
        return True  # No requirements file, nothing to do

    # Read required packages
    required = []
    with open(requirements_path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                # Extract package name (remove version specifiers)
                pkg = line.split('>=')[0].split('==')[0].split('<')[0].strip()
                required.append((pkg, line))

    # Check which are missing
    missing = []
    for pkg_name, pkg_spec in required:
        try:
            __import__(pkg_name)
        except ImportError:
            missing.append(pkg_spec)

    if not missing:
        return True

    print(f"Installing missing dependencies: {', '.join(missing)}")
    try:
        cmd = [sys.executable, '-m', 'pip', 'install', '--quiet'] + missing
        result = subprocess.run(cmd, capture_output=not verbose)
        if result.returncode != 0:
            print(f"Warning: Failed to install dependencies", file=sys.stderr)
            return False
        print("Dependencies installed successfully")
        return True
    except Exception as e:
        print(f"Warning: Failed to install dependencies: {e}", file=sys.stderr)
        return False


def main():
    """Generate Python bindings from ProtoGen XML."""
    parser = argparse.ArgumentParser(
        description="Generate Python bindings from ProtoGen XML"
    )
    parser.add_argument("input_xml", type=Path, help="Input ProtoGen XML file")
    parser.add_argument("output_dir", type=Path, help="Output directory for generated code")
    parser.add_argument(
        "--c-header-path",
        type=Path,
        help="Path to OrionPublicPacket.h for C test generation"
    )
    parser.add_argument(
        "--no-tests",
        action="store_true",
        help="Skip generating cross-language test files"
    )
    parser.add_argument(
        "--compile-tests",
        action="store_true",
        help="Compile the C test harness after generation (requires gcc or cl.exe)"
    )
    parser.add_argument(
        "--project-root",
        type=Path,
        help="Root directory of the project (for finding C sources when compiling tests)"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Print verbose output including compilation commands"
    )
    parser.add_argument(
        "--install-deps",
        action="store_true",
        help="Install missing Python dependencies from requirements.txt"
    )

    args = parser.parse_args()

    # Sanitize project_root path (handles Windows batch file trailing backslash issue
    # where \\" escapes the quote and corrupts the path)
    if args.project_root:
        project_root_str = str(args.project_root)
        # Check for corrupted path containing escaped quote artifacts
        if '" ' in project_root_str or project_root_str.endswith('"'):
            # Extract just the valid path portion before any corruption
            clean_path = project_root_str.split('" ')[0].rstrip('"')
            args.project_root = Path(clean_path)

    # Install dependencies if requested
    if args.install_deps:
        pkg_dir = Path(__file__).parent.parent
        requirements_path = pkg_dir / "requirements.txt"
        if not check_and_install_deps(requirements_path, verbose=args.verbose):
            print("Warning: Some dependencies may be missing", file=sys.stderr)

    # Validate argument combinations
    if args.compile_tests and not args.project_root:
        print("Error: --compile-tests requires --project-root", file=sys.stderr)
        return 1

    if args.compile_tests and args.no_tests:
        print("Error: --compile-tests and --no-tests are mutually exclusive", file=sys.stderr)
        return 1

    if not args.input_xml.exists():
        print(f"Error: Input file not found: {args.input_xml}", file=sys.stderr)
        return 1

    # Import here to allow --help without full load
    from .parser.xml_parser import parse_protocol
    from .generator.code_generator import CodeGenerator

    try:
        protocol = parse_protocol(str(args.input_xml))
        generator = CodeGenerator(protocol, str(args.output_dir))
        c_header_path = str(args.c_header_path) if args.c_header_path else None
        written = generator.write_all(
            include_tests=not args.no_tests,
            c_header_path=c_header_path
        )
    except Exception as e:
        print(f"Error generating code: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1

    # Copy runtime support files
    pkg_dir = Path(__file__).parent
    support_files = [
        ('runtime/bitfields.py', 'bitfields.py'),
        ('parser/type_mapping.py', 'type_mapping.py'),
        ('runtime/scaling_runtime.py', 'scaling_runtime.py'),
    ]
    for src_path, dst_name in support_files:
        src = pkg_dir / src_path
        dst = args.output_dir / dst_name
        if src.exists():
            dst.write_text(src.read_text())
            written.append(str(dst))

    print(f"Generated {len(written)} files in {args.output_dir}")

    # Compile test harness if requested
    if args.compile_tests:
        from .build.compiler import compile_test_harness

        test_dir = args.output_dir / "tests"
        test_harness_c = test_dir / "test_harness.c"
        test_harness_exe = test_dir / "test_harness"

        if not test_harness_c.exists():
            print(f"Error: Test harness source not found: {test_harness_c}", file=sys.stderr)
            return 1

        print("Compiling cross-language test harness...")
        success = compile_test_harness(
            project_root=args.project_root,
            test_harness_c=test_harness_c,
            output_path=test_harness_exe,
            verbose=args.verbose
        )

        if success:
            print("Test harness compiled successfully")
        else:
            print("Warning: Test harness compilation failed", file=sys.stderr)
            return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
