"""Cross-platform test harness compilation.

This module provides functions to compile the C test harness on both
Linux/macOS (gcc) and Windows (MSVC cl.exe).
"""

import os
import shutil
import subprocess
import platform
import sys
from pathlib import Path
from typing import Optional

from .build_config import TEST_HARNESS_SOURCES, INCLUDE_DIRS, GCC_FLAGS, MSVC_FLAGS


def find_compiler(verbose: bool = False) -> Optional[str]:
    """Find available C compiler. Returns 'gcc', 'cl', or None."""
    system = platform.system()

    if verbose:
        print(f"Debug: Platform is {system}", file=sys.stderr)

    if system == "Windows":
        if shutil.which("cl"):
            if verbose:
                print("Debug: Found cl.exe in PATH", file=sys.stderr)
            return "cl"
        if verbose:
            print("Debug: cl.exe not in PATH, trying to set up MSVC environment...", file=sys.stderr)
        if _setup_msvc_env(verbose=verbose):
            return "cl"
        if shutil.which("gcc"):
            if verbose:
                print("Debug: Found gcc in PATH", file=sys.stderr)
            return "gcc"
    else:
        if shutil.which("gcc"):
            return "gcc"
        if shutil.which("clang"):
            return "clang"

    return None


def _setup_msvc_env(verbose: bool = False) -> bool:
    """Try to find and activate MSVC environment on Windows."""
    vswhere = Path(r"C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe")
    if not vswhere.exists():
        if verbose:
            print(f"Debug: vswhere.exe not found at {vswhere}", file=sys.stderr)
        return False

    try:
        result = subprocess.run(
            [str(vswhere), "-latest", "-property", "installationPath"],
            capture_output=True, text=True, timeout=10
        )
        vs_path = result.stdout.strip()
        if not vs_path:
            if verbose:
                print("Debug: vswhere returned empty installation path", file=sys.stderr)
            return False

        if verbose:
            print(f"Debug: Found Visual Studio at {vs_path}", file=sys.stderr)

        vcvarsall = Path(vs_path) / "VC" / "Auxiliary" / "Build" / "vcvarsall.bat"
        if not vcvarsall.exists():
            if verbose:
                print(f"Debug: vcvarsall.bat not found at {vcvarsall}", file=sys.stderr)
            return False

        # Run vcvarsall and capture the resulting environment
        result = subprocess.run(
            f'"{vcvarsall}" x64 >nul 2>nul && set',
            capture_output=True, text=True, shell=True, timeout=30
        )
        if result.returncode != 0:
            if verbose:
                print(f"Debug: vcvarsall.bat failed with return code {result.returncode}", file=sys.stderr)
            return False

        # Apply the environment variables to our process
        for line in result.stdout.splitlines():
            if "=" in line:
                key, _, value = line.partition("=")
                os.environ[key] = value

        cl_found = shutil.which("cl") is not None
        if verbose:
            print(f"Debug: cl.exe found after vcvarsall: {cl_found}", file=sys.stderr)
        return cl_found

    except Exception as e:
        if verbose:
            print(f"Debug: Exception in _setup_msvc_env: {e}", file=sys.stderr)
        return False


def compile_test_harness(
    project_root: Path,
    test_harness_c: Path,
    output_path: Path,
    verbose: bool = False
) -> bool:
    """Compile the test harness.

    Args:
        project_root: Root directory of the project (contains Communications/, Utils/)
        test_harness_c: Path to the generated test_harness.c file
        output_path: Path for the output executable
        verbose: Print compilation command if True

    Returns:
        True on success, False on failure
    """
    compiler = find_compiler()

    if compiler is None:
        print("Warning: No C compiler found (gcc, clang, or cl.exe)", file=sys.stderr)
        return False

    # Build list of source files
    sources = [str(test_harness_c)]
    for src in TEST_HARNESS_SOURCES:
        src_path = project_root / src
        if not src_path.exists():
            print(f"Warning: Source file not found: {src_path}", file=sys.stderr)
            return False
        sources.append(str(src_path))

    # Build include paths
    includes = [project_root / d for d in INCLUDE_DIRS]

    if compiler == "cl":
        return _compile_msvc(sources, includes, output_path, verbose)
    else:
        return _compile_gcc(compiler, sources, includes, output_path, verbose)


def _compile_gcc(
    compiler: str,
    sources: list,
    includes: list,
    output_path: Path,
    verbose: bool
) -> bool:
    """Compile using gcc or clang."""
    include_flags = [f"-I{inc}" for inc in includes]

    cmd = [compiler, "-o", str(output_path)] + sources + include_flags + GCC_FLAGS

    if verbose:
        print(f"Compiling: {' '.join(cmd)}")

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=120
        )

        if result.returncode != 0:
            print(f"Compilation failed:\n{result.stderr}", file=sys.stderr)
            return False

        if result.stderr and verbose:
            print(f"Compiler warnings:\n{result.stderr}")

        return True

    except subprocess.TimeoutExpired:
        print("Compilation timed out", file=sys.stderr)
        return False
    except Exception as e:
        print(f"Compilation error: {e}", file=sys.stderr)
        return False


def _compile_msvc(
    sources: list,
    includes: list,
    output_path: Path,
    verbose: bool
) -> bool:
    """Compile using MSVC cl.exe."""
    include_flags = [f"/I{inc}" for inc in includes]

    cmd = ["cl", f"/Fe:{output_path}"] + sources + include_flags + MSVC_FLAGS

    if verbose:
        print(f"Compiling: {' '.join(cmd)}")

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=120
        )

        if result.returncode != 0:
            print(f"Compilation failed:\n{result.stderr}", file=sys.stderr)
            return False

        if result.stderr and verbose:
            print(f"Compiler warnings:\n{result.stderr}")

        return True

    except subprocess.TimeoutExpired:
        print("Compilation timed out", file=sys.stderr)
        return False
    except Exception as e:
        print(f"Compilation error: {e}", file=sys.stderr)
        return False
