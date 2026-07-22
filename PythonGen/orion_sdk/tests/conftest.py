"""Pytest configuration for orion_sdk tests.

Sets up the path to import generated bindings from Communications/python/orion_sdk.
Run GenerateOrionPublicPacket.sh first to generate the bindings.
"""

import sys
from pathlib import Path


def pytest_configure(config):
    """Set up path before test collection."""
    # Path: PythonGen/orion_sdk/tests/conftest.py
    # Need to get to: Public/Communications/python
    _test_dir = Path(__file__).parent          # PythonGen/orion_sdk/tests
    _orion_sdk_dir = _test_dir.parent          # PythonGen/orion_sdk
    _pythongen_dir = _orion_sdk_dir.parent     # PythonGen
    _public_dir = _pythongen_dir.parent        # Public
    _generated_path = _public_dir / 'Communications' / 'python'

    if _generated_path.exists():
        if str(_generated_path) not in sys.path:
            sys.path.insert(0, str(_generated_path))


# Also run at import time for when conftest is loaded
_test_dir = Path(__file__).parent
_generated_path = _test_dir.parent.parent.parent / 'Communications' / 'python'
if _generated_path.exists() and str(_generated_path) not in sys.path:
    sys.path.insert(0, str(_generated_path))
