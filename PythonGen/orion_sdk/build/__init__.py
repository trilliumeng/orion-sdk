"""Build and compilation support."""

from .build_config import TEST_HARNESS_SOURCES, INCLUDE_DIRS, GCC_FLAGS, MSVC_FLAGS
from .compiler import compile_test_harness, find_compiler

__all__ = [
    'TEST_HARNESS_SOURCES', 'INCLUDE_DIRS', 'GCC_FLAGS', 'MSVC_FLAGS',
    'compile_test_harness', 'find_compiler',
]
