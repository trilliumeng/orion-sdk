"""Build configuration for test harness compilation.

This module centralizes the list of C source files and compiler flags
needed to build the cross-language validation test harness.
"""

# C source files required for test harness (relative to project root)
TEST_HARNESS_SOURCES = [
    "Communications/OrionPublicPacket.c",
    "Communications/fieldencode.c",
    "Communications/fielddecode.c",
    "Communications/scaledencode.c",
    "Communications/scaleddecode.c",
    "Communications/floatspecial.c",
    "Utils/TrilliumPacket.c",
    "Utils/OrionPublicPacketShim.c",
    "Utils/GpsDataReceive.c",
    "Utils/mathutilities.c",
    "Utils/earthposition.c",
    "Utils/linearalgebra.c",
    "Utils/quaternion.c",
    "Utils/dcm.c",
    "Utils/WGS84.c",
    "Utils/earthrotation.c",
]

# Include directories (relative to project root)
INCLUDE_DIRS = [
    "Communications",
    "Utils",
]

# Compiler flags
GCC_FLAGS = ["-lm", "-Wall", "-Wno-unused-variable", "-Wno-format"]
MSVC_FLAGS = ["/W3", "/wd4101", "/wd4477"]
