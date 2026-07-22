#!/bin/bash

# Do a quick system version check and spit a warning if we are greater than the last verified buildable version
# Ensure this is updated once verified for OS version
Var=$(lsb_release -r)
NumOnly=$(cut -f2 <<< "$Var")
VersionMajor=$(($(cut -d'.' -f1 <<< "$NumOnly")))
VersionMinor=$(($(cut -d'.' -f2 <<< "$NumOnly")))
if [ $VersionMajor -gt 18 ]
then
    echo "orionSDK hasn't been verified to build on $Var"
fi

ROOT_DIR=$(dirname "$0")

# Generate C code
"$ROOT_DIR/Protogen/Protogen.sh" "$ROOT_DIR/Communications/OrionPublicProtocol.xml" "$ROOT_DIR/Communications" -no-doxygen

# Generate Python bindings and compile test harness (if python3 is available)
if command -v python3 &> /dev/null; then
    PYTHONPATH="$ROOT_DIR/PythonGen" python3 -m orion_sdk.cli \
        "$ROOT_DIR/Communications/OrionPublicProtocol.xml" \
        "$ROOT_DIR/Communications/python/orion_sdk" \
        --c-header-path "$ROOT_DIR/Communications/OrionPublicPacket.h" \
        --compile-tests \
        --project-root "$ROOT_DIR" \
        --install-deps
else
    echo "Warning: python3 not found, skipping Python bindings generation"
fi

