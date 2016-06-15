#!/bin/sh

PLATFORM=`uname -i`
MACH=`uname -m`
ROOT_DIR=`dirname $0`

if [ "$PLATFORM" == "i386" ]; then
    ARCH=x86
elif [ "$PLATFORM" == "x86_64" ]; then
    ARCH=x64
elif [ "$MACH" != "${MACH/arm/}" ]; then
    ARCH=arm
else
    echo "Unrecognized platform $PLATFORM"
    exit 1
fi

$ROOT_DIR/Protogen/Linux/$ARCH/ProtoGen.sh $ROOT_DIR/Communications/OrionPublicProtocol.xml $ROOT_DIR/Communications -no-doxygen

