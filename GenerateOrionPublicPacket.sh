#!/bin/sh

PLATFORM=`uname -i 2>/dev/null`
ROOT_DIR=`dirname $0`

if [ "$PLATFORM" = "i386" ]; then
    ARCH=Linux/x86
elif [ "$PLATFORM" = "x86_64" ]; then
    ARCH=Linux/x64
elif uname -m | grep arm > /dev/null; then
    ARCH=Linux/arm
elif [ "`uname`" = "Darwin" ]; then
    ARCH=Mac/ProtoGen.app/Contents/MacOS
else
    echo "Unrecognized platform $PLATFORM"
    exit 1
fi

$ROOT_DIR/Protogen/$ARCH/ProtoGen $ROOT_DIR/Communications/OrionPublicProtocol.xml $ROOT_DIR/Communications -no-doxygen

