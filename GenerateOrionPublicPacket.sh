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

ROOT_DIR=`dirname $0`

$ROOT_DIR/Protogen/Protogen.sh $ROOT_DIR/Communications/OrionPublicProtocol.xml $ROOT_DIR/Communications -no-doxygen

