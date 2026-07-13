cmake_minimum_required(VERSION 3.16)
project(OrionUtils VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(OrionUtils
    ${PROTOGEN_EXPORTS}
    GeolocateTelemetry.c GeolocateTelemetry.h
    GpsDataReceive.c GpsDataReceive.h
    OrionPublicPacketShim.c OrionPublicPacketShim.h
    TrilliumPacket.c TrilliumPacket.h
    WGS84.c WGS84.h
    dcm.c dcm.h
    earthposition.c earthposition.h
    earthrotation.c earthrotation.h
    linearalgebra.c linearalgebra.h
    mathutilities.c mathutilities.h
    quaternion.c quaternion.h
)

target_include_directories(OrionUtils PUBLIC
    ../Communications
)

install(TARGETS OrionUtils
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FRAMEWORK DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
