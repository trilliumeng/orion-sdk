#-------------------------------------------------
#
# Project created by QtCreator 2016-07-07T18:04:55
#
#-------------------------------------------------

QT       -= core gui

TARGET = OrionUtils
TEMPLATE = lib
CONFIG += staticlib

SOURCES += dcm.c \
    earthposition.c \
    earthrotation.c \
    GpsDataReceive.c \
    GeolocateTelemetry.c \
    linearalgebra.c \
    mathutilities.c \
    OrionPublicPacketShim.c \
    quaternion.c \
    TrilliumPacket.c \
    WGS84.c

HEADERS += dcm.h \
    earthposition.h \
    earthrotation.h \
    GpsDataReceive.h \
    GeolocateTelemetry.h \
    linearalgebra.h \
    mathutilities.h \
    OrionPublicPacketShim.h \
    quaternion.h \
    TrilliumPacket.h \
    WGS84.h

INCLUDEPATH += ../Communications

CONFIG(debug, debug|release) {
    DESTDIR = debug
} else {
    DESTDIR = release
}

unix {
    target.path = /usr/lib
    INSTALLS += target
}
