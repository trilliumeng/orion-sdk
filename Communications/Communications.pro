#-------------------------------------------------
#
# Project created by QtCreator 2016-07-07T18:00:46
#
#-------------------------------------------------

QT       -= core gui

TARGET = OrionComm
TEMPLATE = lib
CONFIG += staticlib

SOURCES += bitfieldspecial.c \
    fielddecode.c \
    fieldencode.c \
    floatspecial.c \
    OrionCommLinux.c \
    OrionCommWindows.c \
    OrionPublicPacket.c \
    scaleddecode.c \
    scaledencode.c

HEADERS += bitfieldspecial.h \
    fielddecode.h \
    fieldencode.h \
    floatspecial.h \
    OrionComm.h \
    OrionPublicPacket.h \
    scaleddecode.h \
    scaledencode.h

INCLUDEPATH += ../Utils

unix {
    target.path = /usr/lib
    INSTALLS += target
}
