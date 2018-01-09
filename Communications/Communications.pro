#-------------------------------------------------
#
# Project created by QtCreator 2016-07-07T18:00:46
#
#-------------------------------------------------

QT       -= core gui

TARGET = OrionComm
TEMPLATE = lib
CONFIG += staticlib

SOURCES += \
    fielddecode.c \
    fieldencode.c \
    floatspecial.c \
    OrionComm.c \
    OrionCommLinux.c \
    OrionCommWindows.c \
    OrionPublicPacket.c \
    scaleddecode.c \
    scaledencode.c

HEADERS += \
    fielddecode.h \
    fieldencode.h \
    floatspecial.h \
    OrionComm.h \
    OrionPublicPacket.h \
    scaleddecode.h \
    scaledencode.h

INCLUDEPATH += ../Utils

CONFIG(debug, debug|release) {
    DESTDIR = debug
} else {
    DESTDIR = release
}

Public.target = $$PWD/OrionPublic.html
Public.depends = $$PWD/OrionPublicProtocol.xml

unix {
    target.path = /usr/lib
    INSTALLS += target
}

win32 {
    Public.commands = GenerateOrionPublicPacketWin.bat
} else {
    Public.commands = ../Protogen/Protogen.sh $$Public.depends .
}

PRE_TARGETDEPS += $$Public.target

QMAKE_EXTRA_TARGETS += Public
