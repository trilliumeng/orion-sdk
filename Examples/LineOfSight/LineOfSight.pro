TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += LineOfSight.c

INCLUDEPATH += ../../Communications \
    ../../Utils

Debug:LIBS += -L../../Communications/debug -L../../Utils/debug
Release:LIBS += -L../../Communications/release -L../../Utils/release

LIBS += -lOrionComm -lOrionUtils

win32:LIBS += -lws2_32
