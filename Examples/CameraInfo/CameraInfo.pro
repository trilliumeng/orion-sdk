TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += CameraInfo.c

INCLUDEPATH += ../../Communications \
    ../../Utils

CONFIG(debug, debug|release) {
    LIBS += -L../../Communications/debug -L../../Utils/debug
} else {
    LIBS += -L../../Communications/release -L../../Utils/release
}

LIBS += -lOrionComm -lOrionUtils

win32:LIBS += -lws2_32
