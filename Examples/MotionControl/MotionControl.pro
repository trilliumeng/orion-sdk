TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += MotionControl.c \
    timing_loop.c \
    test_signal.c \
    response_metrics.c

HEADERS += timing_loop.h \
    test_signal.h \
    response_metrics.h

INCLUDEPATH += ../../Communications \
    ../../Utils

CONFIG(debug, debug|release) {
    LIBS += -L../../Communications/debug -L../../Utils/debug
} else {
    LIBS += -L../../Communications/release -L../../Utils/release
}

LIBS += -lOrionComm -lOrionUtils

win32:LIBS += -lws2_32 -lwinmm
