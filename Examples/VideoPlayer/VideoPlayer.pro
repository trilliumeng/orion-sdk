TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += VideoPlayer.c \
    FFmpeg.c \
    KlvParser.c \
    KlvTree.c \
    StreamDecoder.c

INCLUDEPATH += ../../Communications \
    ../../Utils

CONFIG(debug, debug|release) {
    LIBS += -L../../Communications/debug -L../../Utils/debug
} else {
    LIBS += -L../../Communications/release -L../../Utils/release
}

LIBS += -lOrionComm -lOrionUtils -lavutil -lavcodec -lavformat -lswscale -ljpeg

win32:LIBS += -lws2_32
