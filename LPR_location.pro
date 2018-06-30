#-------------------------------------------------
#
# Project created by QtCreator 2018-06-19T19:37:46
#
#-------------------------------------------------

QT       += core

QT       -= gui
QT += gui widgets

TARGET = LPR_location
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    image.cpp \
    charpartition.cpp \
    charrecognition.cpp

HEADERS += \
    Image.h \
    charpartition.h \
    def.h \
    charrecognition.h

INCLUDEPATH += /usr/local/include/opencv \
             /usr/local/include/opencv2 \
             /usr/local/include


LIBS += /usr/local/lib/libopencv_*.so \
