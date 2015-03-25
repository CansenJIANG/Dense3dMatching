#-------------------------------------------------
#
# Project created by QtCreator 2014-05-01T14:24:33
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = denseMatch3D_pc
TEMPLATE = app

CONFIG +=link_pkgconfig
CONFIG += console
PKGCONFIG += opencv
PKGCONFIG += eigen3

DEFINES += EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

SOURCES += \
    src/seedpropagation.cpp \
    src/scorefunc.cpp \
    src/pclQviewer.cpp \
    src/main.cpp

HEADERS  += \
    src/seedpropagation.h \
    src/scorefunc.h \
    src/pclQviewer.h \
    src/commonFunc.h \
    src/commonHeader.h

INCLUDEPATH += /usr/include/vtk-5.8 \
                /usr/include/pcl-1.7 \
                /usr/include/boost \
                /usr/include/openni


LIBS += -L/usr/lib \
            -lpcl_io \
            -lpcl_visualization \
            -lpcl_common \
            -lpcl_filters \
            -lpcl_octree \
            -lpcl_kdtree \
            -lpcl_registration \
            -lpcl_features \
            -lpcl_keypoints \
            -lpcl_recognition \
            -lpcl_sample_consensus \
            -lpcl_search  \
            -lpcl_surface \
            -lpcl_outofcore \
            -lpcl_people \
            -lpcl_segmentation \
            -lpcl_tracking \
            -lboost_filesystem \
            -lboost_thread \
            -lboost_system \
            -lQVTK \
            -lvtkRendering \
            -lvtkIO \
            -lvtkCommon \
            -lvtkWidgets \
            -lvtkFiltering \
            -lvtkGraphics

FORMS += \
    src/pclQviewer.ui

OTHER_FILES += \
    src/README.md

