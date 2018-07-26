include(gtest_dependency.pri)

TEMPLATE = app
CONFIG += console c++11

CONFIG -= app_bundle
CONFIG += thread

QT += core

OBJECTS_DIR = obj
MOC_DIR = moc

INCLUDEPATH += $$(EIGEN_INCLUDE) += $$(BOOST_INCLUDE) \
            += $$PWD/../ins_board_pc/

OBJ_PREFIX = $$PWD/../release/ins_board_pc/obj/

LIBS += $$OBJ_PREFIX/quaternion.o \
        $$OBJ_PREFIX/quatutils.o \
        $$OBJ_PREFIX/utils.o

HEADERS += \
        tst_quaterniontest.h \
    tst_utilstest.h

SOURCES += \
        main.cpp \
    tst_quaternion.cpp \
    tst_utilstest.cpp
