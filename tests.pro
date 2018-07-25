include(gtest_dependency.pri)

TEMPLATE = app
CONFIG += console c++11

CONFIG -= app_bundle
CONFIG += thread
CONFIG -= qt

INCLUDEPATH += $$(EIGEN_INCLUDE) += $$(BOOST_INCLUDE) \
            += $$PWD/../ins_board_pc/

OBJ_PREFIX = $$PWD/../release/ins_board_pc/obj/

LIBS += $$OBJ_PREFIX/quaternion.o

HEADERS += \
        tst_quaterniontest.h

SOURCES += \
        main.cpp \
    tst_quaternion.cpp
