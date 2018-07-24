include(gtest_dependency.pri)

TEMPLATE = app
CONFIG += console c++11

CONFIG -= app_bundle
CONFIG += thread
CONFIG -= qt

INCLUDEPATH += C:/Tools/Eigen3/include/eigen3 += C:/Tools/boost_1_66_0 \
            += $$PWD/../ins_board_pc/

OBJ_PREFIX = $$PWD/../release/ins_board_pc/obj/

LIBS += $$OBJ_PREFIX/quaternion.o

HEADERS += \
        tst_quaterniontest.h

SOURCES += \
        main.cpp \
    tst_quaternion.cpp
