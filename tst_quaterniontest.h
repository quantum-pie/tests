#ifndef TST_QUATERNIONTEST_H
#define TST_QUATERNIONTEST_H

#include <gtest/gtest.h>
#include <gmock/gmock-matchers.h>

#include <quaternion.h>

using namespace testing;
using namespace quat;

class QuaternionTest : public Test
{
public:
    QuaternionTest() : q{10, -5, 15, 2},
                       p{ -45, 7, 1, -12 } {}

protected:
    Quaternion q;
    Quaternion p;
};

#endif // TST_QUATERNIONTEST_H

