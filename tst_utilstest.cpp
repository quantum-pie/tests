#include "tst_utilstest.h"

#include <QString>


#include <fstream>

using namespace utils;

TEST_F(UtilsTest, IsRadDegCorrect)
{
    double degrees = radians_to_degrees(greater_than_pi_double);
    EXPECT_NEAR(450.0, degrees, 1e-2);
    EXPECT_DOUBLE_EQ(greater_than_pi_double, degrees_to_radians(degrees));
}

TEST_F(UtilsTest, IsFixAngleCorrect)
{
    EXPECT_NEAR(-3.1416, fix_angle(less_than_minus_pi_double), 1e-4);
    EXPECT_DOUBLE_EQ(between_minus_pi_and_pi_double, fix_angle(between_minus_pi_and_pi_double));
    EXPECT_NEAR(1.5708, fix_angle(greater_than_pi_double), 1e-4);
}

TEST_F(UtilsTest, IsFloatFixedCorrect)
{
    int32_t fixed = float_to_fixed(float_value);
    EXPECT_EQ(static_cast<int32_t>(51470), fixed);
}

TEST_F(UtilsTest, IsDoubleFixedCorrect)
{
    int32_t fixed = double_to_fixed(greater_than_pi_double);
    EXPECT_EQ(static_cast<int32_t>(257351), fixed);
    EXPECT_NEAR(greater_than_pi_double, fixed_to_double(fixed), 1e-4);
}

TEST_F(UtilsTest, IsAngleFixedCorrect)
{
    int32_t fixed = angle_to_fixed(between_minus_pi_and_pi_double);
    EXPECT_EQ(static_cast<int32_t>(10922), fixed);
    EXPECT_NEAR(between_minus_pi_and_pi_double, fixed_to_angle(fixed), 1e-4);
}

TEST_F(UtilsTest, IsMsToKnotsCorrrect)
{
    ASSERT_NEAR(311.015, ms_to_knots(160), 1e-3);
}

TEST_F(UtilsTest, IsDoubleViewCorrect)
{
    ASSERT_STREQ("1.0472", double_view(between_minus_pi_and_pi_double, 4).toLatin1().data());
}

TEST_F(UtilsTest, IsGPSTimeCorrect)
{
    ASSERT_STREQ("8/13/90 9:01 PM", gps_time_string(timestamp).toLatin1().data());
}
