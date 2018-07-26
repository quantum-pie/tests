#ifndef TST_UTILSTEST_H
#define TST_UTILSTEST_H

#include <gtest/gtest.h>
#include <gmock/gmock-matchers.h>

#include <QLocale>

#include <utils.h>
#include <packets.h>

using namespace testing;

class UtilsTest : public Test
{
public:
    UtilsTest() :
        float_value{ M_PI / 2 },
        less_than_minus_pi_double{ - 3.0 * M_PI },
        greater_than_pi_double{ 2.5 * M_PI },
        between_minus_pi_and_pi_double{ M_PI / 3 },
        timestamp{ 1990, 8, 13, 21, 1, 5, 45 }
    {
        QLocale::setDefault(QLocale::English);
    }

protected:
    float float_value;
    double less_than_minus_pi_double;
    double greater_than_pi_double;
    double between_minus_pi_and_pi_double;
    Timestamp timestamp;
};

#endif // TST_UTILSTEST_H
