#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "gtest/gtest.h"
#include "AstroLib.h"

struct TestAngle {
    double rad;
    SxAngle sx;
};

// {radians, {degrees, minutes, seconds, sign}}
TestAngle angles[] = {
    {3.614435, {152, 54, 29, 1}},
    {3.948608, {133, 45, 41, 1}},
    {4.060650, {127, 20, 30, 1}},
    {4.816841, { 84,  0, 55, 1}},
    {1.271555, { 72, 51, 17, 0}},
    {5.297542, { 56, 28, 23, 1}},
    {4.213696, {118, 34, 22, 1}},
    {2.848288, {163, 11, 41, 0}},
    {0.633988, { 36, 19, 29, 0}},
    {3.492877, {159, 52, 22, 1}},
    {4.835365, { 82, 57, 14, 1}},
    {5.955636, { 18, 46,  1, 1}},
    {0.241292, { 13, 49, 30, 0}},
    {1.707204, { 97, 48, 56, 0}},
    {5.555794, { 41, 40, 35, 1}},
    {5.968573, { 18,  1, 33, 1}},
    {6.017326, { 15, 13, 57, 1}},
    {0.537151, { 30, 46, 35, 0}},
    {1.676158, { 96,  2, 12, 0}},
    {4.416605, {106, 56, 49, 1}},
};

struct TestDate {
    double jd;      // Julian date
    double gmst;    // Greenwich mean sidereal time
    //float lst;     // Local sidereal time
    Date date;
};

// Test data generated with pyephem
TestDate dates[] = {
    {2443825.690278, 2.102182, {1978, 11, 13,  4, 34,  0, 0, 0}},
    {2445850.597222, 4.935610, {1984,  5, 30,  2, 20,  0, 0, 0}},
    {2451544.500000, 1.744767, {2000,  1,  1,  0,  0,  0, 0, 0}},
    {2452765.835428, 6.013140, {2003,  5,  6,  8,  3,  1, 0, 0}},
    {2453065.041100, 6.169397, {2004,  2, 29, 12, 59, 11, 0, 0}},
    {2457023.173866, 5.979462, {2014, 12, 31, 16, 10, 22, 0, 0}},
    {2458099.418438, 0.897815, {2017, 12, 11, 22,  2, 33, 0, 0}},
    {2469938.334537, 2.971056, {2050,  5, 11, 20,  1, 44, 0, 0}},
};

/*
// Test data generated with pyephem
EqCoords stars[] = {
    {0.767637, 1.559294},
    {2.900727, 1.075995},
    {2.892403, 0.982305},
    {5.419333, 0.791490},
    {0.826224, 0.715994},
    {4.876056, 0.677254},
    {5.199440, 0.155650},
    {1.376248, -0.142839},
    {3.517354, -0.196366},
    {1.771324, -0.292163},
};
*/


TEST(AstroLib, rad2sx) {
    for (int i=0; i<sizeof(angles)/sizeof(TestAngle); i++) {
        TestAngle angle = angles[i];

        SxAngle sx;
        rad2sx(angle.rad, &sx);

        ASSERT_EQ(angle.sx.deg, sx.deg);
        ASSERT_EQ(angle.sx.min, sx.min);
        ASSERT_EQ(angle.sx.sec, sx.sec);
        ASSERT_EQ(angle.sx.sign, sx.sign);
    }
}

TEST(AstroLib, sx2rad) {
    for (int i=0; i<sizeof(angles)/sizeof(TestAngle); i++) {
        TestAngle angle = angles[i];
        ASSERT_NEAR(angle.rad, sx2rad(angle.sx), 0.00001);
    }
}

TEST(AstroLib, TestJulianDate) {
    for (int i=0; i<sizeof(dates)/sizeof(TestDate); i++) {
        TestDate date = dates[i];

        ASSERT_NEAR(date.jd, getJulianDate(date.date), 0.000001);
        ASSERT_FLOAT_EQ(floor(date.jd - 0.5) + 0.5, getJulianDate0(date.date));
        ASSERT_NEAR(date.jd - 2451545.0, getJ2000Date(date.date), 0.000001);
    }
}

TEST(AstroLib, TestSiderealTime) {
    for (int i=0; i<sizeof(dates)/sizeof(TestDate); i++) {
        TestDate date = dates[i];

        float jd = date.jd - 2451545.0;
        ASSERT_NEAR(date.gmst , getGMST(date.date), 0.00014);
    }
}

/*
TEST(horizToLocal, TestHorizToEq) {
    // Madrid:
    Location loc = {0.705404328965830, -0.06458321069102657};
    Date date = {2017, 12, 11, 22, 2, 33};
    float jd = getJ2000Date(date);
    float stime = getLST(jd, loc);

    for (int i=0; i<sizeof(stars)/sizeof(EqCoords); i++) {
        HorizCoords hc;
        localToHoriz(stime, loc, stars[i], &hc);
        //printf("%d\t%f\t%f\t%f\t%f\n", i, stars[i].ra, stars[i].dec, hc.az, hc.alt);

        LocalCoords eq;
        horizToLocal(stime, loc, hc, &eq);
        printf("%d\t%f\t%f\t%f\t%f\n", i, stars[i].ra, stars[i].dec, eq.ra, eq.dec);
    }
}
*/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
