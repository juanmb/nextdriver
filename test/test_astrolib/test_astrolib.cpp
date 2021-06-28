#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unity.h>
#include "AstroLib.h"
#include "test_data.h"


// Parse a UTC string date into a timestamp value
time_t parse_date(const char *date) {
    struct tm tm;
    strptime(date, "%Y/%m/%d %T", &tm);
    return timegm(&tm);
}

void test_astrolib_limitDec(void) {
    uint8_t over;
    TEST_ASSERT_EQUAL_FLOAT(0.0, limitDec(0.0, &over));
    TEST_ASSERT_FALSE(over);
    TEST_ASSERT_EQUAL_FLOAT(M_PI/4, limitDec(M_PI/4, &over));
    TEST_ASSERT_FALSE(over);
    TEST_ASSERT_EQUAL_FLOAT(M_PI/2, limitDec(M_PI/2, &over));
    TEST_ASSERT_TRUE(over);
    TEST_ASSERT_EQUAL_FLOAT(M_PI/4, limitDec(3*M_PI/4, &over));
    TEST_ASSERT_TRUE(over);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0, limitDec(M_PI, &over));
    TEST_ASSERT_TRUE(over);
    TEST_ASSERT_EQUAL_FLOAT(-M_PI/4, limitDec(-M_PI/4, &over));
    TEST_ASSERT_FALSE(over);
    TEST_ASSERT_EQUAL_FLOAT(-M_PI/4, limitDec(-3*M_PI/4, &over));
    TEST_ASSERT_TRUE(over);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0, limitDec(-M_PI, &over));
    TEST_ASSERT_TRUE(over);
}

void test_astrolib_normalize2Pi(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0, normalize2Pi(0.0));
    TEST_ASSERT_EQUAL_FLOAT(4.0, normalize2Pi(4.0));
    TEST_ASSERT_EQUAL_FLOAT(2*M_PI - 2.0, normalize2Pi(-2.0));
    TEST_ASSERT_EQUAL_FLOAT(3.0, normalize2Pi(4*M_PI + 3.0));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0, normalize2Pi(2*M_PI));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0, normalize2Pi(8*M_PI));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 2*M_PI, normalize2Pi(-8*M_PI));
}

void test_astrolib_normalizePi(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0, normalizePi(0.0));
    TEST_ASSERT_EQUAL_FLOAT(3.0, normalizePi(3.0));
    TEST_ASSERT_EQUAL_FLOAT(-M_PI, normalizePi(M_PI));
    TEST_ASSERT_EQUAL_FLOAT(-3.0, normalizePi(2*M_PI - 3.0));
    TEST_ASSERT_EQUAL_FLOAT(3.0, normalizePi(8*M_PI + 3.0));
    TEST_ASSERT_EQUAL_FLOAT(-3.0, normalizePi(8*M_PI - 3.0));
}

void test_astrolib_rad2sx(void) {
    for (int i=0; i<sizeof(test_angles)/sizeof(TestAngle); i++) {
        TestAngle angle = test_angles[i];

        SxAngle sx;
        rad2sx(angle.rad, &sx);

        TEST_ASSERT_EQUAL(angle.sx.deg, sx.deg);
        TEST_ASSERT_EQUAL(angle.sx.min, sx.min);
        TEST_ASSERT_EQUAL(angle.sx.sec, sx.sec);
        TEST_ASSERT_EQUAL(angle.sx.sign, sx.sign);
    }
}

void test_astrolib_sx2rad(void) {
    for (int i=0; i<sizeof(test_angles)/sizeof(TestAngle); i++) {
        TestAngle angle = test_angles[i];
        TEST_ASSERT_FLOAT_WITHIN(0.00001, angle.rad, sx2rad(angle.sx));
    }
}

void test_astrolib_JulianDate0(void) {
    for (int i=0; i<sizeof(test_dates)/sizeof(TestDate); i++) {
        TestDate date = test_dates[i];
        time_t t = parse_date(date.date);

        TEST_ASSERT_EQUAL_FLOAT(floor(date.jd - 0.5) + 0.5, getJulianDate0(t));
    }
}

void test_astrolib_JulianDate(void) {
    for (int i=0; i<sizeof(test_dates)/sizeof(TestDate); i++) {
        TestDate date = test_dates[i];
        time_t t = parse_date(date.date);

        float jd2000 = date.jd - 2451545.0;
        TEST_ASSERT_FLOAT_WITHIN(0.000001, date.jd, getJulianDate(t));
        TEST_ASSERT_FLOAT_WITHIN(0.000001, jd2000, getJ2000Date(t));
    }
}

void test_astrolib_TestSiderealTime(void) {
    for (int i=0; i<sizeof(test_dates)/sizeof(TestDate); i++) {
        TestDate date = test_dates[i];
        time_t t = parse_date(date.date);

        TEST_ASSERT_FLOAT_WITHIN(0.00015, date.gmst, getGMST(t));
        TEST_ASSERT_FLOAT_WITHIN(0.00015, date.lst, getLST(t, obs_loc));
    }
}

void test_astrolib_localToHoriz(void) {
    float lst = getLST(parse_date(obs_date), obs_loc);

    for (int i=0; i<sizeof(test_stars)/sizeof(TestStar); i++) {
        TestStar star = test_stars[i];
        LocalCoords lc = {lst - star.eq.ra, star.eq.dec};

        HorizCoords hc;
        localToHoriz(obs_loc, lc, &hc);
        TEST_ASSERT_FLOAT_WITHIN(0.001, star.hc.az, hc.az);
        TEST_ASSERT_FLOAT_WITHIN(0.001, star.hc.alt, hc.alt);
    }
}

void test_astrolib_horizToLocal(void) {
    float lst = getLST(parse_date(obs_date), obs_loc);

    for (int i=0; i<sizeof(test_stars)/sizeof(TestStar); i++) {
        TestStar star = test_stars[i];

        LocalCoords lc;
        horizToLocal(obs_loc, star.hc, &lc);
        float test_ha = normalize2Pi(lst - star.eq.ra);
        TEST_ASSERT_FLOAT_WITHIN(0.001, test_ha, lc.ha);
        TEST_ASSERT_FLOAT_WITHIN(0.001, star.eq.dec, lc.dec);
    }
}

struct TestCoord {
    AxisCoords ac;
    LocalCoords lc;
    const char *msg;
};

TestCoord coordsAxisToLocal[] = {
    {{0, M_PI/2}, {0, M_PI/2}, "north pole (1)"},
    {{M_PI, M_PI/2}, {-M_PI, M_PI/2}, "north pole (2)"},
    {{3*M_PI/2, M_PI/2}, {-M_PI/2, M_PI/2}, "north pole (3)"},
    {{M_PI/2, 0}, {-M_PI/2, 0}, "east"},
    {{M_PI/2, M_PI}, {M_PI/2, 0}, "west"},
    {{0, M_PI}, {0, 0}, "intersection of the equator with the meridian (1)"},
    {{M_PI, 0}, {0, 0}, "intersection of the equator with the meridian (2)"},
    {{0, M_PI}, {0, 0}, "intersection of the equator with the meridian (2)"},
    {{0, 0}, {-M_PI, 0}, "intersection of the equator with the meridian (3)"},
    {{0, 3*M_PI/2}, {0, -M_PI/2}, "south pole"},
};

TestCoord coordsLocalToAxis[] = {
    {{0, M_PI/2}, {0, M_PI/2}, "north pole (1)"},
    {{M_PI/2, 0}, {-M_PI/2, 0}, "east"},
    {{M_PI/2, M_PI}, {M_PI/2, 0}, "west"},
    {{0, M_PI}, {0, 0}, "intersection of the equator with the meridian (1)"},
    {{2*M_PI, 0}, {-M_PI, 0}, "intersection of the equator with the meridian (2)"},
    {{0, 3*M_PI/2}, {0, -M_PI/2}, "south pole"},
};

void test_astrolib_axisToLocal(void) {
    for (int i=0; i<sizeof(coordsAxisToLocal)/sizeof(TestCoord); i++) {
        TestCoord point = coordsAxisToLocal[i];
        LocalCoords lc;
        axisToLocalCoords(point.ac, &lc);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-6, point.lc.ha, lc.ha, point.msg);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-6, point.lc.dec, lc.dec, point.msg);
    }
}

void test_astrolib_localToAxis(void) {
    for (int i=0; i<sizeof(coordsLocalToAxis)/sizeof(TestCoord); i++) {
        TestCoord point = coordsLocalToAxis[i];
        AxisCoords ac;
        localToAxisCoords(point.lc, &ac);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-6, point.ac.ra, ac.ra, point.msg);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-6, point.ac.dec, ac.dec, point.msg);
    }
}


int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_astrolib_limitDec);
    RUN_TEST(test_astrolib_normalize2Pi);
    RUN_TEST(test_astrolib_normalizePi);
    RUN_TEST(test_astrolib_rad2sx);
    RUN_TEST(test_astrolib_sx2rad);
    RUN_TEST(test_astrolib_JulianDate0);
    RUN_TEST(test_astrolib_JulianDate);
    RUN_TEST(test_astrolib_TestSiderealTime);
    RUN_TEST(test_astrolib_localToHoriz);
    RUN_TEST(test_astrolib_horizToLocal);
    RUN_TEST(test_astrolib_axisToLocal);
    UNITY_END();

    return 0;
}
