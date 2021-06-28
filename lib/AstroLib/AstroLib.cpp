#include <math.h>
#include <stdio.h>
#include "AstroLib.h"

#define abs(x) (((x) > 0) ? (x) : -(x))
#define sign(x) (((x) > 0) ? 1 : -1)


// Normalize an angle in radians between 0 and 2*pi
float normalize2Pi(float h)
{
    int ih = (int)(h/2/M_PI);
    h = h - (float)ih*2*M_PI;
    return h < 0 ? h + 2*M_PI : h;
}

// Normalize an angle in radians between -pi and pi
float normalizePi(float h)
{
    h = normalize2Pi(h);
    int ih = (int)(h/2/M_PI);
    h = h - (float)ih*2*M_PI;
    return h > M_PI ? h - 2*M_PI: h;
}

// Limit an angle to [-pi/2, pi/2]
// This is useful for transforming spherical coordinates
float limitDec(float h, uint8_t *over)
{
    h = normalizePi(h);

    if (h > M_PI/2) {
        *over = 1;
        return M_PI - h;
    } else if (h < -M_PI/2) {
        *over = 1;
        return -M_PI - h;
    }
    *over = 0;
    return h;
}


// Convert an angle from sexagesimal to radians
float sx2rad(SxAngle sx)
{
    float rad = M_PI*((float)sx.deg +
            (float)sx.min/60.0 +
            (float)sx.sec/3600.0)/180.0;

    if (sx.sign)
        return -rad;
    return rad;
}

// Convert an angle from radians to sexagesimal
void rad2sx(float rad, SxAngle *sx)
{
    sx->sign = (rad < 0);
    rad = fabs(rad);
    float deg = rad*180/M_PI;
    float min = 60*(deg - int(deg));
    float sec = 60*(min - int(min));

    sx->deg = (uint16_t)deg;
    sx->min = (uint8_t)min;
    sx->sec = (uint8_t)round(sec);
}

void sx2string(SxAngle sx, char *buffer)
{
    sprintf(buffer, " %d:%02d:%02d", sx.deg, sx.min, sx.sec);
    if (sx.sign)
        buffer[0] = '-';
}

// Calculate the julian date of the midnight of a given date
float getJulianDate0(time_t t)
{
    float a, b, c, d;
#ifdef ARDUINO_ARCH_AVR
    float _month = month(t);
    float _year = year(t);
    float _day = day(t);
#else
    struct tm *ptm;
    ptm = gmtime(&t);
    float _month = ptm->tm_mon + 1;
    float _year = ptm->tm_year + 1900;
    float _day = ptm->tm_mday;
#endif

    if (_month < 3) {
        _month += 12;
        _year -= 1;
    }

    a = (long)(_year/100);
    b = 2 - a + (long)(a/4);
    c = (long)(365.25*(_year + 4716));
    d = (long)(30.6001*(_month + 1));

    return _day + b + c + d - 1524.5;
}

// Calculate the julian date of a given timestamp
float getJulianDate(time_t t)
{
    float _hour, _min, _sec, h, jd;
#ifdef ARDUINO_ARCH_AVR
    _hour = hour(t);
    _min = minute(t);
    _sec = second(t);
#else
    struct tm *ptm = gmtime(&t);
    _hour = ptm->tm_hour;
    _min = ptm->tm_min;
    _sec = ptm->tm_sec;
#endif

    h = (_hour + _min/60 + _sec/3600);
    jd =  (float)getJulianDate0(t) + (float)h/24;
    //printf("%f, %f, %f\n", getJulianDate0(t), h/24, jd);
    return jd;
}

float getJ2000Date(time_t t)
{
    return getJulianDate(t) - 2451545.0;
}


#if 0
void dateFromJ2000(float jd, time_t *t)
{
    //TODO

    tmElements_t tm;
    tm.Year = 2017 - 1970;
    tm.Month = 12;
    tm.Day = 16;
    tm.Hour = 11;
    tm.Minute = 58;
    tm.Second = 35;
    *t = makeTime(tm);

    /*
    float q = jd + 0.5;
    int z = int(jd + 0.5);

    w = (int)((z - 1867216.25)/36524.25);
    b = 1524 + z + 1 + w - (int)(w/4);
    c = (int)((b - 122.1)/365.25);
    d = (int)(365.25*c);
    e = (int)((b - d)/30.6001);
    f = 30.6001*e;

    tm.Day = b - d - f + q - z;
    tm.Month = e - 1;
    if (tm.Month > 13)
        tm.Month -= 12;

    tm.Year = c - 4715;
    if (tm.Month < 3)
        tm.Year -= 1;
    */
}
#endif

// Calculate Greenwich Mean Sidereal Time in radians
// given a J2000-based julian date
// Reference: http://aa.usno.navy.mil/faq/docs/GAST.php
float getGMST(time_t t)
{
    float _hour, _min, _sec, frac, gmst;

#ifdef ARDUINO_ARCH_AVR
    _hour = hour(t);
    _min = minute(t);
    _sec = second(t);
#else
    struct tm *ptm = gmtime(&t);
    _hour = ptm->tm_hour;
    _min = ptm->tm_min;
    _sec = ptm->tm_sec;
#endif

    long jdx = getJulianDate0(t) - 2451544.5;
    frac = (_hour + _min/60 + _sec/3600)/24 - 0.5;
    gmst = 4.8949612127 + 0.0172027918*jdx + 6.3003880989849*frac;
    return normalize2Pi(gmst);
}

// Calculate Local Sidereal Time in radians
// given a J2000-based julian date
float getLST(time_t t, Location loc)
{
    float lst = getGMST(t) + loc.lon;
    return normalize2Pi(lst);
}

// Convert equatorial to horizontal coordinates
// Reference: http://www.geocities.jp/toshimi_taki/matrix/matrix.htm
void localToHoriz(Location loc, LocalCoords lc, HorizCoords *hor)
{
    float x, y, z, x2, y2, z2;

    // Convert to rectangular coordinates
    x = cos(-lc.ha)*cos(lc.dec);
    y = sin(-lc.ha)*cos(lc.dec);
    z = sin(lc.dec);

    // Rotate the coordinate system along east-west axis
    float pz = M_PI/2 - loc.lat;
    x2 = x*cos(pz) - z*sin(pz);
    y2 = y;
    z2 = x*sin(pz) + z*cos(pz);

    // Convert to horizontal coordinates
    hor->az = M_PI - atan2(y2, x2);
    hor->alt = asin(z2);
}

// Convert horizontal to equatorial coordinates
// Reference: http://www.geocities.jp/toshimi_taki/matrix/matrix.htm
void horizToLocal(Location loc, HorizCoords hor, LocalCoords *lc)
{
    float x, y, z, x2, y2, z2;

    // Convert to rectangular coordinates
    x = cos(M_PI - hor.az)*cos(hor.alt);
    y = sin(M_PI - hor.az)*cos(hor.alt);
    z = sin(hor.alt);

    // Rotate the coordinate system along east-west axis
    float zp = loc.lat - M_PI/2;
    x2 = x*cos(zp) - z*sin(zp);
    y2 = y;
    z2 = x*sin(zp) + z*cos(zp);

    // Convert to equatorial coordinates
    lc->ha = -atan2(y2, x2);
    lc->dec = asin(z2);

    if (lc->ha < 0)
        lc->ha += 2*M_PI;
}

// Convert mechanical coordinates to local coordinates (HA/dec)
void axisToLocalCoords(AxisCoords ac, LocalCoords *lc)
{
    uint8_t over;
    float dec = limitDec(ac.dec, &over);

    float ha;
    if (over) {
        ha = ac.ra;
    } else {
        ha = ac.ra + M_PI;
    }

    lc->ha = normalizePi(ha);
    lc->dec = dec;
}

// Convert local coordinates (HA/dec) to mechanical coordinates
void localToAxisCoords(LocalCoords lc, AxisCoords *ac)

{
    float ha = normalizePi(lc.ha);

    if (lc.ha >= 0) {
        ac->ra = normalize2Pi(ha);
        ac->dec = normalize2Pi(M_PI - lc.dec);
    } else {
        ac->ra = normalize2Pi(ha - M_PI);
        ac->dec = normalize2Pi(lc.dec);
    }
}

// Convert local coordinates to equatorial coordinates
void localToEqCoords(LocalCoords lc, float lst, EqCoords *eq)
{
    eq->ra = normalizePi(lst - lc.ha);
    eq->dec = lc.dec;
}

// Convert equatorial coordinates to local coordinates
void eqToLocalCoords(EqCoords eq, float lst, LocalCoords *lc)
{
    lc->ha = normalizePi(lst - eq.ra);
    lc->dec = eq.dec;
}

// Convert equatorial coordinates to mechanical coordinates
void eqToAxisCoords(EqCoords eq, float lst, AxisCoords *ac)
{
    LocalCoords lc;
    eqToLocalCoords(eq, lst, &lc);
    localToAxisCoords(lc, ac);
}

// Convert mechanical coordinates equatorial to coordinates
void axisToEqCoords(AxisCoords ac, float lst, EqCoords *eq)
{
    LocalCoords lc;
    axisToLocalCoords(ac, &lc);
    localToEqCoords(lc, lst, eq);
}
