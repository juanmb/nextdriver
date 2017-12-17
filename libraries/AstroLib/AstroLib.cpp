#include <math.h>
#include <stdio.h>
#include "AstroLib.h"


double sx2rad(SxAngle sx)
{
    double rad = M_PI*((double)sx.deg + (double)sx.min/60.0 + (double)sx.sec/3600.0)/180.0;

    if (sx.sign)
        return 2*M_PI - rad;
    return rad;
}

void rad2sx(double rad, SxAngle *sx)
{
    double deg = rad*180/M_PI;
    if (deg > 180)
        deg = deg - 360;

    sx->sign = (deg < 0);
    deg = fabs(deg);
    double min = 60*(deg - int(deg));
    double sec = 60*(min - int(min));

    sx->deg = (uint8_t)deg;
    sx->min = (uint8_t)min;
    sx->sec = (uint8_t)sec;
}

void sx2string(SxAngle sx, char *buffer)
{
    sprintf(buffer, " %d:%02d:%02d", sx.deg, sx.min, sx.sec);
    if (sx.sign)
        buffer[0] = '-';
}

inline double to24h(double h)
{
    int ih = (int)h;
    h -= (double)(ih/24) * 24;
    return h < 0 ? h + 24 : h;
}

inline double normalize2pi(double h)
{
    int ih = (int)(h/2/M_PI);
    return h - (double)ih*2*M_PI;
}

// Calculate the julian date of the noon of a given date
double getJulianDate0(Date date)
{
    double a, b, c, d;
    double month = date.month;
    double year = date.year;

    if (month < 3) {
        month += 12;
        year -= 1;
    }

    a = (long)(year/100);
    b = 2 - a + (long)(a/4);
    c = (long)(365.25*(year + 4716));
    d = (long)(30.6001*(month + 1));

    return (double)date.day + b + c + d - 1524.5;
}

double getJulianDate(Date date)
{
    // TODO: apply DST?
    double h = ((double)date.hour - (double)date.offset +
            (double)date.min/60 + (double)date.sec/3600);
    return getJulianDate0(date) + h/24;
}

double getJ2000Date(Date date)
{
    double h = ((double)date.hour - (double)date.offset +
            (double)date.min/60 + (double)date.sec/3600);
    return (getJulianDate0(date) - 2451545) + h/24;
}


void dateFromJ2000(double jd, Date *date)
{
    //TODO
    date->year = 2017;
    date->month = 12;
    date->day = 16;
    date->hour = 11;
    date->min = 58;
    date->sec = 35;
    date->offset = 0;
    date->dst = 0;

    /*
    double q = jd + 0.5;
    int z = int(jd + 0.5);

    w = (int)((z - 1867216.25)/36524.25);
    b = 1524 + z + 1 + w - (int)(w/4);
    c = (int)((b - 122.1)/365.25);
    d = (int)(365.25*c);
    e = (int)((b - d)/30.6001);
    f = 30.6001*e;

    date->day = b - d - f + q - z;
    date->month = e - 1;
    if (date->month > 13)
        date->month -= 12;

    date->year = c - 4715;
    if (date->month < 3)
        date->year -= 1;
    */
}

// Calculate Greenwich Mean Sidereal Time in radians
// given a J2000-based julian date
// Reference: http://aa.usno.navy.mil/faq/docs/GAST.php
double getGMST(double jd)
{
    double gmst = (18.697374558 + 24.0657098*jd)*M_PI/12;
    return normalize2pi(gmst);
}

// Calculate Local Sidereal Time in radians
// given a J2000-based julian date
double getLST(double jd, Location loc)
{
    double lst = getGMST(jd) + loc.longitude;
    return normalize2pi(lst);
}

// Convert equatorial to horizontal coordinates
// lst: Local Sidereal Time (in radians)
// Reference: http://www.geocities.jp/toshimi_taki/matrix/matrix.htm
void eqToHoriz(double lst, Location loc, EqCoords eq, HorizCoords *hor)
{
    double x, y, z, x2, y2, z2;

    // obtain the hour angle
    double ha = lst - eq.ra;

    // Convert to rectangular coordinates
    x = cos(-ha)*cos(eq.dec);
    y = sin(-ha)*cos(eq.dec);
    z = sin(eq.dec);

    // Rotate the coordinate system along east-west axis
    double pz = M_PI/2 - loc.latitude;
    x2 = x*cos(pz) - z*sin(pz);
    y2 = y;
    z2 = x*sin(pz) + z*cos(pz);

    // Convert to horizontal coordinates
    hor->az = M_PI - atan2(y2, x2);
    hor->alt = asin(z2);
}

// Convert horizontal to equatorial coordinates
// lst: Local Sidereal Time (in radians)
// Reference: http://www.geocities.jp/toshimi_taki/matrix/matrix.htm
void horizToEq(double lst, Location loc, HorizCoords hor, EqCoords *eq)
{
    double x, y, z, x2, y2, z2;

    // Convert to rectangular coordinates
    x = cos(M_PI - hor.az)*cos(hor.alt);
    y = sin(M_PI - hor.az)*cos(hor.alt);
    z = sin(hor.alt);

    // Rotate the coordinate system along east-west axis
    double zp = loc.latitude - M_PI/2;
    x2 = x*cos(zp) - z*sin(zp);
    y2 = y;
    z2 = x*sin(zp) + z*cos(zp);

    // Convert to equatorial coordinates
    eq->ra = lst + atan2(y2, x2);
    eq->dec = asin(z2);

    if (eq->ra < 0)
        eq->ra += 2*M_PI;
}
