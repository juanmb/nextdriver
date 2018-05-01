#ifndef _AstroLib_h_
#define _AstroLib_h_

#include <stdint.h>

struct Date {
    int year;
    int month;
    int day;
    int hour;
    int min;
    int sec;
    int offset;
    int dst;
};

// Location in radians
struct Location {
    double latitude;
    double longitude;
};

// Horizontal coordinates in radians
struct HorizCoords {
    double az;
    double alt;
};

// Equatorial coordinates in radians
struct EqCoords {
    double ra;
    double dec;
};

// Equatorial coordinates in radians, with hour angle instead of RA
struct EqHACoords {
    double ha;
    double dec;
};

// Angle in sexagesimal format
struct SxAngle {
	uint8_t deg;
	uint8_t min;
	uint8_t sec;
	uint8_t sign;
};

double sx2rad(SxAngle sx);
void rad2sx(double rad, SxAngle *sx);
void sx2string(SxAngle sx, char *buffer);

double getJulianDate0(Date date);
double getJulianDate(Date date);
double getJ2000Date(Date date);
double getGMST(double jd);
double getLST(double jd, Location loc);
void dateFromJ2000(double jd, Date *date);

void eqToHoriz(Location loc, EqHACoords eq, HorizCoords *hor);
void horizToEq(Location loc, HorizCoords hor, EqHACoords *eq);

#endif
