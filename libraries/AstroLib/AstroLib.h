#ifndef _AstroLib_h_
#define _AstroLib_h_

#include <stdint.h>
#include <TimeLib.h>

// Location in radians
struct Location {
    float latitude;
    float longitude;
};

// Horizontal coordinates in radians
struct HorizCoords {
    float az;
    float alt;
};

// Equatorial coordinates in radians
struct EqCoords {
    float ra;
    float dec;
};

// Equatorial coordinates in radians, with hour angle instead of RA
struct EqHACoords {
    float ha;
    float dec;
};

// Angle in sexagesimal format
struct SxAngle {
	uint8_t deg;
	uint8_t min;
	uint8_t sec;
	uint8_t sign;
};

float sx2rad(SxAngle sx);
void rad2sx(float rad, SxAngle *sx);
void sx2string(SxAngle sx, char *buffer);

float getJulianDate0(time_t t);
float getJulianDate(time_t t);
float getJ2000Date(time_t t);
float getGMST(time_t t);
float getLST(time_t t, Location loc);
void dateFromJ2000(float jd, time_t *t);

void eqToHoriz(Location loc, EqHACoords eq, HorizCoords *hor);
void horizToEq(Location loc, HorizCoords hor, EqHACoords *eq);

#endif
