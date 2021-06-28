#ifndef _AstroLib_h_
#define _AstroLib_h_

#include <stdint.h>

#ifdef ARDUINO_ARCH_AVR
#include <TimeLib.h>
#else
#include <time.h>
#endif

// Location in radians
struct Location {
    float lon;
    float lat;
};

// Horizontal coordinates (Azimuth/altitude) in radians
struct HorizCoords {
    float az;  // azimuth
    float alt; // altitude
};

// Equatorial coordinates (RA/dec) in radians
struct EqCoords {
    float ra;  // right ascension
    float dec; // declination
};

// Local coordinates (HA/dec) in radians
struct LocalCoords {
    float ha;  // hour angle
    float dec; // declination
};

// Mechanical coordinates of an equatorial telescope mount (radians)
//
// The 'ra' angle is:
//  * 0:    counterweight pointing to the East
//  * pi/2: counterweight pointing to the ground
//  * pi:   counterweight pointing to the West
//
// If 'ra' is pi/2, the 'dec' angle is:
//  * 0:    tube to the East
//  * pi/2: tube to the North pole
//  * pi:   tube to the West
struct AxisCoords {
    float ra;  // mechanical angle of RA axis
    float dec; // mechanical angle of dec axis
};

// Angle in sexagesimal format
// TODO: make 'deg' int16_t and remove the sign field
struct SxAngle {
    uint16_t deg;
    uint8_t min;
    uint8_t sec;
    uint8_t sign;
};

/*****************************************************************************
  Angle conversion functions
******************************************************************************/

float limitDec(float, uint8_t*);
float normalizePi(float);
float normalize2Pi(float);
float sx2rad(SxAngle sx);
void rad2sx(float rad, SxAngle *sx);
void sx2string(SxAngle sx, char *buffer);

/*****************************************************************************
  Date conversion functions
******************************************************************************/

float getJulianDate0(time_t t);
float getJulianDate(time_t t);
float getJ2000Date(time_t t);
float getGMST(time_t t);
float getLST(time_t t, Location loc);
//void dateFromJ2000(float jd, time_t *t);

/*****************************************************************************
  Coordinates conversion functions
******************************************************************************/

void localToHoriz(Location loc, LocalCoords lc, HorizCoords *hor);
void horizToLocal(Location loc, HorizCoords hor, LocalCoords *lc);
void axisToLocalCoords(AxisCoords ac, LocalCoords *lc);
void localToAxisCoords(LocalCoords lc, AxisCoords *ac);
void localToEqCoords(LocalCoords lc, float lst, EqCoords *eq);
void eqToLocalCoords(EqCoords eq, float lst, LocalCoords *lc);
void eqToAxisCoords(EqCoords eq, float lst, AxisCoords *ac);
void axisToEqCoords(AxisCoords ac, float lst, EqCoords *eq);

#endif
