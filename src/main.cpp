/******************************************************************
    Author: Juan Menendez <juanmb@gmail.com>

    This code is part of the NextDriver project:
        https://github.com/juanmb/NextDriver

********************************************************************/

#include <Arduino.h>
#ifdef TARGET_DUE
#include <DueFlashStorage.h>
#else
#include <EEPROM.h>
#include <SoftwareSerial.h>
#endif
#include <TimeLib.h>
#include "serial_command.h"
#include "nexstar_aux.h"
#include "AstroLib.h"

#define VERSION_MAJOR 4
#define VERSION_MINOR 21
#define MOUNT_MODEL 14  // CGEM DX
#define CONTROLLER_VARIANT 0x11  // NexStar
#define BAUDRATE 9600
#define GO_BELOW_HORIZON
#define ENABLE_HOMING

///////////////////////////////////////////
// Pin definitions
///////////////////////////////////////////

#ifdef SIMPLE_AUX_INTERFACE
// Use a simple interface circuit containing only a resistor and a diode
#define AUX_BUSY   3
#else
// Use a more elaborate interface circuit built around a 74LS125
#define AUX_BUSY   2
#endif
#define AUX_SELECT 3
#define AUX_TX     4
#define AUX_RX     5

#define LED_STATUS_G 12 // synced/homed
#define LED_STATUS_R 13 // no time set
#define LED_RED      11 // error
#define LED_GREEN    10 // slewing
#define LED_YELLOW    9 // parked

#define HOME_BUTTON   7
#define ABORT_BUTTON   8
#define HOME_RA_PIN   A0
#define LIMIT_RA_PIN  A1
#define HOME_DEC_PIN  A2
#define LIMIT_ALT_PIN A3

#define ABS(a) ((a) < 0 ? -(a) : (a))

#define decHomeSensor() (analogRead(HOME_DEC_PIN) > 512) // true if blocked
#define raHomeSensor() (analogRead(HOME_RA_PIN) > 512)   // true if blocked

enum ScopeState {
    ST_IDLE,
    ST_MERIDIAN_FLIP,
    ST_SLEWING_FAST,
    ST_SLEWING_SLOW,
    ST_HOMING_DEC,
    ST_HOMING_FAST1,
    ST_HOMING_FAST2,
    ST_HOMING_SLOW1,
    ST_HOMING_SLOW2,
};

enum ScopeEvent {
    EV_NONE,
    EV_GOTO,
    EV_ABORT,
    EV_HOME,
};

enum PierSide {
    PIER_EAST,  // Telescope on the east side of pier (looking west)
    PIER_WEST,  // Telescope on the west side of pier (looking east)
};

ScopeState state = ST_IDLE;
ScopeEvent event = EV_NONE;

// Position of the home (or index) sensors in axis coordinates.
// For now, we assume the sensors are always at exactly pi/2, pi/2:
// the telescope looking at the pole and the counterweight pointing downwards
const AxisCoords home_pos = {M_PI/2, M_PI/2};

Location location = {0, 0};      // Default location
uint8_t tracking_mode = 0;
uint32_t ref_t = 0;    // millis() at last cmdSetTime call
float ref_lst = 0.0;  // Last synced LST
float ref_jd = 0.0;   // Last synced julian date refered to J2000
bool synced = false;
bool time_set = false;
bool aligned = false;
bool parked = false;
EqCoords target = {0};

// Addresses in EEPROM/Flash
#define ADDR_LOCATION 0
#define ADDR_HOME 2*4
#define ADDR_TRACKING 2*4

#ifdef TARGET_DUE
#define aux Serial1
DueFlashStorage FS;
#else
SoftwareSerial aux(AUX_RX, AUX_TX);
#endif

NexStarAux nexstar(&aux, AUX_SELECT, AUX_BUSY);
SerialCommand sCmd(&Serial);


/*
// Get current Julian date
static float getCurrentJD()
{
    return ref_jd + (float)(now() - ref_t)/(24.0*60*60);
}
*/

// Obtain current Local Sidereal Time from last synced time
// This is faster than performing the full calculation from JD & location
// by calling getLST()
float getCurrentLST() {
    return ref_lst + 2*M_PI*(float)(now() - ref_t)/(24.0*60*60);
}

/*****************************************************************************
  Get/set/goto position
******************************************************************************/

void getAxisCoords(AxisCoords *ac)
{
    uint32_t ra, dec;
    nexstar.getPosition(DEV_RA, &ra);
    nexstar.getPosition(DEV_DEC, &dec);

    ac->ra = normalizePi(pnex2rad(ra));
    ac->dec = normalizePi(pnex2rad(dec));
}

void getLocalCoords(LocalCoords *lc)
{
    AxisCoords ac;
    getAxisCoords(&ac);
    axisToLocalCoords(ac, lc);
}

void getEqCoords(EqCoords *eq)
{
    LocalCoords lc;
    getLocalCoords(&lc);
    localToEqCoords(lc, getCurrentLST(), eq);
}

void getHorizCoords(HorizCoords *hc)
{
    LocalCoords lc;
    getLocalCoords(&lc);
    localToHoriz(location, lc, hc);
}

void setAxisCoords(const AxisCoords ac)
{
    nexstar.setPosition(DEV_RA, rad2pnex(ac.ra));
    nexstar.setPosition(DEV_DEC, rad2pnex(ac.dec));
    synced = true;
}

void setLocalCoords(const LocalCoords lc)
{
    AxisCoords ac;
    localToAxisCoords(lc, &ac);
    setAxisCoords(ac);
}

void setEqCoords(const EqCoords eq)
{
    AxisCoords ac;
    eqToAxisCoords(eq, getCurrentLST(), &ac);
    setAxisCoords(ac);
}

void gotoAxisCoords(const AxisCoords ac, bool slow)
{
    if (synced) {
        nexstar.gotoPosition(DEV_RA, slow, rad2pnex(ac.ra));
        nexstar.gotoPosition(DEV_DEC, slow, rad2pnex(ac.dec));
    }
}

void gotoEqCoords(const EqCoords eq, bool slow)
{
    AxisCoords ac;
    eqToAxisCoords(eq, getCurrentLST(), &ac);
    gotoAxisCoords(ac, slow);
}

void stopMotors()
{
    nexstar.move(DEV_RA, 0, 0);
    nexstar.move(DEV_DEC, 0, 0);
}

// Check if both axes have reached their target position
bool slewDone()
{
    bool ra_done, dec_done;

    nexstar.slewDone(DEV_RA, &ra_done);
    nexstar.slewDone(DEV_DEC, &dec_done);
    return ra_done && dec_done;
}

// Indicates the pointing state of the mount at a given position
// Reference: https://ascom-standards.org/Help/Platform/html/P_ASCOM_DeviceInterface_ITelescopeV3_SideOfPier.htm
PierSide getPierSide(AxisCoords ac)
{
    float dec = normalizePi(ac.dec);
    return ABS(dec) <= M_PI/2 ? PIER_WEST : PIER_EAST;
}

PierSide getPierSide()
{
    AxisCoords ac;
    getAxisCoords(&ac);
    return getPierSide(ac);
}

// Check if a meridian flip is required
bool checkMeridianFlip(EqCoords eq)
{
    AxisCoords ac;
    PierSide target_side, current_side;

    eqToAxisCoords(eq, getCurrentLST(), &ac);
    target_side = getPierSide(ac);
    current_side = getPierSide();

    // A meridian flip is required if both angles have different sign
    return target_side != current_side;
}

void readLocation(Location *loc)
{
#ifdef __arm__
    byte *b1 = FS.readAddress(ADDR_LOCATION);
    memcpy(loc, b1, sizeof(Location));
#else
    EEPROM.get(ADDR_LOCATION, *loc);
#endif
}

// Store the location in EEPROM
void writeLocation(Location loc)
{
#ifdef __arm__
    byte b[sizeof(Location)];
    memcpy(b, &loc, sizeof(Location));
    FS.write(ADDR_LOCATION, b, sizeof(Location));
#else
    EEPROM.put(ADDR_LOCATION, loc);
#endif
}

// Read the position of the index sensors from EEPROM.
void readHomePosition(AxisCoords *ac)
{
#ifdef __arm__
    byte *b2 = FS.readAddress(ADDR_HOME);
    memcpy(ac, b2, sizeof(LocalCoords));
#else
    EEPROM.get(ADDR_HOME, *ac);
#endif
}

// Save current axis coordinates as the home position
void writeHomePosition()
{
    AxisCoords ac;
    getAxisCoords(&ac);

    // Store the home position in EEPROM
#ifdef __arm__
    byte b[sizeof(AxisCoords)];
    memcpy(b, &ac, sizeof(AxisCoords));
    FS.write(ADDR_HOME, b, sizeof(AxisCoords));
#else
    EEPROM.put(ADDR_HOME, ac);
#endif
}

/*****************************************************************************
  Serial commands
******************************************************************************/

void cmdGetEqCoords(char *cmd)
{
    EqCoords eq;
    getEqCoords(&eq);

    char tmp[19];

    if (cmd[0] == 'E')
        sprintf(tmp, "%04X,%04X#", rad2nex(eq.ra), rad2nex(eq.dec));
    else
        sprintf(tmp, "%08lX,%08lX#", rad2pnex(eq.ra), rad2pnex(eq.dec));

    Serial.write(tmp);
}

void cmdGetAzCoords(char *cmd)
{
    HorizCoords hc;
    getHorizCoords(&hc);

    char tmp[19];

    if (cmd[0] == 'Z')
        sprintf(tmp, "%04X,%04X#", rad2nex(hc.az), rad2nex(hc.alt));
    else
        sprintf(tmp, "%08lX,%08lX#", rad2pnex(hc.az), rad2pnex(hc.alt));

    Serial.write(tmp);
}

void cmdGetPierSide(char *cmd)
{
    // Celestron protocol use the opposite definition of pier side
    Serial.write(getPierSide() == PIER_EAST ? 'W' : 'E');
    Serial.write('#');
}

void cmdSyncEqCoords(char *cmd)
{
    EqCoords eq;

    if (cmd[0] == 'S') {
        short unsigned int ra, dec;
        sscanf(cmd + 1, "%4hx,%4hx", &ra, &dec);
        eq.ra = nex2rad(ra);
        eq.dec = nex2rad(dec);
    } else {
        uint32_t ra, dec;
        sscanf(cmd + 1, "%8lx,%8lx", &ra, &dec);
        eq.ra = pnex2rad(ra);
        eq.dec = pnex2rad(dec);
    }

    setEqCoords(eq);
    Serial.write('#');
}

void cmdGotoEqCoords(char *cmd)
{
    EqCoords eq;

    if (cmd[0] == 'R') {
        short unsigned int ra, dec;
        sscanf(cmd + 1, "%4hx,%4hx", &ra, &dec);
        eq.ra = nex2rad(ra);
        eq.dec = nex2rad(dec);
#ifdef ENABLE_HOMING
        // Home the mount if dec=pi/2
        if (dec == 0x4000) {
            event = EV_HOME;
            Serial.write('#');
            return;
        }
#endif
    } else {
        uint32_t ra, dec;
        sscanf(cmd + 1, "%8lx,%8lx", &ra, &dec);
        eq.ra = pnex2rad(ra);
        eq.dec = pnex2rad(dec);
#ifdef ENABLE_HOMING
        // Home the mount if dec=pi/2
        if (dec == 0x40000000) {
            event = EV_HOME;
            Serial.write('#');
            return;
        }
#endif
    }

#ifdef GO_BELOW_HORIZON
    target.ra = eq.ra;
    target.dec = eq.dec;
    event = EV_GOTO;
#else
    // Obtain the horizontal coordinates of the target
    LocalCoords lc;
    HorizCoords hc;
    eqToLocalCoords(eq, &lc);
    localToHoriz(location, lc, &hc);

    // If target is above the horizon, go
    if (hc.alt >= 0) {
        target.ra = eq.ra;
        target.dec = eq.dec;
        event = EV_GOTO;
    }
#endif
    Serial.write('#');
}

void cmdGotoAzCoords(char *cmd)
{
    HorizCoords hc;

    if (cmd[0] == 'B') {
        short unsigned int az, alt;
        sscanf(cmd + 1, "%4hx,%4hx", &az, &alt);
        hc.az = nex2rad(az);
        hc.alt = nex2rad(alt);
    } else {
        uint32_t az, alt;
        sscanf(cmd + 1, "%8lx,%8lx", &az, &alt);
        hc.az = pnex2rad(az);
        hc.alt = pnex2rad(alt);
    }

    if (hc.alt >= 0) {
        // If target is above the horizon, go
        LocalCoords lc;
        horizToLocal(location, hc, &lc);
        localToEqCoords(lc, getCurrentLST(), &target);
        event = EV_GOTO;
    }
    Serial.write('#');
}

void cmdIsAligned(char *cmd)
{
    Serial.write(aligned ? 1 : 0);
    Serial.write('#');
}

void cmdGotoInProgress(char *cmd)
{
    Serial.write(state == ST_IDLE ? '0' : '1');
    Serial.write('#');
}

void cmdCancelGoto(char *cmd)
{
    event = EV_ABORT;
    Serial.write('#');
}

void cmdGetTrackingMode(char *cmd)
{
    Serial.write(tracking_mode);
    Serial.write('#');
}

void cmdSetTrackingMode(char *cmd)
{
    //TODO: store tracking mode in EEPROM and engage tracking at start
    tracking_mode = cmd[1];
    nexstar.setGuiderate(DEV_RA, GUIDERATE_POS, true, 0);    // stop RA motor
    nexstar.setGuiderate(DEV_DEC, GUIDERATE_POS, true, 0);   // stop DEC motor

    switch(tracking_mode) {
        case TRACKING_EQ_NORTH:
            nexstar.setGuiderate(DEV_RA, GUIDERATE_POS, 0, GUIDERATE_SIDEREAL);
            break;
        case TRACKING_EQ_SOUTH:
            nexstar.setGuiderate(DEV_RA, GUIDERATE_NEG, 0, GUIDERATE_SIDEREAL);
            break;
    }

    Serial.write('#');
}

void cmdGetLocation(char *cmd)
{
    SxAngle sxLat, sxLong;

    rad2sx(normalizePi(location.lat), &sxLat);
    Serial.write(sxLat.deg);
    Serial.write(sxLat.min);
    Serial.write(sxLat.sec);
    Serial.write(sxLat.sign);

    rad2sx(normalizePi(location.lon), &sxLong);
    Serial.write(sxLong.deg);
    Serial.write(sxLong.min);
    Serial.write(sxLong.sec);
    Serial.write(sxLong.sign);
    Serial.write('#');
}

void cmdSetLocation(char *cmd)
{
    SxAngle latitude = {
        (uint8_t)cmd[1], (uint8_t)cmd[2],
        (uint8_t)cmd[3], (uint8_t)cmd[4]
    };

    SxAngle longitude = {
        (uint8_t)cmd[5], (uint8_t)cmd[6],
        (uint8_t)cmd[7], (uint8_t)cmd[8]
    };

    float lat = normalizePi(sx2rad(latitude));
    float lon = normalizePi(sx2rad(longitude));

    if ((lat != location.lat) || (lon != location.lon)) {
        location.lat = lat;
        location.lon = lon;

        ref_lst = getLST(now(), location);
        synced = false;

	writeLocation(location);
    }

    Serial.write('#');
}

void cmdSetTime(char *cmd)
{
    int hour = (int)cmd[1];
    int min = (int)cmd[2];
    int sec = (int)cmd[3];
    int month = (int)cmd[4];
    int day = (int)cmd[5];
    int year = (int)cmd[6] + 2000;
    int offset = (int)cmd[7];
    //int dst = (int)cmd[8];

    setTime(hour, min, sec, day, month, year);
    adjustTime(-offset*3600);

    ref_t = now();
    ref_lst = getLST(ref_t, location);
    time_set = true;
    synced = false;

    Serial.write('#');
}

void cmdGetTime(char *cmd)
{
    time_t t = now();
    Serial.write(hour(t));
    Serial.write(minute(t));
    Serial.write(second(t));
    Serial.write(month(t));
    Serial.write(day(t));
    Serial.write(year(t) % 2000);
    Serial.write((uint8_t)0);
    Serial.write((uint8_t)0);
    Serial.write("#");
}

void cmdPassThrough(char *cmd)
{
    char resp[8];
    uint8_t size;

    if (nexstar.sendRawCommand(cmd, resp, &size) == 0) {
        for (int i=0; i < size; i++) {
            Serial.write(resp[i]);
        }
    } else {
        // indicate an error by returning a response of size = resp_size + 1
        for (int i=0; i <= size; i++) {
            Serial.write('0');
        }
    }

    Serial.write('#');
}

void cmdGetVersion(char *cmd)
{
    Serial.write(VERSION_MAJOR);
    Serial.write(VERSION_MINOR);
    Serial.write('#');
}

void cmdGetVariant(char *cmd)
{
    Serial.write(CONTROLLER_VARIANT);
    Serial.write('#');
}

void cmdGetModel(char *cmd)
{
    Serial.write(MOUNT_MODEL);
    Serial.write('#');
}

void cmdEcho(char *cmd)
{
    Serial.write(cmd[1]);
    Serial.write('#');
}

void cmdHibernate(char *cmd)
{
    //TODO
}

void cmdWakeup(char *cmd)
{
    //TODO
    Serial.write('#');
}

void setup()
{
    // Map serial commands to functions
    sCmd.addCommand('E', 1, cmdGetEqCoords);
    sCmd.addCommand('e', 1, cmdGetEqCoords);
    sCmd.addCommand('Z', 1, cmdGetAzCoords);
    sCmd.addCommand('z', 1, cmdGetAzCoords);
    sCmd.addCommand('p', 1, cmdGetPierSide);

    sCmd.addCommand('S', 10, cmdSyncEqCoords);
    sCmd.addCommand('s', 18, cmdSyncEqCoords);

    sCmd.addCommand('R', 10, cmdGotoEqCoords);
    sCmd.addCommand('r', 18, cmdGotoEqCoords);

    sCmd.addCommand('B', 10, cmdGotoAzCoords);
    sCmd.addCommand('b', 18, cmdGotoAzCoords);

    sCmd.addCommand('L', 1, cmdGotoInProgress);
    sCmd.addCommand('M', 1, cmdCancelGoto);
    sCmd.addCommand('J', 1, cmdIsAligned);

    sCmd.addCommand('T', 2, cmdSetTrackingMode);
    sCmd.addCommand('t', 1, cmdGetTrackingMode);

    sCmd.addCommand('W', 9, cmdSetLocation);
    sCmd.addCommand('w', 1, cmdGetLocation);
    sCmd.addCommand('H', 9, cmdSetTime);
    sCmd.addCommand('h', 1, cmdGetTime);

    sCmd.addCommand('P', 8, cmdPassThrough);
    sCmd.addCommand('V', 1, cmdGetVersion);
    sCmd.addCommand('v', 1, cmdGetVariant);
    sCmd.addCommand('m', 1, cmdGetModel);
    sCmd.addCommand('K', 2, cmdEcho);

    sCmd.addCommand('x', 1, cmdHibernate);
    sCmd.addCommand('y', 1, cmdWakeup);

    pinMode(AUX_SELECT, OUTPUT);
    //pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LED_STATUS_R, OUTPUT);
    pinMode(LED_STATUS_G, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);

    pinMode(HOME_BUTTON, INPUT_PULLUP);
    pinMode(ABORT_BUTTON, INPUT_PULLUP);
    pinMode(LIMIT_RA_PIN, INPUT_PULLUP);
    pinMode(LIMIT_ALT_PIN, INPUT_PULLUP);

    aux.begin(AUX_BAUDRATE);

    Serial.begin(BAUDRATE);
    nexstar.init();

    // read the location from EEPROM
    readLocation(&location);

    setAxisCoords(home_pos);

    nexstar.setGuiderate(DEV_RA, GUIDERATE_POS, true, 0);    // stop RA motor
    nexstar.setGuiderate(DEV_DEC, GUIDERATE_POS, true, 0);   // stop DEC motor
}

// Update the scope status with a simple state machine
void updateFSM()
{
    static uint32_t t_timer;
    static bool ra_homed = 0, dec_homed = 0, dec_homing_dir = 0;

    if (event == EV_ABORT) {
        stopMotors();
        state = ST_IDLE;
        event = EV_NONE;
        return;
    }

    switch(state) {
        case ST_IDLE:
            if (event == EV_GOTO) {
                t_timer = millis();
                if (checkMeridianFlip(target)) {
                    // go to the pole before flipping
                    gotoAxisCoords((AxisCoords){M_PI/2, M_PI/2}, false);
                    state = ST_MERIDIAN_FLIP;
                    break;
                }
                // TODO: home the mount if the target coords are (0, pi/2)
                gotoEqCoords(target, false);
                state = ST_SLEWING_FAST;
            } else if (event == EV_HOME) {
                dec_homing_dir = !decHomeSensor();
                nexstar.move(DEV_DEC, dec_homing_dir, 9);
                aligned = false;
                state = ST_HOMING_DEC;
            }
            break;

        case ST_MERIDIAN_FLIP:
            if (millis() - t_timer > 500) {
                // Every 0.5 seconds, check if we are close to the target
                t_timer = millis();

                if (slewDone()) {
                    gotoEqCoords(target, false);
                    state = ST_SLEWING_FAST;
                }
            }
            break;

        case ST_SLEWING_FAST:
            if (millis() - t_timer > 500) {
                // Every 0.5 seconds, check if we are close to the target
                t_timer = millis();

                if (slewDone()) {
                    gotoEqCoords(target, true);
                    state = ST_SLEWING_SLOW;
                }
            }
            break;

        case ST_SLEWING_SLOW:
            if (millis() - t_timer > 250) {
                // Every 0.25 seconds, check if slew is done
                t_timer = millis();

                if (slewDone()) {
                    state = ST_IDLE;
                }
            }
            break;

        case ST_HOMING_DEC:
            // Move CW dec axis until the photodiode changes
            if (dec_homing_dir == decHomeSensor()) {
                nexstar.move(DEV_DEC, 0, 9);
                nexstar.move(DEV_RA, 0, 9);
                ra_homed = 0;
                dec_homed = 0;
                state = ST_HOMING_FAST1;
            }
            break;

        case ST_HOMING_FAST1:
            // Move CW each axis until the photodiode receives light
            if (!dec_homed && !decHomeSensor()) {
                nexstar.move(DEV_DEC, 0, 0);
                dec_homed = 1;
            }
            if (!ra_homed && !raHomeSensor()) {
                nexstar.move(DEV_RA, 0, 0);
                ra_homed = 1;
            }

            if (ra_homed && dec_homed) {
                ra_homed = 0;
                dec_homed = 0;
                nexstar.move(DEV_DEC, 1, 9);
                nexstar.move(DEV_RA, 1, 9);
                state = ST_HOMING_FAST2;
            }
            break;

        case ST_HOMING_FAST2:
            // Move CCW each axis until the photodiode stops receiving light
            if (!dec_homed && decHomeSensor()) {
                nexstar.move(DEV_DEC, 0, 0);
                dec_homed = 1;
            }
            if (!ra_homed && raHomeSensor()) {
                nexstar.move(DEV_RA, 0, 0);
                ra_homed = 1;
            }

            if (ra_homed && dec_homed) {
                ra_homed = 0;
                dec_homed = 0;
                nexstar.move(DEV_DEC, 0, 6);
                nexstar.move(DEV_RA, 0, 6);
                state = ST_HOMING_SLOW1;
            }
            break;

        case ST_HOMING_SLOW1:
            // Move CCW each axis until the photodiode stops receiving light
            if (!dec_homed && !decHomeSensor()) {
                nexstar.move(DEV_DEC, 0, 0);
                dec_homed = 1;
            }
            if (!ra_homed && !raHomeSensor()) {
                nexstar.move(DEV_RA, 0, 0);
                ra_homed = 1;
            }

            if (ra_homed && dec_homed) {
                ra_homed = 0;
                dec_homed = 0;
                nexstar.move(DEV_DEC, 1, 4);
                nexstar.move(DEV_RA, 1, 4);
                state = ST_HOMING_SLOW2;
            }
            break;

        case ST_HOMING_SLOW2:
            if (!dec_homed && decHomeSensor()) {
                nexstar.move(DEV_DEC, 0, 0);
                dec_homed = 1;
            }
            if (!ra_homed && raHomeSensor()) {
                nexstar.move(DEV_RA, 0, 0);
                ra_homed = 1;
            }

            if (ra_homed && dec_homed) {
                setAxisCoords(home_pos);

                aligned = true;
                state = ST_IDLE;
            }
            break;
    };

    event = EV_NONE;
}

void updateLEDs()
{
    if (!time_set) {
        digitalWrite(LED_STATUS_G, LOW);
        digitalWrite(LED_STATUS_R, HIGH);
    } else if (!synced) {
        digitalWrite(LED_STATUS_G, HIGH);
        digitalWrite(LED_STATUS_R, HIGH);
    } else {
        digitalWrite(LED_STATUS_G, HIGH);
        digitalWrite(LED_STATUS_R, LOW);
    }

    digitalWrite(LED_GREEN, (state != ST_IDLE));
    digitalWrite(LED_YELLOW, parked);
}

void loop()
{
    if (!digitalRead(ABORT_BUTTON) && state != ST_IDLE)
        event = EV_ABORT;
    else if (!digitalRead(HOME_BUTTON) && state != ST_HOMING_DEC)
        event = EV_HOME;

    sCmd.readSerial();
    updateFSM();
    updateLEDs();
    nexstar.run();
}
