/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the NexStarAdapter project:
        https://github.com/juanmb/NexStarAdapter

********************************************************************/

//#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <TimeLib.h>
#include "serial_command.h"
#include "nexstar_aux.h"
#include "nexstar_stepper.h"
#include "AstroLib.h"

#define VERSION_MAJOR 4
#define VERSION_MINOR 21
#define MOUNT_MODEL 10  // GT
#define CONTROLLER_VARIANT 0x11  // NexStar
#define BAUDRATE 9600
#define GO_BELOW_HORIZON
#define USE_HOME_SWITCHES
#define MERIDIAN_MARGIN (M_PI/8)
//#define DEBUG

// pin definitions
#define AUX_SELECT 5
#define AUX_RX 6
#define AUX_TX 7
#define DEBUG_TX 2
//#define BUTTON 3
#define HOME_DEC_PIN A6
#define HOME_RA_PIN A7

// Hour angle and declination of home position in radians
#define HOME_HA (M_PI*(6 + 2.0/60 + 20.0/3600)/12)   // 6h 2m 20s
#define HOME_DEC (M_PI*(71 + 12.0/60)/180)           // 71 12'

#define abs(x) (((x) > 0) ? (x) : -(x))
#define sign(x) (((x) > 0) ? 1 : -1)

#define decHomeSensor() (analogRead(HOME_DEC_PIN) > 512) // returns true if blocked
#define raHomeSensor() (analogRead(HOME_RA_PIN) > 512)   // returns true if blocked

#ifdef DEBUG
#define DEBUG_PRINT(str) debug.println(str)
#else
#define DEBUG_PRINT(str) do {} while(0)
#endif

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

struct AxisCoords {
    float ra;
    float dec;
};

ScopeState state = ST_IDLE;
ScopeEvent event = EV_NONE;

Location location = {0.75894737, -0.10014247};   // Default location
uint8_t tracking_mode = 0;
uint32_t ref_t = 0; // millis() at last cmdSetTime call
double ref_lst = 0.0;  // Last synced LST
double ref_jd = 0.0;     // Last synced julian date refered to J2000
bool synced = false;
EqCoords target = {0};


SerialCommand sCmd;
NexStarAux nexstar(AUX_RX, AUX_TX, AUX_SELECT);
//NexStarStepper nexstar;

#ifdef DEBUG
SoftwareSerial debug(4, 2);
#endif

// Get current Julian date
//static double getCurrentJD()
//{
    //return ref_jd + (double)(now() - ref_t)/(24.0*60*60);
//}

double normalizeAngle2pi(double h)
{
    int ih = (int)(h/2/M_PI);
    h = h - (double)ih*2*M_PI;
    return h < 0 ? h + 2*M_PI : h;
}

// Normalize an angle in radians between -pi and pi
double normalizeAngle(double h)
{
    h = normalizeAngle2pi(h);
    int ih = (int)(h/2/M_PI);
    h = h - (double)ih*2*M_PI;
    return h > M_PI ? h - 2*M_PI: h;
}

// Obtain current Local Sidereal Time from last synced time
// This is faster than performing the full calculation from JD & location
// by calling getLST()
static double getCurrentLST() {
    return ref_lst + 2*M_PI*(double)(now() - ref_t)/(24.0*60*60);
}

// Convert nexstar angle format to radians
double nex2rad(uint16_t angle)
{
    return 2*M_PI*((double)angle / 0x10000);
}

// Convert nexstar precise angle format to radians
double pnex2rad(uint32_t angle)
{
    return 2*M_PI*((double)angle / 0x100000000);
}

// Convert radians to nexstar angle format
uint16_t rad2nex(double rad)
{
    return (uint16_t)(rad * 0x10000 / (2*M_PI));
}

// Convert radians to nexstar precise angle format
uint32_t rad2pnex(double rad)
{
    return (uint32_t)(rad * 0x100000000 / (2*M_PI));
}

void stopMotors()
{
    nexstar.move(DEV_RA, 0, 0);
    nexstar.move(DEV_DEC, 0, 0);
}

void getHADec(EqHACoords *eq)
{
    uint32_t int_ra_pos, int_dec_pos;
    nexstar.getPosition(DEV_RA, &int_ra_pos);
    nexstar.getPosition(DEV_DEC, &int_dec_pos);
    double ra_pos = normalizeAngle(pnex2rad(int_ra_pos));
    double dec_pos = normalizeAngle(pnex2rad(int_dec_pos));

    eq->ha = ra_pos + M_PI/2*(sign(dec_pos) - 1);
    eq->dec = M_PI/2 - abs(dec_pos);
}

// Convert local equatorial coordinates to axis coordinates
void eqHAToAxisCoords(EqHACoords eq, double *ra_pos, double *dec_pos)
{
    *ra_pos = eq.ha + M_PI/2*(1 - sign(eq.ha));
    *dec_pos = (M_PI/2 - eq.dec)*sign(eq.ha);
}

// Convert absolute equatorial coordinates to axis coordinates
void eqToAxisCoords(EqCoords eq, double *ra_pos, double *dec_pos)
{
    EqHACoords eq2 = {normalizeAngle(getCurrentLST() - eq.ra), eq.dec};
    eqHAToAxisCoords(eq2, ra_pos, dec_pos);
}

void getEqCoords(EqCoords *eq)
{
    EqHACoords pos;
    getHADec(&pos);

    eq->ra = normalizeAngle2pi(getCurrentLST() - pos.ha);
    eq->dec = pos.dec;
}

void getAzCoords(HorizCoords *hor)
{
    EqHACoords pos;
    getHADec(&pos);

    eqToHoriz(location, pos, hor);
}

void setEqCoords(EqCoords eq)
{
    double ra_pos, dec_pos;
    eqToAxisCoords(eq, &ra_pos, &dec_pos);

    nexstar.setPosition(DEV_RA, rad2pnex(ra_pos));
    nexstar.setPosition(DEV_DEC, rad2pnex(dec_pos));
    synced = true;
}

void setHACoords(EqHACoords eq)
{
    double ra_pos, dec_pos;
    eqHAToAxisCoords(eq, &ra_pos, &dec_pos);

    nexstar.setPosition(DEV_RA, rad2pnex(ra_pos));
    nexstar.setPosition(DEV_DEC, rad2pnex(dec_pos));
    synced = true;
}

void gotoEqCoords(EqCoords eq, bool slow)
{
    if (!synced)
        return;

    double ra_pos, dec_pos;
    eqToAxisCoords(eq, &ra_pos, &dec_pos);

    nexstar.gotoPosition(DEV_RA, slow, rad2pnex(ra_pos));
    nexstar.gotoPosition(DEV_DEC, slow, rad2pnex(dec_pos));
}

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
    HorizCoords hor;
    getAzCoords(&hor);

    char tmp[19];

    if (cmd[0] == 'Z')
        sprintf(tmp, "%04X,%04X#", rad2nex(hor.az), rad2nex(hor.alt));
    else
        sprintf(tmp, "%08lX,%08lX#", rad2pnex(hor.az), rad2pnex(hor.alt));

    Serial.write(tmp);
}

void cmdSyncEqCoords(char *cmd)
{
    EqCoords eq;

    if (cmd[0] == 'S') {
        uint16_t ra, dec;
        sscanf(cmd + 1, "%4x,%4x", &ra, &dec);
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
        uint16_t ra, dec;
        sscanf(cmd + 1, "%4x,%4x", &ra, &dec);
        eq.ra = nex2rad(ra);
        eq.dec = nex2rad(dec);
    } else {
        uint32_t ra, dec;
        sscanf(cmd + 1, "%8lx,%8lx", &ra, &dec);
        eq.ra = pnex2rad(ra);
        eq.dec = pnex2rad(dec);
    }

#ifdef GO_BELOW_HORIZON
    target.ra = eq.ra;
    target.dec = eq.dec;
    event = EV_GOTO;
#else
    // Obtain the horizontal coordinates of the target
    EqHACoords eqHA;
    HorizCoords hor;
    eqHA.ha = getCurrentLST() - eq.ra;
    eqHA.dec = eq.dec;
    eqToHoriz(location, eqHA, &hor);

    // If target is above the horizon, go
    if (hor.alt >= 0) {
        target.ra = eq.ra;
        target.dec = eq.dec;
        event = EV_GOTO;
    }
#endif
    Serial.write('#');
}

void cmdGotoAzCoords(char *cmd)
{
    HorizCoords hor;

    if (cmd[0] == 'B') {
        uint16_t az, alt;
        sscanf(cmd + 1, "%4x,%4x", &az, &alt);
        hor.az = nex2rad(az);
        hor.alt = nex2rad(alt);
    } else {
        uint32_t az, alt;
        sscanf(cmd + 1, "%8lx,%8lx", &az, &alt);
        hor.az = pnex2rad(az);
        hor.alt = pnex2rad(alt);
    }

    if (hor.alt >= 0) {
        // If target is above the horizon, go
        EqHACoords eq;
        horizToEq(location, hor, &eq);
        target.ra = getCurrentLST() - eq.ha;
        target.dec = eq.dec;

        event = EV_GOTO;

#ifdef USE_HOME_SWITCHES
        // Home the mount if az=0, alt=0
        if (hor.az == 0 && hor.alt == 0) {
            event = EV_HOME;
        }
#endif
    }
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

    rad2sx(location.latitude, &sxLat);
    Serial.write(sxLat.deg);
    Serial.write(sxLat.min);
    Serial.write(sxLat.sec);
    Serial.write(sxLat.sign);

    rad2sx(location.longitude, &sxLong);
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

    float lat = sx2rad(latitude);
    float lon = sx2rad(longitude);

    if ((lat != location.latitude) || (lon != location.longitude)) {
        location.latitude = sx2rad(latitude);
        location.longitude = sx2rad(longitude);

        ref_lst = getLST(now(), location);
        synced = false;

        // TODO: Store the location in EEPROM
        //for (int i = 0; i < 8; i++) {
            //EEPROM.write(i, cmd[i + 1]);
        //}
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
    Serial.write(0);
    Serial.write(0);
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

void cmdDebug1(char *cmd)
{
    // return axis positions
    uint32_t ha_pos, dec_pos;
    nexstar.getPosition(DEV_RA, &ha_pos);
    nexstar.getPosition(DEV_DEC, &dec_pos);

    Serial.print(pnex2rad(ha_pos), 6);
    Serial.print(',');
    Serial.print(pnex2rad(dec_pos), 6);
    Serial.print('#');
}

void cmdDebug2(char *cmd)
{
    Serial.print(getCurrentLST(), 6);
    Serial.print('#');
}

void cmdDebug3(char *cmd)
{
    // return longitude
    Serial.print(location.longitude, 6);
    Serial.print('#');
}

void setup()
{
    // Map serial commands to functions
    sCmd.addCommand('E', 1, cmdGetEqCoords);
    sCmd.addCommand('e', 1, cmdGetEqCoords);
    sCmd.addCommand('Z', 1, cmdGetAzCoords);
    sCmd.addCommand('z', 1, cmdGetAzCoords);

    sCmd.addCommand('S', 10, cmdSyncEqCoords);
    sCmd.addCommand('s', 18, cmdSyncEqCoords);

    sCmd.addCommand('R', 10, cmdGotoEqCoords);
    sCmd.addCommand('r', 18, cmdGotoEqCoords);

    sCmd.addCommand('B', 10, cmdGotoAzCoords);
    sCmd.addCommand('b', 18, cmdGotoAzCoords);

    sCmd.addCommand('L', 1, cmdGotoInProgress);
    sCmd.addCommand('M', 1, cmdCancelGoto);

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

    // custom commands
    sCmd.addCommand('d', 1, cmdDebug1);
    sCmd.addCommand('D', 1, cmdDebug2);
    sCmd.addCommand('?', 1, cmdDebug3);

    //sCmd.addCommand('J', 1, cmdAlignmentComplete);
    sCmd.addCommand('x', 1, cmdHibernate);
    sCmd.addCommand('y', 1, cmdWakeup);

    pinMode(AUX_SELECT, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    //pinMode(BUTTON, INPUT_PULLUP);

#ifdef DEBUG
    debug.begin(9600);
#endif
    Serial.begin(BAUDRATE);
    nexstar.init();

    // TODO: Read the location from EEPROM
}

// Check if both motors have reached their target position
bool slewDone()
{
    bool ra_done, dec_done;

    nexstar.slewDone(DEV_RA, &ra_done);
    nexstar.slewDone(DEV_DEC, &dec_done);
    return ra_done && dec_done;
}

// Check if both motors are less than 10 degrees away from the target
bool isClose()
{
    EqCoords current;
    getEqCoords(&current);

    bool ra_diff = fabs(current.ra - target.ra);
    bool dec_diff = fabs(current.dec - target.dec);

    return fmax(ra_diff, dec_diff) < M_PI/18;
}

// Check if a meridian flip is required
bool checkMeridian()
{
    uint32_t int_ra_pos;
    double curr, ha, tgt;
    nexstar.getPosition(DEV_RA, &int_ra_pos);
    curr = normalizeAngle(pnex2rad(int_ra_pos) - M_PI/2);

    ha = normalizeAngle(getCurrentLST() - target.ra + MERIDIAN_MARGIN);
    tgt = normalizeAngle(ha - M_PI/2*sign(ha));

    // A meridian flip is required if both angles have different sign
    return curr*tgt < 0;
}

// Update the scope status with a simple state machine
void updateFSM()
{
    static uint32_t t_timer;
    static int ra_homed = 0, dec_homed = 0;
    static bool dec_homing_dir = 0;

    if (event == EV_ABORT) {
        stopMotors();
        state = ST_IDLE;
        event = EV_NONE;
        DEBUG_PRINT("ST_IDLE");
        return;
    }

    switch(state) {
        case ST_IDLE:
            if (event == EV_GOTO) {
                t_timer = millis();
                if (checkMeridian()) {
                    // go to the pole before flipping
                    nexstar.gotoPosition(DEV_RA, 0, rad2pnex(M_PI/2));
                    nexstar.gotoPosition(DEV_DEC, 0, 0);
                    state = ST_MERIDIAN_FLIP;
                    DEBUG_PRINT("ST_MERIDIAN_FLIP");
                    break;
                }
                bool slow = isClose();
                gotoEqCoords(target, slow);
                state = slow ? ST_SLEWING_SLOW : ST_SLEWING_FAST;
                DEBUG_PRINT(slow ? "ST_SLEWING_SLOW" : "ST_SLEWING_FAST");
            } else if (event == EV_HOME) {
                dec_homing_dir = !decHomeSensor();
                nexstar.move(DEV_DEC, dec_homing_dir, 9);
                state = ST_HOMING_DEC;
                DEBUG_PRINT("ST_HOMING_DEC");
            }
            break;

        case ST_MERIDIAN_FLIP:
            if (millis() - t_timer > 500) {
                // Every 0.5 seconds, check if we are close to the target
                t_timer = millis();

                if (slewDone()) {
                    bool slow = isClose();
                    gotoEqCoords(target, slow);
                    state = slow ? ST_SLEWING_SLOW : ST_SLEWING_FAST;
                    DEBUG_PRINT(slow ? "ST_SLEWING_SLOW" : "ST_SLEWING_FAST");
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
                    DEBUG_PRINT("ST_SLEWING_SLOW");
                }
            }
            break;

        case ST_SLEWING_SLOW:
            if (millis() - t_timer > 250) {
                // Every 0.25 seconds, check if slew is done
                t_timer = millis();

                if (slewDone()) {
                    state = ST_IDLE;
                    DEBUG_PRINT("ST_IDLE");
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
                DEBUG_PRINT("ST_HOMING_FAST1");
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
                DEBUG_PRINT("ST_HOMING_FAST2");
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
                DEBUG_PRINT("ST_HOMING_SLOW1");
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
                DEBUG_PRINT("ST_HOMING_SLOW2");
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
                //stopMotors();
                EqHACoords eq = {HOME_HA, HOME_DEC};
                setHACoords(eq);
                synced = true;
                state = ST_IDLE;
                DEBUG_PRINT("ST_IDLE");
            }
            break;
    };

    event = EV_NONE;
}

void loop()
{
    sCmd.readSerial();
    updateFSM();
    nexstar.run();
}
