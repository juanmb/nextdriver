/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the NexStarAdapter project:
        https://github.com/juanmb/NexStarAdapter

********************************************************************/

//#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "serial_command.h"
#include "nexstar_aux.h"
#include "AstroLib.h"


#define VERSION_MAJOR 4
#define VERSION_MINOR 21
#define MOUNT_MODEL 10  // GT
#define CONTROLLER_VARIANT 0x11  // NexStar
#define BAUDRATE 9600

#define AUX_SELECT 5
#define AUX_RX 6
#define AUX_TX 7
#define RA_POT_PIN A6
#define DEC_POT_PIN A7

enum ScopeState {
    ST_IDLE,
    ST_GOING_FAST,
    ST_GOING_SLOW,
};

enum ScopeEvent {
    EV_NONE,
    EV_GOTO,
    EV_ABORT,
};

ScopeState state = ST_IDLE;
ScopeEvent event = EV_NONE;

double jdate = 0.0;    // Julian date refered to J2000
Location location = {0};
uint8_t tracking_mode = 0;
uint32_t t_timeset = 0;
uint32_t t_sync = 0;
EqCoords target = {0};

SerialCommand sCmd;
NexStarAux scope(AUX_RX, AUX_TX, AUX_SELECT);


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

// Obtain the difference in RA from the motor position due to Earth rotation
static double getRADiff() {
    return 2*M_PI/8.616e7*(millis() - t_sync);
}

void getEqCoords(EqCoords *eq)
{
    uint32_t ra_pos, dec_pos;
    scope.getPosition(DEV_RA, &ra_pos);
    scope.getPosition(DEV_DEC, &dec_pos);

    eq->ra = 2*M_PI - pnex2rad(ra_pos) + getRADiff();
    eq->dec = pnex2rad(dec_pos);
}

void setEqCoords(EqCoords eq)
{
    scope.setPosition(DEV_RA, rad2pnex(2*M_PI - eq.ra));
    scope.setPosition(DEV_DEC, rad2pnex(eq.dec));
    t_sync = millis();
}

void stopMotors()
{
    scope.move(DEV_RA, 0, 0);
    scope.move(DEV_DEC, 0, 0);
}

void gotoEqCoords(EqCoords eq, bool slow)
{
    uint32_t ra_pos = rad2pnex(2*M_PI - eq.ra + getRADiff());
    uint32_t dec_pos = rad2pnex(eq.dec);

    scope.gotoPosition(DEV_RA, slow, ra_pos);
    scope.gotoPosition(DEV_DEC, slow, dec_pos);
}

void getAzCoords(HorizCoords *hor)
{
    double jd = jdate + (double)(millis() - t_timeset)/(1000.0*60*60*24);
    double lst = getLST(jd, location);

    EqCoords eq;
    getEqCoords(&eq);

    eqToHoriz(lst, location, eq, hor);
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
    if (cmd[0] == 'R') {
        uint16_t ra, dec;
        sscanf(cmd + 1, "%4x,%4x", &ra, &dec);
        target.ra = nex2rad(ra);
        target.dec = nex2rad(dec);
    } else {
        uint32_t ra, dec;
        sscanf(cmd + 1, "%8lx,%8lx", &ra, &dec);
        target.ra = pnex2rad(ra);
        target.dec = pnex2rad(dec);
    }

    event = EV_GOTO;
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

    double jd = jdate + (double)(millis() - t_timeset)/(1000.0*60*60*24);
    double lst = getLST(jd, location);
    horizToEq(lst, location, hor, &target);

    event = EV_GOTO;
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
    //TODO: store tracking mode in EEPROM and start tracking at start
    tracking_mode = cmd[1];
    scope.setGuiderate(DEV_RA, GUIDERATE_POS, true, 0);    // stop RA motor
    scope.setGuiderate(DEV_DEC, GUIDERATE_POS, true, 0);   // stop DEC motor

    switch(tracking_mode) {
        case TRACKING_EQ_NORTH:
            scope.setGuiderate(DEV_RA, GUIDERATE_POS, 0, GUIDERATE_SIDEREAL);
            break;
        case TRACKING_EQ_SOUTH:
            scope.setGuiderate(DEV_RA, GUIDERATE_NEG, 0, GUIDERATE_SIDEREAL);
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
    location.latitude = sx2rad(latitude);

    SxAngle longitude = {
        (uint8_t)cmd[5], (uint8_t)cmd[6],
        (uint8_t)cmd[7], (uint8_t)cmd[8]
    };
    location.longitude = sx2rad(longitude);

    // Store the location in EEPROM
    //for (int i = 0; i < 8; i++) {
        //EEPROM.write(i, cmd[i + 1]);
    //}
    Serial.write('#');
}

void cmdGetTime(char *cmd)
{
    double ms = millis() - t_timeset;
    double jd = jdate + ms/(1000.0*60*60*24);

    Date date;
    dateFromJ2000(jd, &date);

    Serial.write(date.hour);
    Serial.write(date.min);
    Serial.write(date.sec);
    Serial.write(date.month);
    Serial.write(date.day);
    Serial.write(date.year % 2000);
    Serial.write(date.offset);
    Serial.write(date.dst);
    Serial.write("#");
}

void cmdSetTime(char *cmd)
{
    Date date;
    date.hour = (int)cmd[1];
    date.min = (int)cmd[2];
    date.sec = (int)cmd[3];
    date.month = (int)cmd[4];
    date.day = (int)cmd[5];
    date.year = 2000 + (int)cmd[6];
    date.offset = (int)cmd[7];
    date.dst = (int)cmd[8];

    jdate = getJ2000Date(date);
    t_timeset = millis();

    Serial.write('#');
}

void cmdPassThrough(char *cmd)
{
    NexStarMessage resp;
    uint8_t size = cmd[1] - 1;

    // pass the command to the mount
    int ret = scope.sendCommand(cmd[2], cmd[3], size, &cmd[4], &resp);
    if (ret != 0) {
        // TODO: return a response with size = normal_return_size + 1
        Serial.print(ret);
        Serial.write('#');
        return;
    }

    for (int i = 0; i < resp.header.length - 3; i++) {
        Serial.write(resp.payload[i]);
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

// Read the absolute position of the two axes using two potentiometers.
// The returned position is the raw 10-bits value reading from the ADC.
void cmdGetAbsPosition(char *cmd)
{
    int axis1_pos = analogRead(RA_POT_PIN);
    int axis2_pos = analogRead(DEC_POT_PIN);

    char tmp[11];
    sprintf(tmp, "%04X,%04X#", axis1_pos, axis2_pos);
    Serial.write(tmp);
}

void cmdDebugLST(char *cmd)
{
    double jd = jdate + (double)(millis() - t_timeset)/(1000.0*60*60*24);
    double lst = getLST(jd, location);

    Serial.print(lst, 6);
    Serial.print('#');
}

void cmdDebugJulianTime(char *cmd)
{
    Serial.print(jdate, 6);
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
    sCmd.addCommand('U', 1, cmdGetAbsPosition);
    sCmd.addCommand('d', 1, cmdDebugLST);
    sCmd.addCommand('j', 1, cmdDebugJulianTime);

    //sCmd.addCommand('J', 1, cmdAlignmentComplete);
    sCmd.addCommand('x', 1, cmdHibernate);
    sCmd.addCommand('y', 1, cmdWakeup);

    pinMode(AUX_SELECT, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(BAUDRATE);
    scope.begin();
}

// Check if both motors have reached their target position
bool slewDone()
{
    bool ra_done, dec_done;

    scope.slewDone(DEV_RA, &ra_done);
    scope.slewDone(DEV_DEC, &dec_done);
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

// Update the scope status with a simple state machine
void updateFSM()
{
    static uint32_t t_timer;

    switch(state) {
        case ST_IDLE:
            if (event == EV_GOTO) {
                t_timer = millis();
                if (isClose()) {
                    gotoEqCoords(target, true);
                    state = ST_GOING_SLOW;
                } else {
                    gotoEqCoords(target, false);
                    state = ST_GOING_FAST;
                }
            }
            break;

        case ST_GOING_FAST:
            if (event == EV_ABORT) {
                stopMotors();
                state = ST_IDLE;
            } else if (millis() - t_timer > 1000) {
                // Every second, check if we are close to the target
                t_timer = millis();

                if (slewDone()) {
                    gotoEqCoords(target, true);
                    state = ST_GOING_SLOW;
                }
            }
            break;

        case ST_GOING_SLOW:
            if (event == EV_ABORT) {
                stopMotors();
                state = ST_IDLE;
            } else if (millis() - t_timer > 1000) {
                // Every second, check if slew is done
                t_timer = millis();

                if (slewDone())
                    state = ST_IDLE;
            }
            break;
    };

    event = EV_NONE;
}

void loop()
{
    sCmd.readSerial();
    updateFSM();
}
