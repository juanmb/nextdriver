#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "nexstar.h"


#define VERSION_MAJOR 3
#define VERSION_MINOR 1
#define MOUNT_MODEL 10  // GT
#define BAUDRATE 9600

#define AUX_RX_PIN 10
#define AUX_TX_PIN 11
#define AUX_ENABLE_PIN 12


SerialCommand sCmd;
SoftwareSerial auxSerial(AUX_RX_PIN, AUX_TX_PIN);


char calcCRC(char *packet, int length)
{
    //TODO
    return '\0';
}

void cmdGetEqCoords(char *cmd)
{
    //TODO
    Serial.write("34AB,12CE#");
}

void cmdGetAzCoords(char *cmd)
{
    //TODO
    Serial.write("12AB,4000#");
}

void cmdGotoEqCoords(char *cmd)
{
    //TODO
    Serial.write('#');
}

void cmdGotoAzCoords(char *cmd)
{
    //TODO
    Serial.write('#');
}

void cmdGotoInProgress(char *cmd)
{
    //TODO
    Serial.write("0#");
}

void cmdCancelGoto(char *cmd)
{
    //TODO
    Serial.write('#');
}

void cmdSyncEqCoords(char *cmd)
{
    //TODO
    Serial.write('#');
}

void cmdGetLocation(char *cmd)
{
    // Read the location from EEPROM
    for (int i=0; i<8; i++) {
        Serial.write(EEPROM.read(i));
    }
    Serial.write('#');
}

void cmdSetLocation(char *cmd)
{
    // Store the location in EEPROM
    for (int i=0; i<8; i++) {
        EEPROM.write(i, cmd[i+1]);
    }
    Serial.write('#');
}

void cmdGetTime(char *cmd)
{
    //TODO
    Serial.write("\0\0\0\0\0\0\0\0s#");
}

void cmdSetTime(char *cmd)
{
    //TODO
    Serial.write('#');
}

void cmdPassThrough(char *cmd)
{
    char packet[8];
    char length = cmd[1];

    packet[0] = 0x3b;   // start byte
    packet[1] = length; // packet length
    packet[2] = 0x04;   // source device (hand controller)
    memcpy(packet + 3, cmd + 2, length);

    packet[length + 2] = calcCRC(packet, length);

    digitalWrite(AUX_ENABLE_PIN, HIGH);
    for (int i=0; i<length+3; i++) {
        auxSerial.write(packet[i]);
    }
    digitalWrite(AUX_ENABLE_PIN, LOW);

    //TODO
    Serial.write('#');
}

void cmdGetVersion(char *cmd)
{
    Serial.write(VERSION_MAJOR);
    Serial.write(VERSION_MINOR);
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

void setup()
{
    // Map serial commands to functions
    sCmd.addCommand('E', 1, cmdGetEqCoords);
    sCmd.addCommand('Z', 1, cmdGetAzCoords);

    sCmd.addCommand('R', 10, cmdGotoEqCoords);
    sCmd.addCommand('B', 10, cmdGotoAzCoords);
    sCmd.addCommand('L', 1, cmdGotoInProgress);
    sCmd.addCommand('M', 1, cmdCancelGoto);

    sCmd.addCommand('s', 10, cmdSyncEqCoords);

    sCmd.addCommand('w', 1, cmdGetLocation);
    sCmd.addCommand('W', 9, cmdSetLocation);
    sCmd.addCommand('h', 1, cmdGetTime);
    sCmd.addCommand('H', 9, cmdSetTime);

    sCmd.addCommand('P', 8, cmdPassThrough);
    sCmd.addCommand('V', 1, cmdGetVersion);
    sCmd.addCommand('m', 1, cmdGetModel);
    sCmd.addCommand('K', 2, cmdEcho);

    //sCmd.addCommand('J', 1, cmdAlignmentComplete);
    //sCmd.addCommand('x', 1, cmdHibernate);
    //sCmd.addCommand('y', 1, cmdWakeup);

    pinMode(AUX_ENABLE_PIN, OUTPUT);

    Serial.begin(BAUDRATE);
    auxSerial.begin(19200);
}

void loop()
{
    sCmd.readSerial();
}
