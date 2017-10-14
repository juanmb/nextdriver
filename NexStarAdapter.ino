#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "serial_command.h"
#include "nexstar_serial.h"


#define VERSION_MAJOR 3
#define VERSION_MINOR 1
#define MOUNT_MODEL 10  // GT
#define BAUDRATE 9600

#define AUX_SELECT 5
#define AUX_RX 6
#define AUX_TX 7


SerialCommand sCmd;
NexStartSerial auxSerial(AUX_RX, AUX_TX, AUX_SELECT);


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
    for (int i = 0; i < 8; i++) {
        Serial.write(EEPROM.read(i));
    }
    Serial.write('#');
}

void cmdSetLocation(char *cmd)
{
    // Store the location in EEPROM
    for (int i = 0; i < 8; i++) {
        EEPROM.write(i, cmd[i + 1]);
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
    nexstar_message resp;
    uint8_t size = cmd[1] - 1;

    // pass the command to the mount
    int ret = auxSerial.sendMessage(cmd[2], cmd[3], size, &cmd[4], &resp);
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

    pinMode(AUX_SELECT, OUTPUT);

    Serial.begin(BAUDRATE);
    auxSerial.begin();
}

void loop()
{
    sCmd.readSerial();
}
