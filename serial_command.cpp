/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the NexStarAdapter project:
        https://github.com/juanmb/NexStarAdapter

*******************************************************************************/

#include <Arduino.h>
#include "serial_command.h"


SerialCommand::SerialCommand()
{
    nCommands = 0;
    cmdSize = 0;
    bufPos = 0;
}

int SerialCommand::addCommand(const char firstChar, uint8_t size, cbFunction function)
{
    if (nCommands >= MAX_COMMANDS) {
        return 1;
    }

    commandList[nCommands].firstChar = firstChar;
    commandList[nCommands].size = size;
    commandList[nCommands].function = function;
    nCommands++;
    return 0;
}

void SerialCommand::readSerial()
{
    while (Serial.available()) {
        char c = Serial.read();

        if (bufPos == 0) {
            cmdSize = 0;
            for (int i = 0; i < nCommands; i++) {
                if (commandList[i].firstChar == c) {
                    cmdSize = commandList[i].size;
                    cmdFunction = commandList[i].function;
                    break;
                }
            }
            if (cmdSize == 0) {
                continue;
            }
        }
        buffer[bufPos++] = c;
        if (bufPos >= cmdSize) {
            cmdFunction(buffer);
            bufPos = 0;
        }
    }
}
