/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the NexStarAdapter project:
        https://github.com/juanmb/NexStarAdapter

*******************************************************************/


#include <string.h>
#include "nexstar_base.h"


// The number 0x12345678 will be converted into {0x12, 0x34, 0x56}
void uint32To24bits(uint32_t in, char *out)
{
    uint32_t tmp = in;
    for (int i=0; i<3; i++) {
        tmp >>= 8;
        out[2-i] = tmp & 0xff;
    }
}


// The char array {0x12, 0x34, 0x56} will be converted into 0x12345600
uint32_t uint32From24bits(char *data)
{
    uint32_t out = 0;

    for (int i=0; i<3; i++) {
        out |= data[i] & 0xff;
        out <<= 8;
    }
    return out;
}


int NexStarBase::sendRawCommand(char *cmd, char *resp, uint8_t *resp_size) {
    uint8_t size = cmd[1] - 1;
    uint8_t dest = cmd[2];
    uint8_t cmdId = cmd[3];
    char *payload = &cmd[4];
    int ret = 0;

    // expected response length
    *resp_size = cmd[7];

    // process command
    switch(cmdId) {
        case MC_GET_VER:
            ret = getVersion(dest, &resp[0], &resp[1]);
            break;
        case MC_SET_POSITION:
            ret = setPosition(dest, (uint32_t)(*payload));
            break;
        case MC_GET_POSITION:
            ret = getPosition(dest, (uint32_t*)resp);
            break;
        case MC_MOVE_POS:
        case MC_MOVE_NEG:
            ret = move(dest, cmdId == MC_MOVE_POS, payload[0]);
            break;
        default:
            // pass the raw command to the scope
            NexStarMessage msg;
            ret = sendCommand(dest, cmdId, size, payload, &msg);
            if (ret == 0)
                memcpy(resp, msg.payload, *resp_size);
    }
    return ret;
}
