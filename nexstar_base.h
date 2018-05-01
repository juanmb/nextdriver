/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the NexStarAdapter project:
        https://github.com/juanmb/NexStarAdapter

*******************************************************************/

#ifndef _nexstar_scope_h_
#define _nexstar_scope_h_

#include <inttypes.h>

#define MAX_PAYLOAD_SIZE 10

#define ERR_INVALID -1
#define ERR_TIMEOUT -2
#define ERR_BAD_SIZE -3
#define ERR_BAD_HDR -4
#define ERR_CRC -5

#define TRACKING_NONE       0
#define TRACKING_ALTAZ      1
#define TRACKING_EQ_NORTH   2
#define TRACKING_EQ_SOUTH   3

#define GUIDERATE_NEG 0
#define GUIDERATE_POS 1

#define GUIDERATE_SIDEREAL  0xffff
#define GUIDERATE_SOLAR     0xfffe
#define GUIDERATE_LUNAR     0xfffd

// device IDs
#define DEV_MAIN    0x01
#define DEV_HC      0x04
#define DEV_RA      0x10
#define DEV_AZ      0x10
#define DEV_DEC     0x11
#define DEV_ALT     0x11
#define DEV_GPS     0xb0
#define DEV_RTC     0xb2

// command IDs
#define MC_GET_POSITION         0x01
#define MC_GOTO_FAST            0x02
#define MC_SET_POSITION         0x04
#define MC_SET_POS_GUIDERATE    0x06
#define MC_SET_NEG_GUIDERATE    0x07
#define MC_LEVEL_START          0x0b
#define MC_PEC_RECORD_START     0x0c
#define MC_PEC_PLAYBACK         0x0d
#define MC_SET_POS_BACKLASH     0x10
#define MC_SET_NEG_BACKLASH     0x11
#define MC_LEVEL_DONE           0x12
#define MC_SLEW_DONE            0x13
#define MC_PEC_RECORD_DONE      0x15
#define MC_PEC_RECORD_STOP      0x16
#define MC_GOTO_SLOW            0x17
#define MC_AT_INDEX             0x18
#define MC_SEEK_INDEX           0x19
#define MC_MOVE_POS             0x24
#define MC_MOVE_NEG             0x25
#define MC_MOVE_PULSE           0x26
#define MC_GET_PULSE_STATUS     0x27
#define MC_ENABLE_CORDWRAP      0x38
#define MC_DISABLE_CORDWRAP     0x39
#define MC_SET_CORDWRAP_POS     0x3a
#define MC_POLL_CORDWRAP        0x3b
#define MC_GET_CORDWRAP_POS     0x3c
#define MC_GET_POS_BACKLASH     0x40
#define MC_GET_NEG_BACKLASH     0x41
#define MC_SET_AUTOGUIDE_RATE   0x46
#define MC_GET_AUTOGUIDE_RATE   0x47
#define MC_PROGRAM_ENTER        0x81
#define MC_PROGRAM_INIT         0x82
#define MC_PROGRAM_DATA         0x83
#define MC_PROGRAM_END          0x84
#define MC_GET_APPROACH         0xfc
#define MC_SET_APPROACH         0xfd
#define MC_GET_VER              0xfe


void uint32To24bits(uint32_t in, char *out);
uint32_t uint32From24bits(char *data);


typedef struct NexStarHeader {
    uint8_t preamble;
    uint8_t length;
    uint8_t source;
    uint8_t dest;
    uint8_t id;
} NexStarHeader;


struct NexStarMessage {
    NexStarHeader header;
    char payload[MAX_PAYLOAD_SIZE];
    uint8_t crc;
};


class NexStarBase {
protected:
    virtual int sendCommand(uint8_t dest, uint8_t id, uint8_t size, char* data,
            NexStarMessage *resp);

public:
    virtual int setPosition(uint8_t dest, uint32_t pos);
    virtual int getPosition(uint8_t dest, uint32_t *pos);
    virtual int gotoPosition(uint8_t dest, bool slow, uint32_t pos);
    virtual int move(uint8_t dest, bool dir, uint8_t rate);
    virtual int slewDone(uint8_t dest, bool *done);
    virtual int setGuiderate(uint8_t dest, bool dir, bool custom_rate, uint32_t rate);
    virtual int getVersion(uint8_t dest, char *major, char *minor);

    int sendRawCommand(char *cmd, char *resp, uint8_t *resp_size);

    // Call this method periodically in the main loop
    virtual void run();
};

#endif
