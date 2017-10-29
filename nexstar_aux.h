/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the NexStarAdapter project:
        https://github.com/juanmb/NexStarAdapter

*******************************************************************************/

#ifndef _NexStarAux_h_
#define _NexStarAux_h_

#include <SoftwareSerial.h>
#include <inttypes.h>

#define AUX_BAUDRATE 19200
#define MAX_PAYLOAD_SIZE 10
#define RESP_TIMEOUT 800   // timeout in milliseconds

#define ERR_INVALID -1
#define ERR_TIMEOUT -2
#define ERR_BAD_SIZE -3
#define ERR_BAD_HDR -4
#define ERR_CRC -5

#define POS_GUIDERATE 1
#define NEG_GUIDERATE 0

#define GUIDERATE_SIDEREAL 0xffff
#define GUIDERATE_SOLAR 0xfffe
#define GUIDERATE_LUNAR 0xfffd

// device IDs
#define DEV_MAIN 0x01
#define DEV_HC 0x04
#define DEV_RA 0x10
#define DEV_AZ 0x10
#define DEV_DEC 0x11
#define DEV_ALT 0x11
#define DEV_GPS 0xb0
#define DEV_RTC 0xb2


typedef enum deviceId {
    main_board = 1,
    hand_controller = 4,
    ra_motor = 16,
    dec_motor = 17,
    gps_dev = 176,
    rtc_dev = 178,
} deviceId;


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


class NexStarAux {
public:
    NexStarAux(int rx, int tx, int select);
    int sendMessage(uint8_t dest, uint8_t id, uint8_t size, char* data,
            NexStarMessage *resp);
    void begin();

    int setPosition(uint8_t dest, uint32_t pos);
    int getPosition(uint8_t dest, uint32_t *pos);

    int gotoPosition(uint8_t dest, bool slow, uint32_t pos);
    int move(uint8_t dest, bool dir, uint8_t rate);
    int slewDone(uint8_t dest, bool *done);

    int setGuiderate(uint8_t dest, bool dir, bool custom_rate, uint32_t rate);
    int setAutoGuiderate(uint8_t dest, uint8_t rate);
    int getAutoGuiderate(uint8_t dest, uint8_t *rate);

    int setApproach(uint8_t dest, bool dir);
    int getApproach(uint8_t dest, bool *dir);

    int setBacklash(uint8_t dest, bool dir, uint8_t steps);
    int getBacklash(uint8_t dest, bool dir, uint8_t *steps);

    int getVersion(uint8_t dest, bool *done);

protected:
    int newMessage(NexStarMessage *msg, uint8_t dest, uint8_t id,
                   uint8_t size, char* data);

private:
    SoftwareSerial *serial;
    int select_pin;
};

#endif
