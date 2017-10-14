/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the NexStarAdapter project:
        https://github.com/juanmb/NexStarAdapter

*******************************************************************************/

#ifndef _NexStartSerial_h_
#define _NexStartSerial_h_

#include <SoftwareSerial.h>
#include <inttypes.h>

#define AUX_BAUDRATE 19200
#define MAX_PAYLOAD_SIZE 10
#define RESP_TIMEOUT 200   // timeout in milliseconds

#define ERR_INVALID -1
#define ERR_TIMEOUT -2
#define ERR_BAD_SIZE -3
#define ERR_BAD_HDR -4
#define ERR_CRC -5


typedef enum deviceId {
    main_board = 1,
    hand_controller = 4,
    ra_motor = 16,
    dec_motor = 17,
    gps_dev = 176,
    rtc_dev = 178,
} deviceId;


typedef struct nexstar_header {
    uint8_t preamble;
    uint8_t length;
    uint8_t source;
    uint8_t dest;
    uint8_t id;
} nexstar_header;

struct nexstar_message {
    nexstar_header header;
    uint8_t payload[MAX_PAYLOAD_SIZE];
    uint8_t crc;
};


class NexStartSerial {
public:
    NexStartSerial(int rx, int tx, int select);
    int sendMessage(uint8_t dest, uint8_t id, uint8_t size, char* data,
                    nexstar_message *response);
    void begin();

protected:
    int newMessage(nexstar_message *msg, uint8_t dest, uint8_t id,
                   uint8_t size, char* data);

private:
    SoftwareSerial *serial;
    int select_pin;
};

#endif
