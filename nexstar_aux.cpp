/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the NexStarAdapter project:
        https://github.com/juanmb/NexStarAdapter

    This code is based on Andre Paquette's documentation about
    the NexStar AUX protocol:
    http://www.paquettefamily.ca/nexstar/NexStar_AUX_Commands_10.pdf

*******************************************************************/

#include <Arduino.h>
#include "nexstar_aux.h"

#define AUX_BAUDRATE 19200
#define RESP_TIMEOUT 800   // timeout in milliseconds
#ifdef __AVR__
#endif

#ifdef __AVR__
#include <SoftwareSerial.h>

#define serialBegin(x) (serial->begin(x))
#define serialWrite(x) (serial->write(x))
#define serialFlush() (serial->flush())
#define serialAvailable() (serial->available())
#define serialRead() (serial->read())

SoftwareSerial *serial;

#else
// In Arduino DUE, we use Serial1
#define serialBegin(x) (Serial1.begin(x))
#define serialWrite(x) (Serial1.write(x))
#define serialFlush(x) (Serial1.flush())
#define serialAvailable() (Serial1.available())
#define serialRead() (Serial1.read())
#endif

uint8_t calcCRC(NexStarMessage *msg)
{
    int result = 0;
    char *data = (char*)msg;

    for (int i = 1; i < msg->header.length + 2; i++) {
        result += data[i];
    }
    return -result & 0xff;
}


NexStarAux::NexStarAux(int rx, int tx, int select, int busy)
{
#ifdef __AVR__
    serial = new SoftwareSerial(rx, tx);
#endif
    select_pin = select;
    busy_pin = busy;
}

// Initialize pins and setup serial port
void NexStarAux::init()
{
    pinMode(select_pin, OUTPUT);
    pinMode(busy_pin, INPUT);
    serialBegin(AUX_BAUDRATE);
}

// Fill NexStarMessage struct
// data: payload data
// size: size of payload data
int NexStarAux::newMessage(NexStarMessage *msg, uint8_t dest, uint8_t id,
                               uint8_t size, char* data)
{
    if (size > MAX_PAYLOAD_SIZE) {
        return ERR_INVALID;
    }
    msg->header.preamble = 0x3b;
    msg->header.length = size + 3;
    msg->header.source = DEV_HC;
    msg->header.dest = dest;
    msg->header.id = id;
    memcpy(msg->payload, data, size);
    msg->crc = calcCRC(msg);
    return 0;
}

// Send a message and receive its response
int NexStarAux::sendCommand(uint8_t dest, uint8_t id, uint8_t size,
        char* data, NexStarMessage *resp)
{
    NexStarMessage msg;
    char *bytes = (char*)(&msg);

    int ret = newMessage(&msg, dest, id, size, data);
    if (ret != 0) {
        return ret;
    }

    digitalWrite(select_pin, LOW);
    for (int i = 0; i < size + 5; i++) {
        serialWrite(bytes[i]);
    }
    serialWrite(msg.crc);
    serialFlush();
    digitalWrite(select_pin, HIGH);

    long int t0 = millis();

    delay(1);
    // wait while select pin is high
    while(digitalRead(busy_pin) == HIGH) {
        if (millis() - t0 > RESP_TIMEOUT) {
            return ERR_TIMEOUT;
        }
        delay(1);
    }
    // wait while select pin is low
    while(digitalRead(busy_pin) == LOW) {
        delay(1);
        if (millis() - t0 > RESP_TIMEOUT) {
            return ERR_TIMEOUT;
        }
    }

    bytes = (char*)(resp);
    unsigned int pos = 0;
    while (serialAvailable()) {
        bytes[pos++] = serialRead();
        if (pos >= sizeof(NexStarMessage)) {
            return ERR_BAD_SIZE;
        }
    }
    if (pos <= sizeof(NexStarHeader) + 1) {
        return ERR_BAD_SIZE;
    }
    resp->crc = bytes[resp->header.length + 2];
    if (calcCRC(resp) != resp->crc) {
        return ERR_CRC;
    }
    return 0;
}

int NexStarAux::setPosition(uint8_t dest, uint32_t pos)
{
    NexStarMessage resp;
    char payload[3];
    uint32To24bits(pos, payload);
    return sendCommand(dest, MC_SET_POSITION, 3, payload, &resp);
}

int NexStarAux::getPosition(uint8_t dest, uint32_t *pos)
{
    NexStarMessage resp;
    int ret = sendCommand(dest, MC_GET_POSITION, 0, NULL, &resp);
    *pos = uint32From24bits(resp.payload);
    return ret;
}

int NexStarAux::gotoPosition(uint8_t dest, bool slow, uint32_t pos)
{
    NexStarMessage resp;
    char payload[3];
    uint32To24bits(pos, payload);

    char cmdId = slow ? MC_GOTO_SLOW : MC_GOTO_FAST;
    return sendCommand(dest, cmdId, 3, payload, &resp);
}

int NexStarAux::move(uint8_t dest, bool dir, uint8_t rate)
{
    NexStarMessage resp;
    uint8_t payload[1] = { rate };

    char cmdId = dir ? MC_MOVE_POS : MC_MOVE_NEG;
    return sendCommand(dest, cmdId, 1, (char *)payload, &resp);
}

int NexStarAux::slewDone(uint8_t dest, bool *done)
{
    NexStarMessage resp;
    int ret = sendCommand(dest, MC_SLEW_DONE, 0, NULL, &resp);
    *done = (bool)resp.payload[0];
    return ret;
}

int NexStarAux::setGuiderate(uint8_t dest, bool dir, bool custom_rate, uint32_t rate)
{
    NexStarMessage resp;

    char payload[3];
    uint32To24bits(rate << 16, payload);

    char cmdId = dir ? MC_SET_POS_GUIDERATE : MC_SET_NEG_GUIDERATE;
    char msgSize = custom_rate ? 3 : 2;
    return sendCommand(dest, cmdId, msgSize, payload, &resp);
}

int NexStarAux::setApproach(uint8_t dest, bool dir)
{
    NexStarMessage resp;
    char payload[1] = { dir };
    return sendCommand(dest, MC_SET_APPROACH, 1, payload, &resp);
}

int NexStarAux::getApproach(uint8_t dest, bool *dir)
{
    NexStarMessage resp;
    int ret = sendCommand(dest, MC_GET_APPROACH, 0, NULL, &resp);
    *dir = (bool)resp.payload[0];
    return ret;
}

int NexStarAux::getVersion(uint8_t dest, char *major, char *minor)
{
    NexStarMessage resp;
    int ret = sendCommand(dest, MC_GET_VER, 0, NULL, &resp);
    *major = resp.payload[0];
    *minor = resp.payload[1];
    return ret;
}
