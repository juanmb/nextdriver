/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the NexStarAdapter project:
        https://github.com/juanmb/NexStarAdapter

*******************************************************************/

#include <Arduino.h>
#include "nexstar_aux.h"


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


uint8_t calcCRC(NexStarMessage *msg)
{
    int result = 0;
    char *data = (char*)msg;

    for (int i = 1; i < msg->header.length + 2; i++) {
        result += data[i];
    }
    return -result & 0xff;
}


NexStarAux::NexStarAux(int rx, int tx, int select)
{
    serial = new SoftwareSerial(rx, tx);
    select_pin = select;
}

// Initialize pins and setup serial port
void NexStarAux::begin()
{
    pinMode(select_pin, INPUT);
    serial->begin(AUX_BAUDRATE);
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
    msg->header.source = hand_controller;
    msg->header.dest = dest;
    msg->header.id = id;
    memcpy(msg->payload, data, size);
    msg->crc = calcCRC(msg);
    return 0;
}

// Send a message and receive its response
int NexStarAux::sendMessage(uint8_t dest, uint8_t id, uint8_t size,
        char* data, NexStarMessage *resp)
{
    NexStarMessage msg;
    char *bytes = (char*)(&msg);

    int ret = newMessage(&msg, dest, id, size, data);
    if (ret != 0) {
        return ret;
    }

    digitalWrite(select_pin, LOW);
    pinMode(select_pin, OUTPUT);
    for (int i = 0; i < size + 5; i++) {
        serial->write(bytes[i]);
    }
    serial->write(msg.crc);
    pinMode(select_pin, INPUT);

    serial->flush();

    long int t0 = millis();

    delay(1);
    // wait while select pin is low
    while(digitalRead(select_pin) == HIGH) {
        if (millis() - t0 > RESP_TIMEOUT) {
            return ERR_TIMEOUT;
        }
        delay(1);
    }
    // wait while select pin is high
    while(digitalRead(select_pin) == LOW) {
        delay(1);
        if (millis() - t0 > RESP_TIMEOUT) {
            return ERR_TIMEOUT;
        }
    }

    bytes = (char*)(resp);
    unsigned int pos = 0;
    while (serial->available()) {
        bytes[pos++] = serial->read();
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
    return sendMessage(dest, 0x04, 3, payload, &resp);
}

int NexStarAux::getPosition(uint8_t dest, uint32_t *pos)
{
    NexStarMessage resp;
    int ret = sendMessage(dest, 0x01, 0, NULL, &resp);
    *pos = uint32From24bits(resp.payload);
    return ret;
}

int NexStarAux::gotoPosition(uint8_t dest, bool slow, uint32_t pos)
{
    NexStarMessage resp;
    char payload[3];
    uint32To24bits(pos, payload);

    if (slow) {
        return sendMessage(dest, 0x17, 3, payload, &resp);
    }
    return sendMessage(dest, 0x02, 3, payload, &resp);
}

int NexStarAux::move(uint8_t dest, bool dir, uint8_t rate)
{
    NexStarMessage resp;
    uint8_t payload[1] = { rate };

    if (dir) {
        return sendMessage(dest, 0x24, 1, (char *)payload, &resp);
    }
    return sendMessage(dest, 0x025, 1, (char *)payload, &resp);
}

int NexStarAux::slewDone(uint8_t dest, bool *done)
{
    NexStarMessage resp;
    int ret = sendMessage(dest, 0x13, 0, NULL, &resp);
    *done = (bool)resp.payload[0];
    return ret;
}

int NexStarAux::setGuiderate(uint8_t dest, bool dir, bool custom_rate, uint32_t rate)
{
    NexStarMessage resp;

    char payload[3];
    uint32To24bits(rate << 16, payload);

    char msgId = dir ? 0x06 : 0x07;
    char msgSize = custom_rate ? 3 : 2;
    return sendMessage(dest, msgId, msgSize, payload, &resp);
}

int NexStarAux::setApproach(uint8_t dest, bool dir)
{
    NexStarMessage resp;
    char payload[1] = { dir };
    return sendMessage(dest, 0xfd, 1, payload, &resp);
}

int NexStarAux::getApproach(uint8_t dest, bool *dir)
{
    NexStarMessage resp;
    int ret = sendMessage(dest, 0xfc, 0, NULL, &resp);
    *dir = (bool)resp.payload[0];
    return ret;
}
