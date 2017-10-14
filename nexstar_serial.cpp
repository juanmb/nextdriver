/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the NexStarAdapter project:
        https://github.com/juanmb/NexStarAdapter

*******************************************************************************/

#include <Arduino.h>
#include "nexstar_serial.h"


uint8_t calcCRC(nexstar_message *msg)
{
    int result = 0;
    char *data = (char*)msg;

    for (int i = 1; i < msg->header.length + 2; i++) {
        result += data[i];
    }
    return -result & 0xff;
}


NexStartSerial::NexStartSerial(int rx, int tx, int select)
{
    serial = new SoftwareSerial(rx, tx);
    select_pin = select;
}

// Initialize pins and setup serial port
void NexStartSerial::begin()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(select_pin, INPUT);
    serial->begin(AUX_BAUDRATE);
}

// Fill nexstar_message struct
// data: payload data
// size: size of payload data
int NexStartSerial::newMessage(nexstar_message *msg, uint8_t dest, uint8_t id,
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
int NexStartSerial::sendMessage(uint8_t dest, uint8_t id, uint8_t size,
                                char* data, nexstar_message *resp)
{
    nexstar_message msg;
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
    digitalWrite(LED_BUILTIN, HIGH);

    long int t0 = millis();

    delay(1);
    // wait while select pin is low
    while(digitalRead(select_pin) == HIGH) {
        if (millis() - t0 > RESP_TIMEOUT) {
            digitalWrite(LED_BUILTIN, LOW);
            return ERR_TIMEOUT;
        }
        delay(1);
    }
    // wait while select pin is high
    while(digitalRead(select_pin) == LOW) {
        delay(1);
        if (millis() - t0 > RESP_TIMEOUT) {
            digitalWrite(LED_BUILTIN, LOW);
            return ERR_TIMEOUT;
        }
    }
    digitalWrite(LED_BUILTIN, LOW);

    bytes = (char*)(resp);
    unsigned int pos = 0;
    while (serial->available()) {
        bytes[pos++] = serial->read();
        if (pos >= sizeof(nexstar_message)) {
            return ERR_BAD_SIZE;
        }
    }
    if (pos <= sizeof(nexstar_header) + 1) {
        return ERR_BAD_SIZE;
    }
    resp->crc = bytes[resp->header.length + 2];
    if (calcCRC(resp) != resp->crc) {
        return ERR_CRC;
    }
    return 0;
}
