/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the NextDriver project:
        https://github.com/juanmb/NextDriver

    This code is based on Andre Paquette's documentation about
    the NexStar AUX protocol:
    http://www.paquettefamily.ca/nexstar/NexStar_AUX_Commands_10.pdf

*******************************************************************/

#ifndef _nexstar_aux_h_
#define _nexstar_aux_h_

#include "nexstar_base.h"

#define AUX_BAUDRATE 19200

// Convert between nexstar angle format and radians
float nex2rad(uint16_t angle);
float pnex2rad(uint32_t angle);
uint16_t rad2nex(float rad);
uint32_t rad2pnex(float rad);


class NexStarAux : public NexStarBase {
public:
    //NexStarAux(int rx, int tx, int select, int busy);
    NexStarAux(Stream *serial, int select, int busy);
    void init();

    int setPosition(uint8_t dest, uint32_t pos);
    int getPosition(uint8_t dest, uint32_t *pos);

    int gotoPosition(uint8_t dest, bool slow, uint32_t pos);
    int move(uint8_t dest, bool dir, uint8_t rate);
    int slewDone(uint8_t dest, bool *done);

    int setGuiderate(uint8_t dest, bool dir, bool custom_rate, uint32_t rate);

    int getVersion(uint8_t dest, char *major, char *minor);

    int setApproach(uint8_t dest, bool dir);
    int getApproach(uint8_t dest, bool *dir);

    void run() {};

    //int setAutoGuiderate(uint8_t dest, uint8_t rate);
    //int getAutoGuiderate(uint8_t dest, uint8_t *rate);
    //int setBacklash(uint8_t dest, bool dir, uint8_t steps);
    //int getBacklash(uint8_t dest, bool dir, uint8_t *steps);

protected:
    int sendCommand(uint8_t dest, uint8_t id, uint8_t size, char* data,
            NexStarMessage *resp);

private:
    int newMessage(NexStarMessage *msg, uint8_t dest, uint8_t id,
            uint8_t size, char* data);

    Stream *serial;
    int select_pin;
    int busy_pin;
};

#endif
