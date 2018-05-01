/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the NexStarAdapter project:
        https://github.com/juanmb/NexStarAdapter

*******************************************************************/

#ifndef _nexstar_stepper_h_
#define _nexstar_stepper_h_

#include "nexstar_base.h"
#include "AccelStepper.h"


class NexStarStepper : public NexStarBase {
public:
    NexStarStepper();
    void init();

    int setPosition(uint8_t dest, uint32_t pos);
    int getPosition(uint8_t dest, uint32_t *pos);

    int gotoPosition(uint8_t dest, bool slow, uint32_t pos);
    int move(uint8_t dest, bool dir, uint8_t rate);
    int slewDone(uint8_t dest, bool *done);

    int setGuiderate(uint8_t dest, bool dir, bool custom_rate, uint32_t rate);

    int getVersion(uint8_t dest, char *major, char *minor);

    void run() {azMotor.run(); altMotor.run();};

protected:
    int sendCommand(uint8_t dest, uint8_t id, uint8_t size, char* data,
            NexStarMessage *resp) { return 0; };

private:
    AccelStepper azMotor, altMotor;
};

#endif
