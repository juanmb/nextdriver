/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the NexStarAdapter project:
        https://github.com/juanmb/NexStarAdapter

*******************************************************************/

#include <Arduino.h>
#include "nexstar_stepper.h"

#define VERSION_MAJOR 3
#define VERSION_MINOR 4

// Motor parameters
#define GEAR_RATIO 1
#define STEPS 200
#define MICROSTEPS 16
#define USTEPS_REV (GEAR_RATIO*STEPS*MICROSTEPS) // microsteps per revolution
#define POS_FACTOR (0x100000000/USTEPS_REV)

#define GOTO_SPEED 500
#define ACCEL 500

// CNC Shield pins
#define XSTEP 2
#define YSTEP 3
#define ZSTEP 4
#define XDIR 5
#define YDIR 6
#define ZDIR 7
#define ENABLE 8
#define XSTOP 9
#define YSTOP 10
#define ZSTOP 11
#define SPINDLE_ENABLE 12
#define SPINDLE_DIR 13

// Pin for the photo diode -- Map to ZSTEP
#define PHOTO_PIN ZSTEP

// Modulus operator
#define MOD(a, b) ((((a) % (b)) + (b)) % (b))


NexStarStepper::NexStarStepper(void) :
    azMotor(AccelStepper::DRIVER, XSTEP, XDIR),
    altMotor(AccelStepper::DRIVER, YSTEP, YDIR)
{
}

// Initialize pins and setup serial port
void NexStarStepper::init()
{
    // Enable pin
    pinMode(ENABLE, OUTPUT);
    // Configure outputs
    pinMode(XSTEP, OUTPUT);
    pinMode(XDIR, OUTPUT);
    pinMode(YSTEP, OUTPUT);
    pinMode(YDIR, OUTPUT);
    // Photo diode pin
    pinMode(PHOTO_PIN, INPUT);

    // Configure axes logic and dynamics
    azMotor.setPinsInverted(true, false, true);
    azMotor.setAcceleration(ACCEL);
    azMotor.setEnablePin(ENABLE);

    altMotor.setPinsInverted(false, false, true);
    altMotor.setAcceleration(ACCEL);
    altMotor.setEnablePin(ENABLE);

    digitalWrite(ENABLE, HIGH);
}

int NexStarStepper::setPosition(uint8_t dest, uint32_t pos)
{
    AccelStepper *motor = (dest == DEV_AZ) ? &azMotor : &altMotor;

    motor->setCurrentPosition(pos/POS_FACTOR);
    return 0;
}

int NexStarStepper::getPosition(uint8_t dest, uint32_t *pos)
{
    AccelStepper *motor = (dest == DEV_AZ) ? &azMotor : &altMotor;

    long motorPos = motor->currentPosition();
    *pos = motorPos*POS_FACTOR;
    return 0;
}

// Obtain the relative distance for travelling to the target position
long getDiff(long cur, long tgt)
{
    cur = (cur < USTEPS_REV/2) ? cur : cur - USTEPS_REV;
    tgt = (tgt < USTEPS_REV/2) ? tgt : tgt - USTEPS_REV;
    return tgt - cur;
}

int NexStarStepper::gotoPosition(uint8_t dest, bool slow, uint32_t pos)
{
    AccelStepper *motor = (dest == DEV_AZ) ? &azMotor : &altMotor;

    digitalWrite(ENABLE, LOW);
    motor->setMaxSpeed(GOTO_SPEED);

    long current = motor->currentPosition();
    long target = pos/POS_FACTOR;

    motor->move(getDiff(current, target));
    return 0;
}

int NexStarStepper::move(uint8_t dest, bool dir, uint8_t rate)
{
    AccelStepper *motor = (dest == DEV_AZ) ? &azMotor : &altMotor;
    long pos = dir ? USTEPS_REV/2 : -USTEPS_REV/2;

    if (rate == 0) {
        motor->stop();
    } else {
        digitalWrite(ENABLE, LOW);
        motor->setMaxSpeed(10*rate);
        motor->move(pos);
    }
    return 0;
}

int NexStarStepper::slewDone(uint8_t dest, bool *done)
{
    AccelStepper *motor = (dest == DEV_AZ) ? &azMotor : &altMotor;

    *done = !motor->isRunning();

    //if (*done)
        //digitalWrite(ENABLE, HIGH);
    return 0;
}

int NexStarStepper::setGuiderate(uint8_t dest, bool dir, bool custom_rate, uint32_t rate)
{
    return 0;
}

int NexStarStepper::getVersion(uint8_t dest, char *major, char *minor)
{
    *major = VERSION_MAJOR;
    *minor = VERSION_MINOR;
    return 0;
}
