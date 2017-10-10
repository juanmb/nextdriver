# NexStarAdapter

## Description

This is an USB adapter for Celestron NexStar AUX protocol implemented in Arduino.
It acts like a hand controller, receiving the commands from the PC (HC
procotol) and communicating with the mount boards (AUX protocol).

Documentation:

 * [NexStar HC Protocol](http://www.nexstarsite.com/download/manuals/NexStarCommunicationProtocolV1.2.zip)
 * [NexStar AUX Protocol](http://www.paquettefamily.ca/nexstar/NexStar_AUX_Commands_10.pdf)

## Makefile

The code can be compiled and uploaded to the Arduino using `make`.
The provided Makefile requires [Arduino-Makefile](https://github.com/sudar/Arduino-Makefile)
to work. In Debian/Ubuntu/Mint, you can install it with

    sudo apt-get install arduino-mk

## Required libraries

You will need the following third-party Arduino libraries before compiling this
project:

 * [SerialCommand](https://github.com/scogswell/ArduinoSerialCommand)
