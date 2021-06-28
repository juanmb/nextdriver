# NextDriver

An Arduino-based controller for Celestron CG5-series equatorial mounts.

## Disclaimer

:warning: **This is a work in progress!** :warning:

No stable version was released yet, so you can expect some critical bugs to occur.
It shouldn't be used in unattended telescopes!

Currently NextDriver was tested only with [KStars/Ekos](https://www.indilib.org/about/ekos.html)
using the [Celestron NexStar](https://www.indilib.org/telescopes/celestron/celestron-nexstar.html) INDI driver to control a
[Celestron CG-5 GT Advanced Series](https://s3.amazonaws.com/celestron-site-support-files/support_files/1196122269_cg5mount9151791.pdf) mount.

## Description

NextDriver is a USB controller for the Celestron NexStar AUX protocol
implemented in Arduino. The Arduino replaces the hand controller, receiving
the commands from the PC (HC procotol) and controlling the mount (AUX protocol).

With **NextDriver** you won't need a hand control to control your Celestron mount anymore.
It allows a fully remote operation of the mount by removing the tedious manual startup proceeding.

![](images/block_diagram.png)

Documentation on the NexStar HC and Aux protocols:
- [NexStar HC Protocol](http://www.nexstarsite.com/download/manuals/NexStarCommunicationProtocolV1.2.zip)
- [NexStar AUX Protocol](http://www.paquettefamily.ca/nexstar/NexStar_AUX_Commands_10.pdf)

### LEDs
- Dual color red-green status LED:
    - Red: time not set
    - Yellow: time set. no synced/aligned
    - Green: synced/aligned
- Green LED: slewing
- Red LED: Error (limits, communication)
- Yellow: not used yet

## Hardware

![](images/nextdriver_schematic.png)

Required parts:
- Arduino UNO
- [Prototype Expansion Board for Arduino UNO](https://www.aliexpress.com/item/33045177683.html)
- Infrared sensor modules. You can build the circuit shown in the schematics or
  you can buy [already-built modules](https://www.aliexpress.com/item/32886394063.html)
- [Limit switch](https://www.aliexpress.com/item/4001033375208.html) for RA axis
- [Tilt switch](https://www.aliexpress.com/item/33040347015.html) for declination axis
- [6P6C female connector](https://www.aliexpress.com/item/33041450076.html)
- [RJ12 male to male cable](https://www.aliexpress.com/item/1005001412508363.html)
- 1N4148 (or similar) diode
- Red, green, yellow LEDs
- Dual red-green LED
- Resistors (shown in the schematics)

## Features

- **Home (index) sensors** to allow remote startup without requiring manual alignment
- **Limit sensors** to avoid collisions beween the telescope and the mount
- Automatic **meridian flip** when slewing to a target
- Home and abort buttons
- Status LEDs

Supported HC commands:
  - GetEqCoords
  - GetAzCoords
  - GetPierSide
  - SyncEqCoords
  - GotoEqCoords
  - GotoAzCoords
  - GotoInProgress
  - CancelGoto
  - IsAligned
  - SetTrackingMode
  - GetTrackingMode
  - SetLocation
  - GetLocation
  - SetTime
  - GetTime
  - GetVersion
  - GetVariant
  - GetModel
  - Hibernate (not working yet)
  - Wakeup
  - Echo
  - PassThrough

## Building

The code is built using [PlatformIO](https://platformio.org/), so you'll need to install it
in order to build the Arduino firmware and upload it to the board.

Test the code in your computer (it does not require an Arduino board):

    pio platform install native
    pio test -e native

Build the firmware for Arduino UNO (default target):

    pio run

Program the board:

    pio run -t upload


## Current limitations

Currently, only CG-5 mounts in the North hemisphere are supported.
