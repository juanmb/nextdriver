# This Makefile is used to compile and upload the code to the Arduino via
# Make, instead of using the IDE.
#
# The user should replace ARDUINO_PORT and the path to Arduino.mk for the
# rigth values if this Makefile is going to be used.
#
# Source code can be downloaded from:
# 	https://github.com/sudar/Arduino-Makefile

BOARD_TAG    = atmega328
ARDUINO_LIBS = SoftwareSerial EEPROM
ARDUINO_LIBS += Wire
ARDUINO_PORT = /dev/ttyUSB0

include /usr/share/arduino/Arduino.mk

format:
	astyle --style=stroustrup -p -c -s4 *.cpp *.h *.ino
