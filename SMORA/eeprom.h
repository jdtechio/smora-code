#ifndef EEPROM_H
#define EEPROM_H

#include <Arduino.h>
#include <Wire.h>

#if defined(ARDUINO_SAMD_ZERO) // Required for Serial on Zero based boards
    #include "defines_atsamd21g18a.h"
#else
    #include "defines_atmega328p.h"
#endif

// Where to store the config data in EEPROM
#define CONFIG_START 0

byte getEEPROMHighAddress(int memAddress);
byte getEEPROMLowAddress(int memAddress);
byte getEEPROMDevAddress(int memAddress);
void writeEEPROM(int memAddress, const byte data);
void writeEEPROM(int memAddress, const byte* buffer, int length);
byte readEEPROM(int memAddress);
void readEEPROM(int memAddress, byte* buffer, int length);

#endif // EEPROM_H