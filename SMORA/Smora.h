#ifndef SMORA_H
#define SMORA_H 

#include <Arduino.h>
#include <Wire.h>

/* Patched I2Cdev.h by adding #define BUFFER_LENGTH 64 because Wire.h for SAMD doesn't have it */
#include <I2Cdev.h>
#include <MPU6050.h>

#include <Adafruit_INA219.h>
#include <ams_as5048b.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#if defined(ARDUINO_SAMD_ZERO) // Required for Serial on Zero based boards
    #include "defines_atsamd21g18a.h"
#else
    #include "defines_atmega328p.h"
#endif

// ID of the settings block
#define CONFIG_VERSION 20161202

// Where to store the config data in EEPROM
#define CONFIG_START 0

class SMORA {
    public:
        short ax, ay, az;
        short gx, gy, gz;

        struct StoreStruct {
            unsigned long version;

            int16_t Speed_P;
            int16_t Speed_I;
            int16_t Speed_D;

            int16_t Position_P;
            int16_t Position_I;
            int16_t Position_D;

            float vars[6];
        } storage = {
          CONFIG_VERSION,
          // The default values
          0, 0, 0,
          0, 0, 0,
          {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
        };

        SMORA();
        //~SMORA();
        void init(void);

        byte getEEPROMHighAddress(int memAddress);
        byte getEEPROMLowAddress(int memAddress);
        byte getEEPROMDevAddress(int memAddress);
        void writeEEPROM(int memAddress, const byte data);
        void writeEEPROM(int memAddress, const byte* buffer, int length);
        byte readEEPROM(int memAddress);
        void readEEPROM(int memAddress, byte* buffer, int length);

        int loadConfig();
        int saveConfig();
        int getConfigSize();

        void setRGB(unsigned char RGB);
        void led_animation(unsigned int speed);
        void test_sensors(void);
        
        float getAngle_Degrees(void);
        float getAngle_Radians(void);
        unsigned int getAngle_RAW(void);

        float getCurrent_mA(void);
        unsigned int getCurrent_RAW(void);

        float getVoltage_V(void);
        unsigned int getVoltage_RAW(void);

        float getTemperatureBlocking(void);
        float getTemperature(void);
        void requestTemperatureConversion(void);
        bool isTemperatureAvailable(void);
        unsigned long timeTemperatureConversion(void);

        void readMPU6050(void);

        void setMotorPWM(short pwm);
        void testMotor1(short pwm, short samples);
        void testMotorMovement(unsigned char d);
};

#endif // SMORA_H
