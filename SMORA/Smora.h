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

typedef struct PIDstate {
    float Input, Output, Reference;
    float error, previous_error;
    float integrator;
} PIDstate;

typedef struct PID {
    float Kp, Ki, Kd;
    float integrator_limit;
    float frequency;
    PIDstate state;
} PID;

typedef struct StoreStruct {
    unsigned long version;
    PID positionPID;
} StoreStruct;

typedef union byteFloat {
    float F;
    byte B[sizeof(float)];
} byteFloat;

typedef union byteInt {
    int I;
    byte B[sizeof(int)];
} byteInt;

class SMORA {    
    public:
        short ax, ay, az;
        short gx, gy, gz;

        float temperature;

        StoreStruct storage = {
          CONFIG_VERSION,
          {40., 1., 1., 1023, 100., {0., 0., 0.,   0., 0.,   0.}}
        };

        SMORA();
        //~SMORA();
        void init(void);

        void halfDuplexWrite(byte);
        byte halfDuplexRead(void);
        byte halfDuplexPeek(void);
        int halfDuplexAvailable(void);

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
        void ledAnimation(unsigned int speed);
        void testSensors(void);
        
        float unwrapAngleDegrees(float prevAngle, float newAngle);
        float getVelocity(float prevAngle, float newAngle, float interval);
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

        void testMotorIVW();

        void setPositionPID_Gains(float Kp, float Ki, float Kd);
        void setPositionPID_Frequency(float frequency);
        void resetPositionPID_Integrator(void);
        void computePositionPID(float position);
        short convertPositionPIDOutputToPWM(void);

        void plotPositionPID(float initAngle, float finalAngle, unsigned int duration_ms);
};

#endif // SMORA_H
