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

#include "util.h"
#include "PID.h"
#include "eeprom.h"

#if defined(ARDUINO_SAMD_ZERO) // Required for Serial on Zero based boards
    #include "defines_atsamd21g18a.h"
#else
    #include "defines_atmega328p.h"
#endif

// ID of the settings block
#define CONFIG_VERSION 20161210

typedef struct STORAGE {
    unsigned long version;
    PID position;
    PID speed;
} STORAGE;

typedef struct ATTITUDE {
    short ax, ay, az;
    short gx, gy, gz;
} ATTITUDE;

class SMORA {    
    public:
        float temperature;

        ATTITUDE attitude;

        STORAGE storage = { .version=CONFIG_VERSION };

        PID* speedPID = &(storage).speed;
        PID* positionPID = &(storage).speed;

        SMORA();
        //~SMORA();
        void init(void);

        void halfDuplexWrite(byte);
        byte halfDuplexRead(void);
        byte halfDuplexPeek(void);
        int halfDuplexAvailable(void);

        int loadConfig();
        int saveConfig();
        int getConfigSize();

        void setRGB(unsigned char RGB);
        void ledAnimation(unsigned int speed);
        void testSensors(void);
        
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

        void getAttitude(void);

        void setMotorPWM(int pwm);
        void testMotor1(int pwm, unsigned int samples);
        void testMotorMovement(unsigned char d);

        void testMotorIVW();

        //void testPositionPID(float initAngle, float finalAngle, unsigned int duration_ms);
        //void positionPIDTest(void);

        void SpeedPIDLoop(float initSpeed, float finalSpeed, unsigned int duration_ms);
        void speedTestParser(void);
};

#endif // SMORA_H
