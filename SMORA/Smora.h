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
    float PIDOutput, FeedforwardOutput;
    float error, previous_error;
    float integrator;
} PIDstate;

typedef struct PID {
    float Kp, Ki, Kd, Kf;
    float integrator_limit;
    float frequency;
    PIDstate state;
} PID;

typedef struct PIVstate {
    float Input, Output, Reference;
    float PIVOutput, FeedforwardOutput;
    float position_error, previous_position_error;
    float speed_error, previous_speed_error;
    float position, previous_position;
    float speed, previous_speed;
    float integrator;
};

typedef struct PIV {
    float Kp, Ki, Kv, Kf;
    float speed_limit;
    float frequency;
    PIVstate state;
} PIV;

typedef struct StoreStruct {
    unsigned long version;
    PID positionPID;
    PID speedPID;
    PIV speedPIV;
} StoreStruct;

typedef union byteFloat {
    float F;
    byte B[sizeof(float)];
} byteFloat;

typedef union byteInt {
    int I;
    byte B[sizeof(int)];
} byteInt;

typedef struct ATTITUDE {
    short ax, ay, az;
    short gx, gy, gz;
} ATTITUDE;

class SMORA {    
    public:
        float temperature;

        ATTITUDE motion;

        StoreStruct storage = {
          CONFIG_VERSION,
          {40., 2., 1., 1.,  360.0, 100., {0., 0., 0.,   0., 0.,   0., 0.,   0.}},
          {40., 2., 1., 1.,  250.0, 100., {0., 0., 0.,   0., 0.,   0., 0.,   0.}},
          
          { 1., 1., 1., 0.,  100.,  100., {0., 0., 0.,   0., 0.,   0., 0.,   0., 0.,   0., 0.,  0., 0.,  0.}}
        };

        PID* positionPID = &(storage).positionPID;
        PID* speedPID = &(storage).speedPID;
        PIV* speedPIV = &(storage).speedPIV;

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

        void getAttitude(void);

        void setMotorPWM(int pwm);
        void testMotor1(int pwm, unsigned int samples);
        void testMotorMovement(unsigned char d);

        void testMotorIVW();

        void setPIDGains(PID* controller, float Kp, float Ki, float Kd, float Kf);
        void setPIDFrequency(PID* controller, float frequency);
        void resetPIDIntegrator(PID* controller);
        void computePID(PID* controller, float input, float reference);
        int convertPIDOutputToPWM(PID* controller);

        void testPositionPID(float initAngle, float finalAngle, unsigned int duration_ms);
        void positionPIDTest(void);

        void testSpeedPID(float initSpeed, float finalSpeed, unsigned int duration_ms);
        void speedPIDTest(void);

        void setPIVGains(PIV* controller, float Kp, float Ki, float Kv, float Kf);
        void setPIVFrequency(PIV* controller, float frequency);
        void setPIVSpeed(PIV* controller, float speed);
        void resetPIVIntegrator(PIV* controller);
        void computePIV(PIV* controller, float reference, float input);
        int convertPIVOutputToPWM(PIV* controller);

        void testspeedPIV(float initAngle, float finalAngle, unsigned int duration_ms);
        void speedPIVTest(void);

};

#endif // SMORA_H
