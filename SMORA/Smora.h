#ifndef SMORA_H
#define SMORA_H 

#include <Arduino.h>

#include <Wire.h>

/* Patched I2Cdev.h by adding #define BUFFER_LENGTH 32 because Wire.h for SAMD doesn't have it */
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

class SMORA {
    //int8_t frameState;
    //char curChannel;
    //char frameHexData[8];
    
    public:
        short ax, ay, az;
        short gx, gy, gz;

        SMORA();
        //~SMORA();
        void init(void);

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

        
        /*
        void init(void (*process_frame_function)(char channel, uint32_t value, channels_t& obj),
                  void (*serial_write_function)(uint8_t b)
                  );
        */
};

#endif // SMORA_H
