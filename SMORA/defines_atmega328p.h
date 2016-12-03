#ifndef DEFINES_ATMEGA328P_H
#define DEFINES_ATMEGA328P_H

#include <TimerOne.h>

#define HalfDuplex 		Serial
#define NativeSerial	Serial

/* Led pins and colors RGB*/
#define LED_R_PIN       A1      // PC1
#define LED_G_PIN       A0      // PC0
#define LED_B_PIN       A2      // PC2

#define BLACK           0b000
#define BLUE            0b001
#define GREEN           0b010
#define CYAN            0b011
#define RED             0b100
#define PURPLE          0b101
#define YELLOW          0b110
#define WHITE           0b111

/* Motor pins */
#define M_MODE_PIN      8       // PB0 MODE=1 - Phase/Enable; MODE=0 - IN/IN
#define M_DIR_PIN       9       // PB1 IN1/PHASE
#define M_PWM_PIN       10      // PB2 IN2/ENBL

/* I2C */
#define TWI_FREQ        400000L
// Temperature sensor
#define ONE_WIRE_BUS    4
// INA219 current/voltage sensor
#define INA219_addr     0x42
// MPU6050 accelerometer/gyro
#define MPU6050_addr    0x68
#define MPU6050_INT_PIN 2
// AS5048B magnetic encoder
#define AS5048B_addr    0x40
// 24LC16BT I2C EEPROM
#define E_24LC16BT_addr 0x50
#define E_24LC16BT_size	2000 // bytes

/* TestPad 1 PD5 connected to pin 4 of the half-duplex connector */
#define TESTPAD_1_PIN   5
/* TestPad 2 PD6 disconnected */
#define TESTPAD_2_PIN   6
/* CTS PD7 pin from external programming adapter */
#define CTS_PIN         7

/* VIN_ADC */
#define VIN_ADC_PIN     A3

/* I2C pins - can also be used to connect a potenciometer */
#define SDA_PIN         A4  // PC4
#define SCL_PIN         A5  // PC5

/* Serial baudrate */
//#define SERIAL_BAUDRATE 1000000
#define SERIAL_BAUDRATE 115200
#define RX_PIN          0
#define TX_PIN          1
#define DIR_PIN         3
// Set DIR_PIN to HIGH to transmit and LOW to receive
#define HalfDuplex_Transmit()   digitalWrite(DIR_PIN, HIGH)
#define HalfDuplex_Receive()    digitalWrite(DIR_PIN, LOW)

#endif // DEFINES_ATMEGA328P_H
