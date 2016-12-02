#ifndef DEFINES_ATMEGA328P_H
#define DEFINES_ATMEGA328P_H

#define mySerial 	SerialUSB
#define Serial 		SerialUSB

/* Led pins */
#define LED_R_PIN       7      // PA21
#define LED_G_PIN       12     // PA19
#define LED_B_PIN       6      // PA20

#define BLACK           0b000
#define BLUE            0b001
#define GREEN           0b010
#define CYAN            0b011
#define RED             0b100
#define PURPLE          0b101
#define YELLOW          0b110
#define WHITE           0b111

/* Motor pins */
#define M_MODE_PIN      5       // PA15 MODE=1 - Phase/Enable; MODE=0 - IN/IN
#define M_DIR_PIN       13      // PA17 IN1/PHASE
#define M_PWM_PIN       11      // PA16 IN2/ENBL

/* I2C */
#define TWI_FREQ        400000L
// Temperature sensor
#define ONE_WIRE_BUS    2		// PA14
// INA219 current/voltage sensor
#define INA219_addr     0x42
// MPU6050 accelerometer/gyro
#define MPU6050_addr    0x68
#define MPU6050_INT_PIN 3		// PA09
// AS5048B magnetic encoder
#define AS5048B_addr    0x40
// 24LC16BT I2C EEPROM
#define E_24LC16BT_addr 0x50
#define E_24LC16BT_size	2000 // bytes

#define TESTPAD_2_PIN   9		// PA07 - not connected
#define TESTPAD_4_PIN   22		// PA12 - not connected
#define TESTPAD_5_PIN	38 		// PA13 - connected to pin 4 of half-duplex connector
#define TESTPAD_6_PIN   4		// PA08 - not connected


/* VIN_ADC */
#define VIN_ADC_PIN     A4 		// PA05

/* I2C pins - can also be used to connect a potenciometer */
#define SDA_PIN         20  	// PA22
#define SCL_PIN         21  	// PA23

/* Serial baudrate */
//#define SERIAL_BAUDRATE 1000000
#define SERIAL_BAUDRATE 115200
#define RX_PIN          31		// PB23
#define TX_PIN          30		// PB22
#define DIR_PIN         26		// PA27
// Set DIR_PIN to HIGH to transmit and LOW to receive
#define halfduplex_transmit()   digitalWrite(DIR_PIN, HIGH)
#define halfduplex_receive()    digitalWrite(DIR_PIN, LOW)

#endif // DEFINES_ATMEGA328P_H
