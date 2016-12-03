#include <Arduino.h>
#include "Smora.h"

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature ds18b20(&oneWire);
// arrays to hold device address
DeviceAddress sensorAddress;

Adafruit_INA219 ina219(INA219_addr);
MPU6050 mpu6050(MPU6050_addr);
AMS_AS5048B as5048b(AS5048B_addr);

SMORA::SMORA(){
}

void SMORA::init(){
  /* Init RGB Led */
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  digitalWrite(LED_R_PIN, HIGH);
  digitalWrite(LED_G_PIN, HIGH);
  digitalWrite(LED_B_PIN, HIGH);

  /* Init motor */
  pinMode(M_MODE_PIN, OUTPUT);
  digitalWrite(M_MODE_PIN, HIGH);

  digitalWrite(M_DIR_PIN, LOW);
  pinMode(M_DIR_PIN, OUTPUT);

  digitalWrite(M_PWM_PIN, LOW);
  pinMode(M_PWM_PIN, OUTPUT);

  /* Init testpads */
#if defined(ARDUINO_SAMD_ZERO)
  //pinMode(TESTPAD_2_PIN, OUTPUT);
  //pinMode(TESTPAD_4_PIN, OUTPUT);
  pinMode(TESTPAD_5_PIN, OUTPUT);
  //pinMode(TESTPAD_6_PIN, OUTPUT);
#else
  pinMode(TESTPAD_1_PIN, OUTPUT);
  //pinMode(TESTPAD_2_PIN, OUTPUT);
  pinMode(CTS_PIN, OUTPUT);
#endif

  /* Serial */
  NativeSerial.begin(SERIAL_BAUDRATE);
  while (!NativeSerial);
#if defined(ARDUINO_SAMD_ZERO)
  HalfDuplex.begin(SERIAL_BAUDRATE);
  while (!HalfDuplex);
#endif
  pinMode(DIR_PIN, OUTPUT);
  HalfDuplex_Receive();

  /* DS18B20 Temperature sensor */
  ds18b20.begin();
  ds18b20.getAddress(sensorAddress, 0);
  ds18b20.setResolution(sensorAddress, 12);
  ds18b20.setWaitForConversion(false);

  /* INA219AIDCNR Current sensor */
  ina219.begin();

  /* AS5048B magnetic position encoder */
  as5048b.begin();
  //consider the current position as zero
  //as5048b.setZeroReg();

  /* MPU6050 accel/gyro */
  mpu6050.initialize();
  pinMode(MPU6050_INT_PIN, INPUT); // also an interrupt pin
  //pinMode(MPU6050_INT_PIN, INPUT_PULLUP); // use this line instead if a pull-up is needed. Compatible with SAMD and AVR
  // supply your own gyro offsets here, scaled for min sensitivity
  /* 
  mpu6050.setXGyroOffset(220);
  mpu6050.setYGyroOffset(76);
  mpu6050.setZGyroOffset(-85);
  mpu6050.setZAccelOffset(1788); // 1688 factory default
  */

#if defined(ARDUINO_SAMD_ZERO)
  //analogWriteResolution(10);
  //analogWrite(M_DIR_PIN, 0);
  //analogWrite(M_PWM_PIN, 0);

  // http://forum.arduino.cc/index.php?topic=346731.5

  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for the 2 PWM channels: timer TCC0, TCC1 and TCC2 outputs
  const uint8_t CHANNELS = 2;
  const uint8_t pwmPins[] = { M_PWM_PIN, M_DIR_PIN };
  for (uint8_t i = 0; i < CHANNELS; i++)
  {
     PORT->Group[g_APinDescription[pwmPins[i]].ulPort].PINCFG[g_APinDescription[pwmPins[i]].ulPin].bit.PMUXEN = 1;
  }
  // Connect the TCC timers to the port outputs - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  //PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F; 
  //PORT->Group[g_APinDescription[4].ulPort].PMUX[g_APinDescription[4].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
  //PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
  PORT->Group[g_APinDescription[11].ulPort].PMUX[g_APinDescription[11].ulPin >> 1].reg = PORT_PMUX_PMUXO_E | PORT_PMUX_PMUXE_E;

  // Feed GCLK4 to TCC2 (and TC3)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC2 (and TC3)
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC2_TC3;    // Feed GCLK4 to TCC2 (and TC3)
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC2_WAVE |= TCC_WAVE_POL(0xF) |         // Reverse the output polarity on all TCC2 outputs
                    TCC_WAVE_WAVEGEN_DSBOTH;    // Setup dual slope PWM on TCC2
  while (TCC2->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation: 
  REG_TCC2_PER = 1023;         // Set the frequency of the PWM on TCC2 to 23.4375kHz
  while (TCC2->SYNCBUSY.bit.PER);                // Wait for synchronization
  
  // Set the PWM signal to output 0% duty cycle
  REG_TCC2_CCB0 = 0;         // TCC2 CCB0 - M_PWM_PIN 11
  while (TCC2->SYNCBUSY.bit.CCB0);                // Wait for synchronization
  REG_TCC2_CCB1 = 0;         // TCC2 CCB1 - M_DIR_PIN 13
  while (TCC2->SYNCBUSY.bit.CCB1);                // Wait for synchronization
  
  // Divide the 48MHz signal by 1 giving 48MHz (20.83ns) TCC2 timer tick and enable the outputs
  REG_TCC2_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC2 output
  while (TCC2->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
#else
  Timer1.initialize(50); //uS
  Timer1.pwm(TIMER1_B_PIN, 0);
  //Timer1.attachInterrupt(timer_interrupt);
#endif

  /* After all I2C sensors and initialized, switch frequency to TWI_FREQ */
  Wire.setClock(TWI_FREQ);

  ledAnimation(125);
}

void SMORA::halfDuplexWrite(byte data){
  HalfDuplex_Transmit();
  HalfDuplex.write(data);
#if defined(ARDUINO_SAMD_ZERO)
  // Silly hack to get around a bug in the way flush is used in SAMD
  // When only 1 byte is transmitted, flushUART() inside SERCOM.cpp returns without checking if the transmission is actually done
  // This means that when HalfDuplex_Receive is called, the byte gets corrupted...
  while(!SERCOM5->USART.INTFLAG.bit.TXC); 
#else 
  HalfDuplex.flush();
#endif
  HalfDuplex_Receive();
}

byte SMORA::halfDuplexRead(){
  HalfDuplex_Receive();
  return HalfDuplex.read();
}

byte SMORA::halfDuplexPeek(){
  return HalfDuplex.peek();
}

int SMORA::halfDuplexAvailable(){
  // Another silly workaround for SAMD
  // For some reason calling HalfDuplex.available() followed by NativeSerial.write() hangs after reading 1 byte if not done as follows... Why?
  HalfDuplex_Receive();
  int data = HalfDuplex.available();
  return data;
}

byte SMORA::getEEPROMHighAddress(int memAddress){
    return (byte)((memAddress) >> 8);
}

byte SMORA::getEEPROMLowAddress(int memAddress){
    return (byte)(memAddress & 0xFF);
}

byte SMORA::getEEPROMDevAddress(int memAddress){
    return E_24LC16BT_addr | (this->getEEPROMHighAddress(memAddress) & 0x07);
}

void SMORA::writeEEPROM(int memAddress, const byte data){
  Wire.beginTransmission( getEEPROMDevAddress(memAddress) );
  Wire.write( memAddress );
  Wire.write(data);
  Wire.endTransmission();
  delay(5);
}

void SMORA::writeEEPROM(int memAddress, const byte* buffer, int length){
    int page_i = 0;
    uint8_t c = 0;
    while (c < length){
        Wire.beginTransmission( getEEPROMDevAddress(memAddress + c) );
        Wire.write( getEEPROMLowAddress(memAddress + c) );
        page_i = 0;
        while ((page_i < 16) && (c < length)){
            Wire.write(buffer[c]);
            page_i++;
            c++;
        }
        Wire.endTransmission();
        delay(5);
    }
}

byte SMORA::readEEPROM(int memAddress){
    Wire.beginTransmission( getEEPROMDevAddress(memAddress) );
    Wire.write( memAddress );
    Wire.endTransmission();
    byte rdata = 0x00;
    Wire.requestFrom( getEEPROMDevAddress(memAddress), 1 );
    if (Wire.available()) 
    {
        rdata = Wire.read();
    }
    return rdata;
}

void SMORA::readEEPROM(int memAddress, byte* buffer, int length){
    Wire.beginTransmission( getEEPROMDevAddress(memAddress) );
    Wire.write( getEEPROMLowAddress(memAddress) );
    Wire.endTransmission();
    Wire.requestFrom( getEEPROMDevAddress(memAddress), (byte)length );
    int c = 0;
    for ( c = 0; c < length; c++ )
      if (Wire.available()) buffer[c] = Wire.read();
}

int SMORA::loadConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  // read first 4 bytes to get the CONFIG_VERSION
  unsigned int i = 0;
  unsigned long version;
  byte version_buffer[4];

  readEEPROM(CONFIG_START, version_buffer, 4);
  version = (version_buffer[3]<<24) + (version_buffer[2]<<16) + (version_buffer[1]<<8) + (version_buffer[0]);

  byte* data = (byte*)(void*)&storage;

  if (version == CONFIG_VERSION){
    for (i=0; i<sizeof(storage); i++)
      *data++ = readEEPROM(CONFIG_START+i);
  }
  return i;
}

int SMORA::saveConfig() {
  unsigned int i = 0;

  if (sizeof(storage) > E_24LC16BT_size)
    return -1;

  const byte* data = (const byte*)(const void*)&storage;

  for (i=0; i<sizeof(storage); i++)
    writeEEPROM(CONFIG_START+i, *data++);
  return i;
}

int SMORA::getConfigSize(){
  return sizeof(storage);
}

float SMORA::unwrapAngleDegrees(float prevAngle, float newAngle){
  float a = newAngle - prevAngle;
  if (a > 180.0)
    a -= 360.0;
  if (a < -180.0)
    a += 360.0;
  return a;
}

float SMORA::getVelocity(float prevAngle, float newAngle, float interval){
  return unwrapAngleDegrees(prevAngle, newAngle) / (interval / (float)1000000);
}

float SMORA::getAngle_Degrees(){
  return as5048b.angleR(3, true); // U_DEG = 3
}

float SMORA::getAngle_Radians(){
  return as5048b.angleR(4, true); // U_RAD = 4
}

unsigned int SMORA::getAngle_RAW(){
  unsigned int raw_angle;
  Wire.beginTransmission(AS5048B_addr);
  Wire.write(0xFE);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5048B_addr, 2);
  raw_angle = (unsigned int)(Wire.read() << 6) + (Wire.read() & 0x3F);
  return raw_angle;
}

float SMORA::getCurrent_mA(){
  return ina219.getCurrent_mA();
}

unsigned int SMORA::getCurrent_RAW(){
  unsigned int raw_current;
  
  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  //wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue);
  /*Wire.beginTransmission(INA219_addr);
  Wire.write(0x05);                       // Register - INA219_REG_CALIBRATION
  Wire.write((4096 >> 8) & 0xFF);         // Upper 8-bits - ina219_calValue
  Wire.write(4096 & 0xFF);                // Lower 8-bits - ina219_calValue
  Wire.endTransmission();*/
  
  // Now we can safely read the CURRENT register!
  //wireReadRegister(INA219_REG_CURRENT, &raw_current);
  Wire.beginTransmission(INA219_addr);
  Wire.write(0x04);                       // Register - INA219_REG_CURRENT
  Wire.endTransmission();

  //delay(1); // Max 12-bit conversion time is 586us per sample
  //delayMicroseconds(1172); // considering a 2x averaging configuration
  //delayMicroseconds(586); // considering a 1x configuration
  //delay(1);

  Wire.requestFrom(INA219_addr, 2);
  raw_current = ((Wire.read() << 8) | Wire.read());
  
  return (unsigned int)raw_current;
}

float SMORA::getVoltage_V(){
  return ina219.getBusVoltage_V();
}

unsigned int SMORA::getVoltage_RAW(){
  unsigned int raw_voltage;
  
  //wireReadRegister(INA219_REG_BUSVOLTAGE, &raw_voltage);
  Wire.beginTransmission(INA219_addr);
  Wire.write(0x02);                       // Register - INA219_REG_BUSVOLTAGE
  Wire.endTransmission();

  //delay(1); // Max 12-bit conversion time is 586us per sample
  //delayMicroseconds(1172); // considering a 2x averaging configuration
  //delayMicroseconds(586); // considering a 1x configuration

  Wire.requestFrom(INA219_addr, 2);
  raw_voltage = ((Wire.read() << 8) | Wire.read());

  // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
  return (unsigned int)((raw_voltage >> 3) * 4);
}

float SMORA::getTemperatureBlocking(){
  ds18b20.requestTemperatures();
  delay(750);
  return ds18b20.getTempC(sensorAddress);
}

float SMORA::getTemperature(){
  return ds18b20.getTempC(sensorAddress);
}

void SMORA::requestTemperatureConversion(){
  ds18b20.requestTemperatures();
}

bool SMORA::isTemperatureAvailable(){
  return (bool)oneWire.read();
}

unsigned long SMORA::timeTemperatureConversion(){
  unsigned long time = millis();
  requestTemperatureConversion();
  while(!isTemperatureAvailable()){}
  getTemperature();
  return millis() - time;
}

void SMORA::setRGB(unsigned char RGB){
  if (RGB & 0b100)
    digitalWrite(LED_R_PIN, LOW);
  else
    digitalWrite(LED_R_PIN, HIGH);
  
  if (RGB & 0b010)
    digitalWrite(LED_G_PIN, LOW);
  else
    digitalWrite(LED_G_PIN, HIGH);
  
  if (RGB & 0b001)
    digitalWrite(LED_B_PIN, LOW);
  else
    digitalWrite(LED_B_PIN, HIGH);
}

void SMORA::ledAnimation(unsigned int speed){
  setRGB(WHITE);  delay(speed);
  setRGB(YELLOW); delay(speed);
  setRGB(PURPLE); delay(speed);
  setRGB(RED);    delay(speed);
  setRGB(CYAN);   delay(speed);
  setRGB(GREEN);  delay(speed);
  setRGB(BLUE);   delay(speed);
  setRGB(BLACK);  delay(speed);
}

void SMORA::testSensors(){
  unsigned char error;

  struct {
    unsigned char addr;
    char name[10]; 
  } sensor[4] = {
    {INA219_addr, "INA219"},
    {MPU6050_addr, "MPU6050"},
    {AS5048B_addr, "AS5048B"},
    {E_24LC16BT_addr, "24LC16BT"}
  };

  for(int i=0; i<4; i++){
    Wire.beginTransmission(sensor[i].addr);
    error = Wire.endTransmission();
    if (error == 0) {
      NativeSerial.print("* Found sensor "); NativeSerial.print(sensor[i].name); NativeSerial.print(" @ 0x"); NativeSerial.println(sensor[i].addr, HEX);
    } else if (error == 2) {
      NativeSerial.print("! UNKNOWN sensor "); NativeSerial.print(sensor[i].name); NativeSerial.print(" @ 0x"); NativeSerial.println(sensor[i].addr, HEX);
    }
  }
}

void SMORA::readMPU6050(void){
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

void SMORA::setMotorPWM(short pwm){
  bool reverse = 0;
  
  if (pwm < 0){
    pwm = -pwm;  // make speed a positive quantity
    reverse = 1;  // preserve the direction
  }
  if (pwm > 1023)  // max PWM duty cycle
    pwm = 1023;
#if defined(ARDUINO_SAMD_ZERO)
  REG_TCC2_CCB0 = pwm;              // TCC2 CCB0 - M_PWM_PIN 11
  while (TCC2->SYNCBUSY.bit.CCB0);  // Wait for synchronization
#else
  Timer1.setPwmDuty(TIMER1_B_PIN, pwm);
#endif

  if (reverse) // flip if speed was negative or _flipM2 setting is active, but not both
    digitalWrite(M_DIR_PIN, HIGH);
  else
    digitalWrite(M_DIR_PIN, LOW);
}

void SMORA::testMotor1(short pwm, short samples){
  unsigned short raw_angle = 0;
  unsigned short raw_current = 0;
  unsigned short raw_bus_voltage = 0;

  unsigned long currentMicros = micros();   // us
  unsigned long previousMicros = micros();  // us
  const unsigned long interval = 50000;     // 50ms
    
  setRGB(RED);
  setMotorPWM(pwm);
  delay(1000);
  while(samples--){
    while(currentMicros - previousMicros < interval){
      currentMicros = micros();
    }
    previousMicros = currentMicros;

    // returns the RAW value. Needs to be converted using the formulas below
    raw_angle = getAngle_RAW();  
    /*
    #define AS5048B_RESOLUTION 16384.0 //14 bits
    angleConv = (angle / AS5048B_RESOLUTION) * 360.0;     // degrees
    angleConv = (angle / AS5048B_RESOLUTION) * 2 * M_PI;  // radian
    */

    //current = ina219.getCurrent_mA();
    //current = ina219.getCurrent_raw(); 
    // returns the RAW value in uint16_t. Needs to be divided by ina219_currentDivider_mA=10 (depending on the configuration)
    raw_current = getCurrent_RAW();
    
    //bus_voltage = ina219.getBusVoltage_V(); 
    //bus_voltage = ina219.getShuntVoltage_raw();  
    // returns the RAW value in uint16_t. Needs to be multiplied by 0.01
    raw_bus_voltage = getVoltage_RAW();
    
    NativeSerial.print(currentMicros); NativeSerial.print(',');
    NativeSerial.print(samples); NativeSerial.print(',');
    NativeSerial.print(pwm); NativeSerial.print(',');
    NativeSerial.print(raw_angle); NativeSerial.print(',');
    NativeSerial.print(raw_current); NativeSerial.print(',');
    NativeSerial.print(raw_bus_voltage); NativeSerial.print(',');
    NativeSerial.print(micros()-currentMicros); NativeSerial.println(); // Last conversion time for sensor measurements
    NativeSerial.flush();
  }
  setMotorPWM(0);
  setRGB(BLACK);
}

void SMORA::testMotorMovement(unsigned char wait){
  // linearly from 0 to max for x between 0 and max
  // linearly from max to 0 for x between max and 2*max
  int max = 1023;
  int pwm = 0;
  unsigned long time = millis();

  for(int x=0; x<=2*max; x++){
    while (millis() - time < wait){}
    pwm = abs(((x + max) % (2*max) - max));
#if defined(ARDUINO_SAMD_ZERO)
    REG_TCC2_CCB0 = pwm;              // TCC2 CCB0 - M_PWM_PIN 11
    while (TCC2->SYNCBUSY.bit.CCB0);  // Wait for synchronization
    //REG_TCC2_CCB1 = 0;                // TCC2 CCB1 - M_DIR_PIN 13 - 0 rotates one way, 1023 rotates the other
    //while (TCC2->SYNCBUSY.bit.CCB1);  // Wait for synchronization
#else
    Timer1.setPwmDuty(TIMER1_B_PIN, pwm);
#endif
    time = millis();
  }
}

void SMORA::testMotorIVW(){
  float newAngle = getAngle_Degrees();    // degrees
  float prevAngle = getAngle_Degrees();   // degrees
  float diffAngle = 0.0;
  float current = getCurrent_mA();        // mA
  float bus_voltage = getVoltage_V();     // V
  float velocity = 0.0;                   // degrees/s
  float instantVelocity = 0.0;
  float alpha = 0.05;                     // averaging factor

  unsigned short pwm = 31+32+32;
  unsigned char samples = 50;
  unsigned int count;
  unsigned char turns = 1;
  float voltage = 0.0;

  unsigned long currentMicros = micros();   // us
  unsigned long previousMicros = micros();  // us
  const unsigned long interval = 50000;     // 50ms

  setRGB(RED);
  setMotorPWM(pwm);
  delay(100);

  while(pwm < 1024){
    setMotorPWM(pwm);
    prevAngle = getAngle_Degrees();
    previousMicros = micros();
    delay(100);
    currentMicros = micros();
    current = getCurrent_mA();
    bus_voltage = getVoltage_V();
    count = samples;

    if(pwm > 415)
      turns = 2;
    else if (pwm > 831)
      turns = 3;

    //while(diffAngle < 360.0 && count < samples){
    while(diffAngle < turns*360.0){
      while(currentMicros - previousMicros < interval)
        currentMicros = micros();
      
      newAngle = getAngle_Degrees();
      current = (1-alpha)*current + alpha*getCurrent_mA();
      bus_voltage = (1-alpha)*bus_voltage + alpha*getVoltage_V();

      velocity = (1-alpha)*velocity + alpha*( getVelocity(prevAngle, newAngle, (float)interval) );

      diffAngle += abs(unwrapAngleDegrees(prevAngle, newAngle));

      prevAngle = newAngle;
      previousMicros = currentMicros;
      count++;
    }

    voltage = bus_voltage*(float)pwm/(float)1024;

    NativeSerial.print(pwm); NativeSerial.print(',');
    NativeSerial.print(count); NativeSerial.print(',');
    NativeSerial.print(voltage); NativeSerial.print(',');
    NativeSerial.print(current); NativeSerial.print(',');
    NativeSerial.print(bus_voltage); NativeSerial.print(',');
    NativeSerial.print(velocity); NativeSerial.println(); // Last conversion time for sensor measurements
    NativeSerial.flush();

    pwm += 32;
    diffAngle = 0.0;
    count = 0;
  }

  setMotorPWM(0);
  NativeSerial.println("done");
  setRGB(BLACK);
}
