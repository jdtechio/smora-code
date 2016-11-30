#include "Smora.h"

SMORA smora;

void setup() {
  smora.init();
  //smora.test_sensors();
  //Serial.println();
  Serial.println("* Ready!");
}

void loop() {
  // A-rev.1 - arduino nano 57600
  // A-rev.2 - arduino mini 115200
  // B-rev.1 - arduino zero (Native USB)
  
  //smora.led_animation(125);
  //smora.setRGB(BLUE);
  //smora.testMotorMovement(1);
  //smora.setRGB(BLACK);
  /*
  smora.setRGB(YELLOW);
  float angle_degrees = smora.getAngle_Degrees();
  float angle_radians = smora.getAngle_Radians();
  float current = smora.getCurrent_mA();
  float voltage = smora.getVoltage_V();
  float temperature = smora.getTemperatureBlocking();
  Serial.print("Angle degrees: "); Serial.println(angle_degrees);
  Serial.print("Angle radians: "); Serial.println(angle_radians);
  Serial.print("Current: "); Serial.println(current);
  Serial.print("Voltage: "); Serial.println(voltage);
  Serial.print("Temperature: "); Serial.println(temperature);
  Serial.println();
  smora.setRGB(BLACK);

  smora.setRGB(BLUE);
  smora.readMPU6050();
  Serial.print("Accelerometer: \t");
  Serial.print(smora.ax); Serial.print("\t");
  Serial.print(smora.ay); Serial.print("\t");
  Serial.println(smora.az);
  Serial.print("Gyroscope: \t");
  Serial.print(smora.gx); Serial.print("\t");
  Serial.print(smora.gy); Serial.print("\t");
  Serial.println(smora.gz);
  Serial.println();
  smora.setRGB(BLACK);
  */
  /*
  smora.setRGB(RED);
  smora.testMotorMovement(5);
  smora.setRGB(BLACK);
  
  delay(1000);*/
  unsigned short pwm_value = 0;
  unsigned short count = 0;
  unsigned char start_byte;
  unsigned char availableBytes = 0;
  
  if (Serial.available()) {
    smora.setRGB(GREEN);
    availableBytes = Serial.available();
    start_byte = Serial.peek();

    if (start_byte == 0xFF && availableBytes == 5){
      Serial.read();
      pwm_value = (Serial.read()<<8) + Serial.read();
      count = (Serial.read()<<8) + Serial.read();
      
      if (pwm_value <= 1023) {
        smora.testMotor1(pwm_value, count);
      } else {
        Serial.print("Error: "); 
        Serial.print("#BYTES:"), Serial.print(availableBytes, DEC); Serial.print(',');
        Serial.print("STARTBYTE:");Serial.print(start_byte, HEX); Serial.print(',');
        Serial.print("PWM:"); Serial.print(pwm_value, DEC); Serial.print(',');
        Serial.print("COUNT:"); Serial.print(count, DEC); Serial.println();
        Serial.flush();
      }
    }
    smora.setRGB(BLACK);
  }
    
}




