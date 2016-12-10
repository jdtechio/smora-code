#include "eeprom.h"

byte getEEPROMHighAddress(int memAddress){
    return (byte)((memAddress) >> 8);
}

byte getEEPROMLowAddress(int memAddress){
    return (byte)(memAddress & 0xFF);
}

byte getEEPROMDevAddress(int memAddress){
    return E_24LC16BT_addr | (getEEPROMHighAddress(memAddress) & 0x07);
}

void writeEEPROM(int memAddress, const byte data){
  Wire.beginTransmission( getEEPROMDevAddress(memAddress) );
  Wire.write( memAddress );
  Wire.write(data);
  Wire.endTransmission();
  delay(5);
}

void writeEEPROM(int memAddress, const byte* buffer, int length){
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

byte readEEPROM(int memAddress){
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

void readEEPROM(int memAddress, byte* buffer, int length){
    Wire.beginTransmission( getEEPROMDevAddress(memAddress) );
    Wire.write( getEEPROMLowAddress(memAddress) );
    Wire.endTransmission();
    Wire.requestFrom( getEEPROMDevAddress(memAddress), (byte)length );
    int c = 0;
    for ( c = 0; c < length; c++ )
      if (Wire.available()) buffer[c] = Wire.read();
}