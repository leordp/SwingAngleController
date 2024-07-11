#ifndef COMMOM
#define COMMOM
#include<Arduino.h>
#include<Wire.h>


void writeRegister(int device_addr, byte addr, byte val){
    Wire.beginTransmission(device_addr);
    Wire.write(addr);
    Wire.write(val);
    Wire.endTransmission();
}

unsigned long readRegister(int device_addr, byte addr){
    unsigned long v;
    Wire.beginTransmission(device_addr);
    Wire.write(addr);
    Wire.endTransmission();

    Wire.requestFrom(device_addr, 1);
    while(!Wire.available()){
        // Serial.print(device_addr);
        // Serial.println(" not available");
        //wait for address to be availabledelay(1);
    }
    v = Wire.read();
    return v;
}
#endif