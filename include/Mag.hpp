#include<Arduino.h>
#include<Wire.h>

const int mag_addr = 0x1E;//Bussola HMC5883L
void setupMag(){
    Wire.beginTransmission(mag_addr);
    Wire.write(0x02);//Seleciona o modo
    Wire.write(0x00);//Modo de mecição continua
    Wire.endTransmission();
}

void readMag(int16_t &mx,int16_t &my,int16_t &mz){

    Wire.beginTransmission(mag_addr);
    Wire.write(0x03);
    Wire.endTransmission();

    Wire.requestFrom(mag_addr,6);
    if(6 <= Wire.available()){
        mx = Wire.read()<<8;
        mx = int16_t(mx|Wire.read());
        my = Wire.read()<<8;
        my = int16_t(my|Wire.read());
        mz = Wire.read()<<8;
        mz = int16_t(mz|Wire.read());
    }
}

// void setup(){
//     Wire.begin();
//     Serial.begin(9600);
//     setupMag();
// }

// void loop(){
//     double mx,my,mz;
//     readMag(mx,my,mz);
//     Serial.print("x: ");
//     Serial.print(mx);
//     Serial.print("  y: ");
//     Serial.print(my);
//     Serial.print("  z: ");
//     Serial.println(mz);
//     delay(250);
// }