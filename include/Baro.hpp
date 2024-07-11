#include<Arduino.h>
#include<Wire.h>
#include "Common.hpp"

#define REFERENCE_PRESSURE 101352

const int baro_addr = 0x77;
const unsigned char OSS = 0;//Oversampling

//Callibration
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;

long b5;

int readInt(unsigned char addr){
    unsigned char msb, lsb;
    Wire.beginTransmission(baro_addr);
    Wire.write(addr);
    Wire.endTransmission();

    Wire.requestFrom(baro_addr,2);
    while(Wire.available()<2) continue;
    msb = Wire.read();
    lsb = Wire.read();

    return msb<<8|lsb;
}

void setupBaro(){
    ac1 = readInt(0xAA);
    ac2 = readInt(0xAC);
    ac3 = readInt(0xAE);
    ac4 = readInt(0xB0);
    ac5 = readInt(0xB2);
    ac6 = readInt(0xB4);
    b1 = readInt(0xB6);
    b2 = readInt(0xB8);
    mb = readInt(0xBA);
    mc = readInt(0xBC);
    md = readInt(0xBE);
    
}

// Calcula a temperatue em °C
float getTemperature(unsigned int ut){
    long x1,x2;
    x1 = (((long)ut - (long)ac6)*(long)ac5)>>15;
    x2 = ((long)mc << 11)/(x1+md);
    b5 = x1 + x2;

    float temp = ((b5 + 8)>>4)/10;

    return temp;
}
//Calcula a pressão em Pa
long getPressure(unsigned long up){
    long x1,x2,x3,b3,b6,p;
    unsigned long b4,b7;

    b6 = b5 - 4000;

    x1 = (b2 * (b6 * b6)>>12)>>11;
    x2 = (ac2 * b6)>>11;
    x3 = x1 + x2;
    b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

    x1 = (ac3 * b6)>>13;
    x2 = (b1 * ((b6 * b6)>>12))>>16;
    x3 = ((x1 + x2) + 2)>>2;
    b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

    b7 = ((unsigned long)(up - b3) * (50000>>OSS));
    if (b7 < 0x80000000){
        p = (b7<<1)/b4;
    }else{
        p = (b7/b4)<<1;
    }
    x1 = (p>>8) * (p>>8);
    x1 = (x1 * 3038)>>16;
    x2 = (-7357 * p)>>16;
    p += (x1 + x2 + 3791)>>4;
    return p;
}

unsigned int getUncompensatedTemp(){
    unsigned ut;
    Wire.beginTransmission(baro_addr);
    Wire.write(0xF4);
    Wire.write(0X2E);
    Wire.endTransmission();

    delay(5);

    ut = readInt(0xF6);
    return ut;
}

unsigned long getUncompensatedPressure(){
    unsigned long msb, lsb, xlsb;
    unsigned long up = 0;

    Wire.beginTransmission(baro_addr);
    Wire.write(0xF4);
    Wire.write(0x34 + (OSS<<6));
    Wire.endTransmission();

    delay(2 + (3<<OSS));
    msb = readRegister(baro_addr,0xF6);
    lsb = readRegister(baro_addr,0xF7);
    xlsb = readRegister(baro_addr,0xF8);

    up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

    return up;
}


float getAltitude(float pressure){
    return 44330*(1-pow(pressure/REFERENCE_PRESSURE,1/5.255));

}

// void setup(){
//     Serial.begin(9600);
//     Wire.begin();
//     setupBaro();
// }

// void loop(){
//     unsigned int ut = getUncompensatedTemp();
//     float temperature = getTemperature(ut);

//     unsigned long up = getUncompensatedPressure();
//     float pressure = getPressure(up);

//     float atm = pressure/101352;
//     float altitude = getAltitude(pressure);

//     Serial.print("Temperatura: ");
//     // Mostra a temperatura com 2 casas decimais
//     Serial.print(temperature, 2); 
//     Serial.println(" C");
//     Serial.print("Pressao: ");
//     Serial.print(pressure, 0); 
//     Serial.println(" Pa");
//     Serial.print("Atmosfera padrao : ");
//     // Mostra o valor com 4 casas decimais
//     Serial.println(atm, 4); //display 4 decimal places
//     Serial.print("Altitude: ");
//     // Mostra o valor com 2 casas decimais
//     Serial.print(altitude, 2); 
//     Serial.println(" M");
//     Serial.println();
//     //Aguarda 5 segundos e reinicia o processo
//     delay(1000);
// }