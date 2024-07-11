#include <Arduino.h>
#include <Wire.h>
// #include <I2Cdev.h>
// #include <MPU6050.h>
#include "Common.hpp"

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24


const int gyro_addr = 105;//Sensor L3G4200D


void setupGyro(int scale){
    // Enable x,y,z and turn off power down
    writeRegister(gyro_addr,CTRL_REG1,0b00001111);

    //Adjust High Pass Filter
    writeRegister(gyro_addr, CTRL_REG2,0b00000000);

    //Generate data ready interrupt on INT2
    writeRegister(gyro_addr, CTRL_REG3,0b00001000);

    //Controls the full-scale range, among other things
    if(scale == 250){
        writeRegister(gyro_addr, CTRL_REG4,0b00000000);
    }else if(scale == 500){
        writeRegister(gyro_addr, CTRL_REG4,0b00010000);
    }else{
        writeRegister(gyro_addr, CTRL_REG4,0b00110000);
    }

    //Controls high-pass filter of output
    writeRegister(gyro_addr,CTRL_REG5,0b00000000);
}


void readGyro(int16_t &gx, int16_t &gy, int16_t &gz){
    // Rotina para leitura dos valores de X, Y e Z
    byte xMSB = readRegister(gyro_addr, 0x29);
    byte xLSB = readRegister(gyro_addr, 0x28);
    gx = ((xMSB << 8) | xLSB);
    byte yMSB = readRegister(gyro_addr, 0x2B);
    byte yLSB = readRegister(gyro_addr, 0x2A);
    gy = ((yMSB << 8) | yLSB);
    byte zMSB = readRegister(gyro_addr, 0x2D);
    byte zLSB = readRegister(gyro_addr, 0x2C);
    gz = ((zMSB << 8) | zLSB);

}

// void setup(){
//     Wire.begin();
//     Serial.begin(9600);
    
//     setupGyro(2000);

//     delay(1500);
// }


// void loop(){
//     double gx,gy,gz;
//     readGyro(gx,gy,gz);
//     Serial.print("X:");
//     Serial.print(gx);
//     Serial.print(" Y:");
//     Serial.print(gy);
//     Serial.print(" Z:");
//     Serial.println(gz);
//     delay(100);
//   // Aguarda 100ms e reinicia o processo

// }