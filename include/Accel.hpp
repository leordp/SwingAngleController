#include<Arduino.h>
#include<Wire.h>

#define Register_ID 0
#define Register_Data 0x31
#define Register_2D 0x2D
#define Register_X0 0x32
#define Register_X1 0x33
#define Register_Y0 0x34
#define Register_Y1 0x35
#define Register_Z0 0x36
#define Register_Z1 0x37

const int accel_addr = 0x53;//ADXL345
int16_t X,Y,Z;

void setupAccel(){
    Wire.beginTransmission(accel_addr);
    Wire.write(Register_2D);
    Wire.write(8);//Mesuring Enable
    Wire.endTransmission();
}

int readRegisters(int reg1, int reg2){
    Wire.beginTransmission(accel_addr);
    Wire.write(reg1);
    Wire.write(reg2);
    Wire.endTransmission();
    Wire.requestFrom(accel_addr,2);
    int x1 = 0,x2 = 0,xo = 0;
    if(Wire.available() <= 2){
        x1 = Wire.read();
        x2 = Wire.read();
        x2 = x2<<8;
        xo = x1|x2;
    }
    return xo;
}

void readAccel(double &ax, double &ay, double &az){
    X = readRegisters(Register_X0,Register_X1);//Read x value
    Y = readRegisters(Register_Y0,Register_Y1);//Read y value
    Z = readRegisters(Register_Z0,Register_Z1);//Read z value

    ax = X/256.f;
    ay = Y/256.f;
    az = Z/256.f;
}

// void setup(){
//     Wire.begin();
//     Serial.begin(9600);
//     setupAccel();
// }

// void loop(){
//     double ax,ay,az;
//     readAccel(ax,ay,az);
//     Serial.print(" X= ");
//     Serial.print(ax);
//     Serial.print("    ");
//     Serial.print("Y= ");
//     Serial.print(ay);
//     Serial.print("    ");
//     Serial.print("Z= ");
//     Serial.print(az);
//     Serial.println("  ");
//     delay(200);
// }