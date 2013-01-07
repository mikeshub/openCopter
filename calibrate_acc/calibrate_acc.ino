/*
Usage of the calibration program:
This will require something like a set of helping hands. The idea is we want to find out what gravity is detected as
for earch plus and minus for each axis. The method I use is as follows.

Using the helping hands rotate the device until you are reading gravity only on the positive X axis. 

Capture the output of this program as a .csv file for a few second being very careful not to move the device

Take the average of those values. This is what the device detects gravity as.

Repeat for the other axes

*/
#include <SPI.h>

//general SPI defines
#define READ 0x80
#define WRITE 0x00
#define MULTI 0x40
#define SINGLE 0x00


#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define DATAX0 0x32

#define AccSSOutput() DDRL |= 1<<1 //this is the same as pinMode(49,OUTPUT)
#define AccSSHigh() PORTL |= 1<<1 //this is like digitalWrite(49,HIGH) but faster
#define AccSSLow() PORTL &= ~(1<<1)

typedef union{
  struct{
    int16_t x;
    int16_t y;
    int16_t z;
  }
  v;
  uint8_t buffer[6];
}
Sensor_t;

Sensor_t acc;


void setup(){
  Serial.begin(115200);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2); 
  AccSSOutput();
  AccInit();
}

void loop(){
  GetAcc();
  Serial.print(acc.v.x);
  Serial.print(",");
  Serial.print(acc.v.y);
  Serial.print(",");
  Serial.println(acc.v.z);
  delay(10);
}
void GetAcc(){
  AccSSLow();
  //delayMicroseconds(1);
  SPI.transfer(DATAX0 | READ | MULTI);
  for (int i = 0; i < 6; i++){//the endianness matches as does the axis order
    acc.buffer[i] = SPI.transfer(0x00);
  }
  //delayMicroseconds(1);
  AccSSHigh();  
}

void AccInit(){
  SPI.setDataMode(SPI_MODE3);
  AccSSOutput();

  AccSSLow();
  delayMicroseconds(1);
  SPI.transfer(WRITE | SINGLE | BW_RATE);
  SPI.transfer(0x0C);//400hz
  delayMicroseconds(1);
  AccSSHigh();
  delay(5);
  AccSSLow();
  delayMicroseconds(1);
  SPI.transfer(WRITE | SINGLE | POWER_CTL);
  SPI.transfer(0x08);//start measurment
  delayMicroseconds(1);
  AccSSHigh();
  delay(5);
  AccSSLow();
  delayMicroseconds(1);
  SPI.transfer(WRITE | SINGLE | DATA_FORMAT);
  SPI.transfer(0x0B);//full resolution + / - 16g
  delayMicroseconds(1);
  AccSSHigh();
  delay(5);

}

