/*openCopter
 An open source flight controller based on the openIMU
 The sensors used in this example are the ADXL345, the L3G3200, the LSM303DLHC, and the BMP085
 If the device is moved during startup it will not function properly. 
 
 Copyright (C) 2013  Michael Baker
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
 
 Gyro must be in RAD/s
 Sensors must be in the North East Down convention
 This example code will only work on the MEGA
 To use on a different arduino change the slave select defines or use digitalWrite 
 */
#include <I2C.h>
#include <SPI.h>
#include "openIMUL.h"
#include "MPIDL.h"//the L is for local incase there is already a library by that name
#include "UBLOXL.h"

//LED defines
#define RED 38
#define YELLOW 40
#define GREEN 42

//general SPI defines
#define READ 0x80
#define WRITE 0x00
#define MULTI 0x40
#define SINGLE 0x00

//gyro defines - ST L3G2
#define L3G_CTRL_REG1 0x20
#define L3G_CTRL_REG2 0x21
#define L3G_CTRL_REG3 0x22
#define L3G_CTRL_REG4 0x23
#define L3G_CTRL_REG5 0x24
#define L3G_OUT_X_L 0x28

//acc defines - Analog Devices ADXL345
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define DATAX0 0x32
//use calibrate_acc to find these values
//take the max and min from each axis
//then use these formulas
//ACC_SC_etc_NEG = 9.8 / axis min
//ACC_SC_etc_POS = 9.8 / axis max
//gravity is sensed differently in each direction
//that is why datasheets list a range of LSB/g
//the values from the datasheets can be used but it will hurt the performance of the filter
//these calibration values will change over time due to factors like tempature
#define ACC_SC_X_NEG 0.038735177f
#define ACC_SC_X_POS 0.036981132f
#define ACC_SC_Y_NEG 0.037984496f
#define ACC_SC_Y_POS 0.036981132f
#define ACC_SC_Z_NEG 0.041525423f
#define ACC_SC_Z_POS 0.037262357f

//mag defines ST LSM303DLHC - will possibly work with the HMC5883L
#define MAG_ADDRESS 0x1E
#define LSM303_CRA_REG (uint8_t)0x00 //??? Wire.h needs to be fixed
#define LSM303_CRB_REG 0x01
#define LSM303_MR_REG 0x02
#define LSM303_OUT_X_H 0x03
//use calibrate_mag to find these values
#define compassXMax 289.0f
#define compassXMin -368.0f
#define compassYMax 246.0f
#define compassYMin -550.0f
#define compassZMax 642.0f
#define compassZMin -245.0f
#define inverseXRange (float)(2.0 / (compassXMax - compassXMin))
#define inverseYRange (float)(2.0 / (compassYMax - compassYMin))
#define inverseZRange (float)(2.0 / (compassZMax - compassZMin))

//barometer defines
//the code for the BMP085 uses the data ready interrupt so the program isn't blocked 
#define BMP085_ADDRESS 0x77
#define OSS 0x00
#define READY_PIN 47
#define POLL_RATE 0

//RC defines
#define DSM2 0
#define DSMX 1
#define SBUS 2
#define RC 3
#define HEX_ZERO 0x00
//telemetry defines
#define radio Serial2
#define TRANSMIT_TIME 20
//control defines
#define TAKEOFF 0 
#define HH_ON 1
#define HH_OFF 2
#define LAND 3
#define LIFTOFF 1175 //3s

//using these macros in place of digitalWrite is much faster
//however digitalWrite will work when using SPI 
#define GyroSSOutput() DDRL |= 1<<0 //this is the same as pinMode(49,OUTPUT)
#define GyroSSHigh() PORTL |= 1<<0 //this is like digitalWrite(49,HIGH) but faster
#define GyroSSLow() PORTL &= ~(1<<0)


#define AccSSOutput() DDRL |= 1<<1 //this is the same as pinMode(48,OUTPUT)
#define AccSSHigh() PORTL |= 1<<1 //this is like digitalWrite(48,HIGH) but faster
#define AccSSLow() PORTL &= ~(1<<1)

#define DebugOutput() DDRA |= 1<<1
#define DebugHigh() PORTA |= 1<<1
#define DebugLow() PORTA &= ~(1<<1)

//motor defines
#define FREQ 400
#define PRESCALE 8
#define PERIOD ((F_CPU/PRESCALE/FREQ) - 1)

#define Motor1WriteMicros(x) OCR3B = x * 2//motor 1 is attached to pin2
#define Motor2WriteMicros(x) OCR3C = x * 2//motor 2 is attached to pin3
#define Motor3WriteMicros(x) OCR3A = x * 2//motor 3 is attached to pin5
#define Motor4WriteMicros(x) OCR4A = x * 2//motor 4 is attached to pin6

//GPS defines
#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f
#define GPSPort Serial3

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

Sensor_t gyro;
Sensor_t acc;
Sensor_t mag;

//barometer variables
int pressureState;
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
unsigned char msb;
unsigned char lsb;
unsigned char xlsb;
long x1;
long x2;
long x3;
long b3;
long b5;
long b6;
long p;
unsigned long b4;
unsigned long b7;
unsigned int ut;
unsigned long up;
unsigned long baroTimer;
float pressureRatio;
long pressure;
short temperature;
boolean newBaro;
int baroCount;
float baroSum;
long pressureInitial; 
float rawAltitude;

//RC vars
uint8_t rcType,readState,inByte;
boolean detected = false;
boolean newRC = false;


const uint8_t syncArray1[14] = {
  1,0,3,2,5,4,7,6,9,8,11,10,13,12};
const uint8_t syncArray2[14] = {
  1,0,3,2,7,6,5,4,11,10,13,12,9,8};

int bufferIndex=0;

typedef union{
  struct{
    uint16_t aileron;//A8
    uint16_t aux1;//A9
    uint16_t elevator;//A10
    uint16_t gear;//A11
    uint16_t rudder;//A12
    uint16_t aux2;//A13
    uint16_t throttle;//A14
    uint16_t aux3;//A15 only for futaba or standard RC
  }
  values;
  byte buffer[16];
  uint16_t standardRCBuffer[8];
}
RadioControl_t;


RadioControl_t rcCommands;

uint8_t currentPinState = 0;
uint8_t previousPinState = 0;
uint8_t changeMask = 0;
uint8_t lastPinState = 0;
uint16_t currentTime = 0;
uint16_t timeDifference = 0;
uint16_t changeTime[8];
int16_t offset = 0;

uint8_t sBusData[25];

//for the calibration of the gyro
int32_t gyroSumX,gyroSumY,gyroSumZ;
int16_t offsetX,offsetY,offsetZ;

//IMU related vars
float radianGyroX,radianGyroY,radianGyroZ;
float degreeGyroX,degreeGyroY,degreeGyroZ;
float floatMagX,floatMagY,floatMagZ;//needs to be a float so the vector can be normalized
int16_t rawX,rawY,rawZ;
float scaledAccX,scaledAccY,scaledAccZ;
float smoothAccX,smoothAccY,smoothAccZ;
float accToFilterX,accToFilterY,accToFilterZ;
float dt,g_dt;

//radio vars
byte inGainBuffer[7];
byte radioInByte;
uint8_t inGainBufferIndex = 0;
uint8_t gainBufferIndex = 0;
long transmitTimer;

//control related vars
boolean integrate = false;
boolean startHold = true;
boolean startLoiter = true;
float zero = 0.0;//so that the MPID library can be used
float motorCommand1,motorCommand2,motorCommand3,motorCommand4;

typedef union {
  struct{
    long timestamp;
    int32_t latInit;
    int32_t lonInit;
    int32_t lat;
    int32_t lon;
    float dist;
    float pitchError;
    float rollError;
    float heading;
    float bodyHeading;
    float yawAngle;
    float pitchSetPoint;
    float rollSetPoint;
    uint8_t GPSfix;
  }
  v;
  byte buffer[53];
}
transmissionUnion_t;

transmissionUnion_t t;//contains all the control vars to be monitored


float pitchAngle;
float rollAngle;
float yawSetPoint;
float rateSetPointX;    
float rateSetPointY;
float rateSetPointZ;
float adjustmentX;
float adjustmentY;
float adjustmentZ; 
float altitudeSetPoint;
float throttleAdjustment;    


typedef union{
  struct{

    float kp_loiter;
    float ki_loiter;
    float kd_loiter;
    float n_loiter;


  }
  v;
  byte buffer[16];
}
gainUnion_t;

gainUnion_t g;//contains the gains

float kp_alt_p = 10;
float ki_alt_p = 1.75;
float kd_alt_p = 15;
float nAlt_p = 15;


float kp_r_p = 0.64409;
float ki_r_p = 0.041982;
float kd_r_p = 0.01423;
float nPitch = 19.5924;

float kp_r_r = 0.64409;
float ki_r_r = 0.041982;
float kd_r_r = 0.01423;
float nRoll = 19.5924;

float kp_r_y = 6.9389;
float ki_r_y = 0.22747;
float kd_r_y = -0.42597;
float nYaw = 4.4174;

float kp_a_p = 4.6527;
float ki_a_p = 0.2005;
float kd_a_p = 0.11256;
float nPitchA = 47.9596;

float kp_a_r = 4.6527;
float ki_a_r = 0.2005;
float kd_a_r = 0.11256;
float nRollA = 47.9596;

float kp_a_y = 3.0564;
float ki_a_y = 0.099342;
float kd_a_y = 0.51023;
float nYawA = 36.9853;


int flightState = 3;
float yawInput;

uint8_t loopCount;
uint16_t i;//index for buffering in the data
uint16_t j;
uint8_t k;//index for RC signals
uint32_t timer,printTimer;
//this is how you use the AHRS and Altimeter
openIMU imu(&radianGyroX,&radianGyroY,&radianGyroZ,&accToFilterX,&accToFilterY,&accToFilterZ,&scaledAccX,&scaledAccY,&scaledAccZ,&floatMagX,&floatMagY,&floatMagZ,&rawAltitude,&dt);

MPID PitchAngle(&t.v.pitchSetPoint,&imu.pitch,&rateSetPointY,&integrate,&kp_a_p,&ki_a_p,&kd_a_p,&nPitchA,&dt,200,300);
MPID RollAngle(&t.v.rollSetPoint,&imu.roll,&rateSetPointX,&integrate,&kp_a_r,&ki_a_r,&kd_a_r,&nRollA,&dt,200,300);
MYAW YawAngle(&yawSetPoint,&imu.yaw,&rateSetPointZ,&integrate,&kp_a_y,&ki_a_y,&kd_a_y,&nYawA,&dt,200,300);

MPID PitchRate(&rateSetPointY,&degreeGyroY,&adjustmentY,&integrate,&kp_r_p,&ki_r_p,&kd_r_p,&nPitch,&dt,200,200);
MPID RollRate(&rateSetPointX,&degreeGyroX,&adjustmentX,&integrate,&kp_r_r,&ki_r_r,&kd_r_r,&nRoll,&dt,200,200);
MPID YawRate(&rateSetPointZ,&degreeGyroZ,&adjustmentZ,&integrate,&kp_r_y,&ki_r_y,&kd_r_y,&nYaw,&dt,200,200);

MPID AltHoldPosition(&altitudeSetPoint,&imu.altitude,&throttleAdjustment,&integrate,&kp_alt_p,&ki_alt_p,&kd_alt_p,&nAlt_p,&dt,200,200);

MPID LoiterRollPosition(&zero,&t.v.rollError,&t.v.rollSetPoint,&integrate,&g.v.kp_loiter,&g.v.ki_loiter,&g.v.kd_loiter,&g.v.n_loiter,&g_dt,15,20);
MPID LoiterPitchlPosition(&zero,&t.v.pitchError,&t.v.pitchSetPoint,&integrate,&g.v.kp_loiter,&g.v.ki_loiter,&g.v.kd_loiter,&g.v.n_loiter,&g_dt,15,20);

boolean failSafe = false;
long failSafeTimer;

UBLOX gps;
long prevGPSTime;

void setup(){
  pinMode(RED,OUTPUT);
  pinMode(YELLOW,OUTPUT);
  pinMode(GREEN,OUTPUT);
  digitalWrite(YELLOW,HIGH);
  digitalWrite(RED,HIGH);
  Serial.begin(115200);
  Serial.println("start");
  radio.begin(115200);
  MotorInit();
  DetectRC();
  if (rcType == RC){
    DDRK = 0;//PORTK as input
    PORTK |= 0xFF;//turn on pull ups
    PCMSK2 |= 0xFF;//set interrupt mask for all of PORTK
    PCICR = 1<<2;//enable the pin change interrupt for K
    delay(100);//wait for a few frames to come in if the aileron channel has not been recieved when the Center() function is called the system does not work
    Center();
  }  
  Serial.println("RC done");
  //arming procedure - Gear switch has to be toggled to arm
  newRC = false;
  while (newRC == false){
    if (rcType == RC){
      delay(100);
    }
    if (rcType != RC){
      FeedLine();
    }
  }

  while (rcCommands.values.gear < 1850){
    if (rcType == RC){
      delay(100);
    }
    if (rcType != RC){
      FeedLine();
    }
  } 
  newRC = false;

  Serial.println("armed");
  digitalWrite(RED,LOW);

  I2c.begin();
  I2c.setSpeed(1);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);  

  AccSSOutput();//this was moved from the init
  AccSSHigh();//if high isn't written to both devices befor config 
  GyroSSOutput();//the SPI bus will be addressing both devices 
  GyroSSHigh();

  GyroInit();
  AccInit();
  MagInit();

  GetGyro();
  GetAcc();
  GetMag();
  imu.InitialQuat();

  LevelAngles();

  BaroInit();
  Serial.println("cal complete");
  PitchAngle.reset();
  RollAngle.reset();
  YawAngle.reset();

  PitchRate.reset();
  RollRate.reset();
  YawRate.reset();

  AltHoldPosition.reset();

  LoiterRollPosition.reset();
  LoiterPitchlPosition.reset();

  g.v.kp_loiter = 6.5;
  g.v.ki_loiter = 0.1;
  g.v.kd_loiter = 0;
  g.v.n_loiter = 0;


  gps.init();

  while (rcCommands.values.throttle > 1020){
    //Serial.println(rcCommands.values.throttle);
    if (rcType != RC){
      FeedLine();
    }
    digitalWrite(GREEN,HIGH);
    delay(500);
    digitalWrite(GREEN,LOW);
    delay(500);
  }
  digitalWrite(YELLOW,LOW);
  digitalWrite(GREEN,HIGH);

  loopCount = 0;
  DebugOutput();
  printTimer = millis();
  failSafeTimer = millis();
  timer = micros();
}

void loop(){
  PollPressure();
  if (newBaro == true){
    newBaro = false;
    GetAltitude(&pressure,&pressureInitial,&rawAltitude);
    imu.BaroKalUpdate();
  }  
  gps.Monitor();
  if (gps.newData == true){
    gps.newData = false;
    t.v.GPSfix = gps.data.vars.gpsFix;
    if (rcCommands.values.aux1 < 1100){

      g_dt = (millis() - prevGPSTime) / 1000.0;
      prevGPSTime = millis();
      if (startLoiter == true){
        startLoiter = false;
        t.v.latInit = gps.data.vars.lat;
        t.v.lonInit = gps.data.vars.lon;
        LoiterRollPosition.reset();
        LoiterPitchlPosition.reset();
      }
      else{
        Loiter();
      }
    }
    else{
      startLoiter = true;
    }

  }  
  if (micros() - timer >= 5000){//attempt to run at 200 hz  
    //DebugHigh();
    dt = ((micros() - timer) / 1000000.0);
    timer = micros();
    GetGyro();
    GetAcc();
    if (loopCount == 6){
      GetMag();
      imu.AHRSupdate();
      loopCount = 0;
    }
    else{
      imu.IMUupdate();
      loopCount++;
    }
    imu.AccKalUpdate();
    imu.GetEuler();
    Angle();
    Rate();
    if (rcCommands.values.aux1 < 1600){
      HeadingHold();
      if(startHold == true){
        AltHoldPosition.reset();
        startHold = false;
        altitudeSetPoint = imu.altitude;
      }
      AltitudeHold();
    }
    else{
      startHold = true;
      throttleAdjustment = 0;
    }
    MotorHandler();
    //DebugLow();
  }

  if (rcType != RC){
    FeedLine();
  }
  if (newRC == true){
    newRC = false;
    failSafeTimer = millis();
    ProcessChannels();
    //imu.GetEuler();
    //for debugging purposes
    /*if (rcCommands.values.gear > 1600){
     Serial.print(millis());
     Serial.print(",");
     Serial.print(imu.pitch);
     Serial.print(",");
     Serial.print(imu.roll);
     Serial.print(",");
     Serial.print(imu.yaw);
     Serial.print(",");
     Serial.println(imu.altitude);
     }*/
    /* Serial.print(",");*/
    /*Serial.print(rcCommands.values.aileron);
    Serial.print(",");
    Serial.print(rcCommands.values.elevator);
    Serial.print(",");
    Serial.print(rcCommands.values.throttle);
    Serial.print(",");
    Serial.print(rcCommands.values.rudder);
    Serial.print(",");
    Serial.print(rcCommands.values.gear);
    Serial.print(",");
    Serial.print(rcCommands.values.aux1);
    Serial.print(",");
    Serial.print(rcCommands.values.aux2);
    Serial.print(",");
    Serial.println(rcCommands.values.aux3); */


  }  
  if (millis() - failSafeTimer > 1000){
    failSafe = true;
  }
  if (failSafe == true ){
    Motor1WriteMicros(1000);//set the output compare value
    Motor2WriteMicros(1000);
    Motor3WriteMicros(1000);
    Motor4WriteMicros(1000);
    digitalWrite(GREEN,LOW);
    while(1){

      digitalWrite(RED,HIGH);
      delay(500);
      digitalWrite(RED,LOW);
      delay(500);
    }
  }

  if (rcCommands.values.gear < 1100){
    Gains();
  }
  else{
    if (rcCommands.values.gear < 1600){
    }
    else{
      Transmit();
    }
  }

}

void Smoothing(int16_t *raw, float *smooth){
  *smooth = (*raw * (0.10)) + (*smooth * 0.9);
}
void MapVar (float *x, float *y, float in_min, float in_max, float out_min, float out_max){
  *y = (*x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void MapVar (int16_t *x, float *y, float in_min, float in_max, float out_min, float out_max){
  *y = (*x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



void MapVar (uint16_t *x, float *y, float in_min, float in_max, float out_min, float out_max){
  *y = (*x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


























