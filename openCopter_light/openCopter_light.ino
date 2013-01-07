/*openCopter light
 An open source flight controller based on the openIMU
 The sensors used in this example are the ADXL345 and the L3G3200
 If the device is moved during startup it will not function properly. 
 
 Copyright (C) 2012  Michael Baker
 
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
#include <SPI.h>
#include "MPIDL.h"//the L is for local in case there is already a library by that name
#include "openIMUL.h"

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
#define ACC_SC_X_NEG 0.038431372f
#define ACC_SC_X_POS 0.036029411f
#define ACC_SC_Y_NEG 0.035897435f
#define ACC_SC_Y_POS 0.03828125f
#define ACC_SC_Z_NEG 0.037984496f
#define ACC_SC_Z_POS 0.039516129f


//RC defines
#define DSM2 0
#define DSMX 1
#define SBUS 2
#define RC 3
#define HEX_ZERO 0x00

//control defines
#define LIFTOFF 1175

//using these macros in place of digitalWrite is much faster
//however digitalWrite will work when using SPI 
#define GyroSSOutput() DDRL |= 1<<0 //this is the same as pinMode(49,OUTPUT)
#define GyroSSHigh() PORTL |= 1<<0 //this is like digitalWrite(49,HIGH) but faster
#define GyroSSLow() PORTL &= ~(1<<0)


#define AccSSOutput() DDRL |= 1<<1 //this is the same as pinMode(48,OUTPUT)
#define AccSSHigh() PORTL |= 1<<1 //this is like digitalWrite(48,HIGH) but faster
#define AccSSLow() PORTL &= ~(1<<1)

//motor defines
#define FREQ 400
#define PRESCALE 8
#define PERIOD ((F_CPU/PRESCALE/FREQ) - 1)

#define Motor1WriteMicros(x) OCR3B = x * 2//motor 1 is attached to pin2
#define Motor2WriteMicros(x) OCR3C = x * 2//motor 2 is attached to pin3
#define Motor3WriteMicros(x) OCR3A = x * 2//motor 3 is attached to pin5
#define Motor4WriteMicros(x) OCR4A = x * 2//motor 4 is attached to pin6

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
int16_t rawX,rawY,rawZ;
float smoothAccX,smoothAccY,smoothAccZ;
float accToFilterX,accToFilterY,accToFilterZ;
float dt;

//control related vars
boolean integrate = false;
float motorCommand1,motorCommand2,motorCommand3,motorCommand4;



float pitchAngle;
float rollAngle;
float yawSetPoint;
float rateSetPointX;    
float rateSetPointY;
float rateSetPointZ;
float adjustmentX;
float adjustmentY;
float adjustmentZ; 
float pitchSetPoint,rollSetPoint;


float kp_r_p = 0.75;
float ki_r_p = 0.06;
float kd_r_p = 0.01423;
float nPitch = 19.5924;

float kp_r_r = 0.75;
float ki_r_r = 0.06;
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

uint8_t loopCount;
uint16_t i;//index for buffering in the data
uint16_t j;
uint32_t timer;

openIMU imu(&radianGyroX,&radianGyroY,&radianGyroZ,&accToFilterX,&accToFilterY,&accToFilterZ,&dt);

MPID PitchAngle(&pitchSetPoint,&imu.pitch,&rateSetPointY,&integrate,&kp_a_p,&ki_a_p,&kd_a_p,&nPitchA,&dt,200,300);
MPID RollAngle(&rollSetPoint,&imu.roll,&rateSetPointX,&integrate,&kp_a_r,&ki_a_r,&kd_a_r,&nRollA,&dt,200,300);

MPID PitchRate(&rateSetPointY,&degreeGyroY,&adjustmentY,&integrate,&kp_r_p,&ki_r_p,&kd_r_p,&nPitch,&dt,200,200);
MPID RollRate(&rateSetPointX,&degreeGyroX,&adjustmentX,&integrate,&kp_r_r,&ki_r_r,&kd_r_r,&nRoll,&dt,200,200);
MPID YawRate(&rateSetPointZ,&degreeGyroZ,&adjustmentZ,&integrate,&kp_r_y,&ki_r_y,&kd_r_y,&nYaw,&dt,200,200);


void setup(){
  Serial.begin(115200);
  MotorInit();
  DetectRC();
  if (rcType == RC){
    DDRK = 0;//PORTK as input
    PORTK |= 0xFF;//turn on pull ups
    PCMSK2 |= 0xFF;//set interrupt mask for all of PORTK
    PCICR = 1<<2;//enable the pin change interrupt for K
    Center();
  }  
  Serial.println("RC done");
  
  //arming procedure - Gear switch has to be toggled to arm
  newRC = false;
  while (newRC == false){
    if (rcType != RC){
      FeedLine();
    }
  }

  while (rcCommands.values.gear < 1850){
    if (rcType != RC){
      FeedLine();
    }
  } 
  newRC = false;
  Serial.println("armed");
  

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);  
  
  AccSSOutput();//this was moved from the init
  AccSSHigh();//if high isn't written to both devices befor config 
  GyroSSOutput();//the SPI bus will be addressing both devices 
  GyroSSHigh();
  
  GyroInit();
  AccInit();

  GetGyro();
  GetAcc();
  
  LevelAngles();
  
  Serial.println("cal complete");
  PitchAngle.reset();
  RollAngle.reset();

  PitchRate.reset();
  RollRate.reset();
  YawRate.reset();

  //pulse the motors to indicate that the craft is armed
  Motor1WriteMicros(1175);//set the output compare value
  Motor2WriteMicros(1175);
  Motor3WriteMicros(1175);
  Motor4WriteMicros(1175);  
  delay(500);

  Motor1WriteMicros(1000);//set the output compare value
  Motor2WriteMicros(1000);
  Motor3WriteMicros(1000);
  Motor4WriteMicros(1000); 

  loopCount = 0;

  timer = micros();
}

void loop(){

  if (micros() - timer >= 2500){//attempt to run at 400 hz - reality on the mega is more like 380  
    dt = ((micros() - timer) / 1000000.0);
    timer = micros();
    GetGyro();
    GetAcc();
    imu.IMUupdate();
    imu.GetEuler();
    if (rcCommands.values.aux1 < 1600){
      Angle();

    }
    Rate();
    MotorHandler();
  }
  if (rcType != RC){
    FeedLine();
  }
  if (newRC == true){
    newRC = false;
    ProcessChannels();
    //imu.GetEuler();
    //for debugging purposes
    /*Serial.print(imu.pitch);
    Serial.print(",");
    Serial.print(imu.roll);
    Serial.print(",");
    Serial.print(rcCommands.values.aileron);
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
    Serial.println(rcCommands.values.aux3);*/

  }  


}

void Smoothing(int16_t *raw, float *smooth){
  *smooth = (*raw * (0.15)) + (*smooth * 0.85);
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



















