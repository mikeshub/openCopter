// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//This code is provided under the GNU General Public Licence
//12/19/2012
//Gyro must be in RAD/s
//Sensors must be in the North East Down convention
#ifndef openIMUL_h
#define openIMUL_h

#include <Arduino.h>

#define ToRad(x) ((x) * 0.01745329252)  // *pi/180
#define ToDeg(x) ((x) * 57.2957795131)  // *180/pi
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f

#define betaDef1 0.10f//gain for the IMU
#define betaDef2 0.30f//gain for the AHRS

#define v_ 2.0f//gains for the altimeter
#define w_ 0.10f



class openIMU{
public:
  openIMU(float*, float*, float*, float*, float*, float*, float*, float*, float*, float*, float*, float*, float*, float*);
  openIMU(float*, float*, float*, float*, float*, float*, float*, float*, float*, float*);
  openIMU(float*, float*, float*, float*, float*, float*, float*);
  void IMUupdate(void);
  void AHRSupdate(void);
  void GetEuler(void);
  void InitialQuat(void);
  void BaroKalUpdate(void);
  void AccKalUpdate(void);
  void Rotate(void);
  float q0,q1,q2,q3;
  float pitch,roll,yaw,altitude;
  float pitchOffset,rollOffset;
private:
  float *gx;
  float *gy;
  float *gz;
  float *ax;
  float *ay;
  float *az;
  float *mx;
  float *my;
  float *mz;
  float *dt;
  float *sax;//scaled accelerometer value
  float *say;
  float *saz;
  float *altRaw;

  //IMU & AHRS vars
  float squareSum;
  float magnitude;
  float _4q0, _4q1, _4q2, _8q1, _8q2;
  float beta1,beta2;
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  //Altimeter vars
  float inertialZ;
  float altEst,velZ;
  float p11;
  float p12;
  float p21;
  float p22;
  float k1,k2;

  float FastAtan2( float , float );
  float InvSqrt(float );
};


#endif


