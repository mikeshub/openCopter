// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//This code is provided under the GNU General Public Licence
//12/19/2012
#include "openIMUL.h"

openIMU::openIMU(float *gyroX, float *gyroY, float *gyroZ, float *accX, float *accY, float *accZ, float *scAccX, float *scAccY,float *scAccZ, float *magX, float *magY, float *magZ, float *rawAlt ,float *G_Dt){
  //constructor for the 10DOF system
  gx = gyroX;
  gy = gyroY;
  gz = gyroZ;

  ax = accX;
  ay = accY;
  az = accZ;

  mx = magX;
  my = magY;
  mz = magZ;

  sax = scAccX;
  say = scAccY;
  saz = scAccZ;

  altRaw = rawAlt;

  dt = G_Dt;
  //set the feedback gain and initial quaternion
  beta1 = betaDef1;
  beta2 = betaDef2;

  pitchOffset = 0;
  rollOffset = 0;

  q0 = 1;
  q1 = 0;
  q2 = 0;
  q3 = 0;
  //Kalman stuff
  altEst = 0;
  velZ = 0;
  p11 = 0.01;
  p12 = 0;
  p21 = 0;
  p22 = 0.01;
}

openIMU::openIMU(float *gyroX, float *gyroY, float *gyroZ, float *accX, float *accY, float *accZ, float *magX, float *magY, float *magZ, float *G_Dt){
  //constructor for the full AHRS
  //assign the pointers
  gx = gyroX;
  gy = gyroY;
  gz = gyroZ;

  ax = accX;
  ay = accY;
  az = accZ;

  mx = magX;
  my = magY;
  mz = magZ;

  dt = G_Dt;
  //set the feedback gain and initial quaternion
  beta1 = betaDef1;
  beta2 = betaDef2;

  pitchOffset = 0;
  rollOffset = 0;

  q0 = 1;
  q1 = 0;
  q2 = 0;
  q3 = 0;
}

openIMU::openIMU(float *gyroX, float *gyroY, float *gyroZ, float *accX, float *accY, float *accZ, float *G_Dt){
  //constructor for the IMU
  gx = gyroX;
  gy = gyroY;
  gz = gyroZ;

  ax = accX;
  ay = accY;
  az = accZ;

  dt = G_Dt;

  beta1 = betaDef1;

  pitchOffset = 0;
  rollOffset = 0;

  q0 = 1;
  q1 = 0;
  q2 = 0;
  q3 = 0;
}

void openIMU::BaroKalUpdate(){
  static float temp;
  temp = p11 + v_;
  k1 = p11 / temp;
  k2 = p21 / temp;
  velZ += k2 * (*altRaw - altEst);
  altEst += k1 * (*altRaw - altEst);
  p22 = p22 - k2 * p12;//run P21 and P22 first because they depend on the pervious values for p11 and p21
  p21 = p21 - k2 * p11;
  p11 = -p11*(k1 - 1);
  p21 = -p12*(k1 - 1);

  altitude = altEst;
}
void openIMU::AccKalUpdate(void){
  //first rotate the accelerometer reading from the body to the intertial frame
  static float q0q0,q1q1,q2q2,q3q3;
  static float q0q2x2,q1q3x2,q0q1x2,q2q3x2;

  q0q0 = q0 * q0;
  q1q1 = q1 * q1;
  q2q2 = q2 * q2;
  q3q3 = q3 * q3;

  q0q2x2 = q0 * q2 * 2;
  q1q3x2 = q1 * q3 * 2;
  q0q1x2 = q0 * q1 * 2;
  q2q3x2 = q2 * q3 * 2;
  //remember gravity is sensed with the sign negated - that is why it is added into the equation instead of subtracted
  inertialZ = (*saz * (q0q0 - q1q1 - q2q2 + q3q3) - *sax * (q0q2x2 - q1q3x2) + *say * (q0q1x2 + q2q3x2)) + 9.8;
  //Kalman filter stuff - makes more sense looking at it in matrix form
  velZ += inertialZ * *dt;
  altEst += velZ * *dt;
  p11 = p11 + *dt * p21 + *dt * w_ + *dt * (p12 + *dt * p22);
  p12 = p12 + *dt * p22;
  p21 = p21 + *dt * p22;
  p22 = p22 + *dt * w_;

  altitude = altEst;
}

void openIMU::Rotate(){

}


void openIMU::InitialQuat(){
  //this function does not have to be called
  //first calculate the pitch and roll from the accelerometer
  pitch = ToDeg(FastAtan2(*ax , sqrt(*ay * *ay + *az * *az)));
  roll = ToDeg(FastAtan2(-1 * *ay ,sqrt(*ax  * *ax  + *az * *az)));


  if (*az > 0){
    if (*ax> 0){
      pitch = 180.0 - pitch;
    }
    else{
      pitch = -180.0 - pitch;
    }
    if (*ay > 0){
      roll = -180.0 - roll;
    }
    else{
      roll = 180.0 - roll;
    }
  }


  //tilt compensate the compass
  float xMag = (*mx * cos(ToRad(pitch))) + (*mz * sin(ToRad(pitch)));
  float yMag = -1 * ((*mx * sin(ToRad(roll))  * sin(ToRad(pitch))) + (*my * cos(ToRad(roll))) - (*mz * sin(ToRad(roll)) * cos(ToRad(pitch))));
  //calculate the yaw from the tilt compensated compass
  yaw = ToDeg(FastAtan2(yMag,xMag));

  if (yaw < 0){
    yaw += 360;
  }
  //calculate the rotation matrix
  float cosPitch = cos(ToRad(pitch));
  float sinPitch = sin(ToRad(pitch));

  float cosRoll = cos(ToRad(roll));
  float sinRoll = sin(ToRad(roll));

  float cosYaw = cos(ToRad(yaw));
  float sinYaw = sin(ToRad(yaw));

  //need the transpose of the rotation matrix
  float r11 = cosPitch * cosYaw;
  float r21 = cosPitch * sinYaw;
  float r31 = -1.0 * sinPitch;

  float r12 = -1.0 * (cosRoll * sinYaw) + (sinRoll * sinPitch * cosYaw);
  float r22 = (cosRoll * cosYaw) + (sinRoll * sinPitch * sinYaw);
  float r32 = sinRoll * cosPitch;

  float r13 = (sinRoll * sinYaw) + (cosRoll * sinPitch * cosYaw);
  float r23 = -1.0 * (sinRoll * cosYaw) + (cosRoll * sinPitch * sinYaw);
  float r33 = cosRoll * cosPitch;


  //convert to quaternion
  q0 = 0.5 * sqrt(1 + r11 + r22 + r33);
  q1 = (r32 - r23)/(4 * q0);
  q2 = (r13 - r31)/(4 * q0);
  q3 = (r21 - r12)/(4 * q0);
}

void openIMU::IMUupdate() {
  //this function handles pitch and roll

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * *gx - q2 * *gy - q3 * *gz);
  qDot2 = 0.5f * (q0 * *gx + q2 * *gz - q3 * *gy);
  qDot3 = 0.5f * (q0 * *gy - q1 * *gz + q3 * *gx);
  qDot4 = 0.5f * (q0 * *gz + q1 * *gy - q2 * *gx);

  squareSum = *ax * *ax + *ay * *ay + *az * *az;

  magnitude = sqrt(squareSum);

  //make sure that the total sum of gravity is 1 +/- .1gs
  //this filter works based on the assumption that only gravity is being detected
  //if gravity is not roughly 256 according to the devices datasheet
  //the below line must be fixed
  if ((magnitude < 281) && (magnitude > 231)){
    // Normalise accelerometer measurement
    recipNorm = InvSqrt(squareSum);
    *ax *= recipNorm;
    *ay *= recipNorm;
    *az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * *ax + _4q0 * q1q1 - _2q1 * *ay;
    s1 = _4q1 * q3q3 - _2q3 * *ax + 4.0f * q0q0 * q1 - _2q0 * *ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * *az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * *ax + _4q2 * q3q3 - _2q3 * *ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * *az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * *ax + 4.0f * q2q2 * q3 - _2q2 * *ay;
    recipNorm = InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta1 * s0;
    qDot2 -= beta1 * s1;
    qDot3 -= beta1 * s2;
    qDot4 -= beta1 * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * *dt;
  q1 += qDot2 * *dt;
  q2 += qDot3 * *dt;
  q3 += qDot4 * *dt;

  // Normalise quaternion
  recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void openIMU::AHRSupdate() {

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * *gx - q2 * *gy - q3 * *gz);
  qDot2 = 0.5f * (q0 * *gx + q2 * *gz - q3 * *gy);
  qDot3 = 0.5f * (q0 * *gy - q1 * *gz + q3 * *gx);
  qDot4 = 0.5f * (q0 * *gz + q1 * *gy - q2 * *gx);

  squareSum = *ax * *ax + *ay * *ay + *az * *az;

  magnitude = sqrt(squareSum);
  //if gravity is not roughly 256 according to the devices datasheet
  //the below line must be fixed
  if ((magnitude < 281) && (magnitude > 231)){
    // Normalise accelerometer measurement
    recipNorm = InvSqrt(magnitude);
    *ax *= recipNorm;
    *ay *= recipNorm;
    *az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = InvSqrt(*mx * *mx + *my * *my + *mz * *mz);
    *mx *= recipNorm;
    *my *= recipNorm;
    *mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * *mx;
    _2q0my = 2.0f * q0 * *my;
    _2q0mz = 2.0f * q0 * *mz;
    _2q1mx = 2.0f * q1 * *mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = *mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + *mx * q1q1 + _2q1 * *my * q2 + _2q1 * *mz * q3 - *mx * q2q2 - *mx * q3q3;
    hy = _2q0mx * q3 + *my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - *my * q1q1 + *my * q2q2 + _2q2 * *mz * q3 - *my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + *mz * q0q0 + _2q1mx * q3 - *mz * q1q1 + _2q2 * *my * q3 - *mz * q2q2 + *mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - *ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - *ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - *mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - *my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - *mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - *ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - *ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - *az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - *mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - *my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - *mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - *ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - *ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - *az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - *mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - *my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - *mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - *ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - *ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - *mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - *my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - *mz);
    recipNorm = InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta2 * s0;
    qDot2 -= beta2 * s1;
    qDot3 -= beta2 * s2;
    qDot4 -= beta2 * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * *dt;
  q1 += qDot2 * *dt;
  q2 += qDot3 * *dt;
  q3 += qDot4 * *dt;

  // Normalise quaternion
  recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void openIMU::GetEuler(void){
  //this function provied the Euler angles
  //Euler angles are easier to incorporte into PID control than the quaternion

  roll= ToDeg(FastAtan2(2 * (q0 * q1 + q2 * q3),1 - 2 * (q1 * q1 + q2 * q2))) - rollOffset;
  pitch = ToDeg(asin(2 * (q0 * q2 - q3 * q1))) - pitchOffset;
  yaw = ToDeg(FastAtan2(2 * (q0 * q3 + q1 * q2) , 1 - 2* (q2 * q2 + q3 * q3)));

  if (yaw < 0){
    yaw +=360;
  }


}


//auxilliary functions
float openIMU::FastAtan2( float y, float x)
{
  static float atan;
  static float z;
  if ( x == 0.0f )
  {
    if ( y > 0.0f ) return PIBY2_FLOAT;
    if ( y == 0.0f ) return 0.0f;
    return -PIBY2_FLOAT;
  }
  z = y / x;
  if ( fabs( z ) < 1.0f )
  {
    atan = z/(1.0f + 0.28f*z*z);
    if ( x < 0.0f )
    {
      if ( y < 0.0f ) return atan - PI_FLOAT;
      return atan + PI_FLOAT;
    }
  }
  else
  {
    atan = PIBY2_FLOAT - z/(z*z + 0.28f);
    if ( y < 0.0f ) return atan - PI_FLOAT;
  }
  return atan;
}

float openIMU::InvSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
}



