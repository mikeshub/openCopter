#include "MPIDL.h"

MPID::MPID(float *set, float *act, float *adj,boolean *intToggle,float *p, float *i, float *d,float *n,float *delta,float iLim,float lim){
  setPoint = set;
  actual = act;
  adjustment = adj;
  integrate = intToggle;
  kp = p;
  ki = i;
  kd = d;
  fc = n;
  integralLimitHigh = iLim;
  integralLimitLow = -1*iLim;
  outputLimitHigh = lim;
  outputLimitLow = -1*lim;
  dt = delta;
  prevActual = 0;
  dErrorPrev = 0;

}

void MPID::calculate(){
  error = *setPoint - *actual;


  if (*integrate == true){
    iError += *ki * *dt * error;
  }
  if (iError > integralLimitHigh){
    iError = integralLimitHigh;
  }
  if (iError < integralLimitLow){
    iError = integralLimitLow;
  }

  dError = dErrorPrev - *fc * *dt * dErrorPrev + *kd * *fc * (error - prevError);

  *adjustment = *kp * error  + iError +  dError;

  if (*adjustment > outputLimitHigh){
    *adjustment  = outputLimitHigh;
  }
  if (*adjustment < outputLimitLow){
    *adjustment = outputLimitLow;
  }

  prevError = error;
  dErrorPrev = dError;
}

void MPID::reset(){
  error = 0;
  iError = 0;
  dError = 0;
  *adjustment = 0;
  prevActual = *actual;
  prevError =0;
  dErrorPrev = 0;
}

MYAW::MYAW(float *set, float *act, float *adj,boolean *intToggle,float *p, float *i, float *d,float *n,float *delta,float iLim,float lim){
  setPoint = set;
  actual = act;
  adjustment = adj;
  integrate = intToggle;
  kp = p;
  ki = i;
  kd = d;
  fc = n;
  integralLimitHigh = iLim;
  integralLimitLow = -1*iLim;
  outputLimitHigh = lim;
  outputLimitLow = -1*lim;
  dt = delta;
  prevActual = 0;
  prevError =0;
  singularityState =0;
  dErrorPrev = 0;

}

void MYAW::calculate(){
  PIDAngle = *actual;

  error = *setPoint - PIDAngle;

  errorDiff = prevError - error;

  if (errorDiff > 180.0){
    //singularityState = 1;
    PIDAngle = *actual -360;
    error = *setPoint - PIDAngle;
  }
  if (errorDiff < -180.0){
    //singularityState = 2;
    PIDAngle = *actual  +360;
    error = *setPoint - PIDAngle;
  }




  //dError = (error - prevError) / *dt;
  dError = dErrorPrev - *fc * *dt * dErrorPrev + *kd * *fc * (error - prevError);
  if (*integrate == true){
    iError += *ki * *dt * error;
  }


  if (iError > integralLimitHigh){
    iError = integralLimitHigh;
  }
  if (iError < integralLimitLow){
    iError = integralLimitLow;
  }




  *adjustment = *kp * error + iError +  dError;

  if (*adjustment > outputLimitHigh){
    *adjustment  = outputLimitHigh;
  }
  if (*adjustment < outputLimitLow){
    *adjustment = outputLimitLow;
  }
  prevActual = *actual;
  prevError = error;
  dErrorPrev = dError;

}

void MYAW::reset(){
  error = 0;
  prevError=0;
  iError = 0;
  dError = 0;
  *adjustment = 0;
  prevActual = *actual;
  dErrorPrev = 0;
}

