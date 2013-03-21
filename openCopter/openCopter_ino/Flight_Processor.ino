void Loiter(){
  static float temp;
  t.v.lat = gps.data.vars.lat;
  t.v.lon = gps.data.vars.lon;

  gps.Heading(&gps.data.vars.lat,&gps.data.vars.lon,&t.v.latInit,&t.v.lonInit,&t.v.heading);
  gps.Distance(&gps.data.vars.lat,&gps.data.vars.lon,&t.v.latInit,&t.v.lonInit,&t.v.dist);
  
  t.v.bodyHeading = t.v.heading - imu.yaw;
  
  t.v.rollError = t.v.dist * -1.0 * sin(ToRad(t.v.bodyHeading));
  t.v.pitchError = t.v.dist * cos(ToRad(t.v.bodyHeading));
  
  LoiterRollPosition.calculate();
  LoiterPitchlPosition.calculate();


}

void Rate(){
  PitchRate.calculate();
  RollRate.calculate();
  YawRate.calculate();
}
void Angle(){
  PitchAngle.calculate();
  RollAngle.calculate();  
}

void AltitudeHold(){
  AltHoldPosition.calculate();
}

void HeadingHold(){
  switch (flightState){
  case TAKEOFF:
    rateSetPointZ = yawInput;
    if (rcCommands.values.throttle > LIFTOFF && abs(yawInput) < 1){
      yawSetPoint = imu.yaw;
      flightState = HH_ON;
    }
    if (rcCommands.values.throttle > LIFTOFF && abs(yawInput) > 1){
      flightState = HH_OFF;
    }
    break;
  case HH_ON:
    YawAngle.calculate();
    if (abs(yawInput) > 1){
      flightState = HH_OFF;
    }
    break;
  case HH_OFF:
    rateSetPointZ = yawInput;
    if (abs(yawInput) < 1){
      yawSetPoint = imu.yaw;
      flightState = HH_ON;
    }
    break;
  case LAND:
    yawSetPoint = imu.yaw;
    if (rcCommands.values.throttle > 1100){
      flightState = TAKEOFF;
    }
    break;
  }  
}
