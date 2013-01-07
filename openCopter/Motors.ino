void MotorInit(){
  DDRE |= B00111000;//set the ports as outputs
  DDRH |= B00001000;

  
  // Init PWM Timer 3                                       
  // WGMn1 WGMn2 WGMn3  = Mode 14 Fast PWM, TOP = ICRn ,Update of OCRnx at BOTOM
  TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  // Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mode)
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);                 // Prescaler set to 8, that gives us a resolution of 0.5us
  ICR3 = PERIOD;                                // Clock_speed / ( Prescaler * desired_PWM_Frequency) #defined above.  

  TCCR4A = (1<<WGM41)|(1<<COM4A1);
  TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
  ICR4 = PERIOD;
  
  Motor1WriteMicros(1000);//set the output compare value
  Motor2WriteMicros(1000);
  Motor3WriteMicros(1000);
  Motor4WriteMicros(1000);
  
}

void MotorHandler(){
  if (rcCommands.values.throttle < 1100){
    integrate = false;
    flightState = LAND;

    PitchAngle.reset();
    RollAngle.reset();
    YawAngle.reset();

    PitchRate.reset();
    RollRate.reset();
    YawRate.reset();

    AltHoldPosition.reset();
    //AltHoldRate.reset();

    LoiterRollPosition.reset();
    LoiterPitchlPosition.reset();

    Motor1WriteMicros(1000);
    Motor2WriteMicros(1000);
    Motor3WriteMicros(1000);
    Motor4WriteMicros(1000); 
  }
  else{
    motorCommand1 = constrain((rcCommands.values.throttle + throttleAdjustment + adjustmentX + adjustmentY - adjustmentZ),1000,2000);
    motorCommand2 = constrain((rcCommands.values.throttle + throttleAdjustment - adjustmentX + adjustmentY + adjustmentZ),1000,2000);
    motorCommand3 = constrain((rcCommands.values.throttle + throttleAdjustment - adjustmentX - adjustmentY - adjustmentZ),1000,2000);
    motorCommand4 = constrain((rcCommands.values.throttle + throttleAdjustment + adjustmentX - adjustmentY + adjustmentZ),1000,2000);
    
    Motor1WriteMicros(motorCommand1);
    Motor2WriteMicros(motorCommand2);
    Motor3WriteMicros(motorCommand3);
    Motor4WriteMicros(motorCommand4);

  }

}


