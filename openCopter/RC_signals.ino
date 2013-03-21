void ProcessChannels(){
  if (rcCommands.values.aux1 < 1100){
    MapVar(&rcCommands.values.rudder,&yawInput,1000,2000,-300,300);
  }
  else{
    if (rcCommands.values.aux1 < 1600){
      MapVar(&rcCommands.values.aileron,&t.v.rollSetPoint,1000,2000,-45,45);
      MapVar(&rcCommands.values.elevator,&t.v.pitchSetPoint,1000,2000,-45,45);
      MapVar(&rcCommands.values.rudder,&yawInput,1000,2000,-300,300);
    }
    else{
      MapVar(&rcCommands.values.aileron,&t.v.rollSetPoint,1000,2000,-45,45);
      MapVar(&rcCommands.values.elevator,&t.v.pitchSetPoint,1000,2000,-45,45);
      MapVar(&rcCommands.values.rudder,&rateSetPointZ,1000,2000,-300,300);
      flightState = HH_OFF;
    }
    if (t.v.rollSetPoint < 1 && t.v.rollSetPoint > -1){
      t.v.rollSetPoint = 0;
    }
    if (t.v.pitchSetPoint < 1 && t.v.pitchSetPoint > -1){
      t.v.pitchSetPoint = 0;
    }
  }
  if (rcCommands.values.throttle > LIFTOFF){
    integrate = true;
  }

}


void Center(){

  while (newRC == false){
    delay(1);
  }

  offset = rcCommands.values.aileron - 1500;
}

ISR(PCINT2_vect){
  currentPinState = PINK;
  changeMask = currentPinState ^ lastPinState;
  lastPinState = currentPinState;
  currentTime = micros();
  for(k = 0;k<8;k++){
    if(changeMask & 1<<k){//has there been a change
      if(!(currentPinState & 1<<k)){//is the pin in question logic low?
        timeDifference = currentTime - changeTime[k];//if so then calculate the pulse width
        if (900 < timeDifference && timeDifference < 2200){//check to see if it is a valid length
          rcCommands.standardRCBuffer[k] = (constrain((timeDifference - offset),1080,1920) - 1080) * 1.19 + 1000;
          if (k != 6){ //for DSM2 fail safe - in loss of signal all channels stop except for the throttle
            newRC = true;
          }
          if (k == 6 && ((timeDifference ) < 1025)){//fail safe for futaba / DSMx
            failSafe = true;
          }
        }
      }
      else{//the pin is logic high implying that this is the start of the pulse
        changeTime[k] = currentTime;
      }
    }
  }
}

void FeedLine(){
  switch(rcType){
  case 0:
    DSM2Parser();
    break;
  case 1:
    DSMXParser();
    break;
  case 2:
    SBusParser();
    break;
  }
}

void SBusParser(){
  if (Serial1.available() > 24){
    while(Serial1.available() > 0){
      inByte = Serial1.read();
      switch (readState){
      case 0:
        if (inByte != 0x0f){
          while(Serial1.available() > 0){//read the contents of in buffer this should resync the transmission
            inByte = Serial1.read();
          }
          return;
        }
        else{
          bufferIndex = 0;
          sBusData[bufferIndex] = inByte;
          sBusData[24] = 0xff;
          readState = 1;
        }
        break;
      case 1:
        bufferIndex ++;
        sBusData[bufferIndex] = inByte;
        if (bufferIndex < 24 && Serial1.available() == 0){
          readState = 0;
        }
        if (bufferIndex == 24){
          readState = 0;
          if (sBusData[0]==0x0f && sBusData[24] == 0x00){
            newRC = true;
          }
        }
        break;
      }
    }
  }  

  if (newRC == true){
    //credit to the folks at multiWii for this sBus parsing algorithm
    rcCommands.values.aileron  = constrain(((sBusData[1]|sBusData[2]<< 8) & 0x07FF),352,1695) ;
    rcCommands.values.aileron  = (rcCommands.values.aileron  - 352) * 0.7446 + 1000;
    rcCommands.values.elevator  = constrain(((sBusData[2]>>3|sBusData[3]<<5) & 0x07FF),352,1695);
    rcCommands.values.elevator  = (rcCommands.values.elevator  - 352) * 0.7446 + 1000;
    rcCommands.values.throttle  = constrain(((sBusData[3]>>6|sBusData[4]<<2|sBusData[5]<<10) & 0x07FF),352,1695);
    rcCommands.values.throttle  = (rcCommands.values.throttle  - 352) * 0.7446 + 1000;
    rcCommands.values.rudder  = constrain(((sBusData[5]>>1|sBusData[6]<<7) & 0x07FF),352,1695);
    rcCommands.values.rudder  = (rcCommands.values.rudder  - 352) * 0.7446 + 1000;
    rcCommands.values.gear = constrain(((sBusData[6]>>4|sBusData[7]<<4) & 0x07FF),352,1695);
    rcCommands.values.gear  = (rcCommands.values.gear  - 352) * 0.7446 + 1000;
    rcCommands.values.aux1 = constrain(((sBusData[7]>>7|sBusData[8]<<1|sBusData[9]<<9) & 0x07FF),352,1695);
    rcCommands.values.aux1  = (rcCommands.values.aux1  - 352) * 0.7446 + 1000;
    rcCommands.values.aux2  = constrain(((sBusData[9]>>2|sBusData[10]<<6) & 0x07FF),352,1695);
    rcCommands.values.aux2  = (rcCommands.values.aux2  - 352) * 0.7446 + 1000;
    rcCommands.values.aux3  = constrain(((sBusData[10]>>5|sBusData[11]<<3) & 0x07FF),352,1695);
    rcCommands.values.aux3  = (rcCommands.values.aux3  - 352) * 0.7446 + 1000;
    if (sBusData[23] & (1<<2)) {
      failSafe = true;
    }
    if (sBusData[23] & (1<<3)) {
      failSafe = true;
    }

  }

}

void DSMXParser(){

  if (Serial1.available() > 14){
    while(Serial1.available() > 0){
      inByte = Serial1.read();
      switch (readState){
      case 0:
        if (inByte == HEX_ZERO || inByte == 0x2D || inByte == 0x5A || inByte == 0x87 || inByte == 0xB4 || inByte == 0xE1 || inByte == 0xFF){
          readState = 1;
        }
        break;
      case 1:
        if (inByte == 0xA2){
          readState = 2;
          bufferIndex = 0;
        }
        break;
      case 2:
        if (bufferIndex % 2 == 0){
          rcCommands.buffer[syncArray1[bufferIndex]] = inByte & 0x07;
        }
        else{
          rcCommands.buffer[syncArray1[bufferIndex]] = inByte;
        }
        bufferIndex++;
        if(bufferIndex == 14){
          readState = 0;
          newRC = true;
        }
        break;
      default:
        break;
      }
    }
    rcCommands.values.aileron  = (constrain(rcCommands.values.aileron,306,1738)  - 306) * 0.69832 + 1000;
    rcCommands.values.elevator  = (constrain(rcCommands.values.elevator,306,1738)  - 306) * 0.69832 + 1000;
    rcCommands.values.throttle  = (constrain(rcCommands.values.throttle,306,1738)  - 306) * 0.69832 + 1000;
    rcCommands.values.rudder  = (constrain(rcCommands.values.rudder,306,1738)  - 306) * 0.69832 + 1000;
    rcCommands.values.gear  = (constrain(rcCommands.values.gear,306,1738)  - 306) * 0.69832 + 1000;
    rcCommands.values.aux1  = (constrain(rcCommands.values.aux1,306,1738)  - 306) * 0.69832 + 1000;
    if (rcCommands.values.aux2 != 0){
      rcCommands.values.aux2  = (constrain(rcCommands.values.aux2,306,1738)  - 306) * 0.69832 + 1000;
    }
  }

}
void DSM2Parser(){
  if (Serial1.available() > 14){
    while(Serial1.available() > 0){
      inByte = Serial1.read();
      switch (readState){
      case 0:
        if (inByte == 0x03 || 0x21){
          readState = 1;
        }
        break;
      case 1:
        if (inByte == 0xA2){
          readState = 2;
          bufferIndex = 0;
        }
        if (inByte == 0xB2){
          readState = 3;
          bufferIndex = 0;
        }
        break;
      case 2:
        if (bufferIndex % 2 == 0){
          rcCommands.buffer[syncArray1[bufferIndex]] = (inByte & 0x07) | 0x04;
        }
        else{
          rcCommands.buffer[syncArray1[bufferIndex]] = inByte;
        }
        bufferIndex++;
        if(bufferIndex == 14){
          readState = 0;
          newRC = true;
        }
        break;
      case 3:
        if (bufferIndex % 2 == 0){
          rcCommands.buffer[syncArray2[bufferIndex]] = (inByte & 0x07) | 0x04;
        }
        else{
          rcCommands.buffer[syncArray2[bufferIndex]] = inByte;
        }
        bufferIndex++;
        if(bufferIndex == 14){
          readState = 0;
          newRC = true;
        }
        break;
      default:
        break;

      }
    }
    rcCommands.values.aileron  = (constrain(rcCommands.values.aileron,1177,1893)  - 1177) * 1.3908 + 1003;
    rcCommands.values.elevator  = (constrain(rcCommands.values.elevator,1177,1893)  - 1177) * 1.3908 + 1003;
    rcCommands.values.throttle  = (constrain(rcCommands.values.throttle,1177,1893)  - 1177) * 1.3908 + 1000;
    rcCommands.values.rudder  = (constrain(rcCommands.values.rudder,1177,1893)  - 1177) * 1.3908 + 1003;
    rcCommands.values.gear  = (constrain(rcCommands.values.gear,1177,1893)  - 1177) * 1.3908 + 1000;
    rcCommands.values.aux1  = (constrain(rcCommands.values.aux1,1177,1893)  - 1177) * 1.3908 + 1000;
    rcCommands.values.aux2  = (constrain(rcCommands.values.aux2,1177,1893)  - 1177) * 1.3908 + 1000;
  }
}

void DetectRC(){
  readState = 0;
  Spektrum();
  if (detected == true){
    return;
  }
  readState = 0;
  SBus();
  if (detected == true){
    return;
  }
  else{
    rcType = RC;
  }
  readState = 0;
}

void SBus(){
  Serial1.begin(100000);
  timer = millis();
  while (Serial1.available() == 0){
    if (millis() - timer > 1000){
      return;
    }
  }
  rcType = SBUS;
  detected = true;
}

void Spektrum(){
  Serial1.begin(115200);
  timer = millis();
  while (Serial1.available() == 0){
    if (millis() - timer > 1000){
      return;
    }
  }  
  delay(1);
  while (Serial1.available() > 0){
    inByte = Serial1.read();
    switch(readState){
    case 0:
      if (inByte == 0x03 || 0x21){
        readState = 1;
      }
      if (inByte == HEX_ZERO || inByte == 0x2D || inByte == 0x5A || inByte == 0x87 || inByte == 0xB4 || inByte == 0xE1 || inByte == 0xFF){
        readState = 2;
      }
      break;
    case 1:
      if (inByte == 0xA2 || inByte == 0xB2){
        rcType = DSM2;
        detected = true;
        return;
      }
      else{
        readState = 0;
      }
      break;
    case 2:
      if (inByte == 0xA2){
        rcType = DSMX;
        detected = true;
        return;
      }
      else{
        readState = 0;
      }
      break;
    }

  }
}






