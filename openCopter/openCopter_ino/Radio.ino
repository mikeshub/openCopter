void Transmit(){
  if (millis() - transmitTimer >= TRANSMIT_TIME){
    transmitTimer = millis();
    t.v.timestamp = transmitTimer;
    radio.write(t.buffer,53);
    radio.write(0x55);
    radio.write(0xAA); 
  }

}
void Gains(){
  while (radio.available() > 0 ){
    radioInByte = radio.read();
    if (radioInByte == 0xAA){
      inGainBufferIndex = 0;
    }
    inGainBuffer[inGainBufferIndex] = radioInByte;
    inGainBufferIndex++;
    if(inGainBufferIndex == 7){
      ParseBuffer();
      inGainBufferIndex = 0;
    }
  }
}

void ParseBuffer(){
  if (inGainBuffer[0] != 0xAA && inGainBuffer[6] != 0x55){
    radio.flush();
    return;
  }
  switch (inGainBuffer[1]){
  case 1:
    gainBufferIndex = 0;
    for (inGainBufferIndex = 2; inGainBufferIndex < 6; inGainBufferIndex++){
      g.buffer[gainBufferIndex]=inGainBuffer[inGainBufferIndex];
      gainBufferIndex++;
    }
    break;
  case 2:
    gainBufferIndex = 4;
    for (inGainBufferIndex = 2; inGainBufferIndex < 6; inGainBufferIndex++){
      g.buffer[gainBufferIndex]=inGainBuffer[inGainBufferIndex];
      gainBufferIndex++;
    }
    break;
  case 3:
    gainBufferIndex = 8;
    for (inGainBufferIndex = 2; inGainBufferIndex < 6; inGainBufferIndex++){
      g.buffer[gainBufferIndex]=inGainBuffer[inGainBufferIndex];
      gainBufferIndex++;
    }
    break;
  case 4:
    gainBufferIndex = 12;
    for (inGainBufferIndex = 2; inGainBufferIndex < 6; inGainBufferIndex++){
      g.buffer[gainBufferIndex]=inGainBuffer[inGainBufferIndex];
      gainBufferIndex++;
    }
    break;   

  default:
    break;
  }
  TransmitGains();
}


void TransmitGains(){
  radio.println("Gians:");
  radio.println("Position:");
  radio.print("kp_loiter: ");
  radio.println(g.v.kp_loiter,4);
  radio.print("ki_loiter: ");
  radio.println(g.v.ki_loiter,4);
  radio.print("kd_loiter: ");
  radio.println(g.v.kd_loiter,4);
  radio.print("n_loiter: ");
  radio.println(g.v.n_loiter,4);

}









