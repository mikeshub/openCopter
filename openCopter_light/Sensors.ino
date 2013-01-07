void LevelAngles(){
  timer = micros();
  delay(5);
  for( int i = 0; i < 1000; i++){//run the IMU so that the error can be driven to zero - keep it still for this
    dt = ((micros() - timer) / 1000000.0);
    timer = micros();
    GetGyro();
    GetAcc();

    imu.IMUupdate();
    delay(5);
  }
  imu.GetEuler();
  imu.pitchOffset = imu.pitch;
  imu.rollOffset = imu.roll;
}



void AccInit(){
  SPI.setDataMode(SPI_MODE3);

  AccSSLow();
  SPI.transfer(WRITE | SINGLE | BW_RATE);
  SPI.transfer(0x0C);//400hz
  AccSSHigh();

  AccSSLow();
  SPI.transfer(WRITE | SINGLE | POWER_CTL);
  SPI.transfer(0x08);//start measurment
  AccSSHigh();

  AccSSLow();
  SPI.transfer(WRITE | SINGLE | DATA_FORMAT);
  SPI.transfer(0x0B);//full resolution + / - 16g
  AccSSHigh();

  for(j = 0; j < 300; j++){
    GetAcc();//to get the smoothing filters caugt up
    delay(5);
  }
}

void GyroInit(){
  SPI.setDataMode(SPI_MODE0);
  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG2 | WRITE | SINGLE);
  SPI.transfer(0x00); //high pass filter disabled
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG3 | WRITE | SINGLE);
  SPI.transfer(0x00); //not using interrupts
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG4 | WRITE | SINGLE);
  SPI.transfer(0x20); //2000dps scale
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG5 | WRITE | SINGLE);
  SPI.transfer(0x02); //out select to use the second LPF
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG1 | WRITE | SINGLE);
  SPI.transfer(0xCF); //fastest update rate 30Hz cutoff
  GyroSSHigh();
  //this section takes an average of 500 samples to calculate the offset
  //if this step is skipped the IMU will still work, but this simple step gives better results
  offsetX = 0;
  offsetY = 0;
  offsetZ = 0;
  for (j = 0; j < 500; j ++){//give the internal LPF time to warm up
    GetGyro();
    delay(3);
  }
  for (j = 0; j < 500; j ++){//give the internal LPF time to warm up
    GetGyro();
    gyroSumX += gyro.v.x;
    gyroSumY += gyro.v.y;
    gyroSumZ += gyro.v.z;
    delay(3);
  }
  offsetX = gyroSumX / 500.0;
  offsetY = gyroSumY / 500.0;
  offsetZ = gyroSumZ / 500.0;
  
}

void GetGyro(){
  SPI.setDataMode(SPI_MODE0);
  GyroSSLow();
  SPI.transfer(L3G_OUT_X_L  | READ | MULTI);
  for (i = 0; i < 6; i++){//the endianness matches as does the axis order
    gyro.buffer[i] = SPI.transfer(0x00);
  }
  GyroSSHigh();
  //don't forget to convert to radians per second. This absolutely will not work otherwise
  //check the data sheet for more info on this
  degreeGyroX = (gyro.v.x - offsetX) * 0.07;
  degreeGyroY = (gyro.v.y - offsetY) * 0.07;
  degreeGyroZ = (gyro.v.z - offsetZ) * 0.07;
  radianGyroX = ToRad(degreeGyroX);
  radianGyroY = ToRad(degreeGyroY);
  radianGyroZ = ToRad(degreeGyroZ);
}

void GetAcc(){
  SPI.setDataMode(SPI_MODE3);
  AccSSLow();
  SPI.transfer(DATAX0 | READ | MULTI);
  for (i = 0; i < 6; i++){//the endianness matches as does the axis order
    acc.buffer[i] = SPI.transfer(0x00);
  }
  AccSSHigh();  

  //the filter expects gravity to be in NED coordinates
  //switching the sign will fix this
  //the order and signs have been switched due to the accelerometer being mounted off by 90 degrees
  //one must be careful to make sure that all of the sensors are in the North, East, Down convention
  rawX = acc.v.y;
  Smoothing(&rawX,&smoothAccX);//this is a very simple low pass digital filter
  rawY = acc.v.x * -1;
  Smoothing(&rawY,&smoothAccY);//it helps significiantlly with vibrations. 
  rawZ = acc.v.z;
  Smoothing(&rawZ,&smoothAccZ);//if the applicaion is not prone to vibrations this can skipped and the raw value simply recast as a float
  accToFilterX = -1.0 * smoothAccX;//if the value from the smoothing filter is sent it will not work when the algorithm normalizes the vector
  accToFilterY = -1.0 * smoothAccY;
  accToFilterZ = -1.0 * smoothAccZ;


}



