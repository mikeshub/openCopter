#include "UBLOXL.h"

void UBLOX::init(void){
  GPSPort.begin(115200);
  GPSState=0;
  newData = false;
}

void UBLOX::Heading(int32_t *lat1, int32_t *lon1, int32_t *lat2, int32_t *lon2,float *bearing){
  dLon = ToRad((*lon2 - *lon1)/10000000.0);
  lat1_f = ToRad(*lat1/10000000.0);
  lat2_f= ToRad(*lat2/10000000.0);

  y = sin(dLon)*cos(lat2_f);
  x = cos(lat1_f)*sin(lat2_f)-sin(lat1_f)*cos(lat2_f)*cos(dLon);
  *bearing = ToDeg(fastAtan2(y,x));
  if(*bearing < 0){
    *bearing += 360;
  }

}

void UBLOX::Distance(int32_t *lat1, int32_t *lon1, int32_t *lat2, int32_t *lon2,float *dist){
  dLon = ToRad((*lon2 - *lon1)/10000000.0);
  dLat = ToRad((*lat2 - *lat1)/10000000.0);
  lat2_f = ToRad((*lat2)/10000000.0);
  x = dLon*cos(lat2_f);
  *dist = sqrt(x*x + dLat*dLat)*6372795;
}

void UBLOX::Monitor(){
    while (GPSPort.available() > 0){
      switch (GPSState){
      case 0:
        inByte = GPSPort.read();
        if (inByte == 0xB5){
          GPSState = 1;
        }
        break;
      case 1:
        inByte = GPSPort.read();
        if (inByte == 0x62){
          GPSState = 2;
        }
        else{
          GPSState = 0;
        }
        break;
      case 2:
        inByte = GPSPort.read();
        if (inByte == 0x01){
          GPSState = 3;
          lengthIndex = 0;
          dataIndex = 0;
        }
        else{
          GPSState = 0;
        }
        break;
      case 3:
        inByte = GPSPort.read();
        if (inByte == 0x02){
          GPSState = 4;
        }
        else{
          if (inByte == 0x12){
            GPSState = 5;
          }
          else{
            if(inByte == 0x03){
              GPSState = 6;
            }
            else{
              GPSState = 0;
            }
          }
        }
        break;
      case 4://POSLLH
        if (lengthIndex < 2 && dataIndex == 0){
          inByte = GPSPort.read();
          lengthIndex++;
          break;
        }
        if (lengthIndex == 2 && dataIndex < 28){
          inByte = GPSPort.read();
          data.buffer[dataIndex] = inByte;
          dataIndex++;
          break;
        }
        if (lengthIndex == 2 && dataIndex == 28){
          GPSState = 0;
          newData = true;
          break;
        }

        break;
      case 5://VELNED
        //Serial.println("VELNED");
        if (lengthIndex < 2 && dataIndex == 0){
          inByte = GPSPort.read();
          lengthIndex++;
          break;
        }
        if (lengthIndex == 2 && dataIndex < 4){
          inByte = GPSPort.read();
          data.buffer[dataIndex] = inByte;
          dataIndex++;
          if (dataIndex == 4){
            dataIndex = 28;
          }
          break;
        }
        if (lengthIndex == 2 && dataIndex < 28){
          inByte = GPSPort.read();
          data.buffer[dataIndex] = inByte;
          dataIndex++;
          break;
        }
        if (lengthIndex == 2 && dataIndex == 60){
          GPSState = 0;
          newData = true;
          break;
        }

        break;
      case 6://
        if (lengthIndex < 2 && dataIndex == 0){
          inByte = GPSPort.read();
          lengthIndex++;
          break;
        }
        if (lengthIndex == 2 && dataIndex < 4){
          inByte = GPSPort.read();
          data.buffer[dataIndex] = inByte;
          dataIndex++;
          break;
        }
        if (lengthIndex == 2 && dataIndex == 4){
          inByte = GPSPort.read();
          data.vars.gpsFix = inByte;
          dataIndex++;
          break;
        }
        if (lengthIndex == 2 && dataIndex == 5){
          GPSState = 0;
          break;
        }

        break;
      }
    }

}


float UBLOX::fastAtan2( float y, float x)
{
  static float atan;
  static float z;
  if ( x == 0.0f )
  {
    if ( y > 0.0f ) return PIBY2_FLOAT;
    if ( y == 0.0f ) return 0.0f;
    return -PIBY2_FLOAT;
  }
  //atan;
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








