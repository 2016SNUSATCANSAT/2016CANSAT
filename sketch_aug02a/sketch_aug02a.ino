
#include "GPS.h"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>

  
#define ADDRESS 0x20

#define WHO_AM_I 0x01
#define REV_ID_MAJOR 0x02
#define REV_ID_MINOR 0x03
#define STATUS 0x04
#define I_ACC_X_LOW 0x10
#define I_ACC_X_HIGH 0x11
#define I_ACC_Y_LOW 0x12
#define I_ACC_Y_HIGH 0x13
#define I_ACC_Z_LOW 0x14
#define I_ACC_Z_HIGH 0x15
#define I_GYRO_X_LOW 0x16
#define I_GYRO_X_HIGH 0x17
#define I_GYRO_Y_LOW 0x18
#define I_GYRO_Y_HIGH 0x19
#define I_GYRO_Z_LOW 0x1A
#define I_GYRO_Z_HIGH 0x1B
#define I_MAGNET_X_LOW 0x1C
#define I_MAGNET_X_HIGH 0x1D
#define I_MAGNET_Y_LOW 0x1E
#define I_MAGNET_Y_HIGH 0x1F
#define I_MAGNET_Z_LOW 0x20
#define I_MAGNET_Z_HIGH 0x21
#define C_ACC_X_LOW 0x22
#define C_ACC_X_HIGH 0x23
#define C_ACC_Y_LOW 0x24
#define C_ACC_Y_HIGH 0x25
#define C_ACC_Z_LOW 0x26
#define C_ACC_Z_HIGH 0x27
#define C_GYRO_X_LOW 0x28
#define C_GYRO_X_HIGH 0x29
#define C_GYRO_Y_LOW 0x2A
#define C_GYRO_Y_HIGH 0x2B
#define C_GYRO_Z_LOW 0x2C
#define C_GYRO_Z_HIGH 0x2D
#define C_MAGNET_X_LOW 0x2E
#define C_MAGNET_X_HIGH 0x2F
#define C_MAGNET_Y_LOW 0x30
#define C_MAGNET_Y_HIGH 0x31
#define C_MAGNET_Z_LOW 0x32
#define C_MAGNET_Z_HIGH 0x33
#define TEMPERATURE_LOW 0x34
#define TEMPERATURE_HIGH 0x35
#define ROLL_LOW 0x36
#define ROLL_HIGH 0x37
#define PITCH_LOW 0x38
#define PITCH_HIGH 0x39
#define YAW_LOW 0x3A
#define YAW_HIGH 0x3B
#define QUATERNION_X_LOW 0x3C
#define QUATERNION_X_HIGH 0x3D
#define QUATERNION_Y_LOW 0x3E
#define QUATERNION_Y_HIGH 0x3F
#define QUATERNION_Z_LOW 0x40
#define QUATERNION_Z_HIGH 0x41
#define QUATERNION_W_LOW 0x42
#define QUATERNION_W_HIGH 0x43

   double status;
   double who_am_i;
   double Status;

   double roll;
   double rollL;
   double rollH;

   double pitch;
   double pitchL;
   double pitchH;

   double yaw;
   double yawL;
   double yawH;

   static char atcommand[32];
   const int chipSelect = 53;
   GPS gpsEx;

   // control variable
   double r0 = 15;
   double x, y, z;
   double r;
   double a, b, c; // 목표지점
   double yaw_c;
   double pitch_c;
   double theta;
   double theta_abs;
   double theta_sign;
   int V[8] = {13, 55, 75, 97, 55, 75, 97, 97};

   Servo bldc;
   Servo servo_left;
   Servo servo_right;
   
  int servo_l = 0, servo_r = 0;  
   
   double phi;

uint8_t readRegister8(uint8_t Reg, double ft){
   uint8_t value;
  Wire.beginTransmission(ADDRESS);
   
   #if ARDUINO >= 100
      Wire.write(Reg);
   #else
      Wire.send(Reg);
   #endif

   Wire.endTransmission(false);

   Wire.beginTransmission(ADDRESS);
   Wire.requestFrom(ADDRESS, 1);
   if(Wire.available() == false){ //stuck
      Serial.print("error");
   }

   #if ARDUINO >= 100
      value = Wire.read();

   #else
      value = Wire.receive();
   #endif

   Wire.endTransmission();

   return value;
}

void setup(){
  Serial.begin(9600);
  Serial1.begin(115200);
  Serial2.begin(9600);
  Wire.begin();
    Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  Wire.begin();

  servo_left.attach(3);
  servo_right.attach(5);
  bldc.attach(7);
}

void hex2char(void *src, int bytes, char* dst)
{
  char hex[17] = "0123456789ABCDEF";
  char *data = (char *)src;
  
  for (int i = 0; i < bytes; i++)
  {
    dst[i * 2]  = hex[(data[i] >> 4) & 0x0F];
    dst[i * 2 + 1] = hex[data[i] & 0x0F];
  }
  
  dst[bytes * 2] = 0;
}

void loop(){
  char startBytes[4] = { 0x04, 0xC7, 0x31, 0xC1 };
  char c;
  char buf[4];
  char strbuf[10];
  uint16_t tmp;

  
  //strcpy(atcommand, "at+unicast=dffd, hi");
  //Serial3.print(atcommand);
  //Serial3.write('\r');
  gpsEx.renew();
  double lat = gpsEx.getLat();
  double lng = gpsEx.getLng();
  double hgt = gpsEx.getHgt();
  String LAT = gpsEx.getSLat();
  String LNG = gpsEx.getSLng();
  String HGT = gpsEx.getSHgt();
  int a=lat/100;
  lat-=a*100;
  lat=a+(lat/60);
  int n=lng/100;
  lng-=n*100;
  lng=n+(lng/60);
  int g=hgt/100;
  hgt-=g*100;
  hgt=g+(lat/60);

   rollL = readRegister8(ROLL_LOW, rollL);
   rollH = readRegister8(ROLL_HIGH, rollH);

   roll = (rollL+(256*rollH))*180/32767;

   pitchL = readRegister8(PITCH_LOW, pitchL);
   pitchH = readRegister8(PITCH_HIGH, pitchH);

   pitch = (pitchL+(256*pitchH))*180/32767;

   yawL = readRegister8(YAW_LOW, yawL);
   yawH = readRegister8(YAW_HIGH, yawH);

   yaw = (yawL+(256*yawH))*180/32767;


  File dataFile = SD.open("one.txt", FILE_WRITE);
  if (dataFile) {
      dataFile.print(lat,6);
      dataFile.print(" ");
      dataFile.print(lng,6);
      dataFile.print(" ");
      dataFile.print(hgt,6);
      
      dataFile.print("roll : ");
      dataFile.print(roll);
      dataFile.print(", pitch : ");
      dataFile.print(pitch);
      dataFile.print(", yaw : ");
      dataFile.print(yaw);

      dataFile.print("servo_l : ");
      dataFile.print(servo_l);
      dataFile.print("servo_r : ");
      dataFile.println(servo_r);
      
    //Serial.print(ch);
    dataFile.close();
    // print to the serial port too:
   //Serial.println(dataString);
  }
  //if the file isn't open, pop up an error:
  else {
    Serial.println("error opening txt");
   }
  //Zigbee
  strcpy(atcommand, "at+unicast=cf12, hi");
  Serial1.print(atcommand);
  hex2char(startBytes, 4, strbuf);
  Serial1.print(strbuf);
  Serial1.print(" ");

    Serial1.print(pitch);
    Serial1.print(" ");
    Serial1.print(roll);
    Serial1.print(" ");
    Serial1.print(yaw);
    Serial1.print(" ");
    Serial1.print(lat,6);
    Serial1.print(" ");
    Serial1.print(lng,6);
    Serial1.print(" ");
    Serial1.print(hgt,6);
    Serial1.print(" ");  
    Serial1.print(servo_l);
    Serial1.print(" ");
    Serial1.print(servo_r);
    Serial1.print(" ");
    
         c = '0';
      Serial1.write(c);
      Serial1.print(" ");
      Serial1.write(c);
   Serial1.write('\r');
  //delay(1000);
  Serial.write(c);

  static int i = 0;
  static int j = 0;
  static int k;
  
  if(pitch>180){
    pitch_c = (-1) * pitch + 360;
  }

  if ((pitch_c>-100)&&(pitch_c<-80)&&(i==0)){
    servo_left.write(0);
    servo_right.write(0);
  }
  else{
    i=1;
  }
  
  pitch_c = (-1) * pitch;                 
  x = lng;
  y = lat;
  z = hgt;
  r = sqrt((x - a) * (x - a) + (y - b) * (y - b));
  yaw_c = (-1) * yaw;
  theta = atan2(b - y, a - x) - atan2(sin(yaw_c - 90) ,cos(yaw_c - 90));
  theta_abs = abs(theta);
  theta_sign = theta;

  if ((r>50 - r0) && (j == 0)) {
    if( theta_abs < 5.7) {
      servo_left.write(V[0]);
      servo_right.write(V[0]);
      k=0;
      servo_l=V[k];
      servo_r=V[k];
    }
    else if(theta_abs > 5.7 + 150) {
      servo_left.write(V[7]);
      servo_right.write(0);
      k=7;
      servo_l=V[k];
      servo_r=0;
    }
    else{
      for (n=1; n<4; n++) {
        if (( theta_abs > 5.7 + 30 * (n - 1) + 5 * n * (n - 1) )  &&  ( theta_abs <= 30 * n + 5 * n * (n + 1) )) { //if-1
          if (theta_sign > 0) {
              servo_left.write(V[n]);
              servo_right.write(0);
              k=n;
              servo_l=V[k];
              servo_r=0;
          }
          else{
            servo_left.write(0);
            servo_right.write(V[3+n]);
            k=n+3;
            servo_l=0;
            servo_r=V[k];
          }
          break;  
        } // if-1 end
        } // for end
    } // else end
  } //

  else{
    if (r > 60) {
      j = 0;
    }
    else if(j == 0) {
      if (theta_sign > 0)  {
        servo_left.write(V[7]);
        servo_right.write(0);
      }
      else {
        servo_left.write(0);
        servo_right.write(V[7]);
      }
      j = 1;
    }
  }

  //pitch control
  phi = atan2(r,z-c);
  if (phi>2.5){
    bldc.write(90);
  }
  else{
    bldc.write(0);
  }
}
