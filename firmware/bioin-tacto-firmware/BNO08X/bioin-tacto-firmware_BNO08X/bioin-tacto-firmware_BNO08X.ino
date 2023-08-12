//#define USE_TEENSY_HW_SERIAL
#include <ros.h>
#include <ros/time.h>
#include <bioin_tacto/raw_barometer.h>
#include <bioin_tacto/raw_imu.h>

byte accAccuracy = 0;
byte gyroAccuracy = 0;
byte magAccuracy = 0;
float ax, ay, az, gx, gy, gz, mx, my, mz;

ros::NodeHandle nh;

#include <Wire.h> // I2C functionality for Arduinos and Teensy4.0
#include <MPL115A2MUX.h>
#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080

#define NUM_SENSORS 1

BNO080 myMARG;
MPL115A2MUX mpl115a2(NUM_SENSORS);

short sensorID = 0;
uint16_t status[NUM_SENSORS];

bioin_tacto::raw_barometer rbaro;
bioin_tacto::raw_imu rimu;
ros::Publisher rir_reads("raw_imus_teensy", &rimu);
ros::Publisher rbr_reads("raw_barometers_teensy", &rbaro);

long seq = 0;
float pressure = 0, tempe = 21;

void setup()
{ 
  //  When using more than one module,
  //  the following lines set the state of the multiplex pins
  //
  //  pinMode(S0, OUTPUT);
  //  pinMode(S1, OUTPUT);
  //  pinMode(S2, OUTPUT);
  //  pinMode(S3, OUTPUT);
  //  digitalWrite(S0, LOW);
  //  digitalWrite(S1, LOW);
  //  digitalWrite(S2, LOW);
  //  digitalWrite(S3, LOW);
  // Configure serial and ros node baud
  Serial.begin(250000);
  nh.getHardware()->setBaud(250000);

  pinMode(13, OUTPUT);

  digitalWrite(13, HIGH);
  delay(750);
  digitalWrite(13, LOW);
  delay(750);

 
  Wire.begin();
  delay(50);
  while (myMARG.begin() == false)
  {
//    while(1){
//          Serial.println("ERROR, CHECK WIRING");
          delay(3);
//    }
  }
//  Serial.println("Success");
  Wire.setClock(400000); //Increase I2C data rate to 400kHz
  myMARG.enableAccelerometer(1); // m/s^2 no plus gravity
  myMARG.enableGyro(1);  // rad/s
  myMARG.enableMagnetometer(1);


  mpl115a2.begin();
  mpl115a2.upateSensorLevelID(0);

  nh.initNode();

  rimu.header.frame_id = "imus_frames";
  nh.advertise(rir_reads);

  rbaro.header.frame_id = "baros_frames";
  nh.advertise(rbr_reads);

  delay(5);
 
  digitalWrite(13, HIGH);
}

void loop()
{
    for (int sensorID = 0; sensorID < NUM_SENSORS; sensorID++)
    {
      mpl115a2.upateSensorLevelID(sensorID);
      rbaro.sensor_id = sensorID;
      rbaro.baro_level = mpl115a2.pressureHistory[sensorID];
      rbaro.tempe = tempe;
      rbaro.header.seq = seq;
      rbaro.header.stamp = nh.now();
      rbr_reads.publish(&rbaro);
      nh.spinOnce();
    }
    
    rimu.sensor_id = sensorID;

    if (myMARG.dataAvailable() == true)
      {
        myMARG.getAccel(ax, ay, az, accAccuracy);
        rimu.ax=ax; rimu.ay=ay; rimu.az=az;
        myMARG.getGyro(gx, gy, gz, gyroAccuracy);
        rimu.gx=gx; rimu.gy=gy; rimu.gz=gz;
        myMARG.getMag(mx, my, mz, magAccuracy);
        rimu.mx=mx; rimu.my=my; rimu.mz=mz;
      
        rimu.tempe = tempe;
    
        rimu.header.seq = seq;
        rimu.header.stamp = nh.now();
        rir_reads.publish(&rimu);
        nh.spinOnce();
    }
  seq++;
}
