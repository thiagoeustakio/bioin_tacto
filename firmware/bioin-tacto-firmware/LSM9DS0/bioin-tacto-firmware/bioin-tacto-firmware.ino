//#define USE_TEENSY_HW_SERIAL

#include <ros.h>
#include <ros/time.h>
#include <bioin_tacto/raw_barometer.h>
#include <bioin_tacto/raw_imu.h>

ros::NodeHandle nh;

// #include <i2c_t3.h> //teensy 3.6- alternative to Wire
#include <Wire.h> // I2C functionality for Arduinos and Teensy4.0
#include <MPL115A2MUX.h>
#include <LSMMUX.h>

#define NUM_SENSORS 1

#define LSM9DS0_XM 0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G 0x6B  // Would be 0x6A if SDO_G is LOW

MPL115A2MUX mpl115a2(NUM_SENSORS);

short sensorID = 0;
uint16_t status[NUM_SENSORS];

LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

bioin_tacto::raw_barometer rbaro;
bioin_tacto::raw_imu rimu;
ros::Publisher rir_reads("raw_imus_teensy", &rimu);
ros::Publisher rbr_reads("raw_barometers_teensy", &rbaro);

long seq = 0;
float pressure = 0, tempe = 0;

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
  pinMode(13, OUTPUT);

  digitalWrite(13, HIGH);
  delay(750);
  digitalWrite(13, LOW);
  delay(750);

  mpl115a2.begin();
  mpl115a2.upateSensorLevelID(0);

  status[sensorID] = dof.beginID(sensorID);
  dof.setAccelScale(dof.A_SCALE_2G);
  dof.setAccelODR(dof.A_ODR_200);
  dof.setAccelABW(dof.A_ABW_50);
  dof.setGyroScale(dof.G_SCALE_245DPS);
  dof.setGyroODR(dof.G_ODR_190_BW_125);
  dof.setMagScale(dof.M_SCALE_2GS);
  dof.setMagODR(dof.M_ODR_125);

  // Configure serial and ros node baud
  Serial.begin(250000);
  nh.getHardware()->setBaud(250000);

  nh.initNode();
  rbaro.header.frame_id = "baros_frames";
  nh.advertise(rbr_reads);

  rimu.header.frame_id = "imus_frames";
  nh.advertise(rir_reads);

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

  for (int sensorID = 0; sensorID < NUM_SENSORS; sensorID++)
  {
    dof.readMux(sensorID);

    rimu.sensor_id = sensorID;
    dof.readGyro(sensorID);
    rimu.gx = (float)(dof.calcGyro(dof.gx[sensorID]));
    rimu.gy = (float)(dof.calcGyro(dof.gy[sensorID]));
    rimu.gz = (float)(dof.calcGyro(dof.gz[sensorID]));

    dof.readAccel(sensorID);
    rimu.ax = (float)(dof.calcAccel(dof.ax[sensorID]));
    rimu.ay = (float)(dof.calcAccel(dof.ay[sensorID]));
    rimu.az = (float)(dof.calcAccel(dof.az[sensorID]));

    dof.readMag(sensorID);
    rimu.mx = dof.calcMag(dof.mx[sensorID]);
    rimu.my = dof.calcMag(dof.my[sensorID]);
    rimu.mz = dof.calcMag(dof.mz[sensorID]);

    dof.readTemp(sensorID);
    rimu.tempe = 21.0 + (float)dof.temperature[sensorID] / 8.;

    rimu.header.seq = seq;
    rimu.header.stamp = nh.now();
    rir_reads.publish(&rimu);
    nh.spinOnce();
  }

  seq++;
}
