// Copyright © 2015, K.Townsend (Adafruit Industries) and Rick Sellens (https://github.com/adafruit/Adafruit_MPL115A2/blob/master/Adafruit_MPL115A2.h)
// Copyright © 2016, Eric Bakan & Yaroslav Tenzer 
// Parts of the code below are derived from the work of the aforementioned authors
// Copyright © 2022, Thiago Eustaquio Alves de Oliveira and Vinicius Prado da Fonseca

// #include <i2c_t3.h> //teensy 3.6- alternative to Wire
#include <Wire.h> //I2C funcionality for Arduinos and Teensy4.0+
/*=========================================================================
	MUX SETTINGS
-----------------------------------------------------------------------*/
#define MAX_MUX 16
#define S0 12
#define S1 11
#define S2 10
#define S3 9

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define MPL115A2_ADDRESS                       (0x60)    // 1100000
/*=========================================================================*/
/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define MPL115A2_REGISTER_PRESSURE_MSB         (0x00)
    #define MPL115A2_REGISTER_PRESSURE_LSB         (0x01)
    #define MPL115A2_REGISTER_TEMP_MSB             (0x02)
    #define MPL115A2_REGISTER_TEMP_LSB             (0x03)
    #define MPL115A2_REGISTER_A0_COEFF_MSB         (0x04)
    #define MPL115A2_REGISTER_A0_COEFF_LSB         (0x05)
    #define MPL115A2_REGISTER_B1_COEFF_MSB         (0x06)
    #define MPL115A2_REGISTER_B1_COEFF_LSB         (0x07)
    #define MPL115A2_REGISTER_B2_COEFF_MSB         (0x08)
    #define MPL115A2_REGISTER_B2_COEFF_LSB         (0x09)
    #define MPL115A2_REGISTER_C12_COEFF_MSB        (0x0A)
    #define MPL115A2_REGISTER_C12_COEFF_LSB        (0x0B)
    #define MPL115A2_REGISTER_STARTCONVERSION      (0x12)
/*=========================================================================*/
#define FREESCALE_ADDRESS 0xC0
#define SENSOR_ALL_ON 0x0C
#define SENSOR_ALL_OFF 0x0D

class MPL115A2MUX{
 public:
  MPL115A2MUX(short n_sensors = 1);
  void begin(void);
  void getPT(float *P, float *T, short sensorID);
  void upateSensorLevelID(short sensorID);
  void readMux(short sensorID);
  void initialize();

  boolean flagHistoryExists=false;

  float pressureHistory[MAX_MUX];

  float oTemp=0;
  float oPressure=0;
  float p_current=0;
  float p_history=0;
  float delta_up=0;
  float delta_down=0;

 private:
  short num_sensors;
  float _mpl115a2_a0[MAX_MUX];
  float _mpl115a2_b1[MAX_MUX];
  float _mpl115a2_b2[MAX_MUX];
  float _mpl115a2_c12[MAX_MUX];

  void readCoefficients(short sensorID);
};
