// Copyright © 2015, K.Townsend (Adafruit Industries) and Rick Sellens (https://github.com/adafruit/Adafruit_MPL115A2/blob/master/Adafruit_MPL115A2.h)
// Copyright © 2016, Eric Bakan & Yaroslav Tenzer 
// Parts of the code below are derived from the work of the aforementioned authors
// Copyright © 2022, Thiago Eustaquio Alves de Oliveira and Vinicius Prado da Fonseca

#include "MPL115A2MUX.h"

MPL115A2MUX::MPL115A2MUX(short n_sensors) {
	oTemp=0;
	oPressure=0;
	p_current=0;
	p_history=0;
	delta_up=0;
	delta_down=0;

    num_sensors = (n_sensors<=MAX_MUX) ? n_sensors : MAX_MUX;
	for(short i = 0; i<num_sensors; i++){

		pressureHistory[i] = 0;

		_mpl115a2_a0[i] = 0.0F;
		_mpl115a2_b1[i] = 0.0F;
		_mpl115a2_b2[i] = 0.0F;
		_mpl115a2_c12[i] = 0.0F;
	}
}

void MPL115A2MUX::begin() {
	for(short i = 0; i<num_sensors; i++){
	  readMux(i);
	  delay(5);

	  Wire.begin();
	  // Wire.setRate(I2C_RATE_400); //Teensy3.6-

      readCoefficients(i);
	}
}

/**************************************************************************/
/*!
    @brief  Gets the factory-set coefficients for this particular sensor
*/
/**************************************************************************/
void MPL115A2MUX::readCoefficients(short sensorID) {
  readMux(sensorID);
  int16_t a0coeff;
  int16_t b1coeff;
  int16_t b2coeff;
  int16_t c12coeff;

  Wire.beginTransmission(MPL115A2_ADDRESS);
  Wire.write((uint8_t)MPL115A2_REGISTER_A0_COEFF_MSB);
  Wire.endTransmission();

  Wire.requestFrom(MPL115A2_ADDRESS, 8);
  a0coeff = (( (uint16_t) Wire.read() << 8) | Wire.read());
  b1coeff = (( (uint16_t) Wire.read() << 8) | Wire.read());
  b2coeff = (( (uint16_t) Wire.read() << 8) | Wire.read());
  c12coeff = (( (uint16_t) (Wire.read() << 8) | Wire.read())) >> 2;

  _mpl115a2_a0[sensorID] = (float)a0coeff / 8;
  _mpl115a2_b1[sensorID] = (float)b1coeff / 8192;
  _mpl115a2_b2[sensorID] = (float)b2coeff / 16384;
  _mpl115a2_c12[sensorID] = (float)c12coeff;
  _mpl115a2_c12[sensorID] /= 4194304.0;
}

/**************************************************************************/
/*!
    @brief  Gets both at once and saves a little time
*/
/**************************************************************************/
void MPL115A2MUX::getPT(float *P, float *T, short sensorID) {
  readMux(sensorID);
  // Get raw pressure and temperature settings
  Wire.beginTransmission(MPL115A2_ADDRESS);
  Wire.write((uint8_t)MPL115A2_REGISTER_STARTCONVERSION);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission();
  // Wait a bit for the conversion to complete (3ms max)
  delay(1);
  Wire.beginTransmission(MPL115A2_ADDRESS);
  Wire.write((uint8_t)MPL115A2_REGISTER_PRESSURE_MSB);  // Register
  Wire.endTransmission();

  Wire.requestFrom(MPL115A2_ADDRESS, 4);
  uint16_t pressure = (( (uint16_t) Wire.read() << 8) | Wire.read()) >> 6;
  uint16_t temp = (( (uint16_t) Wire.read() << 8) | Wire.read()) >> 6;

  // See datasheet p.6 for evaluation sequence
  //pressureComp = _mpl115a2_a0 + (_mpl115a2_b1 + _mpl115a2_c12 * temp ) * pressure + _mpl115a2_b2 * temp;

  *P = pressure;
  *T = temp;
  // Return pressure and temperature as floating point values
  //*P = pressure*1.0F;//((65.0F / 1023.0F) * pressureComp) + 50.0F;        // kPa
  //*T = ((float) temp - 498.0F) / -5.35F +25.0F;           // C
}

void MPL115A2MUX::upateSensorLevelID(short sensorID){
	readMux(sensorID);
	getPT(&oPressure, &oTemp, sensorID);

	// the calculations of the wrapping
	// that are used to calculate the minus signs
	if (flagHistoryExists){
	  p_current=oPressure;
	  p_history=pressureHistory[sensorID];
	  delta_up=p_current-p_history;
	  delta_down=p_history-(p_current-1024);
	  if (delta_up<delta_down){
		oPressure=p_history+delta_up;
	  }else{
		oPressure=p_history-delta_down;
	  }
	}
	pressureHistory[sensorID]=oPressure;
	flagHistoryExists=true;
}

void MPL115A2MUX::readMux(short sensorID) {
	int controlPin[] = { S0, S1, S2, S3 };
	int muxChannel[16][4] = {
			{ 0, 0, 0, 0 }, //channel 0
			{ 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 1, 1, 0, 0 }, { 0, 0, 1, 0 }, { 1,
					0, 1, 0 }, { 0, 1, 1, 0 }, { 1, 1, 1, 0 }, { 0, 0, 0, 1 }, {
					1, 0, 0, 1 }, { 0, 1, 0, 1 }, { 1, 1, 0, 1 },
			{ 0, 0, 1, 1 }, { 1, 0, 1, 1 }, { 0, 1, 1, 1 }, { 1, 1, 1, 1 } //channel 15
	};
	for (int i = 0; i < 4; i++) {
		digitalWrite(controlPin[i], muxChannel[sensorID][i]);
	}
	delay(1);
}

void MPL115A2MUX::initialize() {
  Wire.beginTransmission(0xC0>>1);
  Wire.write(0x12);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(5);
}

