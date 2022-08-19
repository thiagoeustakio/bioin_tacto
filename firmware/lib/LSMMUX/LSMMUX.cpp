/******************************************************************************
 SFE_LSM9DS0.cpp
 SFE_LSM9DS0 Library Source File
 Jim Lindblom @ SparkFun Electronics
 Original Creation Date: February 14, 2014 (Happy Valentines Day!)
 https://github.com/sparkfun/LSM9DS0_Breakout

 This file implements all functions of the LSM9DS0 class. Functions here range
 from higher level stuff, like reading/writing LSM9DS0 registers to low-level,
 hardware reads and writes. Both SPI and I2C handler functions can be found
 towards the bottom of this file.

 Development environment specifics:
 IDE: Arduino 1.0.5
 Hardware Platform: Arduino Pro 3.3V/8MHz
 LSM9DS0 Breakout Version: 1.0

 This code is beerware; if you see me (or any other SparkFun employee) at the
 local, and you've found our code helpful, please buy us a round!

 Distributed as-is; no warranty is given.
 ******************************************************************************/
// Copyright © 2014, Jim Lindblom @ SparkFun Electronics (https://github.com/sparkfun/LSM9DS0_Breakout/tree/master/Libraries/Arduino)
// The code below is derived from the work of the aforementioned authors
// Copyright © 2022, Thiago Eustaquio Alves de Oliveira and Vinicius Prado da Fonseca


#include "LSMMUX.h"

LSM9DS0::LSM9DS0(interface_mode interface, uint8_t gAddr, uint8_t xmAddr) {
	// interfaceMode will keep track of whether we're using SPI or I2C:
	interfaceMode = interface;

	// xmAddress and gAddress will store the 7-bit I2C address, if using I2C.
	// If we're using SPI, these variables store the chip-select pins.
	xmAddress = xmAddr;
	gAddress = gAddr;
  
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  
}

uint16_t LSM9DS0::begin(gyro_scale gScl, accel_scale aScl, mag_scale mScl,
		gyro_odr gODR, accel_odr aODR, mag_odr mODR) {
	// Store the given scales in class variables. These scale variables
	// are used throughout to calculate the actual g's, DPS,and Gs's.
	gScale = gScl;
	aScale = aScl;
	mScale = mScl;

	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
	calcmRes(); // Calculate Gs / ADC tick, stored in mRes variable
	calcaRes(); // Calculate g / ADC tick, stored in aRes variable

	initI2C();					// Initialize I2C

	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	uint8_t gTest = gReadByte(WHO_AM_I_G);		// Read the gyro WHO_AM_I
	uint8_t xmTest = xmReadByte(WHO_AM_I_XM);	// Read the accel/mag WHO_AM_I

	// Gyro initialization stuff:
	initGyro();	// This will "turn on" the gyro. Setting up interrupts, etc.
	setGyroODR(gODR); // Set the gyro output data rate and bandwidth.
	setGyroScale (gScale); // Set the gyro range

	// Accelerometer initialization stuff:
	initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.
	setAccelODR(aODR); // Set the accel data rate.
	setAccelScale (aScale); // Set the accel range.

	// Magnetometer initialization stuff:
	initMag(); // "Turn on" all axes of the mag. Set up interrupts, etc.
	setMagODR(mODR); // Set the magnetometer output data rate.
	setMagScale (mScale); // Set the magnetometer's range.

	// Once everything is initialized, return the WHO_AM_I registers we read:
	return (xmTest << 8) | gTest;
}

uint16_t LSM9DS0::beginID(short sensorID, gyro_scale gScl, accel_scale aScl, mag_scale mScl,
		gyro_odr gODR, accel_odr aODR, mag_odr mODR) {
	readMux(sensorID);
	delay(1);
	return begin(gScl, aScl, mScl,	gODR, aODR, mODR);
}

void LSM9DS0::initGyro() {
	gWriteByte(CTRL_REG1_G, 0x0F); // Normal mode, enable all axes

	gWriteByte(CTRL_REG2_G, 0x00); // Normal mode, high cutoff frequency

	gWriteByte(CTRL_REG3_G, 0x88);

	gWriteByte(CTRL_REG4_G, 0x00); // Set scale to 245 dps

	gWriteByte(CTRL_REG5_G, 0x00);

	// Temporary !!! For testing !!! Remove !!! Or make useful !!!
	configGyroInt(0x2A, 0, 0, 0, 0); // Trigger interrupt when above 0 DPS...
}

void LSM9DS0::initAccel() {

	xmWriteByte(CTRL_REG0_XM, 0x00);

	xmWriteByte(CTRL_REG1_XM, 0x57); // 100Hz data rate, x/y/z all enabled

	xmWriteByte(CTRL_REG2_XM, 0x00); // Set scale to 2g

	// Accelerometer data ready on INT1_XM (0x04)
	xmWriteByte(CTRL_REG3_XM, 0x04);
}

void LSM9DS0::initMag() {

	xmWriteByte(CTRL_REG5_XM, 0x94); // Mag data rate - 100 Hz, enable temperature sensor

	xmWriteByte(CTRL_REG6_XM, 0x00); // Mag scale to +/- 2GS

	xmWriteByte(CTRL_REG7_XM, 0x00); // Continuous conversion mode

	xmWriteByte(CTRL_REG4_XM, 0x04); // Magnetometer data ready on INT2_XM (0x08)

	xmWriteByte(INT_CTRL_REG_M, 0x09); // Enable interrupts for mag, active-low, push-pull
}

void LSM9DS0::calLSM9DS0(short sensorID) {
	readMux(sensorID);
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	int16_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
	int samples, ii;

	// First get gyro bias
	byte c = gReadByte(CTRL_REG5_G);
	gWriteByte(CTRL_REG5_G, c | 0x40);         // Enable gyro FIFO  
	delay(20);                                 // Wait for change to take effect
	gWriteByte(FIFO_CTRL_REG_G, 0x20 | 0x1F); // Enable gyro FIFO stream mode and set watermark at 32 samples
	delay(1000);  // delay 1000 milliseconds to collect FIFO samples

	samples = (gReadByte(FIFO_SRC_REG_G) & 0x1F); // Read number of stored samples

	for (ii = 0; ii < samples; ii++) {  // Read the gyro data stored in the FIFO
		gReadBytes(OUT_X_L_G, &data[0], 6);
		gyro_bias[0] += (((int16_t) data[1] << 8) | data[0]);
		gyro_bias[1] += (((int16_t) data[3] << 8) | data[2]);
		gyro_bias[2] += (((int16_t) data[5] << 8) | data[4]);
	}

	gyro_bias[0] /= samples; // average the data
	gyro_bias[1] /= samples;
	gyro_bias[2] /= samples;

	gbias[sensorID][0] = (float) gyro_bias[0] * gRes; // Properly scale the data to get deg/s
	gbias[sensorID][1] = (float) gyro_bias[1] * gRes;
	gbias[sensorID][2] = (float) gyro_bias[2] * gRes;

	c = gReadByte(CTRL_REG5_G);
	gWriteByte(CTRL_REG5_G, c & ~0x40);  // Disable gyro FIFO  
	delay(20);
	gWriteByte(FIFO_CTRL_REG_G, 0x00);   // Enable gyro bypass mode

	//  Now get the accelerometer biases
	c = xmReadByte(CTRL_REG0_XM);
	xmWriteByte(CTRL_REG0_XM, c | 0x40);      // Enable accelerometer FIFO  
	delay(20);                                // Wait for change to take effect
	xmWriteByte(FIFO_CTRL_REG, 0x20 | 0x1F); // Enable accelerometer FIFO stream mode and set watermark at 32 samples
	delay(1000);  // delay 1000 milliseconds to collect FIFO samples

	samples = (xmReadByte(FIFO_SRC_REG) & 0x1F); // Read number of stored accelerometer samples

	for (ii = 0; ii < samples; ii++) { // Read the accelerometer data stored in the FIFO
		xmReadBytes(OUT_X_L_A, &data[0], 6);
		accel_bias[0] += (((int16_t) data[1] << 8) | data[0]);
		accel_bias[1] += (((int16_t) data[3] << 8) | data[2]);
		accel_bias[2] += (((int16_t) data[5] << 8) | data[4])
				- (int16_t)(1. / aRes); // Assumes sensor facing up!
	}

	accel_bias[0] /= samples; // average the data
	accel_bias[1] /= samples;
	accel_bias[2] /= samples;

	abias[sensorID][0] = (float) accel_bias[0] * aRes; // Properly scale data to get gs
	abias[sensorID][1] = (float) accel_bias[1] * aRes;
	abias[sensorID][2] = (float) accel_bias[2] * aRes;

	c = xmReadByte(CTRL_REG0_XM);
	xmWriteByte(CTRL_REG0_XM, c & ~0x40);    // Disable accelerometer FIFO  
	delay(20);
	xmWriteByte(FIFO_CTRL_REG, 0x00);       // Enable accelerometer bypass mode
}

void LSM9DS0::readAccel(short sensorID) { 
	uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp	
	xmReadBytes(OUT_X_L_A, temp, 6); // Read 6 bytes, beginning at OUT_X_L_A
	ax[sensorID] = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
	ay[sensorID] = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
	az[sensorID] = (temp[5] << 8) | temp[4]; // Store z-axis values into az
}

void LSM9DS0::readMag(short sensorID) {
	uint8_t temp[6]; // We'll read six bytes from the mag into temp	
	xmReadBytes(OUT_X_L_M, temp, 6); // Read 6 bytes, beginning at OUT_X_L_M
	mx[sensorID] = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
	my[sensorID] = (temp[3] << 8) | temp[2]; // Store y-axis values into my
	mz[sensorID] = (temp[5] << 8) | temp[4]; // Store z-axis values into mz
}

void LSM9DS0::readTemp(short sensorID) { 
	uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp	
	xmReadBytes(OUT_TEMP_L_XM, temp, 2); // Read 2 bytes, beginning at OUT_TEMP_L_M
	temperature[sensorID] = (((int16_t) temp[1] << 12) | temp[0] << 4) >> 4; // Temperature is a 12-bit signed integer
}

void LSM9DS0::readGyro(short sensorID) {
	uint8_t temp[6]; // We'll read six bytes from the gyro into temp
	gReadBytes(OUT_X_L_G, temp, 6); // Read 6 bytes, beginning at OUT_X_L_G
	gx[sensorID] = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
	gy[sensorID] = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
	gz[sensorID] = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
}

float LSM9DS0::calcGyro(int16_t gyro) {
	// Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
	return gRes * gyro;
}

float LSM9DS0::calcAccel(int16_t accel) {
	// Return the accel raw reading times our pre-calculated g's / (ADC tick):
	return aRes * accel;
}

float LSM9DS0::calcMag(int16_t mag) {
	// Return the mag raw reading times our pre-calculated Gs / (ADC tick):
	return mRes * mag;
}

void LSM9DS0::setGyroScale(gyro_scale gScl) {
	// We need to preserve the other bytes in CTRL_REG4_G. So, first read it:
	uint8_t temp = gReadByte(CTRL_REG4_G);
	// Then mask out the gyro scale bits:
	temp &= 0xFF ^ (0x3 << 4);
	// Then shift in our new scale bits:
	temp |= gScl << 4;
	// And write the new register value back into CTRL_REG4_G:
	gWriteByte(CTRL_REG4_G, temp);

	// We've updated the sensor, but we also need to update our class variables
	// First update gScale:
	gScale = gScl;
	// Then calculate a new gRes, which relies on gScale being set correctly:
	calcgRes();
}

void LSM9DS0::setAccelScale(accel_scale aScl) {
	// We need to preserve the other bytes in CTRL_REG2_XM. So, first read it:
	uint8_t temp = xmReadByte(CTRL_REG2_XM);
	// Then mask out the accel scale bits:
	temp &= 0xFF ^ (0x3 << 3);
	// Then shift in our new scale bits:
	temp |= aScl << 3;
	// And write the new register value back into CTRL_REG2_XM:
	xmWriteByte(CTRL_REG2_XM, temp);

	// We've updated the sensor, but we also need to update our class variables
	// First update aScale:
	aScale = aScl;
	// Then calculate a new aRes, which relies on aScale being set correctly:
	calcaRes();
}

void LSM9DS0::setMagScale(mag_scale mScl) {
	// We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
	uint8_t temp = xmReadByte(CTRL_REG6_XM);
	// Then mask out the mag scale bits:
	temp &= 0xFF ^ (0x3 << 5);
	// Then shift in our new scale bits:
	temp |= mScl << 5;
	// And write the new register value back into CTRL_REG6_XM:
	xmWriteByte(CTRL_REG6_XM, temp);

	// We've updated the sensor, but we also need to update our class variables
	// First update mScale:
	mScale = mScl;
	// Then calculate a new mRes, which relies on mScale being set correctly:
	calcmRes();
}

void LSM9DS0::setGyroODR(gyro_odr gRate) {
	// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
	uint8_t temp = gReadByte(CTRL_REG1_G);
	// Then mask out the gyro ODR bits:
	temp &= 0xFF ^ (0xF << 4);
	// Then shift in our new ODR bits:
	temp |= (gRate << 4);
	// And write the new register value back into CTRL_REG1_G:
	gWriteByte(CTRL_REG1_G, temp);
}

void LSM9DS0::setAccelODR(accel_odr aRate) {
	// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
	uint8_t temp = xmReadByte(CTRL_REG1_XM);
	// Then mask out the accel ODR bits:
	temp &= 0xFF ^ (0xF << 4);
	// Then shift in our new ODR bits:
	temp |= (aRate << 4);
	// And write the new register value back into CTRL_REG1_XM:
	xmWriteByte(CTRL_REG1_XM, temp);
}

void LSM9DS0::setAccelABW(accel_abw abwRate) {
	// We need to preserve the other bytes in CTRL_REG2_XM. So, first read it:
	uint8_t temp = xmReadByte(CTRL_REG2_XM);
	// Then mask out the accel ABW bits:
	temp &= 0xFF ^ (0x3 << 7);
	// Then shift in our new ODR bits:
	temp |= (abwRate << 7);
	// And write the new register value back into CTRL_REG2_XM:
	xmWriteByte(CTRL_REG2_XM, temp);
}

void LSM9DS0::setMagODR(mag_odr mRate) {
	// We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
	uint8_t temp = xmReadByte(CTRL_REG5_XM);
	// Then mask out the mag ODR bits:
	temp &= 0xFF ^ (0x7 << 2);
	// Then shift in our new ODR bits:
	temp |= (mRate << 2);
	// And write the new register value back into CTRL_REG5_XM:
	xmWriteByte(CTRL_REG5_XM, temp);
}

void LSM9DS0::configGyroInt(uint8_t int1Cfg, uint16_t int1ThsX,
		uint16_t int1ThsY, uint16_t int1ThsZ, uint8_t duration) {
	gWriteByte(INT1_CFG_G, int1Cfg);
	gWriteByte(INT1_THS_XH_G, (int1ThsX & 0xFF00) >> 8);
	gWriteByte(INT1_THS_XL_G, (int1ThsX & 0xFF));
	gWriteByte(INT1_THS_YH_G, (int1ThsY & 0xFF00) >> 8);
	gWriteByte(INT1_THS_YL_G, (int1ThsY & 0xFF));
	gWriteByte(INT1_THS_ZH_G, (int1ThsZ & 0xFF00) >> 8);
	gWriteByte(INT1_THS_ZL_G, (int1ThsZ & 0xFF));
	if (duration)
		gWriteByte(INT1_DURATION_G, 0x80 | duration);
	else
		gWriteByte(INT1_DURATION_G, 0x00);
}

void LSM9DS0::calcgRes() {

	switch (gScale) {
	case G_SCALE_245DPS:
		gRes = 245.0 / 32768.0;
		break;
	case G_SCALE_500DPS:
		gRes = 500.0 / 32768.0;
		break;
	case G_SCALE_2000DPS:
		gRes = 2000.0 / 32768.0;
		break;
	}
}

void LSM9DS0::calcaRes() {

	aRes = aScale == A_SCALE_16G ?
			16.0 / 32768.0 : (((float) aScale + 1.0) * 2.0) / 32768.0;
}

void LSM9DS0::calcmRes() {

	mRes = mScale == M_SCALE_2GS ?
			2.0 / 32768.0 : (float) (mScale << 2) / 32768.0;
}

void LSM9DS0::gWriteByte(uint8_t subAddress, uint8_t data) {

	I2CwriteByte(gAddress, subAddress, data);

}

void LSM9DS0::xmWriteByte(uint8_t subAddress, uint8_t data) {

	return I2CwriteByte(xmAddress, subAddress, data);

}

uint8_t LSM9DS0::gReadByte(uint8_t subAddress) {

	return I2CreadByte(gAddress, subAddress);

}

void LSM9DS0::gReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count) {

	I2CreadBytes(gAddress, subAddress, dest, count);

}

uint8_t LSM9DS0::xmReadByte(uint8_t subAddress) {

	return I2CreadByte(xmAddress, subAddress);

}

void LSM9DS0::xmReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count) {

	I2CreadBytes(xmAddress, subAddress, dest, count);

}

void LSM9DS0::initI2C() {
	Wire.begin();	// Initialize I2C library
	// Wire.setRate(I2C_RATE_400); // Teensy3.6-
}

// Wire.h read and write protocols
void LSM9DS0::I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data) {
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

uint8_t LSM9DS0::I2CreadByte(uint8_t address, uint8_t subAddress) {
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	          // Put slave register address in Tx buffer
	Wire.endTransmission(false); // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, (uint8_t) 1); // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                         // Return data read from slave register
}

void LSM9DS0::I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest,
		uint8_t count) {
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	// Next send the register to be read. OR with 0x80 to indicate multi-read.
	Wire.write(subAddress | 0x80);    // Put slave register address in Tx buffer
	Wire.endTransmission(false); // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
	Wire.requestFrom(address, count); // Read bytes from slave register address 
	while (Wire.available()) {
		dest[i++] = Wire.read(); // Put read results in the Rx buffer
	}
}

void LSM9DS0::readMux(short channel) {
	int controlPin[] = { S0, S1, S2, S3 };
	int muxChannel[16][4] = {
			{ 0, 0, 0, 0 }, //channel 0
			{ 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 1, 1, 0, 0 }, { 0, 0, 1, 0 }, { 1,
					0, 1, 0 }, { 0, 1, 1, 0 }, { 1, 1, 1, 0 }, { 0, 0, 0, 1 }, {
					1, 0, 0, 1 }, { 0, 1, 0, 1 }, { 1, 1, 0, 1 },
			{ 0, 0, 1, 1 }, { 1, 0, 1, 1 }, { 0, 1, 1, 1 }, { 1, 1, 1, 1 } //channel 15
	};
	for (int i = 0; i < 4; i++) {
		digitalWrite(controlPin[i], muxChannel[channel][i]);
	}
}
