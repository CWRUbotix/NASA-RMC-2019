#include "LSM6DS3.h"
#include <SPI.h>
#include <inttypes.h>

LSM6DS3::LSM6DS3(int pin){
	this->cs_pin = pin;
	this->settings = SPISettings(LSM6DS3_SPEED, MSBFIRST, SPI_MODE0);
}

void LSM6DS3::begin(){
	pinMode(this->cs_pin, OUTPUT);
	digitalWrite(this->cs_pin, HIGH);

	// wakeup accel and gyro from power-down
	uint8_t data[1] = {};
	data[0] = (XL_ODR_SET << 4);
	this->_write_reg(CTRL1_XL, 1, data);

	data[0] = (GYRO_ODR_SET << 4);
	this->_write_reg(CTRL2_G, 1, data);

	// configure accel and gyro power-modes
	// for now, defaults should be OK

	// zero the sensor sync time frame
	data[0] = 0x00;
	this->_write_reg(SENSOR_SYNC_TIME_FRAME, 1, data);
}

bool LSM6DS3::is_ok(){
	uint8_t data[1] = {};

	this->_read_reg(WHO_AM_I, 1, data);

	return (data[0] == 0x69);
}

float LSM6DS3::get_accel_x(){
	uint8_t data[2] = {};
	int16_t temp = 0;

	this->_read_reg(OUTX_L_XL, 2, data);
	temp = bytearr_to_int16(data);
	return (temp/32767.0)*this->xl_full_scale;
}
float LSM6DS3::get_accel_y(){
	uint8_t data[2] = {};
	int16_t temp = 0;
	
	this->_read_reg(OUTY_L_XL, 2, data);
	temp = bytearr_to_int16(data);
	return (temp/32767.0)*this->xl_full_scale;
}
float LSM6DS3::get_accel_z(){
	uint8_t data[2] = {};
	int16_t temp = 0;
	
	this->_read_reg(OUTZ_L_XL, 2, data);
	temp = bytearr_to_int16(data);
	return (temp/32767.0)*this->xl_full_scale;
}
float LSM6DS3::get_gyro_x(){
	uint8_t data[2] = {};
	int16_t temp = 0;
	
	this->_read_reg(OUTX_L_G, 2, data);
	temp = bytearr_to_int16(data);
	return (temp/32767.0)*this->gyro_full_scale;
}
float LSM6DS3::get_gyro_y(){
	uint8_t data[2] = {};
	int16_t temp = 0;
	
	this->_read_reg(OUTY_L_G, 2, data);
	temp = bytearr_to_int16(data);
	return (temp/32767.0)*this->gyro_full_scale;
}
float LSM6DS3::get_gyro_z(){
	uint8_t data[2] = {};
	int16_t temp = 0;
	
	this->_read_reg(OUTZ_L_G, 2, data);
	temp = bytearr_to_int16(data);
	return (temp/32767.0)*this->gyro_full_scale;
}

void LSM6DS3::_read_reg(uint8_t reg, uint8_t reg_len, uint8_t* data){
	uint8_t cmd[1]  = {};
	cmd[0] 			= reg;
	cmd[0] |= (1 << 7);

	// make sure data buffer is empty
	for(int i = 0; i < reg_len; i++){
		data[i] = 0x00;
	}

	SPI.beginTransaction(this->settings);
	digitalWrite(this->cs_pin, LOW);
	PAUSE_SHORT;
	SPI.transfer(cmd[0]); 			// send single octet command
	SPI.transfer(data, reg_len); 	// now shift in the data from the proper register
	digitalWrite(this->cs_pin, HIGH);
	SPI.endTransaction();
}

void LSM6DS3::_write_reg(uint8_t reg, uint8_t reg_len, uint8_t* data){
	uint8_t cmd[1]  = {};
	cmd[0] 			= reg;
	cmd[0] &= ~(1 << 7);

	// make sure data buffer is empty
	for(int i = 0; i < reg_len; i++){
		data[i] = 0x00;
	}

	SPI.beginTransaction(this->settings);
	digitalWrite(this->cs_pin, LOW);
	PAUSE_SHORT;
	SPI.transfer(cmd[0]); 			// send single octet command
	SPI.transfer(data, reg_len); 	// now shift in the data from the proper register
	digitalWrite(this->cs_pin, HIGH);
	SPI.endTransaction();
}

// LOW ORDER BYTE FIRST
void int_to_bytearr(int n, uint8_t* data){
	for(int i = 0; i < 4; i++ ){
		data[i] = ((n >> i*8) & 0xFF);
	}
}

// expects the data to be LOW ORDER BYTE FIRST
int bytearr_to_int(uint8_t* data){
	int retval = 0;
	for(int i = 0; i < 4; i++ ){
		retval += (data[i] << i*8);
	}
	return retval;
}

int16_t bytearr_to_int16(uint8_t* data){
	int16_t retval = 0;
	retval |= data[0];
	retval |= (data[1] << 8);

	return retval;
}