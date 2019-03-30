#ifndef _LSM6DS3_H_
#define _LSM6DS3_H_


#include <SPI.h>
#include <inttypes.h>

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#define LSM6DS3_SPEED 	1000000
#define XL_ODR_SET 		0b00000101
#define GYRO_ODR_SET 	0b00000101
#define XL_HM_MODE 		0b00000000
#define GYRO_HM_MODE 	0b00000000

#ifndef NOP4
#define NOP4 			"nop\n\t""nop\n\t""nop\n\t""nop\n\t"
#endif

// should pause about 50 ns or so
#ifndef PAUSE_SHORT
#define PAUSE_SHORT 	__asm__(NOP4 NOP4)
#endif

enum REG_MAP {
	SENSOR_SYNC_TIME_FRAME 	= 0x04,
	WHO_AM_I 				= 0x0F,
	CTRL1_XL 				= 0x10,
	CTRL2_G 				= 0x11,
	CTRL3_C 				= 0x12,
	CTRL4_C 				= 0x13,
	CTRL5_C 				= 0x14,
	CTRL6_C 				= 0x15,
	CTRL7_G 				= 0x16,
	CTRL8_XL 				= 0x17,
	CTRL9_XL 				= 0x18,
	CTRL10_C  				= 0x19,
	STATUS_REG 				= 0x1E,
	OUTX_L_G 				= 0x22,
	OUTY_L_G 				= 0x24,
	OUTZ_L_G 				= 0x26,
	OUTX_L_XL 				= 0x28,
	OUTY_L_XL 				= 0x2A,
	OUTZ_L_XL 				= 0x2C
};


void int_to_bytearr(int i, uint8_t* data);
int bytearr_to_int(uint8_t* data);
int16_t bytearr_to_int16(uint8_t* data);

class LSM6DS3{
private:
	int cs_pin;
	float xl_full_scale 	= 2.0; 		// in g's
	float gyro_full_scale 	= 250.0; 	// in dps
	SPISettings settings;
	void _read_reg(uint8_t reg, uint8_t reg_len, uint8_t* data);
	void _write_reg(uint8_t reg, uint8_t reg_len, uint8_t* data);

public:
	LSM6DS3(int pin);
	void begin();
	bool is_ok();
	float get_accel_x();
	float get_accel_y();
	float get_accel_z();
	float get_gyro_x();
	float get_gyro_y();
	float get_gyro_z();

};

#endif