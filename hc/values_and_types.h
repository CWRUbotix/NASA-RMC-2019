

#ifndef VALUES_AND_TYPES_H_
#define VALUES_AND_TYPES_H_

#include <SPI.h>
#include "VESC/VESC.h"
#include "LSM6DS3.h"
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <XYZrobotServo.h>
////////////////////////////////////////////////////////////////////////////////
//	PREPROCESSOR DEFINES
////////////////////////////////////////////////////////////////////////////////
#define NUM_SENSORS 	33
#define NUM_SENSOR_VALS 33
#define NUM_MOTORS 		9
#define NUM_DEVICES 	15
#define NUM_SPI_DEVICES 9

#define CLIENT 			Serial
#define CMD_MAX_LEN 	1024
#define RPY_MAX_LEN 	1024
#define HEADER_LEN 		5
#define CMD_BLOCK_LEN 	5
#define RPY_BLOCK_LEN 	5
#define RPY_BLOCK_LEN_L 9


#define DEBUG_SELECT_PIN 		2
#define ADC_2_CS_PIN 			14
#define ADC_1_CS_PIN 			15
#define ADC_0_CS_PIN 			16
#define DAC_2_CS_PIN 			17
#define DAC_1_CS_PIN 			18
#define DAC_0_CS_PIN 			19
#define ENCODER_CS_PIN 			22
#define ENCODER_INDEX_PIN 		23
#define EXC_ROT_FORE_LIM_PIN 	24
#define EXC_ROT_AFT_LIM_PIN 	25
#define EXC_TRANS_LOWER_LIM_PIN	29
#define EXC_TRANS_UPPER_LIM_PIN	30
#define DEP_LOWER_LIM_PIN 		36
#define IMU_0_CS_PIN 			35
#define IMU_1_CS_PIN 			38
#define DEP_UPPER_LIM_PIN 		37
#define ESTOP_SENSE_PIN 		39

#define MOTOR_ANLG_CENTER_V 	2.5
#define DAC8551_SPEED 			1000000
#define ADS1120_SPEED 			1000000
#define IMU_SPEED 				1000000
#define ENCODER_SPEED 			1000000
#define DEBUG_SPEED 			1000000

#define DEBUG 					Serial5

#define NOP3 			"nop\n\t""nop\n\t""nop\n\t"
// should pause about 50 ns or so
#define PAUSE_SHORT 	__asm__(NOP3 NOP3 NOP3)

#define ROT_ENC_RD_POS 	0x10
////////////////////////////////////////////////////////////////////////////////
//  DEFINE TYPES
////////////////////////////////////////////////////////////////////////////////
enum DEVICE_INDICES {
	ADC_0,
	ADC_1,
	ADC_2,
	DAC_0,
	DAC_1,
	DAC_2,
	TRANS_ENCODER,
	IMU_0,
	IMU_1,
	VESC_1,
	VESC_2,
	VESC_3,
	VESC_4,
	LOOKY_0,
	LOOKY_1
};

enum SENSOR_INDICES {
	DRIVE_PORT_ENC,
	DRIVE_STBD_ENC,
	DEP_WINCH_ENC,
	EXC_BELT_ENC,
	EXC_TRANS_ENC,
	EXC_ROT_PORT_ENC,
	EXC_ROT_STBD_ENC,
	LOOKY_PORT_ENC,
	LOOKY_STBD_ENC,
	DEP_PORT_LOAD,
	DEP_STBD_LOAD,
	GYRO_0_X,
	GYRO_0_Y,
	GYRO_0_Z,
	ACCEL_0_X,
	ACCEL_0_Y,
	ACCEL_0_Z,
	GYRO_1_X,
	GYRO_1_Y,
	GYRO_1_Z,
	ACCEL_1_X,
	ACCEL_1_Y,
	ACCEL_1_Z,
	DEP_LIMIT_LOWER,
	DEP_LIMIT_UPPER,
	EXC_LIMIT_FORE,
	EXC_LIMIT_AFT,
	EXC_CONV_LIMIT_LOWER,
	EXC_CONV_LIMIT_UPPER,
	ESTOP_SENSE_INDEX,
	ANGULAR_DISP_X,
	ANGULAR_DISP_Y,
	ANGULAT_DISP_Z
};

enum MOTOR_INDICES {
	PORT_VESC,
	STBD_VESC,
	DEP_VESC,
	EXC_VESC,
	EXC_TRANS,
	EXC_ROT_PORT,
	LOOKY_PORT,
	LOOKY_STBD,
	EXC_ROT_STBD
};

enum COMMANDS {
	CMD_SET_OUTPUTS = 0x51,
	CMD_READ_VALUES = 0x52,
	CMD_TEST 		= 0x53,
	CMD_T_SYNC 		= 0x54
};

enum REPLYS {
	RPY_SET_OUTPUTS 	= CMD_SET_OUTPUTS,
	RPY_READ_VALUES 	= CMD_READ_VALUES,
	RPY_TEST 			= CMD_TEST,
	RPY_T_SYNC 			= CMD_T_SYNC,
	RPY_INVALID_CMD 	= 0xA1,
	RPY_LEN_MISMATCH 	= 0xA2,
	RPY_CHKSUM_MISMATCH = 0xA3
};

typedef enum FAULT {
	NO_FAULT,
	INVALID_CMD,
	LEN_MISMATCH,
	CHKSUM_MISMATCH
} FAULT_T;

typedef enum Interface {
	NONE,
	DIGITAL_IO,
	LOOKY_UART,
	VESC_UART,
	SPI_BUS
} Interface;

typedef enum SensorType {
	SENS_NONE,
	SENS_DIGITAL_INPUT,
	SENS_LIMIT,
	SENS_LOAD_CELL,
	SENS_GYRO,
	SENS_ACCEL,
	SENS_ROT_ENC,
	SENS_BLDC_ENC,
	SENS_POT_ENC,
	SENS_LOOKY_ENC
} SensorType;

typedef enum MotorType {
	MTR_NONE,
	MTR_VESC,
	MTR_SABERTOOTH,
	MTR_LOOKY
} MotorType;

typedef struct Device{
	Interface interface = NONE;
	SPISettings* spi_settings;
	VESC* vesc;
	LSM6DS3* imu;
	HardwareSerial* serial;
  XYZrobotServo* servo;
  XYZrobotServoStatus* servo_status;
	uint8_t spi_cs 		= 0;
	uint8_t id 			= 0;
	bool is_setup 		= false; // field to prevent unnecessary setup
}SensorDevice;

typedef struct SensorInfo{
	SensorType type 	= SENS_NONE;
	Device* device;
	char* name = "--- NO NAME ----";
	uint8_t pin 		= 0;
	int n_value; 	// holds any relevant integer value
	float value; 	// holds the relevant value, updated at t_stamp
	int t_stamp; 	// update time-stamp
	float rots;
	float (*get_value)(void);
	char imu_axis;
} SensorInfo;

typedef struct MotorInfo{
	MotorType type 		= MTR_NONE;
	Device* device;
	SensorInfo* sensor;
	float setpt 		= 0.0; 	
	float volts 		= MOTOR_ANLG_CENTER_V;
	int t_stamp 		= 0; 	// time-stamp of last update
	float kp 			= 0.0;
	float ki 			= 0.0;
	float max_setpt 	= 0.0;
	float max_delta 	= 0.0;
	float deadband 		= 0.0;
} MotorInfo;

////////////////////////////////////////////////////////////////////////////////
//
//  GLOBAL VARIABLES
//
////////////////////////////////////////////////////////////////////////////////
SensorInfo 	sensor_infos 	[NUM_SENSORS] 		= {};
MotorInfo 	motor_infos 	[NUM_MOTORS] 		= {};
Device 		device_infos 	[NUM_DEVICES] 		= {};
Device 		SPI_devices 	[NUM_SPI_DEVICES] 	= {};

bool estop_state 		= false; 	// false means off
bool estop_state_last 	= false; 	// false means off

int t_micros 			= 0; 		// will be updated with micros()
int t_offset 			= 0; 		// offset = micros() - t_sync,   t_stamp = micros() - t_offset

XYZrobotServo looky_servo_port(&Serial6, 128);
XYZrobotServo looky_servo_starboard(&Serial6, 129);
XYZrobotServoStatus looky_servoStatus_port;
XYZrobotServoStatus looky_servoStatus_starboard;

VESC vesc1(&Serial1);
VESC vesc2(&Serial2);
VESC vesc3(&Serial3);
VESC vesc4(&Serial4);

LSM6DS3 imu0(IMU_0_CS_PIN);
LSM6DS3 imu1(IMU_1_CS_PIN);

SPISettings DAC_SPI_settings(DAC8551_SPEED, MSBFIRST, SPI_MODE1);
SPISettings ADC_SPI_settings(ADS1120_SPEED, MSBFIRST, SPI_MODE1);
SPISettings IMU_SPI_settings(IMU_SPEED, MSBFIRST, SPI_MODE0);
SPISettings Encoder_SPI_settings(ENCODER_SPEED, MSBFIRST, SPI_MODE0);
SPISettings Debug_SPI_settings(DEBUG_SPEED, MSBFIRST, SPI_MODE0);

#define TIME_STAMP (micros() - t_offset)

void Encoder_ISR(){
	sensor_infos[EXC_TRANS_ENC].rots += 1;
	sensor_infos[EXC_TRANS_ENC].value = sensor_infos[EXC_TRANS_ENC].rots * 2.0 * PI; 	// rotation in radians
}


#endif
