#ifndef DATA_SETUP_H_
#define DATA_SETUP_H_


#include "values_and_types.h"
#include <inttypes.h>

void setup_sensors(void);
void setup_motors(void);
void setup_devices(void);

void setup_devices(){
	Device* device = NULL;

	// ADC 0
	device_infos[ADC_0].interface 		= SPI_BUS;
	device_infos[ADC_0].spi_cs 			= ADC_0_CS_PIN;
	device_infos[ADC_0].spi_settings 	= &ADC_SPI_settings;

	// ADC 1
	device_infos[ADC_1].interface 		= SPI_BUS;
	device_infos[ADC_1].spi_cs 			= ADC_1_CS_PIN;
	device_infos[ADC_1].spi_settings 	= &ADC_SPI_settings;

	// ADC 2
	device_infos[ADC_2].interface 	= SPI_BUS;
	device_infos[ADC_2].spi_cs 		= ADC_2_CS_PIN;
	device_infos[ADC_2].spi_settings= &ADC_SPI_settings;

	// SPI TRANS ENCODER
	device_infos[TRANS_ENCODER].interface 	= SPI_BUS;
	device_infos[TRANS_ENCODER].spi_cs 		= ENCODER_CS_PIN;
	device_infos[TRANS_ENCODER].spi_settings= &Encoder_SPI_settings;

	// IMU 0 DEVICE SETUP
	device_infos[IMU_0].interface 	= SPI_BUS;
	device_infos[IMU_0].spi_cs 		= IMU_0_CS_PIN;
	device_infos[IMU_0].spi_settings= &IMU_SPI_settings;

	// IMU 1 DEVICE SETUP
	device_infos[IMU_1].interface 	= SPI_BUS;
	device_infos[IMU_1].spi_cs 		= IMU_1_CS_PIN;
	device_infos[IMU_1].spi_settings= &IMU_SPI_settings;

	// DAC 0
	device_infos[DAC_0].interface 	= SPI_BUS;
	device_infos[DAC_0].spi_cs 		= DAC_0_CS_PIN;
	device_infos[DAC_0].spi_settings= &DAC_SPI_settings;

	// DAC 1
	device_infos[DAC_1].interface 	= SPI_BUS;
	device_infos[DAC_1].spi_cs 		= DAC_1_CS_PIN;
	device_infos[DAC_1].spi_settings= &DAC_SPI_settings;

	// DAC 2
	device_infos[DAC_2].interface 	= SPI_BUS;
	device_infos[DAC_2].spi_cs 		= DAC_2_CS_PIN;
	device_infos[DAC_2].spi_settings= &DAC_SPI_settings;

	// VESC 1
	device_infos[VESC_1].interface 	= VESC_UART;
	device_infos[VESC_1].serial 	= &Serial4;
	device_infos[VESC_1].vesc 		= &vesc4;

	// VESC 2
	device_infos[VESC_2].interface 	= VESC_UART;
	device_infos[VESC_2].serial 	= &Serial4;
	device_infos[VESC_2].vesc 		= &vesc4;

	// VESC 3
	device_infos[VESC_3].interface 	= VESC_UART;
	device_infos[VESC_3].serial 	= &Serial4;
	device_infos[VESC_3].vesc 		= &vesc4;

	// VESC 4
	device_infos[VESC_4].interface 	= VESC_UART;
	device_infos[VESC_4].serial 	= &Serial4;
	device_infos[VESC_4].vesc 		= &vesc4;

	// LOOKY PORT
	device_infos[LOOKY_0].interface 	= LOOKY_UART;
	device_infos[LOOKY_0].serial 		= &Serial6;

	// LOOKY STBD
	device_infos[LOOKY_1].interface 	= LOOKY_UART;
	device_infos[LOOKY_1].serial 		= &Serial6;
}


void setup_sensors(){
	SensorInfo* sensor = &(sensor_infos[DRIVE_PORT_ENC]);

	// DRIVE PORT ENCODER
	sensor 				= &(sensor_infos[DRIVE_PORT_ENC]);
	sensor->device 		= &(device_infos[VESC_1]);
	sensor->type 		= SENS_BLDC_ENC;

	// DRIVE STBD ENC
	sensor 				= &(sensor_infos[DRIVE_STBD_ENC]);
	sensor->device 		= &(device_infos[VESC_2]);
	sensor->type 		= SENS_BLDC_ENC;

	// DEP WINCH ENC
	sensor 				= &(sensor_infos[DEP_WINCH_ENC]);
	sensor->device 		= &(device_infos[VESC_3]);
	sensor->type 		= SENS_BLDC_ENC;

	// EXC BELT ENC
	sensor 				= &(sensor_infos[EXC_BELT_ENC]);
	sensor->device 		= &(device_infos[VESC_4]);
	sensor->type 		= SENS_BLDC_ENC;

	// EXC TRANSLATION ENCODER
	sensor 				= &(sensor_infos[EXC_TRANS_ENC]);
	sensor->device 		= &(device_infos[TRANS_ENCODER]);
	sensor->type 		= SENS_ROT_ENC;

	// EXC ROTATION PORT ENCODER
	sensor 				= &(sensor_infos[EXC_ROT_PORT_ENC]);
	sensor->device 		= &(device_infos[ADC_0]);
	sensor->type 		= SENS_POT_ENC;

	// EXC ROTATION STBD ENCODER
	sensor 				= &(sensor_infos[EXC_ROT_STBD_ENC]);
	sensor->device 		= &(device_infos[ADC_0]);
	sensor->type 		= SENS_POT_ENC;

	// LOOKY PORT ENC
	sensor 				= &(sensor_infos[LOOKY_PORT_ENC]);
	sensor->device 		= &(device_infos[LOOKY_0]);
	sensor->type 		= SENS_LOOKY_ENC;

	// LOOKY STBD ENC
	sensor 				= &(sensor_infos[LOOKY_STBD_ENC]);
	sensor->device 		= &(device_infos[LOOKY_1]);
	sensor->type 		= SENS_LOOKY_ENC;

	// DEP PORT LOAD-CELL
	sensor 				= &(sensor_infos[DEP_PORT_LOAD]);
	sensor->device 		= &(device_infos[ADC_1]);
	sensor->type 		= SENS_LOAD_CELL;

	// DEP STBD LOAD-CELL
	sensor 				= &(sensor_infos[DEP_STBD_LOAD]);
	sensor->device 		= &(device_infos[ADC_2]);
	sensor->type 		= SENS_LOAD_CELL;

	// GYRO-0 X
	sensor 				= &(sensor_infos[GYRO_0_X]);
	sensor->type 		= SENS_GYRO;
	sensor->device 		= &(device_infos[IMU_0]);

	// GYRO-0 Y
	sensor 				= &(sensor_infos[GYRO_0_Y]);
	sensor->type 		= SENS_GYRO;
	sensor->device 		= &(device_infos[IMU_0]);

	// GYRO-0 Z
	sensor 				= &(sensor_infos[GYRO_0_Z]);
	sensor->type 		= SENS_GYRO;
	sensor->device 		= &(device_infos[IMU_0]);

	// ACCEL-0 X
	sensor 				= &(sensor_infos[ACCEL_0_X]);
	sensor->type 		= SENS_ACCEL;
	sensor->device 		= &(device_infos[IMU_0]);

	// ACCEL-0 Y
	sensor 				= &(sensor_infos[ACCEL_0_Y]);
	sensor->type 		= SENS_ACCEL;
	sensor->device 		= &(device_infos[IMU_0]);

	// ACCEL-0 Z
	sensor 				= &(sensor_infos[ACCEL_0_Z]);
	sensor->type 		= SENS_ACCEL;
	sensor->device 		= &(device_infos[IMU_0]);

	// GYRO-1 X
	sensor 				= &(sensor_infos[GYRO_1_X]);
	sensor->type 		= SENS_GYRO;
	sensor->device 		= &(device_infos[IMU_1]);

	// GYRO-1 Y
	sensor 				= &(sensor_infos[GYRO_1_Y]);
	sensor->type 		= SENS_GYRO;
	sensor->device 		= &(device_infos[IMU_1]);

	// GYRO-1 Z
	sensor 				= &(sensor_infos[GYRO_1_Z]);
	sensor->type 		= SENS_GYRO;
	sensor->device 		= &(device_infos[IMU_1]);

	// ACCEL-1 X
	sensor 				= &(sensor_infos[ACCEL_1_X]);
	sensor->type 		= SENS_ACCEL;
	sensor->device 		= &(device_infos[IMU_1]);

	// ACCEL-1 Y
	sensor 				= &(sensor_infos[ACCEL_1_Y]);
	sensor->type 		= SENS_ACCEL;
	sensor->device 		= &(device_infos[IMU_1]);

	// ACCEL-1 Z
	sensor 				= &(sensor_infos[ACCEL_1_Z]);
	sensor->type 		= SENS_ACCEL;
	sensor->device 		= &(device_infos[IMU_1]);

	// DEP. LOWER LIMIT SWITCH
	sensor 				= &(sensor_infos[DEP_LIMIT_LOWER]);
	sensor->type 		= SENS_LIMIT;
	sensor->pin 		= DEP_LOWER_LIM_PIN;

	// DEP. UPPER LIMIT SWITCH
	sensor 				= &(sensor_infos[DEP_LIMIT_UPPER]);
	sensor->type 		= SENS_LIMIT;
	sensor->pin 		= DEP_UPPER_LIM_PIN;

	// EXC. FORE LIMIT SWITCH
	sensor 				= &(sensor_infos[EXC_LIMIT_FORE]);
	sensor->type 		= SENS_LIMIT;
	sensor->pin 		= EXC_ROT_FORE_LIM_PIN;

	// EXC. AFT LIMIT SWITCH
	sensor 				= &(sensor_infos[EXC_LIMIT_AFT]);
	sensor->type 		= SENS_LIMIT;
	sensor->pin 		= EXC_ROT_AFT_LIM_PIN;

	// EXC bucket conveyor LOWER LIMIT SWITCH
	sensor 				= &(sensor_infos[EXC_CONV_LIMIT_LOWER]);
	sensor->type 		= SENS_LIMIT;
	sensor->pin 		= EXC_TRANS_LOWER_LIM_PIN;

	// EXC bucket conveyor UPPER LIMIT SWITCH
	sensor 				= &(sensor_infos[EXC_CONV_LIMIT_UPPER]);
	sensor->type 		= SENS_LIMIT;
	sensor->pin 		= EXC_TRANS_UPPER_LIM_PIN;

	// E-Stop
	sensor 				= &(sensor_infos[ESTOP_SENSE_INDEX]);
	sensor->type 		= SENS_DIGITAL_INPUT;
	sensor->pin 		= ESTOP_SENSE_PIN;
}

void setup_motors(void){
	MotorInfo* motor = &(motor_infos[PORT_VESC]);

	// PORT VESC (VESC 1)
	motor 				= &(motor_infos[PORT_VESC]);
	motor->device 		= &(device_infos[VESC_1]);

	// STBD VESC (VESC 2)
	motor 				= &(motor_infos[STBD_VESC]);
	motor->device 		= &(device_infos[VESC_2]);

	// DEP VESC (VESC 3)
	motor 				= &(motor_infos[DEP_VESC]);
	motor->device 		= &(device_infos[VESC_3]);

	// EXC VESC (VESC 4)
	motor 				= &(motor_infos[EXC_VESC]);
	motor->device 		= &(device_infos[VESC_4]);

	// Exc translation (DAC 0)
	motor 				= &(motor_infos[EXC_TRANS]);
	motor->device 		= &(device_infos[DAC_0]);
	motor->type 		= MTR_SABERTOOTH;

	// Exc Rot Port (DAC 1)
	motor 				= &(motor_infos[EXC_ROT_PORT]);
	motor->device 		= &(device_infos[DAC_1]);
	motor->type 		= MTR_SABERTOOTH;

	// Exc Rot Starboard (DAC 2)
	motor 				= &(motor_infos[EXC_ROT_STBD]);
	motor->device 		= &(device_infos[DAC_2]);
	motor->type 		= MTR_SABERTOOTH;
}



#endif