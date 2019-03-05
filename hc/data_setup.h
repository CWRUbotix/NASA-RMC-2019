#ifndef DATA_SETUP_H_
#define DATA_SETUP_H_


#include "values_and_types.h"
#include <inttypes.h>

void setup_sensors(void);
void setup_motors(void);
void setup_devices(void);

void setup_devices(){
	Device* device = NULL;

	// ADC 0 (Linear actuator pots.)
	device_infos[ADC_0].interface 		= SPI_BUS;
	device_infos[ADC_0].spi_cs 			= ADC_0_CS_PIN;
	device_infos[ADC_0].spi_settings 	= &ADC_SPI_settings;
	device_infos[ADC_0].adc 			= &adc0;

	// ADC 1 (Deposition Load Cell(s))
	device_infos[ADC_1].interface 		= SPI_BUS;
	device_infos[ADC_1].spi_cs 			= ADC_1_CS_PIN;
	device_infos[ADC_1].spi_settings 	= &ADC_SPI_settings;
	device_infos[ADC_1].adc 			= &adc1;

	// ADC 2 (Excavation Load Cell(s))
	device_infos[ADC_2].interface 		= SPI_BUS;
	device_infos[ADC_2].spi_cs 			= ADC_2_CS_PIN;
	device_infos[ADC_2].spi_settings 	= &ADC_SPI_settings;
	device_infos[ADC_2].adc 			= &adc2;

	// SPI TRANS ENCODER
	device_infos[TRANS_ENCODER].interface 	= SPI_BUS;
	device_infos[TRANS_ENCODER].spi_cs 		= ENCODER_CS_PIN;
	device_infos[TRANS_ENCODER].spi_settings= &Encoder_SPI_settings;

	// IMU 0 DEVICE SETUP
	device_infos[IMU_0].imu 		= &imu0;
	device_infos[IMU_0].interface 	= SPI_BUS;
	device_infos[IMU_0].spi_cs 		= IMU_0_CS_PIN;
	device_infos[IMU_0].spi_settings= &IMU_SPI_settings;

	// IMU 1 DEVICE SETUP
	device_infos[IMU_1].imu 		= &imu1;
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
	// device_infos[LOOKY_0].servo = looky_servo_port;
	// device_infos[LOOKY_0].servo_status;

	// LOOKY STBD
	device_infos[LOOKY_1].interface 	= LOOKY_UART;
	device_infos[LOOKY_1].serial 		= &Serial6;
	// device_infos[LOOKY_1].servo = looky_servo_starboard;
	// device_infos[LOOKY_1].servo_status;
}


void setup_sensors(){
	SensorInfo* sensor = &(sensor_infos[DRIVE_PORT_ENC]);

	// DRIVE PORT ENCODER
	sensor 				= &(sensor_infos[DRIVE_PORT_ENC]);
	sensor->name 		= "PORT DRIVE ENC";
	sensor->device 		= &(device_infos[VESC_1]);
	sensor->type 		= SENS_BLDC_ENC;

	// DRIVE STBD ENC
	sensor 				= &(sensor_infos[DRIVE_STBD_ENC]);
	sensor->name 		= "STBD DRIVE ENC";
	sensor->device 		= &(device_infos[VESC_2]);
	sensor->type 		= SENS_BLDC_ENC;

	// DEP WINCH ENC
	sensor 				= &(sensor_infos[DEP_WINCH_ENC]);
	sensor->name 		= "DEP WINCH ENC";
	sensor->device 		= &(device_infos[VESC_3]);
	sensor->type 		= SENS_BLDC_ENC;

	// EXC BELT ENC
	sensor 				= &(sensor_infos[EXC_BELT_ENC]);
	sensor->name 		= "EXC BELT ENC";
	sensor->device 		= &(device_infos[VESC_4]);
	sensor->type 		= SENS_BLDC_ENC;

	// EXC TRANSLATION ENCODER
	sensor 				= &(sensor_infos[EXC_TRANS_ENC]);
	sensor->name 		= "EXC TRANS ENC";
	sensor->device 		= &(device_infos[TRANS_ENCODER]);
	sensor->type 		= SENS_ROT_ENC;

	// EXC ROTATION PORT ENCODER (ADC AIN2)
	sensor 				= &(sensor_infos[EXC_ROT_PORT_ENC]);
	sensor->name 		= "EXC ROT ENC PORT";
	sensor->device 		= &(device_infos[ADC_0]);
	sensor->type 		= SENS_POT_ENC;
	sensor->adc_channel_config = single_2;

	// EXC ROTATION STBD ENCODER (ADC AIN1)
	sensor 				= &(sensor_infos[EXC_ROT_STBD_ENC]);
	sensor->name 		= "EXC ROT ENC STBD";
	sensor->device 		= &(device_infos[ADC_0]);
	sensor->type 		= SENS_POT_ENC;
	sensor->adc_channel_config = single_1;

	// LOOKY PORT ENC
	sensor 				= &(sensor_infos[LOOKY_PORT_ENC]);
	sensor->name 		= "PORT LOOKY ENC";
	sensor->device 		= &(device_infos[LOOKY_0]);
	sensor->type 		= SENS_LOOKY_ENC;

	// LOOKY STBD ENC
	sensor 				= &(sensor_infos[LOOKY_STBD_ENC]);
	sensor->name 		= "STBD LOOKY ENC";
	sensor->device 		= &(device_infos[LOOKY_1]);
	sensor->type 		= SENS_LOOKY_ENC;

	// DEP LOAD-CELL
	sensor 				= &(sensor_infos[DEP_LOAD_CELL]);
	sensor->name 		= "DEP LOAD CELL";
	sensor->device 		= &(device_infos[ADC_1]);
	sensor->type 		= SENS_LOAD_CELL;
	sensor->prev_values = dep_lc_values;

	// EXC LOAD LOAD-CELL
	sensor 				= &(sensor_infos[EXC_LOAD_CELL]);
	sensor->name 		= "EXC LOAD CELL";
	sensor->device 		= &(device_infos[ADC_2]);
	sensor->type 		= SENS_LOAD_CELL;
	sensor->prev_values = exc_lc_values;

	// GYRO-0 X
	sensor 				= &(sensor_infos[GYRO_0_X]);
	sensor->name 		= "GYRO_0 X";
	sensor->type 		= SENS_GYRO;
	sensor->device 		= &(device_infos[IMU_0]);
	sensor->imu_axis 	= 'X';
	// sensor->get_value 	= &(sensor->device->imu->get_gyro_x);

	// GYRO-0 Y
	sensor 				= &(sensor_infos[GYRO_0_Y]);
	sensor->name 		= "GYRO_0 Y";
	sensor->type 		= SENS_GYRO;
	sensor->device 		= &(device_infos[IMU_0]);
	sensor->imu_axis 	= 'Y';
	// sensor->get_value 	= &(sensor->device->imu->get_gyro_y);

	// GYRO-0 Z
	sensor 				= &(sensor_infos[GYRO_0_Z]);
	sensor->name 		= "GYRO_0 Z";
	sensor->type 		= SENS_GYRO;
	sensor->device 		= &(device_infos[IMU_0]);
	sensor->imu_axis 	= 'Z';
	// sensor->get_value 	= &(sensor->device->imu->get_gyro_z);

	// ACCEL-0 X
	sensor 				= &(sensor_infos[ACCEL_0_X]);
	sensor->name 		= "ACCEL_0 X";
	sensor->type 		= SENS_ACCEL;
	sensor->device 		= &(device_infos[IMU_0]);
	sensor->imu_axis 	= 'X';

	// ACCEL-0 Y
	sensor 				= &(sensor_infos[ACCEL_0_Y]);
	sensor->name 		= "ACCEL_0 Y";
	sensor->type 		= SENS_ACCEL;
	sensor->device 		= &(device_infos[IMU_0]);
	sensor->imu_axis 	= 'Y';

	// ACCEL-0 Z
	sensor 				= &(sensor_infos[ACCEL_0_Z]);
	sensor->name 		= "ACCEL_0 Z";
	sensor->type 		= SENS_ACCEL;
	sensor->device 		= &(device_infos[IMU_0]);
	sensor->imu_axis 	= 'Z';

	// GYRO-1 X
	sensor 				= &(sensor_infos[GYRO_1_X]);
	sensor->name 		= "GYRO_1 X";
	sensor->type 		= SENS_GYRO;
	sensor->device 		= &(device_infos[IMU_1]);
	sensor->imu_axis 	= 'X';

	// GYRO-1 Y
	sensor 				= &(sensor_infos[GYRO_1_Y]);
	sensor->name 		= "GYRO_1 Y";
	sensor->type 		= SENS_GYRO;
	sensor->device 		= &(device_infos[IMU_1]);
	sensor->imu_axis 	= 'Y';

	// GYRO-1 Z
	sensor 				= &(sensor_infos[GYRO_1_Z]);
	sensor->name 		= "GYRO_1 Z";
	sensor->type 		= SENS_GYRO;
	sensor->device 		= &(device_infos[IMU_1]);
	sensor->imu_axis 	= 'Z';

	// ACCEL-1 X
	sensor 				= &(sensor_infos[ACCEL_1_X]);
	sensor->name 		= "ACCEL_1 X";
	sensor->type 		= SENS_ACCEL;
	sensor->device 		= &(device_infos[IMU_1]);
	sensor->imu_axis 	= 'X';

	// ACCEL-1 Y
	sensor 				= &(sensor_infos[ACCEL_1_Y]);
	sensor->name 		= "ACCEL_1 Y";
	sensor->type 		= SENS_ACCEL;
	sensor->device 		= &(device_infos[IMU_1]);
	sensor->imu_axis 	= 'Y';

	// ACCEL-1 Z
	sensor 				= &(sensor_infos[ACCEL_1_Z]);
	sensor->name 		= "ACCEL_1 Z";
	sensor->type 		= SENS_ACCEL;
	sensor->device 		= &(device_infos[IMU_1]);
	sensor->imu_axis 	= 'Z';

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
	motor->max_setpt 	= 5000.0; 					// max RPM
	motor->max_delta 	= 50.0; 					// final output RPM/s
	motor->rpm_factor 	= 100.0 * 7; 				// external_reduc * pole_pairs

	// STBD VESC (VESC 2)
	motor 				= &(motor_infos[STBD_VESC]);
	motor->device 		= &(device_infos[VESC_2]);
	motor->max_setpt 	= 5000.0; 					// max RPM
	motor->max_delta 	= 50.0; 					// final output RPM/s
	motor->rpm_factor 	= 100.0 * 7; 				// external_reduc * pole_pairs

	// DEP VESC (VESC 3)
	motor 				= &(motor_infos[DEP_VESC]);
	motor->device 		= &(device_infos[VESC_3]);
	motor->limit_1 		= &(sensor_infos[DEP_LIMIT_UPPER]);
	motor->limit_2 		= &(sensor_infos[DEP_LIMIT_LOWER]);
	motor->max_setpt 	= 5000.0; 					// max RPM
	motor->max_delta 	= 50.0; 					// final output RPM/s
	motor->rpm_factor 	= 50.0 * 7; 				// external_reduc * pole_pairs

	// EXC VESC (VESC 4)
	motor 				= &(motor_infos[EXC_VESC]);
	motor->device 		= &(device_infos[VESC_4]);
	motor->max_setpt 	= 5000.0; 					// max RPM
	motor->max_delta 	= 50.0; 					// final output RPM/s
	motor->rpm_factor 	= 50.0 * 7; 				// external_reduc * pole_pairs

	// Exc translation (DAC 0)
	motor 				= &(motor_infos[EXC_TRANS]);
	motor->device 		= &(device_infos[DAC_0]);
	motor->type 		= MTR_SABERTOOTH;
	motor->limit_1 		= &(sensor_infos[EXC_CONV_LIMIT_UPPER]);
	motor->limit_2 		= &(sensor_infos[EXC_CONV_LIMIT_LOWER]);
	motor->max_setpt 	= 1000.0; 					// max positional value (mm)

	// Exc Rot Port (DAC 1)
	motor 				= &(motor_infos[EXC_ROT_PORT]);
	motor->device 		= &(device_infos[DAC_1]);
	motor->type 		= MTR_SABERTOOTH;
	motor->limit_1 		= &(sensor_infos[EXC_LIMIT_FORE]);
	motor->limit_2 		= &(sensor_infos[EXC_LIMIT_AFT]);
	motor->max_setpt 	= 135.0; 					// max position in degrees

	// Exc Rot Starboard (DAC 2)
	motor 				= &(motor_infos[EXC_ROT_STBD]);
	motor->device 		= &(device_infos[DAC_2]);
	motor->type 		= MTR_SABERTOOTH;
	motor->limit_1 		= &(sensor_infos[EXC_LIMIT_FORE]);
	motor->limit_2 		= &(sensor_infos[EXC_LIMIT_AFT]);
	motor->max_setpt 	= 135.0; 					// max position in degrees

	// Looky PORT SIDE
	motor 				= &(motor_infos[LOOKY_PORT]);
	motor->device 		= &(device_infos[LOOKY_0]);
	motor->type 		= MTR_LOOKY;
	motor->max_setpt 	= 150.0; 					// max position in degrees

	// Looky STBD SIDE
	motor 				= &(motor_infos[LOOKY_STBD]);
	motor->device 		= &(device_infos[LOOKY_1]);
	motor->type 		= MTR_LOOKY;
	motor->max_setpt 	= 150.0; 					// max position in degrees
}



#endif
