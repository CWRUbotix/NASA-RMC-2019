#ifndef MOTOR_SENSOR_SETUP_H_
#define MOTOR_SENSOR_SETUP_H_

#include "values_and_types.h"
#include <inttypes.h>

void init_device_comms(void);
void init_motors(void);
void init_sensors(void);


void init_motors(){
	for(int i = 0; i < NUM_MOTORS; i++){
		MotorInfo* motor = &motor_infos[i];
		
		switch(motor->type){
			case MTR_NONE:{break;}
			case MTR_VESC:{
				if(motor->device != NULL && !motor->device->is_setup){
					motor->device->vesc->begin();
					motor->device->vesc->set_rpm(0);
					motor->device->vesc->request_mc_values();
					motor->device->is_setup = true;
				}
				// delay(10);
				break;}
			case MTR_SABERTOOTH:{
				if(motor->device->spi_cs != 0){
					pinMode(motor->device->spi_cs, OUTPUT);
					digitalWrite(motor->device->spi_cs, HIGH);
				}
				// set the motor to not move
				// remember that 0V makes the motor go full speed in reverse
				uint8_t data[3] = {};
				package_DAC_voltage(MOTOR_ANLG_CENTER_V, data); 	// make sure motor is stopped

				SPI.beginTransaction(*(motor->device->spi_settings));
				digitalWrite(motor->device->spi_cs, LOW);
				PAUSE_SHORT;
				SPI.transfer(data, 3);
				digitalWrite(motor->device->spi_cs, HIGH);
				SPI.endTransaction();

				motor->setpt 	= motor->sensor->value; 	// make the set-point equal to the current position
				break;}
			case MTR_LOOKY:{
				if(motor->device != NULL && !motor->device->is_setup){
					Herkulex.begin(motor->device->serial, 115200);
					Herkulex.initialize();
					motor->device->is_setup = true;
				}
				break;}
		}
	}
}

void init_sensors(){
	// do configurations as necessary
	SensorInfo* sensor = NULL;
	Device* device = NULL;
	for(int i = 0; i < NUM_SENSORS; i++){
		sensor = &sensor_infos[i];
		device = sensor->device;
		debug(String(sensor->name));
		switch(sensor->type){
			case SENS_NONE: break;
			case SENS_DIGITAL_INPUT:{
				if(sensor->pin != 0){
					pinMode(sensor->pin, INPUT);
				}
				break;}
			case SENS_LIMIT: {
				if(sensor->pin != 0){
					pinMode(sensor->pin, INPUT);
				}
				break;}
			case SENS_LOAD_CELL:{
				if(device != NULL && !device->is_setup){
					// configure ADC
					debug("setting up load cell ADC");
					device->adc->setup(CONFIG_FOR_LOAD_CELL);
					device->adc->set_mux_input(diff_1_2);
					device->adc->set_gain(7);
					device->is_setup = true;
				}
				break;}
			case SENS_GYRO: {
				if(device != NULL && !device->is_setup){
					// configure device
					debug("begin IMU");
					device->imu->begin();
					delayMicroseconds(10);
					device->is_setup = device->imu->is_ok();
					if(device->is_setup){debug("IMU setup success");}
					else{debug("IMU setup failed");}
				}
				break;}
			case SENS_ACCEL: {
				if(device != NULL && !device->is_setup){
					// configure device
					device->imu->begin();
					delayMicroseconds(10);
					device->is_setup = device->imu->is_ok();
				}
				break;}
			case SENS_ROT_ENC: {
				// configure device as per this sensor
				if(device != NULL && !device->is_setup){
					device->is_setup = true;
				}
				break;}
			case SENS_BLDC_ENC: {
				// let this get configured in init_motors
				break;}
			case SENS_POT_ENC:{
				if(device != NULL && !device->is_setup){
					// configure ADC 
					debug("setting up pot. ADC");
					device->adc->setup(CONFIG_FOR_POT);
					device->adc->set_mux_input(sensor->adc_channel_config);
					device->adc->set_gain(0);
					device->is_setup = true;
				}
				break;}
		}
	}

}

// configure more critical pins such as SPI chip-select pins
//  basic low-level IO, prevent bus collisions, etc.
void init_device_comms(){
	for(int i = 0; i<NUM_DEVICES; i++){
		Device* device = &device_infos[i];

		switch(device->interface){
			case NONE: {break;}
			case DIGITAL_IO:{ break;}
			case VESC_UART: { 
				device->vesc->begin();
				break;}
			case LOOKY_UART: { 
				device->serial->begin(115200);
				break;}
			case SPI_BUS: {
				if(device->spi_cs != 0){
					pinMode(device->spi_cs, OUTPUT);
					digitalWrite(device->spi_cs, HIGH);
				}
				break;}
		}
		
		
	}
}

#endif
