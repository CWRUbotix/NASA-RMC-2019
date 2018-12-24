#ifndef IO_H_
#define IO_H_

#include "values_and_types.h"
#include <inttypes.h>

void update_sensors(void);
void maintain_motors(void);
void package_DAC_voltage(float v, uint8_t* data);
void debug(void);

void update_sensors(){
	for(int i = 0; i < NUM_SENSORS; i++){
		SensorInfo* sensor = &sensor_infos[i];
		switch(sensor->type){
			case SENS_NONE: break;
			case SENS_DIGITAL_INPUT:{
				sensor->n_value = (digitalRead(sensor->pin) == HIGH ? 1 : 0);
				sensor->t_stamp = TIME_STAMP;
				break;}
			case SENS_LIMIT:{
				sensor->n_value = (digitalRead(sensor->pin) == HIGH ? 1 : 0);
				sensor->t_stamp = TIME_STAMP;
				break;}
			case SENS_LOAD_CELL:{
				// read ADC among other things
				break;}
			case SENS_GYRO:{
				// 
				break;}
			case SENS_ACCEL:{
				// 
				break;}
			case SENS_ROT_ENC:{
				// 
				break;}
			case SENS_BLDC_ENC:{
				sensor->device->vesc->update_mc_values(); 				// read the available packet
				sensor->value = 1.0*sensor->device->vesc->get_rpm();	// get the RPM
				break;}
			case SENS_POT_ENC:{

				break;}
			case SENS_LOOKY_ENC:{

				break;}
		}
	}
}

void maintain_motors(){
	for(int i = 0; i < NUM_MOTORS; i++){
		MotorInfo* motor = &motor_infos[i];

		switch(motor->type){
			case MTR_NONE: break;
			case MTR_VESC:{
				int sign = motor->setpt/fabs(motor->setpt);
				int rpm = (fabs(motor->setpt) > motor->max_setpt ? (int)sign*motor->max_setpt : (int)motor->setpt );
				motor->device->vesc->set_rpm(rpm); 			// set the motor speed
				motor->device->vesc->request_mc_values(); 	// resonse packet should be ready when we call update sensors
				break;}
			case MTR_SABERTOOTH:{

				break;}
			case MTR_LOOKY:{

				break;}
		}
	}
}

void debug(){
	unsigned char debug[1024] 	= {};
	int s_len 			= 8;
	MotorInfo* motor = NULL;
	String temp = String(":\n");
	for(int i = 0; i < NUM_MOTORS; i++){
		motor = &motor_infos[i];
		if(motor->type == MTR_VESC){
			VESC* vesc 		= motor->device->vesc;
			float current 	= vesc->get_current_in();
			temp.concat(String(current, 3));
			temp.concat(String(':'));
		}

	}
	s_len = temp.length();
	temp.getBytes(debug, s_len);
	DEBUG.write(debug, s_len);
	delay(10);

}


void package_DAC_voltage(float v, uint8_t* data){
	// set the mode to normal
	data[0] &= 0x00;

	uint16_t temp = (uint16_t)(((v/5.0)*65535)+0.5);
	data[1] = (temp >> 8) & 0xFF;
	data[2] = (temp) & 0xFF;
}



#endif