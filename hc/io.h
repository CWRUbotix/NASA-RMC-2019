#include <XYZrobotServo.h>

#ifndef IO_H_
#define IO_H_

#include "values_and_types.h"
#include <inttypes.h>

void update_sensors(void);
void maintain_motors(void);
void package_DAC_voltage(float v, uint8_t* data);
void debug(String s);
float get_rot_encoder_value(SensorInfo* encoder);
void Encoder_ISR(void);

void update_sensors(){
	for(int i = 0; i < NUM_SENSORS; i++){
		SensorInfo* sensor 	= &sensor_infos[i];
		Device* device 		= sensor->device;
		switch(sensor->type){
			case SENS_NONE: break;
			case SENS_DIGITAL_INPUT:{
				sensor->value = (digitalRead(sensor->pin) == HIGH ? 1.0 : 0.0);
				sensor->t_stamp = TIME_STAMP;
				break;}
			case SENS_LIMIT:{
				sensor->value = (digitalRead(sensor->pin) == HIGH ? 1.0 : 0.0);
				sensor->t_stamp = TIME_STAMP;
				break;}
			case SENS_LOAD_CELL:{
				if(sensor->device != NULL && sensor->device->is_setup){
	        		uint16_t temp 		= 0;
	        		int16_t signed_temp = 0;
	        		float f_temp 		= 0;
	        		sensor->value_good = sensor->device->adc->read_channel(diff_1_2, &temp);
					if(sensor->value_good){
						memcpy(&signed_temp, &temp, 2);
						f_temp = (signed_temp/32767.0)*5.0; 					// voltage
						sensor->value = sensor->slope*f_temp + sensor->offset;	// apply linear correction
						sensor->t_stamp = TIME_STAMP;
						sensor->prev_values[val_ind] = sensor->value;
						sensor->val_ind++;
						if(sensor->val_ind >= NUM_PREV_VALUES){
							sensor->val_ind = 0;}
						debug("Load cell v:\t"+String(sensor->value, 3));
					}else{
						debug("Load cell read ERROR");
					}
				}
				break;}
			case SENS_GYRO:{
				if(sensor->device != NULL && sensor->device->is_setup){
					switch(sensor->imu_axis){
						case 'X': sensor->value = device->imu->get_gyro_x(); break;
						case 'Y': sensor->value = device->imu->get_gyro_y(); break;
						case 'Z': sensor->value = device->imu->get_gyro_z(); break;
					}
				}
				sensor->t_stamp = TIME_STAMP;
				break;}
			case SENS_ACCEL:{
				if(sensor->device != NULL && sensor->device->is_setup){
					switch(sensor->imu_axis){
						case 'X': sensor->value = device->imu->get_accel_x(); break;
						case 'Y': sensor->value = device->imu->get_accel_y(); break;
						case 'Z': sensor->value = device->imu->get_accel_z(); break;
					}
				}
				sensor->t_stamp = TIME_STAMP;
				break;}
			case SENS_ROT_ENC:{
				if(sensor->device != NULL && sensor->device->is_setup){
					float temp = get_rot_encoder_value(sensor);
					sensor->value = 3.14159265;
					sensor->t_stamp = TIME_STAMP;
				}
				break;}
			case SENS_BLDC_ENC:{
				if(sensor->device != NULL && sensor->device->is_setup){
					// vesc->request_mc_values() should have been called during maintain motors
					sensor->device->vesc->update_mc_values(); 				// read the available packet
					sensor->value = 1.0*sensor->device->vesc->get_rpm();	// get the RPM
					sensor->t_stamp = TIME_STAMP;
				}
				break;}
			case SENS_POT_ENC:{
				if(sensor->device != NULL && sensor->device->is_setup){
					uint16_t raw 		= 0;
	        		sensor->value_good = sensor->device->adc->read_channel(sensor->adc_channel_config, &raw);
					if(sensor->value_good){
						sensor->t_stamp = TIME_STAMP;
						sensor->value = signed_raw/32767.0;
						debug("Pot frac:\t"+String(sensor->value, 3));
					}else{
						debug("Pot. read ERROR");
					}
				}
				break;}
			case SENS_LOOKY_ENC:{
				//to get status:
				//XYZrobotServoStatus status = servo.readStatus();
				// sensor->device->servo_status = sensor->device->servo->readStatus();
				sensor->t_stamp = TIME_STAMP;
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
				int dt 			= (TIME_STAMP - motor->t_stamp) * 1000000; 	// delta-T in sec
				int sign 		= (int)(motor->setpt/fabs(motor->setpt));
				float proposed_delta = (motor->setpt - motor->last_rpm)/dt; // calculate proposed delta-RPM
				if(fabs(proposed) =< motor->max_delta){ 					// check if the proposed delta is allowed
					motor->last_rpm = motor->setpt; 						// cool, it's allowed, so this will be our rpm to send
				}else{ 														// TOO MUCH DELTA
					motor->last_rpm += (sign * motor->max_delta * dt); 		// increment by max, and consider the sign
				}
				int rpm = (int)(motor->last_rpm * motor->rpm_factor); 		// convert to "eRPM" and cast to int
				debug("Updating VESC: "+String(rpm, DEC));
				motor->device->vesc->set_rpm(rpm); 							// actually send the rpm
				motor->device->vesc->request_mc_values(); 					// resonse packet should be ready when we call update sensors
				break;}
			case MTR_SABERTOOTH:{
				//PID controls:
				//rot_val = dx*(target-actual) + dy*(
				debug("Updating sabertooth motor");

				uint8_t data[3] = {};
				package_DAC_voltage(MOTOR_ANLG_CENTER_V, data); 	// make sure motor is stopped

				SPI.beginTransaction(*(motor->device->spi_settings));
				digitalWrite(motor->device->spi_cs, LOW);
				PAUSE_SHORT;
				SPI.transfer(data, 3);
				digitalWrite(motor->device->spi_cs, HIGH);
				SPI.endTransaction();

				break;}
			case MTR_LOOKY:{
				if(motor->setpt != motor->last_setpt){
					// Herkulex.moveOneAngle(servoID, angle, time_ms, iLed??)
					Herkulex.moveOneAngle(motor->looky_id, motor->setpt, 200, 2);
				}
				debug("Updating Looky");
				break;}
		}
	}
}

void debug(String s){
	DEBUG.println(s);

	DEBUG.flush();
	// delay((int)(s.length()*0.1));
	// s.concat('\n');
	// int len = s.length();
	// unsigned char data[len] = {};
	// s.getBytes(data, len);

	// SPI.beginTransaction(Debug_SPI_settings);
	// digitalWrite(DEBUG_SELECT_PIN, LOW);
	// PAUSE_SHORT;
	// SPI.transfer(data, len);
	// digitalWrite(DEBUG_SELECT_PIN, HIGH);
	// SPI.endTransaction();
}


void package_DAC_voltage(float v, uint8_t* data){
	// set the mode to normal
	data[0] &= 0x00;

	uint16_t temp = (uint16_t)(((v/5.0)*65535)+0.5);
	debug("DAC voltage: " + String(temp));
	data[1] = (temp >> 8) & 0xFF;
	data[2] = (temp) & 0xFF;
}

void Encoder_ISR(){
	sensor_infos[EXC_TRANS_ENC].rots += 1;
	sensor_infos[EXC_TRANS_ENC].value = sensor_infos[EXC_TRANS_ENC].rots * 2.0 * PI; 	// rotation in radians
}

// @return radians of rotation
float get_rot_encoder_value(SensorInfo* encoder){
	int n = 0;
	uint8_t rpy = 0;
	uint16_t value = 0;
	SPI.beginTransaction(*encoder->device->spi_settings);
	digitalWrite(encoder->device->spi_cs, LOW);
	PAUSE_SHORT;

	rpy = SPI.transfer(ROT_ENC_RD_POS); 	// send the command

	while(rpy != ROT_ENC_RD_POS && n < 10){ // wait for a response or timeout
		rpy = SPI.transfer(0x00);
		n++;
	}
	if(rpy == ROT_ENC_RD_POS){
		value = ((SPI.transfer(0x00) & 0x0F) << 8 ); 	// get the high-order byte
		value |= (SPI.transfer(0x00) & 0xFF); 			// add the low-order byte
	}

	digitalWrite(encoder->device->spi_cs, HIGH);
	SPI.endTransaction();

	float retval = (value/4095.0)*2.0*PI; 	// radians of rotation
	return retval;
}

#endif
