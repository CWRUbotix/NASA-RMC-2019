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
				// read ADC among other things
				break;}
			case SENS_GYRO:{
				switch(sensor->imu_axis){
					case 'X': sensor->value = device->imu->get_gyro_x(); break;
					case 'Y': sensor->value = device->imu->get_gyro_y(); break;
					case 'Z': sensor->value = device->imu->get_gyro_z(); break;
				}
				sensor->t_stamp = TIME_STAMP;
				break;}
			case SENS_ACCEL:{
				switch(sensor->imu_axis){
					case 'X': sensor->value = device->imu->get_accel_x(); break;
					case 'Y': sensor->value = device->imu->get_accel_y(); break;
					case 'Z': sensor->value = device->imu->get_accel_z(); break;
				}
				sensor->t_stamp = TIME_STAMP;
				break;}
			case SENS_ROT_ENC:{
				float temp = get_rot_encoder_value(sensor);
				sensor->value = 3.14159265;
				sensor->t_stamp = TIME_STAMP;
				break;}
			case SENS_BLDC_ENC:{
				sensor->device->vesc->update_mc_values(); 				// read the available packet
				sensor->value = 1.0*sensor->device->vesc->get_rpm();	// get the RPM
				sensor->t_stamp = TIME_STAMP;
				break;}
			case SENS_POT_ENC:{
        
        //sensor->value = sensor->device->adc->get_value();
				// sensor->t_stamp = TIME_STAMP;
				break;}
			case SENS_LOOKY_ENC:{
        //to get status:
        //XYZrobotServoStatus status = servo.readStatus();
        sensor->device->servo_status = sensor->device->servo->readStatus();
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
				debug("Updating VESC");
				int sign = motor->setpt/fabs(motor->setpt);
				int rpm = (fabs(motor->setpt) > motor->max_setpt ? (int)sign*motor->max_setpt : (int)motor->setpt );
				motor->device->vesc->set_rpm(rpm); 			// set the motor speed
				motor->device->vesc->request_mc_values(); 	// resonse packet should be ready when we call update sensors
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
				//if(somevalue change direction){}
				motor->device->servo->setPosition(int(setpt+0.5), 0);
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
