////////////////////////////////////////////////////////////////////////////////
// 	MAIN
////////////////////////////////////////////////////////////////////////////////

#include "VESC/VESC.h"
#include "LSM6DS3.h"
#include "values_and_types.h"
#include "data_setup.h"
#include "io.h"
#include "motor_sensor_setup.h"
#include "client_comms.h"

void setup(){
	DEBUG.begin(115200);
	SPI.begin();
	attachInterrupt(digitalPinToInterrupt(ENCODER_INDEX_PIN), Encoder_ISR, RISING);
	
	debug("CWRUbotix Hardware Controller");
	
	debug("setup devices");
	setup_devices();
	
	debug("setup sensors");
	setup_sensors(); 		// sets up sensor data structures
	
	debug("setup motors");
	setup_motors(); 		// sets up motor data structures
	
	debug("init device comms");
	init_device_comms(); 	// configures pins according to motor & sensor data
	
	debug("init sensors");
	init_sensors(); 		// initializes the sensors, this happens as long as the re
	
	debug("check estop pin");
	if(digitalRead(sensor_infos[ESTOP_SENSE_INDEX].pin) == HIGH){
		debug("init motors");
		init_motors();
	}else{
		debug("wait to init motors");
	}
	// wait on initializing motors
	debug("setup complete");
}

void loop(){
	int start_time = micros();
	estop_state = digitalRead(sensor_infos[ESTOP_SENSE_INDEX].pin);
	sensor_infos[ESTOP_SENSE_INDEX].t_stamp = TIME_STAMP;

	if(estop_state==HIGH && estop_state!=estop_state_last){
		debug("init motors");
		init_motors();
	} estop_state_last = estop_state;

	FAULT_T read_fault = NO_FAULT;
	if(CLIENT.available()){
		// do the client receive stuff
		debug("reading from client");
		read_fault = read_from_client();

		// answer the client
		debug("answering client");
		reply_to_client(read_fault);
	}

	update_sensors();

	if(estop_state == HIGH){
		// do motor things
		debug("maintaining motors");
		maintain_motors();
	}
	float loop_t = (micros() - start_time)/1000;
	if(loop_t > 0.0009){
		// String time_str = String("Loop-time in microsec: ");
		// time_str.concat(String(loop_t, 3));
		// debug(time_str);
	}
	
}