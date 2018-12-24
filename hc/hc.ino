////////////////////////////////////////////////////////////////////////////////
// 	MAIN
////////////////////////////////////////////////////////////////////////////////

#include "values_and_types.h"
#include "data_setup.h"
#include "io.h"
#include "motor_sensor_setup.h"
#include "client_comms.h"
#include "VESC/VESC.h"



void setup(){
	DEBUG.begin(115200);
	DEBUG.println("CWRUbotix Hardware Controller");
	delay(10);
	attachInterrupt(digitalPinToInterrupt(ENCODER_INDEX_PIN), Encoder_ISR, RISING);
	DEBUG.println("setup devices");
	delay(10);
	setup_devices();
	DEBUG.println("setup sensors");
	delay(10);
	setup_sensors(); 		// sets up sensor data structures
	DEBUG.println("setup motors");
	delay(10);
	setup_motors(); 		// sets up motor data structures
	DEBUG.println("init device comms");
	delay(10);
	init_device_comms(); 	// configures pins according to motor & sensor data
	DEBUG.println("init sensors");
	delay(10);
	init_sensors(); 		// initializes the sensors, this happens as long as the re
	DEBUG.println("check estop pin");
	delay(10);
	if(digitalRead(sensor_infos[ESTOP_SENSE_INDEX].pin) == HIGH){
		DEBUG.println("init motors");
		delay(10);
		init_motors();
	}
	// wait on initializing motors
	DEBUG.println("setup complete");
	delay(10);
}

void loop(){
	SensorInfo* EStop = &sensor_infos[ESTOP_SENSE_INDEX];
	estop_state = digitalRead(EStop->pin);

	if(estop_state==HIGH && estop_state!=estop_state_last){
		init_motors();
	} estop_state_last = estop_state;

	FAULT_T read_fault = NO_FAULT;
	if(CLIENT.available()){
		// do the client receive stuff
		read_fault = read_from_client();

		// answer the client
		reply_to_client(read_fault);
	}

	update_sensors();

	if(estop_state == HIGH){
		// do motor things
		maintain_motors();
	}

	debug();

	delay(10);

}