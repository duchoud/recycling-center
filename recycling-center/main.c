#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <motors.h>
#include <camera/po8030.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <main.h>
#include <pi_regulator.h>
#include <process_image.h>

//finite state machine to describe the current action of the robot
enum FSM {
	FIND_OBJECT,
	GET_OBJECT,
	PICK_OBJECT,
	FIND_WALL,
	GET_WALL,
	DROP_OBJECT,
	FIND_BASE,
	GET_BASE,
	STEP_BACK,
	FSM_END
};

/*
static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}*/

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    //serial_start();
    //start the USB communication
    //usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();
	//inits the TOF
	VL53L0X_start();

	//stars the threads for the pi regulator and the processing of the image
	pi_regulator_start();
	process_image_start();

	// The robot goes to its base at the beginning as initialization, it will then return there a 2nd time
	// at the end and should stay there, we use a counter to track this
	uint8_t end_fsm = 0;


	//when we start, the robot has to find his base. Thus we start with the state "looking for target"
	enum PI_State new_pi_state = LOOKING_FOR_TARGET;
	enum FSM current_state = FIND_BASE;
	//we look for the base
	bool is_looking_for_base = true;
	// +1: clockwise and -1: counterclockwise
	int16_t look_dir = 1;

	//init the state of the regulators
	switch_state(new_pi_state, is_looking_for_base, look_dir);

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
    	//the main is the master of the FSM so it checks if the robot has finished his current action
    	//if so it calls next
    	if (is_action_done()) {

    		is_looking_for_base = false;
    		look_dir = 1;

    		if (current_state != FSM_END) {
    			//has not finished the loop
    			//this switch is to set the right variable depending on which is the next state
				switch (current_state) {
				case FIND_OBJECT:
					current_state = GET_OBJECT;
					new_pi_state = GO_TO_TARGET;
					break;

				case GET_OBJECT:
					current_state = PICK_OBJECT;
					new_pi_state = PICKING_OBJ;
					break;

				case PICK_OBJECT:
					current_state = FIND_WALL;
					new_pi_state = LOOKING_FOR_TARGET;
					look_dir = 1;
					break;

				case FIND_WALL:
					current_state = GET_WALL;
					new_pi_state = GO_TO_TARGET;
					break;

				case GET_WALL:
					current_state = DROP_OBJECT;
					new_pi_state = DROPPING_OBJ;
					break;

				case DROP_OBJECT:
					current_state = FIND_BASE;
					new_pi_state = LOOKING_FOR_TARGET;
					look_dir = -1;
					is_looking_for_base = true;
					break;

				case FIND_BASE:
					current_state = GET_BASE;
					new_pi_state = GO_TO_TARGET;
					is_looking_for_base = true;
					break;

				case GET_BASE:
					end_fsm++;
					//in our FSM the robot starts by searching the base and it ends by doing the same task
					//so we count how many time it does this task to end the FSM
					if (end_fsm == 1) {
						current_state = STEP_BACK;
						new_pi_state = STEPPING_BACK;
					} else if (end_fsm == 2) {
						current_state = FSM_END;
						new_pi_state = PI_END;
					}
					break;
				case STEP_BACK:
					current_state = FIND_OBJECT;
					new_pi_state = LOOKING_FOR_TARGET;
					look_dir = -1;
					break;
				default:
					break;
				}

				switch_state(new_pi_state, is_looking_for_base, look_dir);
    		}
    	}
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
