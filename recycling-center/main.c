#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "spi_comm.h"
#include "spi_comm.h"
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>

#include <pi_regulator.h>
#include <process_image.h>
#include <distances.h>

enum FSM {
	FIND_OBJECT,
	GET_OBJECT,
	PICK_OBJECT,
	FIND_WALL,
	GET_WALL,
	DROP_OBJECT,
	FIND_BASE,
	GET_BASE,
	FSM_END
};

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();
	//inits the sensors
	distances_start();
	spi_comm_start();

	//stars the threads for the pi regulator and the processing of the image
	pi_regulator_start();
	process_image_start();

	// The robot goes to its base at the beginning as initialization, it will then return there a 2nd time
	// at the end and should stay there, we use a counter to track this
	uint8_t end_fsm = 0;
	enum FSM current_state = FIND_BASE;
	switch_state(LOOKING_FOR_TARGET, true, 1);

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
    	//chprintf((BaseSequentialStream *)&SDU1, "state=%d  ", current_state);
    	if (is_action_done()) {

    		enum PI_State new_pi_state = END;
    		bool is_looking_for_base = false;

    		if (current_state != FSM_END) {
				// If it equals 1, the robot turns clockwise
				int16_t look_dir = 1;

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
					if (end_fsm == 1) {
						current_state = FIND_OBJECT;
						new_pi_state = LOOKING_FOR_TARGET;
						look_dir = -1;
					} else if (end_fsm == 2) {
						current_state = FSM_END;
						new_pi_state = PI_END;
					}
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
