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
	GET_BASE
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
	//pi_regulator_start();
	process_image_start();

	enum FSM current_state = FIND_BASE;
	switch_state(LOOKING_FOR_TARGET, true);

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
    	if (is_action_done()) {
    		switch (current_state) {
    		case FIND_OBJECT:
    			current_state = GET_OBJECT;
    			switch_state(GO_TO_TARGET, false);
    			break;

    		case GET_OBJECT:
    			current_state = PICK_OBJECT;
    			switch_state(PICKING_OBJ, false);
    			break;

    		case PICK_OBJECT:
    			current_state = FIND_WALL;
    			switch_state(LOOKING_FOR_TARGET, false);
    			break;

    		case FIND_WALL:
    			current_state = GET_WALL;
    			switch_state(GO_TO_TARGET, false);
    			break;

    		case GET_WALL:
    			current_state = DROP_OBJECT;
    			switch_state(DROPPING_OBJ, false);
    			break;

    		case DROP_OBJECT:
    			current_state = FIND_BASE;
    			switch_state(LOOKING_FOR_TARGET, true);
    			break;

    		case FIND_BASE:
    			current_state = GET_BASE;
    			switch_state(GO_TO_TARGET, true);
    			break;

    		case GET_BASE:
    			current_state = FIND_OBJECT;
    			switch_state(LOOKING_FOR_TARGET, false);
    			break;

    		default:
    			break;
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
