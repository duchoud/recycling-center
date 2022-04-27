#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <distances.h>

//simple PI regulator implementation
int16_t pi_regulator(int16_t distance, int16_t goal){

	if (distance > MAX_DISTANCE) {
		return 0;
	}

	int16_t error = 0;
	int16_t speed = 0;

	static int16_t sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(abs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return speed;
}

bool check_object_center(void) {
	if (abs(get_line_position() - IMAGE_BUFFER_SIZE/2) < TOF_LATERAL_THRESHOLD) {
		return true;
	}
	return false;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();
        
        //computes the speed to give to the motors
        //distance is modified by the time_of_flight thread
        if (check_object_center()) {
        	speed = pi_regulator(get_distance(), GOAL_DISTANCE);

        	if(abs(speed) < MIN_SPEED) {
				speed = 0;
			}
        } else {
        	speed = 0;
        	speed_correction = MIN_SPEED;
        }

        //applies the speed from the PI regulator and the correction for the rotation
		right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
		left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
