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

static enum PI_State current_state = LOOKING_FOR_TARGET;
static bool current_action_done = true;
static int16_t look_direction = 1;

//simple PI regulator implementation
int16_t distance_pi_regulator(int16_t distance, int16_t goal){

	if (distance > MAX_DISTANCE) {
		return 0;
	}

	int16_t error = 0;
	int16_t speed = 0;

	static int16_t sum_error_dist = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want
	if(abs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error_dist += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error_dist > MAX_SUM_ERROR){
		sum_error_dist = MAX_SUM_ERROR;
	}else if(sum_error_dist < -MAX_SUM_ERROR){
		sum_error_dist = -MAX_SUM_ERROR;
	}

	speed = KP_DIST * error + KI_DIST * sum_error_dist;

    return speed;
}

int16_t rotate_pi_regulator(uint16_t line_position){

	int16_t error = 0;
	int16_t speed = 0;

	// we want the object to be in the center
	error = line_position - (IMAGE_BUFFER_SIZE / 2);

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(abs(error) < ERROR_THRESHOLD){
		return 0;
	}

	speed = KP_ROTA * error ;

    return speed;
}

bool check_object_center(void) {
	if (abs(get_line_position() - (IMAGE_BUFFER_SIZE/2)) < TOF_LATERAL_THRESHOLD) {
		return true;
	}
	return false;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t r_speed = 0;
    int16_t l_speed = 0;

    while(1){
        time = chVTGetSystemTime();
        if (current_state == LOOKING_FOR_TARGET) {

        	r_speed = -look_direction * ROTATION_SPEED;
        	l_speed =  look_direction * ROTATION_SPEED;

        	if (get_line_position() != NOTFOUND) {
        		r_speed = 0;
				l_speed = 0;
				current_state = WAIT;
        	}

        } else if (current_state == GO_TO_TARGET) {
			//computes the speed to give to the motors
			//distance is modified by the time_of_flight thread
			if ((get_distance() < TOF_ONLY_DIST) || check_object_center()) {

				if (abs(get_distance() - GOAL_DISTANCE) < GOAL_THRESHOLD) {
					r_speed = 0;
					l_speed = 0;
					current_state = WAIT;
				} else {
					r_speed = distance_pi_regulator(get_distance(), GOAL_DISTANCE);

					if(abs(r_speed) < MIN_SPEED) {
						r_speed = 0;
					}
					l_speed = r_speed;
				}

			} else {
				if (get_line_position() == NOTFOUND) {
					r_speed = 0;
					l_speed = 0;
				} else {
					r_speed = -rotate_pi_regulator(get_line_position());
					l_speed = -r_speed;
				}
			}
        } else if (current_state == PICKING_OBJ) {
        	r_speed = -ROTATION_SPEED;
        	l_speed = ROTATION_SPEED;

        	if (right_motor_get_pos() < -NB_STEPS_PICK) {
        		right_motor_set_pos(0);
				r_speed = 0;
				l_speed = 0;
				current_state = WAIT;
        	}
        } else if (current_state == DROPPING_OBJ) {
			r_speed = -ROTATION_SPEED;
			l_speed = -ROTATION_SPEED;

			if (right_motor_get_pos() < -NB_STEPS_DROP) {
				right_motor_set_pos(0);
				r_speed = 0;
				l_speed = 0;
				current_state = WAIT;
			}
        } else {
        	// in wait or end mode
        	if (current_state == WAIT) {
        		current_action_done = true;
        	}
        	r_speed = 0;
        	l_speed = 0;
        }

        //applies the speed from the PI regulator
		right_motor_set_speed(r_speed);
		left_motor_set_speed(l_speed);

        chThdSleepUntilWindowed(time, time + MS2ST(MOTOR_UPDT_TIME));
    }
}

void pi_regulator_start(void){

	right_motor_set_pos(0);

	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}

bool is_action_done(void) {
	return current_action_done;
}

void switch_state(enum PI_State new_state, bool is_looking_for_base, int16_t look_dir) {
	right_motor_set_pos(0);
	current_action_done = false;

	set_looking_for_base(is_looking_for_base);

	look_direction = look_dir;

	current_state = new_state;
}
