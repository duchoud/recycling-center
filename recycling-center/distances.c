/*
 * distances.c
 *
 *  Created on: 21 avr. 2022
 *      Author: marin
 */

#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h>

#define NB_IR_SENSORS 8
#define DISTANCE_MIN 10

void distances_start(void) {

	proximity_start();
	calibrate_ir();

	VL53L0X_start();

}

uint8_t collision_check(void) {
	uint8_t collisions = 0;
	for(int i = 0; i < NB_IR_SENSORS; i++) {
		if (get_calibrated_prox(i) < DISTANCE_MIN) {
			collisions |= (1 << i);
		}
	}
	return collisions;
}


uint16_t get_distance(void) {
	return VL53L0X_get_dist_mm();
}
