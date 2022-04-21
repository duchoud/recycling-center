/*
 * distances.h
 *
 *  Created on: 21 avr. 2022
 *      Author: marin
 */

#ifndef DISTANCES_H_
#define DISTANCES_H_

/**
* @brief   Calls the start for the IR sensors and the Time Of Flight sensor.
*
*/
void distances_start(void);

/**
* @brief   	Calls the different IR sensors to check if the e-puck is too close to an object.
*
* @return	8 bits number; each one represent one sensor,
* 			it is set to 1 if the sensor detects an object and 0 otherwise
*/
uint8_t collision_check(void);

/**
* @brief   Return the distance of the Time Of Flight sensor.
*
*/
uint16_t get_distance(void);

#endif /* DISTANCES_H_ */
