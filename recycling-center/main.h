#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the camera and process image
#define IMAGE_BUFFER_SIZE		640 			//[px]
#define WIDTH_SLOPE				20				//[px]
#define MIN_LINE_WIDTH			10				//[px]
#define COEFF_MOD_CAM			0.00000055f 	//to multiply with the cube of the distance to modify the error of the edge of the camera
#define HTHRESHOLD				30  			//minimum value of the jump to be detected
#define NOTFOUND				650				//line not found uint16 greater than IMAGE_BUFFER_SIZE
//this thresholds were measured by sending the values to the computer and are dependent of the ambient lighting
#define RED_THRESHOLD			45				//experimentally found value
#define GREEN_THRESHOLD			55				//experimentally found value
#define BLACK_THRESHOLD			35				//experimentally found value
#define MAX_COLOUR				255				//max value on 8 bits

//constants for the pi regulators
#define GOAL_DISTANCE 			40 				// [mm]
#define GOAL_THRESHOLD			10 				// [mm]
#define TOF_ONLY_DIST			100				// [mm]
#define TOF_LATERAL_THRESHOLD	60	  			// [px]
#define MAX_DISTANCE 			1000 			// [mm]
#define KP_DIST					3 				// [steps/s/mm]
#define KI_DIST 				5				// [steps/s/mm]
#define MAX_SUM_ERROR_DIST 		(MOTOR_SPEED_LIMIT/KI_DIST) // to avoid uncontrolled growth
#define KP_ROTA					2				// [steps/s/px]

//speed for the motors
#define MAX_LINEAR_SPEED		450 			// [steps/s]
#define BACKWARD_SPEED			350				// [steps/s]
#define ROTATIONAL_SPEED		200				// [steps/s]

//constants for precise displacement
#define MOTOR_UPDT_TIME			10 				// [ms]
#define WHEEL_DIAMETER			41 				// [mm]
#define ROBOT_DIAMETER			52  			// [mm]
#define NB_STEPS_PER_TURN		1000			// [steps/turn]
#define ANGLE_TO_PICK			215				// [°]
#define ANGLE_TO_DROP			60				// [°]
#define DEG_FULL_TURN			360				// [°]
#define RAD_FULL_TURN			(2 * M_PI)		// [rad]
// calculate the number of steps to do certain actions
#define NB_STEPS_PICK			(NB_STEPS_PER_TURN * ROBOT_DIAMETER / WHEEL_DIAMETER * ANGLE_TO_PICK / DEG_FULL_TURN)
#define NB_STEPS_DROP_ANGLE		(NB_STEPS_PER_TURN * ROBOT_DIAMETER / WHEEL_DIAMETER * ANGLE_TO_DROP / DEG_FULL_TURN)
#define DROP_DIST				80 				// [mm]
#define NB_STEPS_DROP			(DROP_DIST * NB_STEPS_PER_TURN / (WHEEL_DIAMETER / 2) / RAD_FULL_TURN)
#define BACK_DIST				20 				// [mm]
#define NB_STEPS_BACK			(BACK_DIST * NB_STEPS_PER_TURN / (WHEEL_DIAMETER / 2) / RAD_FULL_TURN)

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
