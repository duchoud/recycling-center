#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				20
#define MIN_LINE_WIDTH			10
#define ROTATION_THRESHOLD		10
#define GOAL_DISTANCE 			45.0f // [mm]
#define GOAL_THRESHOLD			10.0f // [mm]
#define TOF_ONLY_DIST			100.0f
#define MIN_SPEED				50
#define MAX_DISTANCE 			1000.0f
#define ERROR_THRESHOLD			10	//[mm]
#define KP_DIST					3.0f
#define KI_DIST 				5.0f	//must not be zero
#define MAX_SUM_ERROR_DIST 		(MOTOR_SPEED_LIMIT/KI_DIST)
#define KP_ROTA					0.5f

#define COEFF_MOD_CAM			0.00000055f //to multiply with the cube of the distance to modify the error of the edge of the camera
#define HTHRESHOLD				30  	//minimum value of the jump to be detected
#define NOTFOUND				650		//line not found uint16 greater than IMAGE_BUFFER_SIZE
#define RED_THRESHOLD			45		//experimentally found value
#define GREEN_THRESHOLD			55		//experimentally found value
#define BLACK_THRESHOLD			35		//experimentally found value
#define THRESHOLD_COLOUR		20		//experimentally found value
#define MAX_COLOUR				255		//max value on 8 bits

#define TOF_LATERAL_THRESHOLD	100

#define MOTOR_UPDT_TIME			10 // [ms]
#define ROTATION_SPEED			200 // [steps/s]
#define WHEEL_DIAMETER			41.0f // [mm]
#define ROBOT_DIAMETER			52.0f  // [mm]
#define NB_STEPS_PER_TURN		1000
// This is the number of motor steps required for the robot to do a u-turn (hence the / 2)
#define NB_STEPS_PICK			(NB_STEPS_PER_TURN * ROBOT_DIAMETER / WHEEL_DIAMETER * 200 / 360)
#define DROP_DIST				80 // [mm]
#define NB_STEPS_DROP			(DROP_DIST / (WHEEL_DIAMETER / 2) * 1000 / (2 * M_PI))

#define BACK_DIST				40 // [mm]
#define NB_STEPS_BACK			(BACK_DIST / (WHEEL_DIAMETER / 2) * 1000 / (2 * M_PI))

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
