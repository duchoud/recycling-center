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
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			50.0f
#define TOF_ONLY_DIST			100.0f
#define MIN_SPEED				100
#define MAX_DISTANCE 			1000.0f
#define ERROR_THRESHOLD			10	//[mm]
#define KP_DIST					10.0f
#define KI_DIST 				1.0f	//must not be zero
#define KP_ROTA					1.0f
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI_DIST)

#define COEFF_MOD_CAM			0.00000055f //to multiply with the cube of the distance to modify the error of the edge of the camera
#define HTHRESHOLD				30  	//minimum value of the jump to be detected
#define NOTFOUND				650		//line not found uint16 greater than IMAGE_BUFFER_SIZE
#define RED_THRESHOLD			35		//experimentally found value
#define GREEN_THRESHOLD			55		//experimentally found value

#define TOF_LATERAL_THRESHOLD	50

#define MOTOR_UPDT_TIME			20 // [ms]
#define ROTATION_SPEED			200 // [steps/s]
#define WHEEL_DIAMETER			41.0f // [mm]
#define ROBOT_DIAMETER			53.0f  // [mm]
#define NB_STEPS_PER_TURN		1000
#define TURN_SPEED				(360 * ROTATION_SPEED / NB_STEPS_PER_TURN) // [°/s]
#define ANGLE_CORRECTION		0.8f
#define ANGLE_PER_UPDATE		(ANGLE_CORRECTION * WHEEL_DIAMETER * TURN_SPEED / ROBOT_DIAMETER * MOTOR_UPDT_TIME / 10) // [c°]
#define DROP_ANGLE				18000 // [c°]

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
