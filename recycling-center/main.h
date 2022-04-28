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
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			65.0f
#define MIN_SPEED				150
#define MAX_DISTANCE 			1000.0f
#define ERROR_THRESHOLD			10	//[mm]
#define KP_DIST					15.0f
#define KI_DIST 				1.5f	//must not be zero
#define KP_ROTA					6.0f
#define KI_ROTA 				1.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI_DIST)

#define COEFF_MOD_CAM			0.00000055f //to multiply with the cube of the distance to modify the error of the edge of the camera
#define HTHRESHOLD				30  	//minimum value of the jump to be detected
#define NOTFOUND				650		//line not found uint16 greater than IMAGE_BUFFER_SIZE
#define RED_THRESHOLD			35		//experimentally found value
#define GREEN_THRESHOLD			45		//experimentally found value

#define TOF_LATERAL_THRESHOLD	10

#define MOTOR_UPDT_TIME			10 // [ms]
#define ROTATION_SPEED			200 // [steps/s]
#define WHEEL_DIAMETER			41 // [mm]
#define ROBOT_DIAMETER			53  // [mm]
#define NB_STEPS_PER_TURN		2000
#define TURN_SPEED				(360 * ROTATION_SPEED / NB_STEPS_PER_TURN) // [°/s]
#define ANGLE_PER_UPDATE		(WHEEL_DIAMETER * TURN_SPEED / ROBOT_DIAMETER * MOTOR_UPDT_TIME / 10) // [c°]
#define DROP_ANGLE				18000 // [c°]

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
