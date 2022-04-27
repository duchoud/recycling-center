#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			65.0f
#define MIN_SPEED				150
#define MAX_DISTANCE 			1000.0f
#define ERROR_THRESHOLD			10	//[mm] because of the noise of the camera
#define KP						15.0f
#define KI 						1.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define TOF_LATERAL_THRESHOLD	10

#define MOTOR_UPDT_TIME			10 // [ms]
#define WHEEL_DIAMETER			41 // [cm]
#define ROBOT_DIAMETER			1  // [cm]
#define NB_STEPS_PER_TURN		1000
#define TURN_SPEED				(2*PI*MIN_SPEED / NB_STEPS_PER_TURN) // [rad/s]
#define ANGLE_PER_UPDATE		(WHEEL_DIAMETER / ROBOT_DIAMETER * TURN_SPEED * MOTOR_UPDT_TIME) // [rad]
#define DROP_ANGLE				PI

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
