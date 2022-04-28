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
#define WIDTH_SLOPE				20
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2 
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			65.0f
#define MIN_SPEED				150
#define MAX_DISTANCE 			1000.0f
#define ERROR_THRESHOLD			10	//[mm] because of the noise of the camera
#define KP						15.0f
#define KI 						1.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define COEFF_MOD_CAM			0.00000055f //to multiply with the cube of the distance to modify the error of the edge of the camera
#define HTHRESHOLD				30  	//minimum value of the jump to be detected
#define NOTFOUND				650		//line not found uint16 greater than IMAGE_BUFFER_SIZE
#define RED_THRESHOLD			35		//experimentally found value
#define GREEN_THRESHOLD			45		//experimentally found value

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
