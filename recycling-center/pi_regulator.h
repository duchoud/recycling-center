#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

// This enum is used to determine what task the robot as to perform currently
enum PI_State{
	LOOKING_FOR_TARGET,
	GO_TO_TARGET,
	PICKING_OBJ,
	DROPPING_OBJ,
	STEPPING_BACK,
	WAIT,
	PI_END
};

/**
* @brief 			Inits the thread to control the motors
*/
void pi_regulator_start(void);

/**
* @brief 			For the main to know when the current action is finished
*
* @return			True if the action is finished
*/
bool is_action_done(void);

/**
 * @brief 						Switch the state of the regulators FSM to indicate what to do
 *
 * @param new_state 			The new state in which the regulators are
 * @param is_looking_for_base	Indicates if we are going for the base as it changes the colour tracking
 * @param look_dir				Depending on the state, we rotate clockwise (=1) or anti-clockwise (=-1)
 */
void switch_state(enum PI_State new_state, bool is_looking_for_base, int16_t look_dir);

#endif /* PI_REGULATOR_H */
