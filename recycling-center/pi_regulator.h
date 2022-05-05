#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

enum PI_State{
	LOOKING_FOR_TARGET,
	GO_TO_TARGET,
	PICKING_OBJ,
	DROPPING_OBJ,
	WAIT
};

//start the PI regulator thread
void pi_regulator_start(void);
bool is_action_done(void);
void switch_state(enum PI_State new_state, bool is_looking_for_base);

#endif /* PI_REGULATOR_H */
