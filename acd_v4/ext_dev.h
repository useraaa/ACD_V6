/*
 * ext_dev.h
 *
 *  Created on: 07.07.2012
 *      Author: user
 */

#ifndef EXT_DEV_H_
#define EXT_DEV_H_


typedef enum extdevs {
	no_extdev = 0,
	lock_dev,
	turnstile1,
	turnstile2
} extdev_t;

typedef enum turn_state {
	TURN_STATE_UNLOCK_A = 1,
	TURN_STATE_UNLOCK_B,
	TURN_STATE_UNLOCK_AB,
	TURN_STATE_STOP,
	TURN_STATE_PASS_A,
	TURN_STATE_PASS_B,
	TURN_STATE_PASS_A_COMPL,
	TURN_STATE_PASS_B_COMPL
} turn_state_t;

extern extdev_t curr_extdev;

int extdev_init (extdev_t dev);
void irq_proc (u8 state);
u8 lock_ctl (u32 arg);
u8 turn_ctl (u32 arg1, u32 arg2);

#endif /* EXT_DEV_H_ */
