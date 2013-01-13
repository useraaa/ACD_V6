/*
 * ext_dev.cpp
 *
 *  Created on: 07.07.2012
 *      Author: user
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>

#include "client.h"
#include "ext_dev.h"

#define TURN_DEF_TMO 5



void update_dev_state (extdev_t dev, u8 new_state);



enum IO_TRIG
{
  EXTI_Trigger_Rising = 0x08,
  EXTI_Trigger_Falling = 0x0C,
  EXTI_Trigger_Rising_Falling = 0x10
};


extdev_t curr_extdev = no_extdev;
volatile u8 input_io_state = 0;
// lock vars
u8 close_door_when_sens = 0;
u8 lock_state = 0;

/*! turnstile vars */
u8 turn_timeout = 0;
u8 turn_state = 0;


void turn_alarm ( int sig_num )
{
	printf("DEBUG %s\n", __func__);

	if (turn_state == TURN_STATE_PASS_A)
		turn_state = TURN_STATE_PASS_A_COMPL;
	else if (turn_state == TURN_STATE_PASS_B)
		turn_state = TURN_STATE_PASS_B_COMPL;

	turn_ctl(TURN_CMD_STOP, 0);
}

/*!
 * Включение/выключение внешних исполнительных устройств
 * @param dev - тип устройства
 * @return
 */
int extdev_init (enum extdevs dev)
{
	u8 stmiobuf[29];
	printf ("%s - %d\n", __func__, dev);

	// setup irq;
	memset (stmiobuf, 0, sizeof(stmiobuf));

	if (curr_extdev != no_extdev) {
		printf("External device was already initialized\n");
		return TURN_ERR_DEV_NOT_READY;
	}

	curr_extdev = dev;

	if (curr_extdev == lock_dev) {
		stmiobuf[0] = EXTI_Trigger_Rising_Falling;	// sens1
	}
	if (curr_extdev == turnstile1) {

		stmiobuf[0] = EXTI_Trigger_Falling;	// sens1
		stmiobuf[1] = EXTI_Trigger_Falling;	// sens2
//		stmiobuf[2] = EXTI_Trigger_Falling;	// next release
		turn_ctl (TURN_CMD_STOP, 0);
		turn_timeout = TURN_DEF_TMO;
		signal(SIGALRM, turn_alarm);
	}

	transmit(13, stmiobuf);

	memset (stmiobuf, 0, sizeof(stmiobuf));
	transmit(12, stmiobuf);
	irq_proc (~stmiobuf[0]);
	return 0;
}
/*!
 * Управление турникетом
 * port map:
 * OUT(0)	- ULA
 * OUT(1)	- ULB
 * OUT(2)	- STOP
 * IN(0)	- SENS1
 * IN(1)	- SENS2
 * IN(2)	- ERR_SENS
 * @param arg команда/запрос
 * @return
 */
u8 turn_ctl (u32 arg1, u32 arg2)
{
	int ret = 0;
	u8 stmiobuf[33];

	printf("TURN CTL: %d %d %d\n", turn_state, arg1, arg2);

	if (curr_extdev != turnstile1) {
		printf("EXT DEV IS NOT TURNSTILE!\n");
		return 0xFF;
	}

	switch (arg1)
	{
	case TURN_CMD_UNLOCK_A:
	case TURN_CMD_UNLOCK_B:
	case TURN_CMD_UNLOCK_AB:
	case TURN_CMD_PASS_A:
	case TURN_CMD_PASS_B:
		if (turn_state != TURN_CMD_STOP) {
			printf("TURNSTILE is not ready!\n");
			return TURN_ERR_DEV_NOT_READY;
		}
		break;
	};

	switch (arg1)
	{
	case TURN_CMD_UNLOCK_A:
		memset (stmiobuf, 0, sizeof(stmiobuf));
		stmiobuf[0] = 1;
		stmiobuf[1] = 1;
		transmit (12, stmiobuf);
		break;
	case TURN_CMD_UNLOCK_B:
		memset (stmiobuf, 0, sizeof(stmiobuf));
		stmiobuf[0] = 1;
		stmiobuf[1] = 2;
		transmit (12, stmiobuf);
		break;
	case TURN_CMD_UNLOCK_AB:
		memset (stmiobuf, 0, sizeof(stmiobuf));
		stmiobuf[0] = 1;
		stmiobuf[1] = 3;
		transmit (12, stmiobuf);
		break;
	case TURN_CMD_STOP:
		memset (stmiobuf, 0, sizeof(stmiobuf));
		stmiobuf[0] = 1;
		stmiobuf[1] = 4;
		transmit (12, stmiobuf);
		break;
	case TURN_CMD_PASS_A:
		memset (stmiobuf, 0, sizeof(stmiobuf));
		stmiobuf[0] = 1;
		stmiobuf[1] = 1;
		transmit (12, stmiobuf);
		alarm (3);
		break;
	case TURN_CMD_PASS_B:
		memset (stmiobuf, 0, sizeof(stmiobuf));
		stmiobuf[0] = 1;
		stmiobuf[1] = 2;
		transmit (12, stmiobuf);
		alarm (3);
		break;
	case TURN_CMD_TIMEOUT:
		turn_timeout = arg2;
		break;
	case TURN_CMD_GET_STATE:
		ret = turn_state;
		break;

	default:
		ret = 0xFF;
	}

	if (arg1 < TURN_CMD_TIMEOUT) turn_state = arg1;



	return ret;
}

/*!
 * Управление замком
 * @param arg - команда/запрос
 * @return
 */
u8 lock_ctl (u32 arg)
{
	int ret = 0;
	u8 stmiobuf[33];
	if (curr_extdev != lock_dev)
		return 0xFF;

	switch (arg)
	{
	case DOOR_LOCK_CMD:
		close_door_when_sens = 1;
		lock_state |= 2;
		stmiobuf[0] = 1;
		transmit (3, stmiobuf);
		break;
	case DOOR_UNLOCK_CMD:
		lock_state &= ~2;
		stmiobuf[0] = 0;
		transmit (3, stmiobuf);
		break;
	case DOOR_PASS_CMD:
		lock_state &= ~2;
		stmiobuf[0] = 0;
		close_door_when_sens = 1;
		transmit (3, stmiobuf);
		break;
	case DOOR_GETS_CMD:
		//memset (stmiobuf, 0, sizeof(stmiobuf));
	//	transmit (12, stmiobuf);
		ret = lock_state;
		break;
	case DOOR_FLOCK_CMD:
		lock_state |= 2;
		stmiobuf[0] = 1;
		transmit (3, stmiobuf);
		break;

	default:
		ret = 0xFF;
	}

	return ret;
}

void update_dev_state (extdev_t dev, u8 new_state)
{
	if (dev == lock_dev)
	{
		if ((new_state) & 1)
		{
			lock_state |= 1;
			if (close_door_when_sens) {
				lock_state |= 2;
				close_door_when_sens = 0;
				lock_ctl (DOOR_FLOCK_CMD);
			}
		} else {
			lock_state &= ~1;
		}
	}
	else if (dev == turnstile1)
	{
		// SENS1 turn on
		if (new_state & 1)
		{
			if (turn_state == TURN_STATE_PASS_A)
				turn_ctl (TURN_CMD_STOP, 0);
			alarm(0);
			io_event (EVT_COMMON, 11, 0, 0, 0);
		}
		// SENS2 turn on
		if (new_state & 2)
		{
			if (turn_state == TURN_STATE_PASS_B)
				turn_ctl (TURN_CMD_STOP, 0);
			alarm(0);
			io_event (EVT_COMMON, 12, 0, 0, 1);
		}
		// ERR_SENS turn on
		if (new_state & 4)
		{
			turn_ctl (TURN_CMD_STOP, 0);
			turn_state = 13;
			io_event (EVT_COMMON, 13, 0, 0, 1);
		}

	}
}

void irq_proc (u8 state)
{
	printf("INPUTS: %d %d %d %d %d %d %d %d\n",
			(bool)(state)&0x80,
			(bool)(state)&0x40,
			(bool)(state)&0x20,
			(bool)(state)&0x10,
			(bool)(state)&0x8,
			(bool)(state)&0x4,
			(bool)(state)&0x2,
			(bool)(state)&0x1
			);

	input_io_state = state;

	update_dev_state(curr_extdev, state);




}
