/*
 * acd_stmio.h
 *
 *  Created on: 27.10.2012
 *      Author: user
 */

#ifndef ACD_STMIO_H_
#define ACD_STMIO_H_

void beep(u8 l, u8 p);
void ioctl_proc(u8 *cmd, u32 *arg1, u32 *arg2, u8 port);

int stmio_start();
void stmio_stop();

#endif /* ACD_STMIO_H_ */
