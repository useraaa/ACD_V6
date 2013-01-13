/*
 * acd_stmio.cpp
 *
 *  Created on: 27.10.2012
 *      Author: user
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netinet/in.h>
#include <termios.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <linux/rtc.h>
#include <linux/ioctl.h>


#include <fcntl.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <pthread.h>

#include <semaphore.h>  /* Semaphore */
#include <linux/watchdog.h>

#include "client.h"
#include "camera.h"
#include "ext_dev.h"
#include "acd_server.h"
#include "acd_proto.h"
#include "acd_stmio.h"

u8 stmiobuf[33];
int stmd = 0;
pthread_t stmio_thread_d;
static void * stmio_thread(void *ptr);

int parse_stmio(u8 *buf) {

	for (int i = 0; i < 32; i++) {
		printf("[%d]", buf[i]);
	}
	printf("\n");

	return 0;
}

void transmit(u8 cmd, u8 *packet)
{
	printf("ST>> ");
	u8 buf[33];
	memset(buf, 0, 32);
	buf[0] = 'S';
	buf[31] = 'E';
	buf[1] = cmd;
	memcpy(&buf[2], packet, 29);
	write(stmd, buf, 32);
	usleep(10000);
	read(stmd, buf, 32);

	if ((buf[0] != 'S') || (buf[31] != 'E')) {
		printf(" STMIO ERROR ");
	}

	usleep(10000);

	memcpy(packet, &buf[2], 29);
	printf("<<\n");
}



int stmio_start()
{
	printf("launch stmio\n");

	stmd = open("/dev/stmio", O_RDWR);

	if (stmd == -1) {
		printf("no device! (stmio) \n");
		return -1;
	}

	if (pthread_create(&stmio_thread_d, NULL, stmio_thread, 0)) {
		printf("stmio thread create fail\n");
		return -1;
	}

	return 0;
}

void stmio_stop()
{
	pthread_kill(stmio_thread_d, 0);
	close(stmd);
}


static void * stmio_thread(void *ptr) {

	u8 io_buf[64];
	u32 events = 0;
	u8 tries = 0;
	int stmio_thread_number = pthread_self();
	printf("STMIO thread started(%d)\n", stmio_thread_number);
	int errors = 0;

	memset(io_buf, 0, 64);
	transmit(0, io_buf);
	transmit(0, io_buf);
	transmit(0, io_buf);
	transmit(0, io_buf);


	printf("STMIO protocol ver:%d.\n", io_buf[0]);
//	parse_stmio(io_buf);
	while (1) {

		// call for events
		if (ioctl(stmd, 1, &events)) {
			printf("error ioctl\n");
			if (++errors > 10)
				return NULL;
		}

		if (events == 1) {

			events = 0;
			read(stmd, io_buf, 32);
			parse_stmio(io_buf);
			tries = 5;
//			do {
//				sleep(1);
//				printf("Error in packet.reading ST :(\n");
//				read(stmd, io_buf, 32);
//				parse_stmio(io_buf);
//			} while ((io_buf[1] == 255) && (--tries));
//
//			if (!tries) continue;

			u32 arg1 = 0; // *((u32 *)&io_buf[3]);
			u32 arg2 = 0; // *((u32 *)&io_buf[7]);

			if (io_buf[2] == OW_EVENT1) {
				memcpy(&arg1, &io_buf[5], 4);
				arg2 = 0;
				io_event(EVT_OW, arg1, arg2, 1, 0);
				arg1 = 3;
				if (fam_setup.beeperMode)
					camera_iocmd(0x35, &arg1, 0);
			} else if (io_buf[2] == OW_EVENT2) {
				memcpy(&arg1, &io_buf[14], 4);
				arg2 = 0;
				io_event(EVT_OW, arg1, arg2, 2, 1);
				arg1 = 3;
				if (fam_setup.beeperMode)
					camera_iocmd(0x35, &arg1, 1);
			} else if (io_buf[2] == IO_EVENT)
				irq_proc(~io_buf[22]);
			else
				printf("Unknown event %d\n", io_buf[2]);
//			pthread_mutex_unlock( &mutex1 );
		}
//
//		usleep(10000);
	}

	return NULL;
}

void ioctl_proc(u8 *cmd, u32 *arg1, u32 *arg2, u8 port)
{

	printf("%d, %d, %d \n", *cmd, *arg1, *arg2);

	// routing devices
	switch (*cmd) {
	case CMD162_BUZZER_CTL:
		// buzzer control
		if (*arg1 == 0)
			fam_setup.beeperMode = 0;
		else if (*arg1 == 1)
			fam_setup.beeperMode = 1;
		else
			camera_iocmd(0x35, arg1, port);
		break;
	case CMD162_LED_CTL:
		// status led control

		//*arg1 = abs(*arg1 - 3);
		camera_iocmd(0x36, arg1, port);
		break;
	case CMD162_EXDEV_INIT:
		// STM-IO route
		if (*arg1 < 3)
			*arg1 = extdev_init((extdev_t) *arg1);
		else
			*arg1 = 0xFF;
		break;
	case CMD162_LOCK_CTL:
		*arg1 = lock_ctl(*arg1);
		break;

	case CMD162_TURNSTILE_CTL:
		*arg1 = turn_ctl(*arg1, *arg2);
		break;

	default:
		printf("CMD ERR\n");
		break;
	}

//	parse_stmio(io_buf);

	return;
}


void beep(u8 l, u8 p)
{
	u32 ll = l;
	camera_iocmd(0x35, &ll, p);
}
