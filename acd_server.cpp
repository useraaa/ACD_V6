/*
 * main.cpp
 *
 *  Created on: 16.05.2012
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
#include "network.h"
#include "fam_setup.h"
#include "rtc.h"

struct FAM_SETUP fam_setup;

int kbhit(void) {
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF) {
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	bool watchdogging = false;

	// hello world!!!
	printf("[%s](%s)\n", BANNER, __DATE__);

	// load settings
	if (load_fam_setup(&fam_setup)) {
		printf("Settings are wrong. Loading defaults. TODO :) \n.");
	}
	// create camera IO and thread
	camera_open();
	// setup image processing
	camera_setup();
	// start IO extension
	stmio_start();
	// start external devices
	extdev_init(no_extdev);
	// start RTC
	init_rtc();
	// start networking
	network_start();


	// start capture...
	camera_start();

	if ((argc > 1) && (!strcmp((const char *)argv[1],(const char *)'w')))
		watchdogging = true;

	if (watchdogging){
		int fd = open("/dev/watchdog", O_WRONLY);
		if (fd == -1) {
			perror("watchdog");
			exit(EXIT_FAILURE);
		}
		int timeout = 3;
		ioctl(fd, WDIOC_SETTIMEOUT, &timeout);
		printf("The timeout was set to %d seconds\n", timeout);
		while (1) {
				ioctl(fd, WDIOC_KEEPALIVE, 0);
				sleep(1);
			}
	} else
		while (!kbhit()) sleep(1);


	printf("exit!\n");
	exit(0);
	return 0;
}

