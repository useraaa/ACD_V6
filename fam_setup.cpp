/*
 * fam_setup.cpp
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

#include <linux/watchdog.h>
#include "client.h"



void save_setup (struct FAM_SETUP *s)
{

	FILE *in;
	system("mv /mnt/sdcard/fam_setup.dat /mnt/sdcard/fam_setup.old");
	in = fopen("/mnt/sdcard/fam_setup.dat", "w");
	if (in == NULL) {
		fprintf(stdout, "FAIL :fopen() error : write \n");
		return;
	}


	fprintf(in, "1 IP %d.%d.%d.%d\n", s->ip[3],
			s->ip[2],
			s->ip[1],
			s->ip[0]);

	fprintf(in, "2 GATEWAY %d.%d.%d.%d\n", s->gw[3],
				s->gw[2],
				s->gw[1],
				s->gw[0]);

	fprintf(in, "3 NETMASK %d.%d.%d.%d\n", s->nm[3],
				s->nm[2],
				s->nm[1],
				s->nm[0]);

	fprintf(in, "4 MAC %x:%x:%x:%x:%x:%x\n", s->mac[5],
			s->mac[4],
			s->mac[3],
			s->mac[2],
			s->mac[1],
			s->mac[0]);

	fprintf(in, "5 PORT %d\n", s->port[0]);
	fprintf(in, "6 PORT %d\n", s->port[1]);
	fprintf(in, "7 CAMERA_GAIN %d\n", s->gain[0]);
	fprintf(in, "8 CAMERA_GAIN %d\n", s->gain[1]);
	fprintf(in, "9 CAMERA_OFFSET %d\n", s->offset[0]);
	fprintf(in, "10 CAMERA_OFFSET %d\n", s->offset[1]);
	fprintf(in, "11 CAMERA_BACKLIGHT %d\n", s->highlight[0]);
	fprintf(in, "12 CAMERA_BACKLIGHT %d\n", s->highlight[1]);
	fprintf(in, "13 IRQ_MODE %d\n", s->irqMode);
	fprintf(in, "14 IMAGE_MODE %d\n", s->imgMode);
	fprintf(in, "15 CLIMIT %d\n", s->contrLimit[0]);
	fprintf(in, "16 CLIMIT %d\n", s->contrLimit[1]);
	fprintf(in, "17 CDELTA %d\n", s->contrDelta[0]);
	fprintf(in, "18 CDELTA %d\n", s->contrDelta[1]);
	fprintf(in, "19 SERIAL %d\n", s->serial);
	fprintf(in, "20 BEEP_MODE %d\n", s->beeperMode);
	fprintf(in, "21 CONN_TOUT %d\n", s->connTout);
	fclose(in);


	printf( "1 IP %d.%d.%d.%d\n", s->ip[3],
			s->ip[2],
			s->ip[1],
			s->ip[0]);

	printf( "2 GATEWAY %d.%d.%d.%d\n", s->gw[3],
				s->gw[2],
				s->gw[1],
				s->gw[0]);

	printf( "3 NETMASK %d.%d.%d.%d\n", s->nm[3],
				s->nm[2],
				s->nm[1],
				s->nm[0]);

	printf( "4 MAC %x:%x:%x:%x:%x:%x\n", s->mac[5],
			s->mac[4],
			s->mac[3],
			s->mac[2],
			s->mac[1],
			s->mac[0]);

	printf( "5 PORT %d\n", s->port[0]);
	printf( "6 PORT %d\n", s->port[1]);
	printf( "7 CAMERA_GAIN %d\n", s->gain[0]);
	printf( "8 CAMERA_GAIN %d\n", s->gain[1]);
	printf( "9 CAMERA_OFFSET %d\n", s->offset[0]);
	printf( "10 CAMERA_OFFSET %d\n", s->offset[1]);
	printf( "11 CAMERA_BACKLIGHT %d\n", s->highlight[0]);
	printf( "12 CAMERA_BACKLIGHT %d\n", s->highlight[1]);
	printf( "13 IRQ_MODE %d\n", s->irqMode);
	printf( "14 IMAGE_MODE %d\n", s->imgMode);
	printf( "15 CLIMIT %d\n", s->contrLimit[0]);
	printf( "16 CLIMIT %d\n", s->contrLimit[1]);
	printf( "17 CDELTA %d\n", s->contrDelta[0]);
	printf( "18 CDELTA %d\n", s->contrDelta[1]);
	printf( "19 SERIAL %d\n", s->serial);
	printf( "20 BEEP_MODE %d\n", s->beeperMode);
	printf( "21 CONN_TOUT %d\n", s->connTout);
}

u8 load_fam_setup(struct FAM_SETUP *s)
{
	char prop[25], value[25];
	int num;
	int a[6];
	int err = 0;
	FILE *in;
	in = fopen("/mnt/sdcard/fam_setup.dat", "rw");

	if (in == NULL) {
		fprintf(stdout, "FAIL :fopen() error : write \n");
		return 0xFF;
	}

	printf("Parsing config file...\n");

	while ((fscanf(in, "%d%s%s", &num, prop, value)) == 3) {
		//fprintf(stdout, "%d:[%s=%s] =>", num, prop, value);
		switch (num) {
		case 1:
			if (sscanf(value, "%d.%d.%d.%d", &a[3], &a[2], &a[1], &a[0]) != 4)
				err = num;

			s->ip[3] = a[3];
			s->ip[2] = a[2];
			s->ip[1] = a[1];
			s->ip[0] = a[0];
			printf("%s: %d.%d.%d.%d\n", prop, a[3], a[2], a[1], a[0]);
			break;
		case 2:
			if (sscanf(value, "%d.%d.%d.%d", &a[3], &a[2], &a[1], &a[0]) != 4)
				err = num;

			s->gw[3] = a[3];
			s->gw[2] = a[2];
			s->gw[1] = a[1];
			s->gw[0] = a[0];
			printf("%s: %d.%d.%d.%d\n", prop, a[3], a[2], a[1], a[0]);
			break;
		case 3:
			if (sscanf(value, "%d.%d.%d.%d", &a[3], &a[2], &a[1], &a[0]) != 4)
				err = num;

			s->nm[3] = a[3];
			s->nm[2] = a[2];
			s->nm[1] = a[1];
			s->nm[0] = a[0];
			printf("%s: %d.%d.%d.%d\n", prop, a[3], a[2], a[1], a[0]);
			break;
		case 4:
			if (sscanf(value, "%X:%X:%X:%X:%X:%X", &a[5], &a[4], &a[3], &a[2],
					&a[1], &a[0]) != 6)
				err = num;
			s->mac[5] = a[5];
			s->mac[4] = a[4];
			s->mac[3] = a[3];
			s->mac[2] = a[2];
			s->mac[1] = a[1];
			s->mac[0] = a[0];
			printf("%s: %d.%d.%d.%d.%d.%d\n", prop, a[5], a[4], a[3], a[2], a[1], a[0]);
			break;
		case 5:
			if (sscanf(value, "%d", &a[0]) != 1)
				err = num;
			s->port[0] = a[0];
			printf("%s: %d\n", prop,a[0]);
			break;
		case 6:
			if (sscanf(value, "%d", &a[0]) != 1)
				err = num;
			s->port[1] = a[0];
			printf("%s: %d\n", prop,a[0]);
			break;
		case 7:
			if (sscanf(value, "%d", &a[0]) != 1)
				err = num;
			s->gain[0] = a[0];
			printf("%s: %d\n", prop,a[0]);
			break;
		case 8:
			if (sscanf(value, "%d", &a[0]) != 1)
				err = num;
			s->gain[1] = a[0];
			printf("%s: %d\n", prop,a[0]);
			break;
		case 9:
			if (sscanf(value, "%d", &a[0]) != 1)
				err = num;
			s->offset[0] = a[0];
			printf("%s: %d\n", prop,a[0]);
			break;
		case 10:
			if (sscanf(value, "%d", &a[0]) != 1)
				err = num;
			s->offset[1] = a[0];
			printf("%s: %d\n", prop,a[0]);
			break;
		case 11:
			if (sscanf(value, "%d", &a[0]) != 1)
				err = num;
			s->highlight[0] = a[0];
			printf("%s: %d\n", prop,a[0]);
			break;
		case 12:
			if (sscanf(value, "%d", &a[0]) != 1)
				err = num;
			s->highlight[1] = a[0];
			printf("%s: %d\n", prop,a[0]);
			break;
		case 13:
			if (sscanf(value, "%d", &a[0]) != 1)
				err = num;
			s->irqMode = a[0];
			printf("%s: %d\n", prop,a[0]);
			break;
		case 14:
			if (sscanf(value, "%d", &a[0]) != 1)
				err = num;
			s->imgMode = a[0];
			printf("%s: %d\n", prop,a[0]);
			break;
		case 15:
			if (sscanf(value, "%d", &a[0]) != 1)
				err = num;
			s->contrLimit[0] = a[0];
			printf("%s: %d\n", prop,a[0]);
			break;
		case 16:
			if (sscanf(value, "%d", &a[0]) != 1)
				err = num;
			s->contrLimit[1] = a[0];
			printf("%s: %d\n", prop,a[0]);
			break;
		case 17:
			if (sscanf(value, "%d", &a[0]) != 1)
				err = num;
			s->contrDelta[0] = a[0];
			printf("%s: %d\n", prop,a[0]);
			break;
		case 18:
			if (sscanf(value, "%d", &a[0]) != 1)
				err = num;
			s->contrDelta[1] = a[0];
			printf("%s: %d\n", prop,a[0]);
			break;
		case 19:
			if (sscanf(value, "%d", &a[0]) != 1)
				err = num;
			s->serial = a[0];
			printf("%s: %d\n", prop,a[0]);
			break;
		case 20:
			if (sscanf(value, "%d", &a[0]) != 1)
				err = num;
			s->beeperMode = a[0];
			printf("%s: %d\n", prop,a[0]);
			break;
		case 21:
			if (sscanf(value, "%d", &a[0]) != 1)
				err = num;
			s->connTout = a[0];
			printf("%s: %d\n", prop,a[0]);
			break;
		default:
			printf(" Unknown property\n");
			break;
		}

		if (err)
			printf(" Error while reading %d config string\n", num);
		//printf(" Parsed OK\n");
	}

	fclose(in);

	return 0;
}
