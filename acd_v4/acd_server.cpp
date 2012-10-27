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


/* Some helper functions to convert between the common RTC notion of time
 * and the internal Blackfin notion that is encoded in 32bits.
 */
static inline u32 rtc_time_to_bfin(unsigned long now)
{
	u32 sec  = (now % 60);
	u32 min  = (now % (60 * 60)) / 60;
	u32 hour = (now % (60 * 60 * 24)) / (60 * 60);
	u32 days = (now / (60 * 60 * 24));
	return (sec  << SEC_BITS_OFF) +
	       (min  << MIN_BITS_OFF) +
	       (hour << HOUR_BITS_OFF) +
	       (days << DAY_BITS_OFF);
}
static inline unsigned long rtc_bfin_to_time(u32 rtc_bfin)
{
	return (((rtc_bfin >> SEC_BITS_OFF)  & 0x003F)) +
	       (((rtc_bfin >> MIN_BITS_OFF)  & 0x003F) * 60) +
	       (((rtc_bfin >> HOUR_BITS_OFF) & 0x001F) * 60 * 60) +
	       (((rtc_bfin >> DAY_BITS_OFF)  & 0x7FFF) * 60 * 60 * 24);
}

void init_rtc(u8 getset, time_t *gstime) {

	int rtc_fd;
	int ret, i;
	struct rtc_time rtc_tm;
	char *rtc_dev = "/dev/rtc0";
	time_t t1, t2;

	printf("====== RTC Test  ====\n");
	printf("0. open and release\n");
	rtc_fd = open(rtc_dev, O_RDONLY);
	if (rtc_fd == -1) {
		printf("failed to open '%s': %s\n", rtc_dev, strerror(errno));
		exit(1);
	} else
		printf("opened '%s': fd = %d\n", rtc_dev, rtc_fd);

	printf("Get RTC Time\n");
	ret = ioctl(rtc_fd, RTC_RD_TIME, &rtc_tm);
	if (ret == -1) {
		perror("rtc ioctl RTC_RD_TIME error");
	}

	printf("Current RTC date/time is %d-%d-%d, %02d:%02d:%02d\n",
		rtc_tm.tm_mday, rtc_tm.tm_mon + 1, rtc_tm.tm_year,
		rtc_tm.tm_hour, rtc_tm.tm_min, rtc_tm.tm_sec);
}


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

void setup_stmio();

void Die(char *mess) {
	perror(mess);
	return;
}
//void init_server(int);

static void * net_stream(void *ptr);
static void * log_stream(void *arg);

struct FAM_SETUP fam_setup;
struct LOG_MESSAGES log_message;

int clientsock, clientsocks[5], log_socket;

pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t cam_mutex = PTHREAD_MUTEX_INITIALIZER;


sem_t gframe_mutex;

u8 *pbuf;

int stmd;

int stmio_thread_number = 0;
pthread_t stmio_thread_d, net_thread_d1, net_thread_d2, log_thread_d;


void save_setup ( ) {

	FILE *in;
	system("mv /mnt/sdcard/fam_setup.dat /mnt/sdcard/fam_setup.old");
	in = fopen("/mnt/sdcard/fam_setup.dat", "w");
	if (in == NULL) {
		fprintf(stdout, "FAIL :fopen() error : write \n");
		return;
	}


	fprintf(in, "1 IP %d.%d.%d.%d\n", fam_setup.ip[3],
			fam_setup.ip[2],
			fam_setup.ip[1],
			fam_setup.ip[0]);

	fprintf(in, "2 GATEWAY %d.%d.%d.%d\n", fam_setup.gw[3],
				fam_setup.gw[2],
				fam_setup.gw[1],
				fam_setup.gw[0]);

	fprintf(in, "3 NETMASK %d.%d.%d.%d\n", fam_setup.nm[3],
				fam_setup.nm[2],
				fam_setup.nm[1],
				fam_setup.nm[0]);

	fprintf(in, "4 MAC %x:%x:%x:%x:%x:%x\n", fam_setup.mac[5],
			fam_setup.mac[4],
			fam_setup.mac[3],
			fam_setup.mac[2],
			fam_setup.mac[1],
			fam_setup.mac[0]);

	fprintf(in, "5 PORT %d\n", fam_setup.port[0]);
	fprintf(in, "6 PORT %d\n", fam_setup.port[1]);
	fprintf(in, "7 CAMERA_GAIN %d\n", fam_setup.gain[0]);
	fprintf(in, "8 CAMERA_GAIN %d\n", fam_setup.gain[1]);
	fprintf(in, "9 CAMERA_OFFSET %d\n", fam_setup.offset[0]);
	fprintf(in, "10 CAMERA_OFFSET %d\n", fam_setup.offset[1]);
	fprintf(in, "11 CAMERA_BACKLIGHT %d\n", fam_setup.highlight[0]);
	fprintf(in, "12 CAMERA_BACKLIGHT %d\n", fam_setup.highlight[1]);
	fprintf(in, "13 IRQ_MODE %d\n", fam_setup.irqMode);
	fprintf(in, "14 IMAGE_MODE %d\n", fam_setup.imgMode);
	fprintf(in, "15 CLIMIT %d\n", fam_setup.contrLimit[0]);
	fprintf(in, "16 CLIMIT %d\n", fam_setup.contrLimit[1]);
	fprintf(in, "17 CDELTA %d\n", fam_setup.contrDelta[0]);
	fprintf(in, "18 CDELTA %d\n", fam_setup.contrDelta[1]);
	fprintf(in, "19 SERIAL %d\n", fam_setup.serial);
	fprintf(in, "20 BEEP_MODE %d\n", fam_setup.beeperMode);
	fclose(in);


	printf( "1 IP %d.%d.%d.%d\n", fam_setup.ip[3],
			fam_setup.ip[2],
			fam_setup.ip[1],
			fam_setup.ip[0]);

	printf( "2 GATEWAY %d.%d.%d.%d\n", fam_setup.gw[3],
				fam_setup.gw[2],
				fam_setup.gw[1],
				fam_setup.gw[0]);

	printf( "3 NETMASK %d.%d.%d.%d\n", fam_setup.nm[3],
				fam_setup.nm[2],
				fam_setup.nm[1],
				fam_setup.nm[0]);

	printf( "4 MAC %x:%x:%x:%x:%x:%x\n", fam_setup.mac[5],
			fam_setup.mac[4],
			fam_setup.mac[3],
			fam_setup.mac[2],
			fam_setup.mac[1],
			fam_setup.mac[0]);

	printf( "5 PORT %d\n", fam_setup.port[0]);
	printf( "6 PORT %d\n", fam_setup.port[1]);
	printf( "7 CAMERA_GAIN %d\n", fam_setup.gain[0]);
	printf( "8 CAMERA_GAIN %d\n", fam_setup.gain[1]);
	printf( "9 CAMERA_OFFSET %d\n", fam_setup.offset[0]);
	printf( "10 CAMERA_OFFSET %d\n", fam_setup.offset[1]);
	printf( "11 CAMERA_BACKLIGHT %d\n", fam_setup.highlight[0]);
	printf( "12 CAMERA_BACKLIGHT %d\n", fam_setup.highlight[1]);
	printf( "13 IRQ_MODE %d\n", fam_setup.irqMode);
	printf( "14 IMAGE_MODE %d\n", fam_setup.imgMode);
	printf( "15 CLIMIT %d\n", fam_setup.contrLimit[0]);
	printf( "16 CLIMIT %d\n", fam_setup.contrLimit[1]);
	printf( "17 CDELTA %d\n", fam_setup.contrDelta[0]);
	printf( "18 CDELTA %d\n", fam_setup.contrDelta[1]);
	printf( "19 SERIAL %d\n", fam_setup.serial);
	printf( "20 BEEP_MODE %d\n", fam_setup.beeperMode);
}

u8 load_fam_setup(struct FAM_SETUP *s) {
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

long get_cur_ms() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

unsigned char check_sum(unsigned char *buff) {
	int temp = 0, i;
	for (i = 0; i < 11; i++) {
		temp += (unsigned char) buff[i];
	}
	return temp % 256;
}

bool process_packet(u8 *buff, u8 *cmd, int *param1, int *param2, u8 *err) {
	*cmd = buff[1];
	*err = buff[10];
	unsigned int temp;
	temp = (unsigned char) buff[5];
	temp = temp * 256 + (unsigned char) buff[4];
	temp = temp * 256 + (unsigned char) buff[3];
	temp = temp * 256 + (unsigned char) buff[2];
	*param1 = temp;
	temp = (unsigned char) buff[9];
	temp = temp * 256 + (unsigned char) buff[8];
	temp = temp * 256 + (unsigned char) buff[7];
	temp = temp * 256 + (unsigned char) buff[6];
	*param2 = temp;

	return 0;
}

void build_packet_data(unsigned char cmd, u8 *data, unsigned char err,
		u8 * txBuff) {
	txBuff[0] = FAM_START_BYTE;
	txBuff[1] = cmd;
	memcpy(&txBuff[2], data, 8);
	txBuff[10] = err;
	txBuff[11] = check_sum(txBuff);
	txBuff[12] = FAM_STOP_BYTE;

	return;
}

void build_packet(unsigned char cmd, unsigned int param1, unsigned int param2,
		unsigned char err, u8 * txBuff) {
	txBuff[0] = FAM_START_BYTE;
	txBuff[1] = cmd;
	txBuff[2] = param1;
	txBuff[3] = param1 >> 8;
	txBuff[4] = param1 >> 16;
	txBuff[5] = param1 >> 24;
	txBuff[6] = param2;
	txBuff[7] = param2 >> 8;
	txBuff[8] = param2 >> 16;
	txBuff[9] = param2 >> 24;
	txBuff[10] = err;
	txBuff[11] = check_sum(txBuff);
	txBuff[12] = FAM_STOP_BYTE;

	return;
}

void transmit(u8 cmd, u8 *packet) {
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
		printf("STMIO ERROR\n");
	}
////
////	if (buf[1] == 255) {
////		usleep(10000);
//		transmit(cmd, packet);
////	}

	memcpy(packet, &buf[2], 29);
}


void make_cmd(u8 * buf, u8 cmd) {
	memset(buf, 0, 32);
	buf[0] = buf[31] = 0xFF;
	buf[1] = cmd;

}

int parse_stmio(u8 *buf) {

	for (int i = 0; i < 32; i++) {
		printf("[%d]", buf[i]);
	}
	printf("\n");

	return 0;
}

static void * stmio_thread(void *ptr) {

	u8 io_buf[64];
	u32 events = 0;
	u8 tries = 0;
	stmio_thread_number = pthread_self();
	printf("STMIO thread started(%d)\n", stmio_thread_number);
	int errors = 0;

	memset(io_buf, 0, 64);
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

int main(int argc, char *argv[])
{
	int ret = 0;
	bool watchdogging = false;

	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setstacksize(&attr, 1024 * 1024 * 2);


	printf("[%s](%s)\n", BANNER, __DATE__);



	if (load_fam_setup(&fam_setup)) {
		printf("Loading defaults TODO\n.");
	}

	if (camera_open()) {
		printf("Camera open fails.\n");
	}

	stmd = open("/dev/stmio", O_RDWR);

	if (stmd == -1) {
		printf("no device! (stmio) \n");
		return -1;
	}

	// load settings
	extdev_init(no_extdev);

	init_rtc(0,NULL);

	sem_init(&gframe_mutex, 0, 1);

	char cmd[64];
	sprintf(cmd, "ifconfig eth0 up %d.%d.%d.%d", fam_setup.ip[3],
			fam_setup.ip[2], fam_setup.ip[1], fam_setup.ip[0]);
	printf("Assign eth0 [%s]\n", cmd);
	system(cmd);

	printf("launch port1\n");
	ret = pthread_create(&net_thread_d1, &attr, net_stream, &fam_setup.port[0]);
	if (ret) {
		printf("stmio thread1 create fail\n");
	}
	printf("launch port2\n");
	ret = pthread_create(&net_thread_d2, &attr, net_stream, &fam_setup.port[1]);
	if (ret) {
		printf("stmio thread2 create fail\n");
	}

	ret = pthread_create(&log_thread_d, NULL, log_stream, 0);
	if (ret) {
		printf("log thread create fail\n");
	}
	printf("launch stmio\n");
	ret = pthread_create(&stmio_thread_d, NULL, stmio_thread, 0);
	if (ret) {
		printf("stmio thread create fail\n");
	}

	camera_start();

	//if ((argc > 1) && (!strcmp((const char *)argv[1],(const char *)'w')))
		watchdogging = true;

	if (watchdogging){
		int fd = open("/dev/watchdog", O_WRONLY);
		if (fd == -1) {
			perror("watchdog");
			exit(EXIT_FAILURE);
		}
		int timeleft = 0, timeout = 3;
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

void stop_acdserver()
{
	pthread_kill(stmio_thread_d, 0);

	close(stmd);

	camera_close();
}

u8 stmiobuf[33];
void beep(u8 l, u8 p) {
	u32 ll = l;
	camera_iocmd(0x35, &ll, p);
}

void ioctl_proc(u8 *cmd, u32 *arg1, u32 *arg2, u8 port) {
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

void send_buf(int sock, u8 *buf, u32 len) {
	int total_len = len;
	while (len) {
		int bytes_written = send(sock, buf, len, 0);
		if (bytes_written < 0) {
			// handle errors
			printf("tcp error %d\n", bytes_written);
			return;
		}
		if (bytes_written == 0) {
			usleep(10);
		}
		len -= bytes_written;
		buf += bytes_written;
	}

}

#define TCP_NODELAY 1
void HandleClient(int sock, u8 port) {
	u8 buffer[CTRL_PACK * 2];
	int received = 0;
	u8 cmd, err;
	int arg1, arg2;
	u16 w = 0, h = 0;
	int flag = 1;
	int ret, sock_buf_size;


	do {
		/* Send back received data */
		/* Check for more data */
		unlock_cam();

		if ((received = recv(sock, buffer, CTRL_PACK, 0)) < 0) {
			Die("Failed to receive additional bytes from client");
		}

		if (received == CTRL_PACK) {
			process_packet((u8 *) buffer, &cmd, &arg1, &arg2, &err);

			printf("%d: CMD:%d, ARG1:%d, ARG2:%d, ERR:%d\n",port, cmd, arg1, arg2, err);

			switch (cmd) {

			case CMD_SET_TIME:
				break;
			case CMD_GET_TIME:

				break;

			case CMD_GET_VERSION:
				arg2 = fam_setup.serial;
				arg1 = 0x601;
				err = ERR_1OK;

				build_packet(cmd, arg1, arg2, err, (u8 *) buffer);
				send(sock, buffer, CTRL_PACK, 0);
				break;

			case CMD_IMAGE_DUMP: {
//				sem_wait(&gframe_mutex);

				//long bg = get_cur_ms();
				int pcklen = arg1;

				if (pcklen < 512)
					pcklen = 512;


				u8 *ptr = get_frame((cam_format_t) err, &w, &h, port);

				if (ptr == NULL) {
					build_packet(cmd, 0, 0, ERR_BAD_ARGUMENT, (u8 *) buffer);
					send(sock, buffer, CTRL_PACK, 0);
					printf("Camera not ready\n");
					break;
				}

				int frame_size = w * h;
				arg1 = frame_size;
				arg2 = w;
				err = ERR_1OK;
				//printf("Send image (%d, %d) sock:%d plen:%d\n", arg1, arg2, sock, pcklen);

				build_packet(cmd, arg1, arg2, err, (u8 *) buffer);

				send_buf(sock, buffer, CTRL_PACK);

				if (pcklen > frame_size)
					pcklen  = frame_size;

				int 	tail = frame_size % pcklen,
						ctrl = frame_size / pcklen;
				printf("NumOfChunks=%d\n", ctrl);

				if (pcklen < frame_size) {

					if (!tail) ctrl--;	//no tail

					for (int i = 0; i < ctrl; i++) {
						arg1 = i * pcklen;
						arg2 = pcklen;
						err = ERR_DATA;

						build_packet(EVT_DATA, arg1, arg2, err, (u8 *) buffer);

						send_buf(sock, buffer, CTRL_PACK);

						send_buf(sock, ptr, pcklen);

						ptr += pcklen;
					}

				}

				if (tail)
					pcklen = tail;


				arg1 = frame_size - pcklen;	// position
				arg2 = pcklen;
				err = ERR_LDATA;

				build_packet(EVT_DATA, arg1, arg2, err, (u8 *) buffer);
				send_buf(sock, buffer, CTRL_PACK);
				send_buf(sock, ptr, pcklen);
				printf("<\n");
//				sem_post(&gframe_mutex);

				break;
			}
			case CMD_CHECK_FINGER: {

				arg1 = check_finger_presense(port) > 0 ? 1 : 0;
				arg2 = 0; //key code
				err = ERR_1OK;
				build_packet(cmd, arg1, arg2, err, (u8 *) buffer);
				send(sock, buffer, CTRL_PACK, 0);
				break;
			}
			case CMD_SET_HIGHLIGHT: {
				err = ERR_1OK;
				fam_setup.highlight[port] = arg1;
				//arg1 = 255 - arg1;
				camera_iocmd(0x34, (u32 *) &arg1, port);
				build_packet(cmd, arg1, 0, err, (u8 *) buffer);
				send(sock, buffer, CTRL_PACK, 0);
				break;
			}

			case CMD_SET_GAIN: {
				err = ERR_1OK;
				fam_setup.gain[port] = arg1;
				arg1 = (((u16) arg1) << 16) | 0x35;
				camera_iocmd(0x32, (u32 *) &arg1, port);
				build_packet(cmd, arg1, 0, err, (u8 *) buffer);
				send(sock, buffer, CTRL_PACK, 0);
				break;
			}

			case CMD_M162_CTL: {
				// err - cmd

				ioctl_proc(&err, (u32 *) &arg1, (u32 *) &arg2, port);
				build_packet(cmd, arg1, arg2, err, (u8 *) buffer);
				send(sock, buffer, CTRL_PACK, 0);
				break;
			}

			case CMD_READ_REG: {
				err = ERR_1OK;
				camera_iocmd(0x31, (u32 *) &arg1, port);
				build_packet(cmd, arg1, 0, err, (u8 *) buffer);
				send(sock, buffer, CTRL_PACK, 0);
				break;
			}

			case CMD_WRITE_REG: {
				err = ERR_1OK;
				camera_iocmd(0x32, (u32 *) &arg1, port);
				build_packet(cmd, arg1, 0, err, (u8 *) buffer);
				send(sock, buffer, CTRL_PACK, 0);
				break;
			}

			case CMD_GET_SETTINGS: {
				u8 data[8];
				memset(data, 0, 8);
				data[0] = 1;
				data[1] = fam_setup.gain[port];

				data[2] = fam_setup.offset[port];
				data[3] = fam_setup.offset[port] >> 8;
				data[4] = fam_setup.highlight[port];
				data[5] = fam_setup.irqMode;
				data[6] = 0;
				data[7] = 0;
				err = ERR_1OK;
				build_packet_data(cmd, data, err, (u8 *) buffer);
				send(sock, buffer, CTRL_PACK, 0);
				break;
			}

			case CMD_READ_ROM: {
//					u8 *data = (u8 *)&arg1;
//					u8 *data2 = (u8 *)&arg2;
				u8 data[8];
				memset(data, 0, 8);
				// set ip
				switch (err) {
				case 0:
					memcpy(data, &fam_setup.serial, 4);
					break;
				case 1:
					memcpy(data, fam_setup.ip, 4);
					break;
				case 2:
					memcpy(data, fam_setup.gw, 4);
					break;
				case 3:
					memcpy(data, fam_setup.nm, 4);
					break;
				case 4:
					memcpy(data, fam_setup.mac, 4);
					memcpy(&data[4], &fam_setup.mac[4], 2);
					break;
				case 5:
					data[0] = 1;
					data[1] = fam_setup.gain[port];
					data[2] = fam_setup.offset[port];
					data[3] = fam_setup.offset[port] >> 8;
					data[4] = fam_setup.highlight[port];
					data[5] = 0;
					data[6] = 0;
					data[7] = 0;
					break;
				case 6:
					data[0] = fam_setup.port[port];
					data[1] = fam_setup.port[port] >> 8;
					data[2] = fam_setup.contrLimit[port];
					data[3] = fam_setup.contrLimit[port] >> 8;
					data[4] = fam_setup.contrDelta[port];
					data[5] = fam_setup.contrDelta[port] >> 8;
					break;
				case 7:
					data[0] = fam_setup.irqMode;
					data[1] = fam_setup.connTout;
					data[2] = fam_setup.beeperMode;
					data[5] = fam_setup.imgMode;
					break;
				}
				err = ERR_1OK;
				build_packet_data(cmd, data, err, (u8 *) buffer);
				send(sock, buffer, CTRL_PACK, 0);
				break;
			}

			case CMD_WRITE_ROM: {
				u8 *data = (u8 *) &arg1;
				u8 *data2 = (u8 *) &arg2;
				// set ip
				if (err == 0)
					memcpy(&fam_setup.serial, data, 4);
				if (err == 1)
					memcpy(&fam_setup.ip, data, 4);
				if (err == 2)
					memcpy(&fam_setup.gw, data, 4);
				if (err == 3)
					memcpy(&fam_setup.nm, data, 4);
				if (err == 4) {
					memcpy(fam_setup.mac, data, 4);
					memcpy(&fam_setup.mac[4], data2, 2);
				}
				if (err == 5) {
					fam_setup.gain[port] = data[1];
					fam_setup.offset[port] = data[2] |  data[3] << 8;
					fam_setup.highlight[port] = data2[0];
				}
				if (err == 6) {
					fam_setup.port[port] = data[0] | (data[1] << 8);
					fam_setup.contrLimit[port] = data[2] | (data[3] << 8);
					fam_setup.contrDelta[port] = data2[0] | (data2[1] << 8);
				}
				if (err == 7) {
					fam_setup.irqMode = data[0];
					fam_setup.connTout = data[1];
					fam_setup.beeperMode = data[2];
					fam_setup.imgMode = data2[1];
				}
				save_setup();
				sleep(1);
				err = ERR_1OK;
				build_packet(cmd, arg1, arg2, err, (u8 *) buffer);
				send(sock, buffer, CTRL_PACK, 0);
				break;

			}
			case CMD_REBOOT:
				close(clientsocks[0]);
				close(clientsocks[1]);
				system("reboot");
				sleep(5000);
				break;

			default:
				build_packet(cmd, arg1, arg2, ERR_UNKNOWN_COMMAND,
						(u8 *) buffer);
				send(sock, buffer, CTRL_PACK, 0);
				break;
			}

		}

	} while ((received > 0));
	printf("Disconnect client\n");
	close(sock);
}

void io_event(u8 event, u32 arg1, u32 arg2, u8 err, u8 port) {
	if (clientsocks[port] == NULL) {
		printf("PORT unconnected!\n");
		return;
	}

	printf("PORT:%d event %d [%X][%X][%x]\n", fam_setup.port[port], event, arg1, arg2, err);
	if (!fam_setup.irqMode)
		return;

	u8 buffer[CTRL_PACK];

	build_packet(event, arg1, arg2, err, (u8 *) buffer);

	send_buf(clientsocks[port], buffer, CTRL_PACK);

}

static void * net_stream(void *port) {
	int serversock;
	struct sockaddr_in echoserver, echoclient;

	/* Create the TCP socket */
	if ((serversock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
		Die("Failed to create socket");
	}
	/* Construct the server sockaddr_in structure */
	memset(&echoserver, 0, sizeof(echoserver)); /* Clear struct */
	echoserver.sin_family = AF_INET; /* Internet/IP */
	echoserver.sin_addr.s_addr = htonl(INADDR_ANY); /* Incoming addr */
	echoserver.sin_port = htons(*(u16 *) port); /* server port */
	clientsocks[0] = NULL;
	clientsocks[1] = NULL;
	/* Bind the server socket */
	int i = 60;
	for (; i > 0; i--){
		if (bind(serversock, (struct sockaddr *) &echoserver, sizeof(echoserver)) < 0) {
			printf(".");
			sleep(3);
		} else break;
	}
	if (i == 0) {
		printf("Failed to bind the server socket");
		exit(EXIT_FAILURE);
	}

	/* Listen on the server socket */
	if (listen(serversock, MAXPENDING) < 0) {
		Die("Failed to listen on server socket");
	}

	/* Run until cancelled */
	while (1) {
		unsigned int clientlen = sizeof(echoclient);

		if ((clientsock = accept(serversock, (struct sockaddr *) &echoclient,
				&clientlen)) < 0) {
			Die("Failed to accept client connection");
		}
		fprintf(stdout, "Client connected: %s\n",
				inet_ntoa(echoclient.sin_addr));

		if (*(u16 *) port == fam_setup.port[0]) {
			clientsocks[0] = clientsock;
			HandleClient(clientsock, 0);
		} else if (*(u16 *) port == fam_setup.port[1]) {
			clientsocks[1] = clientsock;
			HandleClient(clientsock, 1);
		}

	}
}
void _log (const char *fmt, ...)
{
	va_list args;

	va_start (args, fmt);

	int len = vsprintf ( (char *)log_message.buf, fmt, args);

	log_message.count = len;

	va_end (args);

	return;
}

static void * log_stream(void *arg) {
	int log_listen_sock;
	struct sockaddr_in echoserver, echoclient;

	/* Create the TCP socket */
	if ((log_listen_sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
		Die("Failed to create socket");
	}
	/* Construct the server sockaddr_in structure */
	memset(&echoserver, 0, sizeof(echoserver)); /* Clear struct */
	echoserver.sin_family = AF_INET; /* Internet/IP */
	echoserver.sin_addr.s_addr = htonl(INADDR_ANY); /* Incoming addr */
	echoserver.sin_port = htons(10000); /* server port */

	/* Bind the server socket */
	if (bind(log_listen_sock, (struct sockaddr *) &echoserver,
			sizeof(echoserver)) < 0) {
		Die("Failed to bind the server socket");
	}
	/* Listen on the server socket */
	if (listen(log_listen_sock, MAXPENDING) < 0) {
		Die("Failed to listen on server socket");
	}

	/* Run until cancelled */
	while (1) {
		unsigned int clientlen = sizeof(echoclient);

		if ((log_socket = accept(log_listen_sock,
				(struct sockaddr *) &echoclient, &clientlen)) < 0) {
			Die("Failed to accept client connection");
		}
		fprintf(stdout, "Log socket connected to: %s\n",
				inet_ntoa(echoclient.sin_addr));

		while (1) {
			if (log_message.count) {
				send_buf(log_socket, log_message.buf, log_message.count);
				log_message.count = 0;
			}
		}
	}
}

