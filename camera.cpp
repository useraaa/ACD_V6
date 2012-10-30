/*
 * camera.cpp
 *
 *  Created on: 07.07.2012
 *      Author: user
 */
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <termios.h>
#include <signal.h>
#include <semaphore.h>  /* Semaphore */

#include "client.h"
#include "camera.h"




int cam_d; // file descriptor
int camio = 0;
FILE *cam_switch;
u8 *cam_vbuf[2];
sem_t camio_mutex;
u8 stored_buf[2][FRAME_S];
//u8 output_buf[2][FRAME_S];

int active = 0;

u8 hline[FRAME_W];
u8 vline[FRAME_H];

u8 camio_buf[9];

u8 camera_holder[2] = { 0, 0 };
volatile u8 camera_holder_ack[2] = { 0, 0 };
static u8 have_finger_on_camera[2] = { 0, 0 };

u32 current_frame_counter = 0, last_frame_counter = 0;
u8 camera_cmd[2] = { 0, 0 };
u32 camera_arg[2] = { 0, 0 };

u8 camera_rst = 0;
pthread_mutex_t cam0_mutex;
pthread_cond_t cam0_cond_in, cam0_cond_out;

struct cam_control {
	u8 detected;
	int cam_number;
	u8 *buf_ptr;
};

struct format {
	u16 l;
	u16 t;
	u16 w;
	u16 h;
	float kx;
	float ky;
} fmt;

struct format image_fmt[5] = {
		{0,0,384,600, 0.3, 0.586},
		{0,0,640,512, 0.5, 0.5},
		{0,0,576,768, 0.45, 0.75},
		{0,0,576,384, 0.45, 0.375},
		{0,0,1280,1024, 1, 1}
};

void camera_reset(u8 num);
static void * cam_stream(void *ptr);
pthread_t cam_thread_d;

u16 check_finger(u16 w, u16 h, u16 *c1, u8 *buff, u16 contrLimit,
		u16 contrDelta, u16 *last_cx, u16 *last_cy);

void stretch(u32 srcWidth, u32 srcHeight, u8 *src, float kx, float ky,
		u16 *DstWidth, u16 *DstHeight, u8 *dst);

volatile u8 tcp_transfer = 0; // camera_process_delay
volatile u8 tcp_transfer_lock = 0;

void cam_beep(u8 l, u8 p)
{
	u32 ll = l;
	camera_iocmd(0x35, &ll, p);
}

void camera_setformat(int w, int h, int l, int t)
{
	//first column to read 0x02
	fmt.l = l;
	//first row to read 0x01
	fmt.h = h;
	// window height - 1 0x03
	fmt.w = w;
	// window width - 1 0x04
	fmt.t = t;


	if (ioctl(cam_d, ACD_SET_NEW_FORMAT, (void *)&fmt) != 0)
		perror("Set Format ioctl error \n");

}
void cam_set_gain (u8 value, u8 cam_num)
{
	u32 arg = ((value) << 16) | 0x35;
	printf("%d: Set gain %d\n", cam_num ,value);
	camera_iocmd(0x32, (u32 *) &arg, cam_num);

}

void cam_set_bl ( u8 value, u8 cam_num)
{
	u32 arg = value;
	printf("%d: Set bl %d\n", cam_num ,value);
	camera_iocmd(0x34, &arg, cam_num);
}

void camera_iocmd(u8 cmd, u32 *arg, u8 cam_num) {
	//camio_mutex
//	sem_wait(&camio_mutex);

	char msg[64];
	sprintf(msg, "echo %d > /sys/class/gpio/gpio31/value", cam_num);
	system(msg);

	u32 tmo = 500, len;
	u8 bbuf[CAMERA_CMD_PACK_SZ*4];

	bbuf[0] = 'S';
	bbuf[1] = cmd;
	bbuf[10] = 'F';
	memcpy(&bbuf[2], arg, 4);

	// write to camera MC
	write(camio, bbuf, CAMERA_CMD_PACK_SZ);


	tmo = 500;
	len = 0;
	while (len < CAMERA_CMD_PACK_SZ) {
		int q = read(camio, &bbuf[len], CAMERA_CMD_PACK_SZ - len);
		if (q > 0)
			len += q;
		else {
			usleep(10);
			if (--tmo == 0)
				break;
		}
	}

	if (!tmo) {
		printf("Camera IO ERROR\n");
	} else
		memcpy(arg, &bbuf[2], 4);

	//printf("camio %04x %d tmo:%d\n", *arg, cam_num, tmo);

//	sem_post(&camio_mutex);

	return;
}

void camera_alarm(int sig_num) {
	// Capture timeouts
	if (have_finger_on_camera[0]) {
		have_finger_on_camera[0]--;
	}
	if (have_finger_on_camera[1]) {
		have_finger_on_camera[1]--;
	}
//	printf("%d %d\n", have_finger_on_camera[0], have_finger_on_camera[1]);
	alarm(1);
}

int camera_open( void ) {

	system ("echo 31 > /sys/class/gpio/export");
	system ("echo high > /sys/class/gpio/gpio31/direction");

	cam_d = open("/dev/acd6_camera", O_RDWR);

	pthread_mutex_init(&cam0_mutex, NULL);

	if (cam_d == -1) {
		printf("no device! (cam)\n");
		return -1;
	}

	sem_init(&camio_mutex, 0, 1);
	struct termios to;

	camio = open("/dev/ttyBF1", O_RDWR);
	if (camio == -1) {
		printf("ERROR: could not open UART1\n");
		return -1;
	} else
		fcntl(camio, F_SETFL, FNDELAY);

	// set the UART1 baud rate to 57600
	tcgetattr(camio, &to); // get current port settings
	cfsetispeed(&to, B115200);
	cfsetospeed(&to, B115200); // set in & out baudrates to 57600
	to.c_cflag |= (CLOCAL | CREAD); // enable receiver & set local mode
	to.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // set raw mode (we want simple binary bytes)
	// no parity (8 data bits, 1 stop bit, no parity)
	to.c_cflag &= ~PARENB;
	to.c_cflag &= ~CSTOPB;
	to.c_cflag &= ~CSIZE;
	to.c_cflag |= CS8;
	to.c_cflag &= ~CRTSCTS; // no hardware flow control
	to.c_iflag &= ~(IXON | IXOFF | IXANY); // no software flow control
	to.c_oflag &= ~OPOST; // raw output

	tcsetattr(camio, TCSANOW, &to); // set the new port options right now
//	printf("STMIO setted up.\n");

	cam_set_bl(fam_setup.highlight[0], 0);
	cam_set_gain(fam_setup.gain[0], 0);

	cam_set_bl(fam_setup.highlight[1], 1);
	cam_set_gain(fam_setup.gain[1], 1);


	signal(SIGALRM, camera_alarm);

	unlock_cam();

	alarm(1);

	return 0;
}

void camera_close()
{
	if (cam_thread_d != NULL)
		camera_stop();

	close(camio);
	close(cam_d);

}

void camera_start() {
	printf("Starting camera\n");
	if (ioctl(cam_d, ACD_SET_CONTRAST_LIMIT0, fam_setup.contrLimit[0]) != 0)
		perror("ioctl");

	if (ioctl(cam_d, ACD_SET_CONTRAST_LIMIT1, fam_setup.contrLimit[1]) != 0)
			perror("ioctl");

	if (fam_setup.irqMode)
		if (ioctl(cam_d, ACD_START_STOP_STREAM, 1) != 0)
			perror("ioctl");

	int ret = pthread_create(&cam_thread_d, NULL, cam_stream, 0);

	if (ret) {
		printf("camera thread create fail\n");
	}

	return;
}

void camera_stop() {
	if (ioctl(cam_d, ACD_START_STOP_STREAM, 0) != 0)
		perror("ioctl");

	pthread_kill(cam_thread_d, 0);
	cam_thread_d = NULL;
	return;
}

void camera_release_buffer (u8 num) {
	if (ioctl(cam_d, ACD_RELEASE_BUFFER_SYNC, num) != 0)
			perror("ioctl ACD_");
}

volatile u32 cam_fn[2] = {0,0};
static void * cam_stream(void *ptr) {
	u16 detect_score = 0;

	struct cam_control cam_c;
	active = 1;

	printf("%s started\n", __func__);
	u16 w, h;

	while (active) {

		if (tcp_transfer)
			continue;

		cam_c.buf_ptr = NULL;
		cam_c.detected = 0;
		cam_c.cam_number = 0;

		if (fam_setup.irqMode) {

			ioctl(cam_d, ACD_GET_FDETECTED, &cam_c);
			if (cam_c.detected)
			{
				io_event(EVT_VALID_IMAGE, (u32) detect_score, cam_fn[0]++, 0, 0);
				if (fam_setup.beeperMode) 	beep(2, 0);
				camera_release_buffer(0);
			}
		}

		ioctl(cam_d, ACD_GET_BUFFER_SYNC, &cam_c);

		if (cam_c.buf_ptr != NULL) {
			stretch(FRAME_W, FRAME_H, cam_c.buf_ptr , image_fmt[fam_setup.imgMode].kx, image_fmt[fam_setup.imgMode].ky, &w, &h, stored_buf[0]);
		}


		if (tcp_transfer)
			continue;

		cam_c.buf_ptr = NULL;
		cam_c.detected = 0;
		cam_c.cam_number = 1;

		if (fam_setup.irqMode) {
			ioctl(cam_d, ACD_GET_FDETECTED, &cam_c);
			if (cam_c.detected)
			{
				io_event(EVT_VALID_IMAGE, (u32) detect_score, cam_fn[1]++, 0, 1);
				if (fam_setup.beeperMode) 	beep(2, 1);
				camera_release_buffer(1);
			}
		}
		ioctl(cam_d, ACD_GET_BUFFER_SYNC, &cam_c);
		if (cam_c.buf_ptr != NULL) {
			stretch(FRAME_W, FRAME_H, cam_c.buf_ptr , image_fmt[fam_setup.imgMode].kx, image_fmt[fam_setup.imgMode].ky, &w, &h, stored_buf[1]);
		}

	}

	return NULL;
}

void camera_reset(u8 num) {
	printf("Camera reset\n");
	camera_cmd[0] = 1;
	camera_rst = 1;

	return;
}

inline u16 estimate_contrast(u8 *line, u16 len) {
	u32 contr = 0;
	int temp;

	for (int i = 1; i < len - 1; i++) {
		temp = line[i + 1];
		temp -= line[i - 1];
		if (temp < 0)
			temp = -temp;
		contr += temp;
	}

	return contr;

}

//FIX
//	(4*(w0 + w1)*320*256*256)>>sh0) < 2^31
// w0 - вес диаганального соседа
// w1 - вес соседа по стороне
// sh0 - промежуточный сдвиг с потерей точности от переполнения
// sh1 - окончательный сдвиг для нормализации результата
//u16 check_finger1(u16 *contrast, u16 *buff, u16 limit, u16 delta, u8 w0, u8 w1, u8 sh0, u8 sh1)
//{
//	u16 i, j, retval,
//		IW = IMG_W,
//		IH = IMG_H - 256;
//	u32 IS = IMG_S;
//	s32 c = 0, w2 = -4 * ((s32)w0 + (s32)w1);
//
//#define b(x,y) ((s32)(buff[x+y*IW]>>2))
//	for (j = 8; j<IH; j+=4)
//		for (i=8; i<IW; i+=4)
//			c += abs(
//					w0*b(i-8, j-8) + w1*b(i-4,j-8) + w0*b(i, j-8)
//				+	w1*b(i-8, j-4) + w2*b(i-4,j-4) + w1*b(i, j-4)
//				+	w0*b(i-8, j) + w1*b(i-4,j) + w0*b(i, j)
//			) >> sh0;
//#undef b
//	c >>= sh1;
//
//	if (c>limit)
//		if (abs(c-*contrast)<delta)
//			retval = 1;
//		else
//			retval = 2;
//	else
//		retval = 0;
//
//	*contrast = (u16)(c > 0xffff ? 0xffff : c);
//	return retval;
//}

void stretch(u32 srcWidth, u32 srcHeight, u8 *src, float kx, float ky,
		u16 *DstWidth, u16 *DstHeight, u8 *dst) {
//    float k1, k2;
	int k1, k2;

	if ((kx == 0.5) && (ky == 0.5)) {
		*DstHeight = srcHeight >> 1;
		*DstWidth = srcWidth >> 1;
		for (u16 j = 0; j < *DstHeight; j++)
			for (u16 i = 0; i < *DstWidth; i++)
				dst[j * (*DstWidth) + i] = src[2 * j * srcWidth + i * 2];

		return;
	}

	u32 kx1 = ((1. / kx) * 256);
	u32 ky1 = ((1. / ky) * 256);

	unsigned int x = 0, y = 0;
	unsigned int i = 0, j = 0;
	unsigned int dstHeight = (unsigned int) (srcHeight * ky);
	unsigned int dstWidth = (unsigned int) (srcWidth * kx);
	unsigned int dstRow = 0, srcRow = 0;
	int srcShift = 0;
	unsigned char* pd = (unsigned char*) dst;

	dstWidth = dstWidth / 4 * 4;
	dstRow = dstWidth;
	srcRow = ((srcWidth + 3) >> 2) << 2;

	for (j = 0; j < dstHeight; j++) {
		y = (unsigned int) (j * ky1) / 256;
		if (y > srcHeight - 2)
			y = srcHeight - 2;

		k1 = j * ky1 - y * 256;

		srcShift = srcRow * y;
		for (i = 0; i < dstWidth - 1; i++) {
			x = (unsigned int) (i * kx1) / 256;
			if (x > srcWidth - 2)
				x = srcWidth - 2;

			k2 = i * kx1 - x * 256;

			pd[i] = (unsigned char) (((src)[srcShift + x] * (256 - k1)
					* (256 - k2) + (src)[srcShift + x + 1] * k2 * (256 - k1)
					+ (src)[srcShift + srcRow + x] * k1 * (256 - k2)
					+ (src)[srcShift + srcRow + x + 1] * k1 * k2) / (65536));

		}
		pd[dstWidth - 1] = src[srcShift + srcWidth - 1];

		pd += dstRow;
	}

	*DstWidth = dstRow;
	*DstHeight = dstHeight;
}

void lock_cam() {
	// wait for new frame
//	while (last_frame_counter == current_frame_counter)
//		usleep(10000);

	tcp_transfer = 1;
//	tcp_transfer_lock = 0;
//
//	while (tcp_transfer_lock == 0)
//		usleep(10000);
//
//	last_frame_counter = current_frame_counter;

}

void unlock_cam()
{
	tcp_transfer = 0;
}

u8 check_finger_presense(u8 port)
{
	return have_finger_on_camera[port];
}

u8 *get_frame(cam_format_t fmt, u16 * w, u16 * h, u8 port)
{
	u8 * src;

	if ((port != 0) && (port != 1)) {
		printf("Wrong port number - %d\n", port);
		return NULL;
	}

	lock_cam();
//
//	if (!fam_setup.irqMode) {
//		int cf =  cam_fn[port];
//		while (cf == cam_fn[port]) usleep(1000);
//	}
	//usleep(30000);

	src = stored_buf[port];

	*w = image_fmt[fam_setup.imgMode].w;
	*h = image_fmt[fam_setup.imgMode].h;

	printf("FMT:%d, sz=%d\n", fmt, *w**h);

	return src;//output_buf[port];

}
