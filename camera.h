/*
 * camera.h
 *
 *  Created on: 07.07.2012
 *      Author: user
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include <sys/ioctl.h>

#define ACD_MAGIC		'C'
#define ACD_GET_REG		_IOWR(ACD_MAGIC, 0, struct camera_reg *)
#define ACD_SET_REG		_IOW(ACD_MAGIC, 1, struct camera_reg *)

#define ACD_GET_LIGHT		_IOWR(ACD_MAGIC, 2, u16)
#define ACD_SET_LIGHT		_IOW(ACD_MAGIC, 3, u16)

#define ACD_START_STOP_STREAM	_IOW(ACD_MAGIC, 4, u32)
#define ACD_GET_BUFFER_SYNC			_IOWR(ACD_MAGIC, 5, u32)
#define ACD_RELEASE_BUFFER_SYNC 	_IOWR(ACD_MAGIC, 6, u32)
#define ACD_SET_NEW_FORMAT			_IOWR(ACD_MAGIC, 7, u32)
#define ACD_SET_CONTRAST_LIMIT0		_IOWR(ACD_MAGIC, 8, u32)
#define ACD_SET_CONTRAST_LIMIT1		_IOWR(ACD_MAGIC, 9, u32)
#define ACD_GET_FDETECTED			_IOWR(ACD_MAGIC, 10, u32)

#define FRAME_W 1280
#define FRAME_H 1024
#define FRAME_S (FRAME_W*FRAME_H)
#define CAMERA_CMD_PACK_SZ 11


typedef enum cam_format {
	fmt1280x1024 = 0,
	fmt640x512 = 1,
	fmt384x600 = 3
} cam_format_t;

struct _image_state {
    int cx;
    int cy;
    int contrLimit;
    int contrDelta;
};

extern volatile u8 tcp_transfer;

int camera_open ();
void camera_close ();

void camera_stop();
void camera_start();

u8 check_finger_presense(u8 port);
void camera_iocmd(u8 cmd, u32 *arg, u8 cam_num);

void lock_cam();
void unlock_cam();

u8 *get_frame (cam_format_t fmt, u16 * w, u16 * h, u8 port);
void cam_beep(u8 l, u8 p);


#endif /* CAMERA_H_ */
