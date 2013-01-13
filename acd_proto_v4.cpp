/*
 * acd_proto_v4.cpp
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
#include "acd_stmio.h"
#include "fam_setup.h"
#include "acd_proto.h"
#include "rtc.h"


extern int clientsock, clientsocks[5], log_socket;

void send_buf(int sock, u8 *buf, u32 len)
{
	//int total_len = len;
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
#define ARG_READ				1
#define ARG_WRITE				2
#define PR_IMGMODE				1
#define PR_IRQMODE				2
#define PR_BEEPERMODE			3

void do_protocol(int sock, u8* buffer, u8 port)
{
	u8 cmd, err;
	u32 arg1, arg2;

	process_packet((u8 *) buffer, &cmd, &arg1, &arg2, &err);

	printf("%d: CMD:%d, ARG1:%d, ARG2:%d, ERR:%d\n", fam_setup.port[port], cmd, arg1, arg2, err);

	switch (cmd) {

		case CMD_SET_TIME:
			rtc_set_time( ((u8*)&arg1)[0], ((u8*)&arg1)[1], ((u8*)&arg1)[2], ((u16*)&arg2)[0]);
			break;

		case CMD_GET_TIME:
			rtc_get_time(&((u8*)&arg1)[0], &((u8*)&arg1)[1], &((u8*)&arg1)[2], (u16*)&arg2);
			break;
		// legacy command;
		case CMD_SETUP:

			if (err == ARG_READ)
			    {
			        switch (arg1)
			        {
			            case PR_IMGMODE:
			            	arg2 = fam_setup.imgMode;
			            	break;

			            case PR_IRQMODE:
			            	arg2 = fam_setup.irqMode;
			            	break;

			            case PR_BEEPERMODE:
			            	arg2 = fam_setup.beeperMode;
			        }
			    }
			    if (err == ARG_WRITE)
			    {
			        switch (arg1)
			        {
			            case PR_IMGMODE:
			            	fam_setup.imgMode = arg2;
			            	break;

			            case PR_IRQMODE:
			            	fam_setup.irqMode = arg2;
			            	break;

			             case PR_BEEPERMODE:
			            	 fam_setup.beeperMode = arg2;
			        }
			    }
			break;

		case CMD_GET_VERSION:
			arg2 = fam_setup.serial;
			arg1 = 0x601;
			err = ERR_1OK;

			build_packet(cmd, arg1, arg2, err, (u8 *) buffer);
			send(sock, buffer, CTRL_PACK, 0);
			break;

		case CMD_IMAGE_DUMP: {

			u16 w = 0, h = 0;
			long bg = get_cur_ms();
			int pcklen = arg1, frame_size = 0;

			if (pcklen < 512)
				pcklen = 512;

			camera_stop();

			if (arg2 == 1)	// legacy check finger
				if (check_finger_presense(port) == 0) {
					build_packet(cmd, arg1, arg2, ERR_NO_VALID_IMAGE, (u8 *) buffer);
					send(sock, buffer, CTRL_PACK, 0);
					break;
				}

			u8 *ptr = get_frame(err, &w, &h, port);

			if (ptr == NULL) {
				build_packet(cmd, 0, 0, ERR_NO_VALID_IMAGE, (u8 *) buffer);
				send(sock, buffer, CTRL_PACK, 0);
				printf("Camera not ready\n");
				break;
			}

			frame_size = w * h;
			arg1 = frame_size;
			arg2 = w;
			err = ERR_1OK;

			build_packet(cmd, arg1, arg2, err, (u8 *) buffer);

			send_buf(sock, buffer, CTRL_PACK);

			if (pcklen > frame_size)
				pcklen  = frame_size;

			int 	tail = frame_size % pcklen,
					ctrl = frame_size / pcklen;
//			printf("NumOfChunks=%d\n", ctrl);

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

			camera_start();

			printf("Time period %d\n", get_cur_ms() - bg);

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
			err = camera_iocmd(0x34, (u32 *) &arg1, port);
			arg1 = 0;
			err |= camera_iocmd(0x33, (u32 *) &arg1, port);
			build_packet(cmd, arg1, 0, err, (u8 *) buffer);
			send(sock, buffer, CTRL_PACK, 0);
			break;
		}
		case CMD_SET_OFFSET: {
			err = camera_iocmd(0x38, (u32 *) &arg1, port);
			arg1 = 0x61;
			err |= camera_iocmd(0x31, (u32 *) &arg1, port);
			printf("SET OFFSET returns %d \n", arg1);
			build_packet(cmd, arg1, 0, err, (u8 *) buffer);
			send(sock, buffer, CTRL_PACK, 0);
			break;
		}

		case CMD_SET_GAIN: {
			err = ERR_1OK;
			fam_setup.gain[port] = arg1;
			arg2 = arg1;	// save initial value to return it to the host
			arg1 = (((u16) arg1) << 16) | 0x35;
			err = camera_iocmd(0x32, (u32 *) &arg1, port);
			arg1 = 0x35;
			err |= camera_iocmd(0x31, (u32 *) &arg1, port);

			build_packet(cmd, arg1, 0, err, (u8 *) buffer);
			send(sock, buffer, CTRL_PACK, 0);
			break;
		}

		case CMD_M162_CTL: {
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
			data[6] = fam_setup.beeperMode;
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
				data[2] = fam_setup.legacybyte[port];
				data[3] = fam_setup.X0[port];
				data[4] = fam_setup.Y0[port];
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
				fam_setup.legacybyte[port] = data[2];
				fam_setup.X0[port]	= data[3];
				fam_setup.Y0[port]	= data[4];
				fam_setup.imgMode = data2[5];
			}
			save_setup(&fam_setup);
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

u8 check_sum(u8 *buff) {
	u32 temp = 0, i;
	for (i = 0; i < 11; i++) {
		temp += (u8) buff[i];
	}
	return temp % 256;
}

bool process_packet(u8 *buff, u8 *cmd, u32 *param1, u32 *param2, u8 *err)
{
	// TODO no error at wrong checksum in acd_proto
	if (check_sum(buff) != buff[11])
		printf("Error checksum\n");

	*cmd = buff[1];
	*err = buff[10];
	u32 temp;
	temp = buff[5];
	temp = temp * 256 + buff[4];
	temp = temp * 256 + buff[3];
	temp = temp * 256 + buff[2];
	*param1 = temp;
	temp = buff[9];
	temp = temp * 256 +  buff[8];
	temp = temp * 256 +  buff[7];
	temp = temp * 256 +  buff[6];
	*param2 = temp;

	return 0;
}

void build_packet_data(u8 cmd, u8 *data, u8 err, u8 * txBuff) {
	txBuff[0] = FAM_START_BYTE;
	txBuff[1] = cmd;
	memcpy(&txBuff[2], data, 8);
	txBuff[10] = err;
	txBuff[11] = check_sum(txBuff);
	txBuff[12] = FAM_STOP_BYTE;
	return;
}

void build_packet(u8 cmd, u32 param1, u32 param2, u8 err, u8 * txBuff) {
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
