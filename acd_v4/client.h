#pragma once

#define SERV_PORT 5000

#define CAM1_PORT 5001
#define CAM2_PORT 5002

#define EVT_DATA				147//0x93
#define	FAM_START_BYTE			0x40
#define	FAM_STOP_BYTE			0x0D
#define	FAM_ERR_BASE (char)0x00						

#define	ERR_1OK							(FAM_ERR_BASE + 0x40)
#define ERR_DATA						(FAM_ERR_BASE + 0x51)
#define ERR_LDATA						(FAM_ERR_BASE + 0x52)


#define	ERR_ERROR				(FAM_ERR_BASE -1)
#define	ERR_BAD_ARGUMENT		(FAM_ERR_BASE -2)
#define	ERR_CRC_ERROR			(FAM_ERR_BASE -3)
#define	ERR_UNKNOWN_COMMAND		(FAM_ERR_BASE -4)
#define	ERR_INVALID_STOP_BYTE	(FAM_ERR_BASE -5)
#define	ERR_I2C_ERROR			(FAM_ERR_BASE -6)
#define ERR_NO_VALID_IMAGE		(FAM_ERR_BASE -7)
#define ERR_TOUT				(FAM_ERR_BASE -8)
#define ERR_QUE_OVERFLOW		(FAM_ERR_BASE -9)


#define CMD_GET_VERSION			1
#define CMD_DEBUG				5
#define CMD_SETUP				6
#define CMD_FAM_SETUP			7

#define CMD_FLASH								10
#define CMD_GET_DATA_ADDRESS 	11
#define CMD_SLEEP							 	12

#define CMD162_OFFSET		50

#define	CMD162_LED_CTL		1 + CMD162_OFFSET

#define CMD162_PRINT		3 + CMD162_OFFSET
#define	CMD162_BUZZER_CTL	2 + CMD162_OFFSET
#define CMD162_EXDEV_INIT	4 + CMD162_OFFSET

#define CMD162_LOCK_CTL			6 + CMD162_OFFSET
#define CMD162_TURNSTILE_CTL	7 + CMD162_OFFSET
#define CMD162_IO_CTL			8 + CMD162_OFFSET



#define CMD_IMAGE_DUMP			68
#define CMD_IMAGE_DUMP2			69
#define CMD_CHECK_FINGER		70
#define CMD_GETTICKCOUNT		80

#define	CMD_SWITCH_CAM			100

#define CMD_SET_GAIN			116
#define CMD_SET_HIGHLIGHT		117
#define CMD_SET_OFFSET			118

#define CMD_READ_REG			119
#define CMD_WRITE_REG			120

#define CMD_UPLOAD_DATA			122
#define CMD_FLASH_WRITE			123
#define CMD_FLASH_READ			124

#define	CMD_M162_CTL		131

#define CMD_GET_SETTINGS		132//0x84

#define CMD_READ_ROM		133
#define CMD_WRITE_ROM		134
#define CMD_REBOOT			135

#define CMD_GET_TIME			137//0x89
#define CMD_SET_TIME			138//0x8a

#define EVT_VALID_IMAGE			144
#define EVT_M162			146
#define EVT_DATA			147
#define EVT_OW				148
#define EVT_LOG				149
#define EVT_KEY				151
#define EVT_COMMON			152

#define CMD_REMOTE_CAM_SELECT	200
#define CMD_REMOTE_CAM_READ	201
#define CMD_REMOTE_CAM_WRITE	202


#define ERR_EVT_FINGER			0x01
#define ERR_EVT_KEY			0x02
#define ERR_EVT_OW1			0x03
#define ERR_EVT_OW2			0x04

enum {
	LED_OFF,
	LED_GREEN,
	LED_RED,
	LED_ORANGE
};

#define CAM_W 1280
#define CAM_H 1024

enum LOCK_CMD {
	DOOR_LOCK_CMD	=1,
	DOOR_UNLOCK_CMD,
	DOOR_PASS_CMD,
	DOOR_GETS_CMD,
	DOOR_FLOCK_CMD
};

enum TURN_CMD {
	TURN_CMD_UNLOCK_A = 1,
    TURN_CMD_UNLOCK_B,
    TURN_CMD_UNLOCK_AB,
    TURN_CMD_STOP,
    TURN_CMD_PASS_A,
	TURN_CMD_PASS_B,
	TURN_CMD_TIMEOUT,
	TURN_CMD_GET_STATE,
	TURN_ERR_DEV_NOT_READY = 254
};

enum ext_devices {
	NO_DEVICE = 0,
    LOCKER,
    TURNIKET
};

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

typedef char s8;
typedef short s16;
typedef int s32;


void build_packet(unsigned char cmd, unsigned int param1, unsigned int param2, unsigned char err, u8 * txBuff);
void build_packet_data(unsigned char cmd, u8 *data, unsigned char err, u8 * txBuff);
bool process_packet(u8 *buff, u8 *cmd, int *param1, int *param2, u8 *err);
void io_event (u8 event, u32 arg1, u32 arg2, u8 err, u8 port);
void make_cmd( u8 * buf, u8 cmd );
void transmit( u8 cmd, u8 *packet);
void beep(u8 l, u8 p);
struct FAM_SETUP {

	u8 ip[4];
	u8 nm[4];
	u8 gw[4];
	u8 mac[6];

	u16 port[2];

	u8 mediaType;
	u8 gain[2];
	u16 offset[2];
	u8 highlight[2];

	u8 irqMode;
	u8 imgMode; // 0 - 1280*1024, 1 - 1152*768

	u16 contrLimit[2];
	u16 contrDelta[2];

	u8 connTout;
	u8 beeperMode;

	u16 X0;	// смещение картинки 1152*768 относительно начала координат
	u16 Y0;
	u16 dX;
	u16 dY;

	u32 serial;

} ;

struct LOG_MESSAGES {
	int count;
	u8 buf[1024];
};

extern struct FAM_SETUP fam_setup;

struct ip_setup
{
	u8 ip[4];
	u8 nm[4];
	u8 gw[4];
	u8 mac[6];
};

