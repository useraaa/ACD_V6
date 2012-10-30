/*
 * rtc.cpp
 *
 *  Created on: 28.10.2012
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

static int rtc_fd = 0;

/* Shift values for RTC_STAT register */
#define DAY_BITS_OFF    17
#define HOUR_BITS_OFF   12
#define MIN_BITS_OFF    6
#define SEC_BITS_OFF    0

/* Some helper functions to convert between the common RTC notion of time
 * and the internal Blackfin notion that is encoded in 32bits.
 */

#define LEAPS_THRU_END_OF(y) ((y)/4 - (y)/100 + (y)/400)

static const unsigned char rtc_days_in_month[] = {
	31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};
static const unsigned short rtc_ydays[2][13] = {
	/* Normal years */
	{ 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365 },
	/* Leap years */
	{ 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366 }
};

static inline bool is_leap_year(unsigned int year)
{
	return (!(year % 4) && (year % 100)) || !(year % 400);
}

int rtc_month_days(unsigned int month, unsigned int year)
{
	return rtc_days_in_month[month] + (is_leap_year(year) && month == 1);
}

unsigned long
mktime(const unsigned int year0, const unsigned int mon0,
       const unsigned int day, const unsigned int hour,
       const unsigned int min, const unsigned int sec)
{
	unsigned int mon = mon0, year = year0;

	/* 1..12 -> 11,12,1..10 */
	if (0 >= (int) (mon -= 2)) {
		mon += 12;	/* Puts Feb last since it has leap day */
		year -= 1;
	}

	return ((((unsigned long)
		  (year/4 - year/100 + year/400 + 367*mon/12 + day) +
		  year*365 - 719499
	    )*24 + hour /* now have hours */
	  )*60 + min /* now have minutes */
	)*60 + sec; /* finally seconds */
}


void rtc_time_to_tm(unsigned long time, struct rtc_time *tm)
{
	unsigned int month, year;
	int days;

	days = time / 86400;
	time -= (unsigned int) days * 86400;

	/* day of the week, 1970-01-01 was a Thursday */
	tm->tm_wday = (days + 4) % 7;

	year = 1970 + days / 365;
	days -= (year - 1970) * 365
		+ LEAPS_THRU_END_OF(year - 1)
		- LEAPS_THRU_END_OF(1970 - 1);
	if (days < 0) {
		year -= 1;
		days += 365 + is_leap_year(year);
	}
	tm->tm_year = year - 1900;
	tm->tm_yday = days + 1;

	for (month = 0; month < 11; month++) {
		int newdays;

		newdays = days - rtc_month_days(month, year);
		if (newdays < 0)
			break;
		days = newdays;
	}
	tm->tm_mon = month;
	tm->tm_mday = days + 1;

	tm->tm_hour = time / 3600;
	time -= tm->tm_hour * 3600;
	tm->tm_min = time / 60;
	tm->tm_sec = time - tm->tm_min * 60;
}

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
static inline void rtc_bfin_to_tm(u32 rtc_bfin, struct rtc_time *tm)
{
	rtc_time_to_tm(rtc_bfin_to_time(rtc_bfin), tm);
}

int rtc_tm_to_time(struct rtc_time *tm, unsigned long *time)
{
	*time = mktime(tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
			tm->tm_hour, tm->tm_min, tm->tm_sec);
	return 0;
}

void init_rtc()
{
	int ret;
	struct rtc_time rtc_tm;
	char *rtc_dev = "/dev/rtc0";

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

void rtc_set_time (u8 sec, u8 min, u8 hour, u16 day)
{
	struct rtc_time rtc_tm;
	int ret = 0;
    u32 itemp = sec & 63;

    itemp |= ((uint)min & 63)<<6;
    itemp |= ((uint)hour & 31)<<12;
    itemp |= ((uint)day)<<17;

	printf("Set RTC Time\n");
	rtc_bfin_to_tm(itemp, &rtc_tm);

	ret = ioctl(rtc_fd, RTC_SET_TIME, &rtc_tm);
	if (ret == -1) {
		perror("rtc ioctl RTC_SET_TIME error");
	}
}

void rtc_get_time (u8 * sec, u8 * min, u8 * hour, u16 * day)
{
	struct rtc_time rtc_tm;
	int ret = 0;
	ulong itemp = 0;

	printf("Get RTC Time\n");
	ret = ioctl(rtc_fd, RTC_RD_TIME, &rtc_tm);
	if (ret == -1) {
		perror("rtc ioctl RTC_RD_TIME error");
	}

	rtc_tm_to_time(&rtc_tm, &itemp);

    *sec = itemp&63;
    *min = (itemp>>6)&63;
    *hour = (itemp>>12)&31;
    *day = itemp>>17;
}

long get_cur_ms()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

