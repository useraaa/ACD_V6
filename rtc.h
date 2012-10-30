/*
 * rtc.h
 *
 *  Created on: 28.10.2012
 *      Author: user
 */

#ifndef RTC_H_
#define RTC_H_


void init_rtc();
void rtc_get_time (u8 * sec, u8 * min, u8 * hour, u16 * day);
void rtc_set_time (u8 sec, u8 min, u8 hour, u16 day);
long get_cur_ms();

#endif /* RTC_H_ */
