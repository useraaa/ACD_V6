/*
 * acd_server.h
 *
 *  Created on: 01.10.2012
 *      Author: user
 */

#ifndef ACD_SERVER_H_
#define ACD_SERVER_H_

#define BANNER ("PPI TEST v.6.0.1")

#define CTRL_PACK 13

#define OW_EVENT1 12
#define OW_EVENT2 13
#define IO_EVENT 14

#define MAXPENDING 5    /* Max connection requests */
#define BUFFSIZE  13

/* Shift values for RTC_STAT register */
#define DAY_BITS_OFF    17
#define HOUR_BITS_OFF   12
#define MIN_BITS_OFF    6
#define SEC_BITS_OFF    0

void run_acdserver();
void stop_acdserver();


#endif /* ACD_SERVER_H_ */
