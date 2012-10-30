/*
 * acd_proto.h
 *
 *  Created on: 27.10.2012
 *      Author: user
 */

#ifndef ACD_PROTO_H_
#define ACD_PROTO_H_


void build_packet(u8 cmd, u32 param1, u32 param2, u8 err, u8 * txBuff);
void build_packet_data(u8 cmd, u8 *data, u8 err, u8 * txBuff);
bool process_packet(u8 *buff, u8 *cmd, u32 *param1, u32 *param2, u8 *err);
void do_protocol(int sock, u8* buffer, u8 port);
void send_buf(int sock, u8 *buf, u32 len);

#endif /* ACD_PROTO_H_ */
