/*
 * network.cpp
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
#include "camera.h"
#include "ext_dev.h"
#include "acd_server.h"
#include "acd_proto.h"
#include "network.h"


static void * net_stream(void *ptr);
static void * log_stream(void *arg);

int clientsock, clientsocks[5], log_socket;
struct LOG_MESSAGES log_message;
pthread_t net_thread_d1, net_thread_d2, log_thread_d;



void Die(char *mess) {
	perror(mess);
	return;
}

void network_start()
{

	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setstacksize(&attr, 1024 * 1024 * 2);


	char cmd[64];
	sprintf(cmd, "ifconfig eth0 up %d.%d.%d.%d", fam_setup.ip[3],
			fam_setup.ip[2], fam_setup.ip[1], fam_setup.ip[0]);
	printf("Assign eth0 [%s]\n", cmd);
	system(cmd);

	printf("launch port1\n");
	if (pthread_create(&net_thread_d1, &attr, net_stream, &fam_setup.port[0])) {
		printf("stmio thread1 create fail\n");
	}
	printf("launch port2\n");

	if (pthread_create(&net_thread_d2, &attr, net_stream, &fam_setup.port[1])) {
		printf("stmio thread2 create fail\n");
	}

	if (pthread_create(&log_thread_d, NULL, log_stream, 0)) {
		printf("log thread create fail\n");
	}
}

#define TCP_NODELAY 1
void HandleClient(int sock, u8 port)
{
	u8 buffer[CTRL_PACK * 2];
	int received = 0;

	if ((port != 0) && (port != 1)) {
		printf("Wrong port number - %d\n", port);
		return;
	}

	do {
		/* Send back received data */
		/* Check for more data */
		unlock_cam();

		if ((received = recv(sock, buffer, CTRL_PACK, 0)) < 0) {
			Die("Failed to receive additional bytes from client");
		}

		if (received == CTRL_PACK) {


			do_protocol(sock, buffer, port);

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


void _log (const char *fmt, ...)
{
	va_list args;

	va_start (args, fmt);

	int len = vsprintf ( (char *)log_message.buf, fmt, args);

	log_message.count = len;

	va_end (args);

	return;
}



