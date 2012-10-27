#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <syslog.h>
#include <string.h>
#include <sys/wait.h>
#include "acd_server.h"

int main(int argc, char *argv[]) {

	pid_t w, pID = vfork();
	int status = 0;

	if (pID == 0)                // child
	{
		printf("Starting ACD_SERVER\n");
		sleep(1);
		system("./mnt/sdcard/acd_server &");
		exit(EXIT_SUCCESS);
	}
	else if (pID < 0)            // failed to fork
	{
		perror("fork of single user shell failed\n");
		exit(EXIT_FAILURE);
	}
	else                                   // parent
	{
		 while(wait(&status) != pID) /* nothing */;
		 printf("Child destroyed\n");
		 exit(EXIT_SUCCESS);
	}
	exit(EXIT_SUCCESS);
}
