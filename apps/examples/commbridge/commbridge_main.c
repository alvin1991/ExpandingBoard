/****************************************************************************
 * examples/commbridge/commbridge.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

//std
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

//
#include <poll.h>
#include <fcntl.h>
#include <errno.h>

//nuttx
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>

//mavlink
#include "mavlink/minimal/mavlink.h"

/****************************************************************************
 * Public Parameters
 ****************************************************************************/
typedef struct CommBridge
{
	bool				_class_instance;
	bool			    _should_exit;
	int					_cycling_rate;
	int					_pid;
	int					_fd;
	int (*start)(char *[]);
	int (*cycle)(void);
}CommBridge_t;

static CommBridge_t	*g_dev;


/****************************************************************************
 * Public Functions
 ****************************************************************************/

void
cycle(void)
{

	int fd,byte_send,len;

	/*
	 * Mavlink Package
	 */
	mavlink_message_t  msgpacket;
	memset(&msgpacket,0,sizeof(msgpacket));

	/*
	 * Mavlink heartbeat
	 */
	mavlink_heartbeat_t packet_in ={
	0,
	MAV_TYPE_GROUND_ROVER,
	MAV_AUTOPILOT_PIXHAWK,
	MAV_MODE_FLAG_DECODE_POSITION_STABILIZE,
	MAV_STATE_ACTIVE,
	2};
	mavlink_heartbeat_t heartbeat;
	memset(&heartbeat,0,sizeof(heartbeat));
	heartbeat.custom_mode = packet_in.custom_mode;
	heartbeat.type = packet_in.type;
	heartbeat.autopilot = packet_in.autopilot;
	heartbeat.base_mode = packet_in.base_mode;
	heartbeat.system_status = packet_in.system_status;
	heartbeat.mavlink_version = packet_in.mavlink_version;


	while(!g_dev->_should_exit){

		/*Send Heartbeat */
		len = mavlink_msg_heartbeat_encode(0x01,
				0x02,
				&msgpacket,
				&heartbeat);
		byte_send = write(g_dev->_fd,&msgpacket,len);
		if(byte_send > 0)
		{
			printf("write:%d\n",byte_send);
			byte_send = 0;
			usleep(2000000);
		}
	}

}

/*!
 * Local functions in support of the shell command.
 */
/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;


/**
 * Start the driver.
 */
int
start(char *argv[])
{
	/* creat commbrdge task */
	g_dev->_pid = task_create( "commbridge",\
			CONFIG_EXAMPLES_COMMBRIDGE_PRIORITY,\
			CONFIG_EXAMPLES_COMMBRIDGE_STACKSIZE,\
			g_dev->cycle, argv);

	if(g_dev->_pid < 0){
		int errcode = errno;
		fprintf(stderr, "ERROR: Failed to start commbridge: %d\n",\
				errcode);
		return -errcode;
	}
	return OK;
}




/****************************************************************************
 * CommBridge_main
 ****************************************************************************/

/**
 * init the device.
 */
int
init(void)
{
	/* initialize device */
	g_dev->_cycling_rate = 100;
	g_dev->_should_exit = false;
	g_dev->cycle = cycle;
	g_dev->start = start;
	return OK;
}

/**
 * Stop the device.
 */
int
stop(void)
{
	exit(0);
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int commbridge_main(int argc, char *argv[])
#endif
{
	/* invalid ParamSet */
	if (*argv[1] != '-'){
	  printf("Invalid options format: %s\n", argv[1]);
	  exit(EXIT_FAILURE);
	}

	/* start commbridge */
	if('s' == *(++argv[1])){
		/* only exist a device  */
		if (g_dev != NULL) {
			printf("device already started.\n");
			exit(EXIT_FAILURE);
		}

		/* create the device */
		g_dev =  (CommBridge_t *)malloc(sizeof(CommBridge_t));

		if (g_dev == NULL) {
			printf("device implement error.\n");
			goto fail;
		}

		/* initialize the device */
		if (OK != init()) {
			goto fail;
			printf("device initialize error.\n");
			exit(EXIT_FAILURE);
		}

		/* open serial port for the device */
		g_dev->_fd = open("/dev/ttyS1", O_RDWR);
		if (g_dev->_fd < 0){
			printf(stderr, "ERROR: open failed: %d.\n", errno);
			goto fail;
		}

		/* start the device */
		if (g_dev->start(argv) < 0){
			printf(stderr, "ERROR: start failed: %d.\n", errno);
			goto fail;
		}

	}

	/* stop commbridge */
	if('c' == *(++argv[1])){
		//....
	}

	exit(EXIT_SUCCESS);
	return OK;

	fail:
		if (g_dev != NULL) {
			free(g_dev);
			g_dev = NULL;
		}
		printf("device start failed");
        return ERROR;

}

