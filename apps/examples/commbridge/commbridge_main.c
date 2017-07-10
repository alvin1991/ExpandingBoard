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
#include <nuttx/sensors/ccfd16_serial.h>
#include <nuttx/sensors/ccf_rfid_serial.h>

//mavlink
#include "mavlink/minimal/mavlink.h"



/****************************************************************************
 * Public Parameters
 ****************************************************************************/
enum
{
	MAG_FINDER_FRONT = 0,
	MAG_FINDER_BACK,
	RFID_READER,
	ULTRA_SONIC,
	MAX_SENSOR_NUMS
};

typedef struct CommBridge
{
	bool				_class_instance;
	bool			    _should_exit;
	int					_pid;
	int					_fd[MAX_SENSOR_NUMS];
	int (*cycle)(void);
}CommBridge_t;

static CommBridge_t	*g_dev;


/****************************************************************************
 * Public Functions
 ****************************************************************************/

static void
cycle(void)
{

	int fd,byte_send,len,ret = -1;
	struct ccfd16_data_s ccfd16f;
	struct ccfd16_data_s ccfd16b;
	struct ccf_rfid_data_s ccf_rfid;

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


	/* open serial port for magfinder front */
	g_dev->_fd[MAG_FINDER_FRONT] = open("/dev/magf",O_RDWR);
	if (g_dev->_fd[MAG_FINDER_FRONT] < 0){
		printf("[magf] open failed: %s\n", strerror(ret));
		exit(EXIT_FAILURE);
	}

	/* open serial port for magfinder back */
	g_dev->_fd[MAG_FINDER_BACK] = open("/dev/magb", O_RDWR);
	if (g_dev->_fd[MAG_FINDER_BACK] < 0){
		printf("[magb] open failed: %s\n", strerror(ret));
		exit(EXIT_FAILURE);
	}

	/* open serial port for RFID sensor */
	g_dev->_fd[RFID_READER] = open("/dev/rfid",O_RDWR);
	if (g_dev->_fd[RFID_READER] < 0){
		printf("[rfid] open failed: %s\n", strerror(ret));
		exit(EXIT_FAILURE);
	}

	while(!g_dev->_should_exit){

		/*Send Heartbeat */
//		len = mavlink_msg_heartbeat_encode(0x01,
//				0x02,
//				&msgpacket,
//				&heartbeat);
//		byte_send = write(g_dev->_fd,&msgpacket,len);
//		if(byte_send > 0)
//		{
//			printf("write:%d\n",byte_send);
//			byte_send = 0;
//			usleep(2000000);
//		}

		ret = read(g_dev->_fd[MAG_FINDER_FRONT],&ccfd16f,sizeof(struct ccfd16_data_s));
		if(ret < 0){
			printf("[magf] read failed: %s\n", strerror(ret));
		}

		ret = read(g_dev->_fd[MAG_FINDER_BACK],&ccfd16b,sizeof(struct ccfd16_data_s));
		if(ret < 0){
			printf("[magb] read failed: %s\n", strerror(ret));
		}

		ret = read(g_dev->_fd[RFID_READER],&ccf_rfid,sizeof(struct ccf_rfid_data_s));
		if(ret < 0){
			printf("[rfid] read failed: %s\n", strerror(ret));
		}
		printf("[magf]:%x %x {%d-%d}  [magb]:%x %x {%d-%d}   [rfid]:%x {%d-%d}\n",\
				ccfd16f._A,ccfd16f._B,ccfd16f._state,ccfd16f._time_stamp,
				ccfd16b._A,ccfd16b._B,ccfd16b._state,ccfd16b._time_stamp,
				ccf_rfid.ID,ccf_rfid._state,ccf_rfid._time_stamp);
		usleep(1000*1000);
	}

}






/****************************************************************************
 * CommBridge_main
 ****************************************************************************/

/**
 * Initialize the device data struct.
 */
static int
init(void)
{
	/* erase all memory */
	memset(g_dev,0,sizeof(CommBridge_t));

	/* initialize member */
	g_dev->_class_instance = true;
	g_dev->_should_exit = false;
	g_dev->cycle = cycle;

	return OK;
}


/**
 * Start the driver.
 */
static int
start(int argc, FAR char *argv[])
{

	/* creat commbrdge task */
	g_dev->_pid = task_create( "commbridge",\
			CONFIG_EXAMPLES_COMMBRIDGE_PRIORITY,\
			CONFIG_EXAMPLES_COMMBRIDGE_STACKSIZE,\
			g_dev->cycle, argv);

	if(g_dev->_pid < 0){
		printf("ERROR: Failed to start commbridge: %d\n",strerror(g_dev->_pid));
		return g_dev->_pid;
	}
	return OK;
}

/**
 * Stop the device.
 */
int
stop(int argc, FAR char *argv[])
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
			printf("communication bridge is already started.\n");
			exit(EXIT_FAILURE);
		}

		/* create the device */
		g_dev =  (CommBridge_t *)malloc(sizeof(CommBridge_t));

		if (g_dev == NULL) {
			printf("communication bridge implement error.\n");
			goto fail;
		}

		/* initialize the driver */
		init();


		/* start the device */
		if (OK != start(argc, argv)) {
			goto fail;
			printf("communication bridge start error.\n");
			exit(EXIT_FAILURE);
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
		printf("communication bridge start failed");
        return ERROR;

}



//#ifdef CONFIG_BUILD_KERNEL
//int main(int argc, FAR char *argv[])
//#else
//int commbridge_main(int argc, char *argv[])
//#endif
//{
//	int ret = -1;
//	struct ccfd16_data_s ccfd16;
//	struct ccf_rfid_data_s ccf_rfid;
//
//	/* invalid ParamSet */
//	if (*argv[1] != '-'){
//	  printf("[Magfiner]:Invalid options format: %s\n", argv[1]);
//	  exit(EXIT_FAILURE);
//	}
//
//	/* start magfinder */
//	if('s' == *(++argv[1])){
//
//		/*
//		 * open serial port for magfinder front
//		 * */
//		ret = open("/dev/magf",O_RDWR);
//		if (ret < 0){
//			printf("[magf] open failed: %s\n", strerror(ret));
//			exit(EXIT_FAILURE);
//		}
//
//		ret = read(ret,&ccfd16,sizeof(struct ccfd16_data_s));
//		if(ret < 0){
//			printf("[magf] read failed: %s\n", strerror(ret));
//		}else{
//			printf("[magf]:value1:%x value2:%x state:%d timestamp:%d \n",ccfd16._A,ccfd16._B,ccfd16._state,ccfd16._time_stamp);
//		}
//
//		/*
//		 * open serial port for magfinder back
//		 * */
//		ret = open("/dev/magb",O_RDWR);
//		if (ret < 0){
//			printf("[magb] open failed: %s\n", strerror(ret));
//			exit(EXIT_FAILURE);
//		}
//
//		ret = read(ret,&ccfd16,sizeof(struct ccfd16_data_s));
//		if(ret < 0){
//			printf("[magb] read failed: %s\n", strerror(ret));
//		}else{
//			printf("[magb]:value1:%x value2:%x state:%d timestamp:%d \n",ccfd16._A,ccfd16._B,ccfd16._state,ccfd16._time_stamp);
//		}
//
//		/*
//		 * open serial port for RFID sensor
//		 * */
//		ret = open("/dev/rfid",O_RDWR);
//		if (ret < 0){
//			printf("[rfid] open failed: %s\n", strerror(ret));
//			exit(EXIT_FAILURE);
//		}
//
//		ret = read(ret,&ccf_rfid,sizeof(struct ccf_rfid_data_s));
//		if(ret < 0){
//			printf("[rfid] read failed: %s\n", strerror(ret));
//		}else{
//			printf("[rfid]:ID:%x state:%d timestamp:%d \n",ccf_rfid.ID,ccf_rfid._state,ccf_rfid._time_stamp);
//		}
//
//	}
//
//	exit(EXIT_SUCCESS);
//	return OK;
//
//}
