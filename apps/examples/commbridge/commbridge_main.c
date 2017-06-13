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
 * Public Functions
 ****************************************************************************/
typedef struct CommBridge
{
	int					_class_instance;
	bool			    _sensor_ok;
	int					_measure_ticks;
	int					_cycling_rate;
}CommBridge_t;

int
init(void)
{
	//1订阅相关传感器的数据

	//2初始化通信桥串口驱动

	//3测试通信桥工作正常

	//4.创建task或者加入workqueue
	return OK;
}

void
cycle(void)
{
	//1.重新加入一次workqueue

}

/*!
 * Local functions in support of the shell command.
 */
/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

CommBridge_t	*g_dev;

/**
 * Start the driver.
 */
int
start(void)
{
	int fd,val;

	/*
	 *打开串口ttyS1作为通信桥接口
	 */
	fd = open("/dev/ttyS1", O_RDWR);
	if (fd < 0){
		printf(stderr, "ERROR: open failed: %d.\n", errno);
		return EXIT_FAILURE;
	}

	for(;;){
		val = write(fd,"This is CommBridge.\n",20);

		if(val > 0)
		{
			printf("write:%d\n",val);
			val = 0;
			usleep(1000000);
		}
	}
	return EXIT_SUCCESS;


	mavlink_heartbeat_t packet_in ={
	963497464,17,84,151,218,3
	};
	mavlink_heartbeat_t packet1;
	memset(&packet1,0,sizeof(packet1));
	packet1.custom_mode = packet_in.custom_mode;
	packet1.type = packet_in.type;
	packet1.autopilot = packet_in.autopilot;
	packet1.base_mode = packet_in.base_mode;
	packet1.system_status = packet_in.system_status;
	packet1.mavlink_version = packet_in.mavlink_version;

//	mavlink_msg_heartbeat_send(MAVLINK_COMM_1 ,
//			packet1.type ,
//			packet1.autopilot ,
//			packet1.base_mode ,
//			packet1.custom_mode ,
//			packet1.system_status );


	/* only exsit a CommBridge  */
	if (g_dev != NULL) {
		printf("already started.\n");
		exit(1);
	}

	/* create the driver */
	if (g_dev == NULL) {
		printf("device implement error.\n");
		goto fail;
	}
	printf("CommBridge Start.\n");
	exit(0);

	fail:
		printf("driver start failed.\n");
		exit(1);
}

/**
 * Stop the driver.
 */
int
stop(void)
{
	exit(0);
}


/****************************************************************************
 * CommBridge_main
 ****************************************************************************/
//#ifdef CONFIG_BUILD_KERNEL
//int main(int argc, FAR char *argv[])
//#else
//int commbridge_main(int argc, char *argv[])
//#endif
//{
//	if(argc >= 2){
//		/*
//		 * Start/load the driver.
//		 */
//		if (!strcmp(argv[1], "start")) {
//			start();
//			return OK;
//		}
//		/*
//		 * Stop the driver.
//		 */
//		if (!strcmp(argv[1], "stop")) {
//			stop();
//			return OK;
//		}
//		printf("unrecognized command, try 'start', 'test', 'reset' or 'info'.\n");
//		exit(1);
//	}
//
//	printf( "unrecognized format, try 'start', 'test', 'reset' or 'info'.\n");
//	exit(1);
//	return ERROR;
//}

int commbridge_main(int argc, char *argv[]){
	int ble_write_pid, ble_pid;
	int i, ret;
	int fd;
	//打印初始线程信息
	for (i = 0; i <= argc; ++i){
		printf("[BLE]:argv[%d] = %s\n",i, argv[i]);
	}
	//传入参数非法
	if (*argv[1] != '-'){
	  printf("Invalid options format: %s\n", argv[1]);
	  exit(EXIT_SUCCESS);
	}

	//启动读写线程
	if('s' == *(++argv[1])){
		//创建蓝牙read线程
		ble_pid = task_create( "ble",\
								100,\
								512,\
								start, argv);
		if(ble_pid < 0){
			int errcode = errno;
			fprintf(stderr, "ERROR: Failed to ble: %d\n",\
					errcode);
			return -errcode;
		}
		printf("[BLE]:task pid:%d\n",ble_pid);
	}
	printf("[BLE]:main task exit.\n");
	//初始线程退出
	exit(EXIT_SUCCESS);
	return EXIT_SUCCESS;
}
