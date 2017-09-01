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
#include <nuttx/input/buttons.h>
#include <arch/board/board.h>
#include <nuttx/sensors/qencoder.h>

//mavlink
#include "mavlink/minimal/mavlink.h"


/****************************************************************************
 * Public Parameters
 ****************************************************************************/
enum
{
	COMM_SERIAL = 0,
	MAG_FINDER_FRONT,
	MAG_FINDER_BACK,
	RFID_READER,
	ULTRA_SONIC,
	QENCODER_1,
	QENCODER_2,
	MAX_SENSOR_NUMS
};

typedef struct CommBridge
{
	bool				_class_instance;
	bool			    _should_exit;
	int					_pid;
	int					_fd[MAX_SENSOR_NUMS];
	void (*cycle)(void);
}CommBridge_t;

static CommBridge_t	*g_dev;


/****************************************************************************
 * Public Functions
 ****************************************************************************/

static void
cycle(void)
{

	int byte_send,len,ret = -1;
	struct ccfd16_data_s ccfd16f;
	struct ccfd16_data_s ccfd16b;
	struct ccf_rfid_data_s ccf_rfid;
	uint8_t i;
	btn_buttonset_t sample = 0, oldsample = 0;
	int qencoder[4];

	uint8_t buf[50];
	/*
	 * Mavlink Package
	 */
	mavlink_message_t  msgpacket;
	memset(&msgpacket,0,sizeof(msgpacket));


	/*
	 * Mavlink extboard
	 */
	mavlink_ext_board_t ext_board;
	memset(&ext_board,0,sizeof(ext_board));


	/* open serial port for communication with rover */
	g_dev->_fd[COMM_SERIAL] = open("/dev/ttyS1",O_RDWR);
	if (g_dev->_fd[COMM_SERIAL] < 0){
		printf("[comm] open failed: %s\n", strerror(ret));
		exit(EXIT_FAILURE);
	}

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

	/* open IO for ultra sonic sensor */
	g_dev->_fd[ULTRA_SONIC] = open("/dev/buttons", O_RDONLY|O_NONBLOCK);
	if (g_dev->_fd[ULTRA_SONIC] < 0){
		printf("[buttons] open failed:%x %s\n",g_dev->_fd[ULTRA_SONIC], strerror(ret));
		exit(EXIT_FAILURE);
	}

	/* open qencoder for rotary encoder */
	g_dev->_fd[QENCODER_1] = open("/dev/qencoder1", O_RDONLY|O_NONBLOCK);
	if (g_dev->_fd[QENCODER_1] < 0){
		printf("[qencoder1] open failed:%x %s\n",g_dev->_fd[QENCODER_1], strerror(ret));
		exit(EXIT_FAILURE);
	}
	g_dev->_fd[QENCODER_2] = open("/dev/qencoder2", O_RDONLY|O_NONBLOCK);
	if (g_dev->_fd[QENCODER_2] < 0){
		printf("[qencoder2] open failed:%x %s\n",g_dev->_fd[QENCODER_2], strerror(ret));
		exit(EXIT_FAILURE);
	}

	while(!g_dev->_should_exit){

		/* read data from sensors */
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

	    ret = ioctl(g_dev->_fd[QENCODER_1],QEIOC_POSITION,&qencoder[0]);
		if(ret < 0){
			printf("[qencoder1] read failed: %s\n", strerror(ret));
		}		printf("qencoder1:%d\n",qencoder[0]);

		ret = read(g_dev->_fd[ULTRA_SONIC],&sample,sizeof(sample));
		if(ret < 0){
			printf("[buttons] read failed: %s\n", strerror(ret));
		}

		for (i = 0; i < NUM_BUTTONS; i++){
		   if ((sample & (1 << i)) && !(oldsample & (1 << i))){
			   ext_board.ultrasonic[i] = 0;
			   printf("%d was pressed:%d\n", i,qencoder[0]);
			}

		   if (!(sample & (1 << i)) && (oldsample & (1 << i))){
			   ext_board.ultrasonic[i] = 1;
			  printf("%d was released\n", i);
		 	}
		 }

	    oldsample = sample;


		//fill the mavlink package
		ext_board.mag_f = (ccfd16f._A << 8) + ccfd16f._B;//0x11111111;
		ext_board.mag_b = (ccfd16b._A << 8) + ccfd16b._B;//0x22222222;

		ext_board.rfid  = ccf_rfid.ID;//0x33333333;



		//send mavlink package via serial
		mavlink_msg_ext_board_pack(0x01, 0x02, &msgpacket,
				ext_board.mag_f,\
				ext_board.mag_b,\
				ext_board.rfid ,\
				ext_board.ultrasonic,\
				ext_board.Encoder);

		len = mavlink_msg_to_send_buffer(buf, &msgpacket);

		byte_send = write(g_dev->_fd[COMM_SERIAL],&buf,len);
		if(byte_send < 0){
			printf("[comm]:write error: %s\n",strerror(byte_send));
		}

		usleep(1000*100);
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
	return true;

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
