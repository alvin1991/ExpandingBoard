#ifndef __DRIVER_MAGFINDER_CCFD16_H
#define __DRIVER_MAGFINDER_CCFD16_H

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/
#define MAGFINDER_DEV_MAX  (2)

/****************************************************************************
 * Private Types
 ****************************************************************************/
enum
{
	GOOD,
	HEAD,
	LEN_OK,
	DATA_OK,
	CHECK_OK,

	INVALID_STEP = -1,
	LEN_FAILED = -2,
	DATA_FAILED = -3,
	CHECK_FAILED = -4,

};

struct ccfd16_msg_s
{
	int step;
	int len;
	int lencnt;
	char buff[5];
	char ck;
};


struct ccfd16_data_s
{
	int _time_stamp;
	int _A;
	int _B;
	int _state;
};


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int ccfd16_register(void);

#endif
