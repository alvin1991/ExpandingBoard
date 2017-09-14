#ifndef __DRIVER_ORICOD_ABS_ENCODER_H
#define __DRIVER_ORICOD_ABS_ENCODER_H

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/
#define ORICOD_ABS_ENCODER_DEV_MAX  (1)
#define ORICOD_ABS_ENCODER_MSG_LEN  (16)
/****************************************************************************
 * Private Types
 ****************************************************************************/

struct oricod_abs_encoder_msg_s
{
	int step;
	int len;
	int lencnt;
	char buff[20];
	char ck;
};


struct oricod_abs_encoder_data_s
{
	int _time_stamp;
	int _addr;
	int _value;
	int _state;
};

enum
{
	ORC_X = 0,
	ORC_A,
	ORC_B,
	ORC_SEP,
	ORC_SMB,
	ORC_DATA,
	ORC_END
};


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int oricod_abs_encoder_register(void);

#endif
