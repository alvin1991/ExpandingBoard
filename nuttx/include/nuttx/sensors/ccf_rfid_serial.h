#ifndef __DRIVER_MAGFINDER_ccf_rfid_H
#define __DRIVER_MAGFINDER_ccf_rfid_H

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/
#define RFID_DEV_MAX  (1)
#define RFID_MSG_LEN  (8)
/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ccf_rfid_msg_s
{
	int step;
	int len;
	int lencnt;
	char buff[5];
	char ck;
};


struct ccf_rfid_data_s
{
	int _time_stamp;
	int ID;
	int _state;
};


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int ccf_rfid_register(void);

#endif
