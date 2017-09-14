/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/wqueue.h>
#include <time.h>
#include <nuttx/sensors/oricod_abs_encoder_485.h>

/*
 * Character Driver Methods
 * */
static int     oricod_abs_encoder_open (FAR struct file *filep);
static int     oricod_abs_encoder_close(FAR struct file *filep);
static ssize_t oricod_abs_encoder_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t oricod_abs_encoder_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int     oricod_abs_encoder_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);
/****************************************************************************
 * Private Data
 ****************************************************************************/
enum
{
	GOOD = 0,
	HEAD,
	ADDR_OK,
	DATA_OK,
	CHECK_OK,

	INVALID_STEP = -1,
	ADDR_FAILED = -2,
	DATA_FAILED = -3,
	CHECK_FAILED = -4
};

struct oricod_abs_encoder_dev_s
{
  FAR const struct oricod_abs_encoder_ops_s   *ops;
  const char *					  devpath;
  const char *				      serpath;
  int							  serfd;
  struct work_s	    			  _work;
  struct oricod_abs_encoder_msg_s 		  _msg;
  struct oricod_abs_encoder_data_s		  _data;
  bool							  _started;
};

/* file operations interface */
static const struct file_operations g_fops =
{
  oricod_abs_encoder_open,
  oricod_abs_encoder_close,
  oricod_abs_encoder_read,
  oricod_abs_encoder_write,
  NULL,
  oricod_abs_encoder_ioctl,
#ifndef CONFIG_DISABLE_POLL
  NULL,
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  NULL,
#endif
};

struct oricod_abs_encoder_ops_s
{
	CODE int (*config)(FAR struct oricod_abs_encoder_dev_s *priv);
	CODE int (*start)(FAR struct oricod_abs_encoder_dev_s *priv);
	CODE int (*stop)(FAR struct oricod_abs_encoder_dev_s *priv);
	CODE int (*cycle)(FAR struct oricod_abs_encoder_dev_s *priv);
	CODE int (*getmagval)(FAR struct oricod_abs_encoder_dev_s *priv,char *buff);
	CODE int (*decode)(FAR struct oricod_abs_encoder_dev_s *priv,char c);
	CODE int (*sendbuff)(FAR struct oricod_abs_encoder_dev_s *priv,char *buff,int len);
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int oricod_abs_encoder_config(FAR struct oricod_abs_encoder_dev_s *priv);
static int oricod_abs_encoder_start(FAR struct oricod_abs_encoder_dev_s *priv);
static int oricod_abs_encoder_stop(FAR struct oricod_abs_encoder_dev_s *priv);
static int oricod_abs_encoder_cycle(FAR struct oricod_abs_encoder_dev_s *priv);
static int oricod_abs_encoder_getmagval(FAR struct oricod_abs_encoder_dev_s *priv ,char *buff);
static int oricod_abs_encoder_decode(FAR struct oricod_abs_encoder_dev_s *priv,char c);
static int oricod_abs_encoder_sendbuff(FAR struct oricod_abs_encoder_dev_s *priv,char *buff,int len);

/****************************************************************************
 * Device operations interface
 ****************************************************************************/

static const struct oricod_abs_encoder_ops_s g_oricod_abs_encoder_ops = {
   .config = oricod_abs_encoder_config,
   .start = oricod_abs_encoder_start,
   .stop = oricod_abs_encoder_stop,
   .cycle = oricod_abs_encoder_cycle,
   .getmagval = oricod_abs_encoder_getmagval,
   .decode = oricod_abs_encoder_decode,
   .sendbuff = oricod_abs_encoder_sendbuff
};


/****************************************************************************
 * Single linked list to store instances of drivers
 ****************************************************************************/
static struct oricod_abs_encoder_dev_s g_oricod_abs_encoder_dev[ORICOD_ABS_ENCODER_DEV_MAX] = {
		{
				.ops = &g_oricod_abs_encoder_ops,
				.devpath ="/dev/oricod",
				.serpath ="/dev/ttyS5",
				.serfd = -1,
		},
};

/****************************************************************************
 * File operations functions
 ****************************************************************************/

/****************************************************************************
 * Name: oricod_abs_encoder_open
 *
 * Description:
 *   This method is called when the device is opened.
 *
 ****************************************************************************/
static int oricod_abs_encoder_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: oricod_abs_encoder_close
 *
 * Description:
 *   This method is called when the device is closed.
 *
 ****************************************************************************/
static int oricod_abs_encoder_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: oricod_abs_encoder_read
 *
 * Description:
 *   This method is called when the device is closed.
 *
 ****************************************************************************/
static ssize_t oricod_abs_encoder_read(FAR struct file *filep, FAR char *buffer,size_t buflen)
{
	  FAR struct inode        *inode = filep->f_inode;
	  FAR struct oricod_abs_encoder_dev_s *priv  = inode->i_private;
	  int ret = -1;

	  if (!buffer){
		  printf("ERROR: Buffer is null\n");
	      return -1;
	  }

	  /* cycle reading data from device */
	  if(!priv->_started){
		  ret = priv->ops->start(priv);
		  if(!(ret < 0)){
			  priv->_started = true;
			  printf("[%s] start... \n",priv->devpath);
		  }else{
			  printf("[%s]Restart failed :%s \n",priv->devpath,strerror(ret));
		  }

	  }

	  //copy data to buffer
	  memcpy((char *)buffer,(char *)(&(priv->_data)),buflen);

	return sizeof(priv->_data);
}

/****************************************************************************
 * Name: oricod_abs_encoder_write
 *
 * Description:
 *   This method is called when the device is closed.
 *
 ****************************************************************************/
static ssize_t oricod_abs_encoder_write(FAR struct file *filep, FAR const char *buffer,size_t buflen)
{
	FAR struct inode        *inode = filep->f_inode;
	FAR struct oricod_abs_encoder_dev_s *priv  = inode->i_private;

	int ret =-1;

	//check buffer address
	if (!buffer){
	  printf("ERROR: Buffer is null\n");
	  return ret;
	}

    return ret;
}

/****************************************************************************
 * Name: oricod_abs_encoder_ioctl
 *
 * Description:
 *   This method is called when the device is closed.
 *
 ****************************************************************************/
static int  oricod_abs_encoder_ioctl(FAR struct file *filep, int cmd,unsigned long arg)
{
	return 1;
}



/****************************************************************************
 * oricod_abs_encoder Operations
 ****************************************************************************/

/****************************************************************************
 * Name: oricod_abs_encoder_config
 *
 * Description:
 *   Configure the magfinder.
 ****************************************************************************/
static int oricod_abs_encoder_config(FAR struct oricod_abs_encoder_dev_s *priv)
{
  /* Sanity check */
  DEBUGASSERT(priv != NULL);

  return OK;
}

/****************************************************************************
 * Name: oricod_abs_encoder_start
 *
 * Description:
 *   This method is uesd to starting device
 *
 ****************************************************************************/
static int oricod_abs_encoder_start(FAR struct oricod_abs_encoder_dev_s *priv)
{
	int ret = -1;

	/* add worker to workqueue */
	ret = work_queue(HPWORK, &(priv->_work),(worker_t)priv->ops->cycle,priv, USEC2TICK(1000*20));
	if(ret != 0){
		printf("[%s]: Add to workqueue failed:%s\n",priv->devpath,strerror(ret));
		exit(EXIT_FAILURE);
	}
	return ret;
}

static int oricod_abs_encoder_stop(FAR struct oricod_abs_encoder_dev_s *priv)
{
	return OK;
}

/****************************************************************************
 * Name: oricod_abs_encoder_cycle
 *
 * Description:
 *   This method is uesd for workqueue
 *
 ****************************************************************************/
static int oricod_abs_encoder_cycle(FAR struct oricod_abs_encoder_dev_s *priv)
{
	int ret = -1 , nbyte = 0, i = 0;

	static char pool[ORICOD_ABS_ENCODER_MSG_LEN];

	static int abs_encoder_step;
	static int abs_encoder_num;

	/* read cmd on 485bus */
	static char val[][4] = {
			{0x44,0x30,0x31,0x0D},
			{0x44,0x30,0x32,0x0D}
	};
	switch(abs_encoder_step){
	case 0:
		/* read char from 485bus */
		nbyte = priv->ops->getmagval(priv,pool);
		if(nbyte > 0){
			for(i = 0; i < nbyte ;i++){

				/* parse message */
				ret = priv->ops->decode(priv,pool[i]);

				priv->_data._state = ret;

				/* update mag data state*/
				if(ret < 0){
					printf("[%s]:Decode error:%d \n",priv->devpath,ret);
				}
			}
		}

		if(ret != CHECK_OK ){
			abs_encoder_step = 1;
		}

		break;

	case 1:
		/* open serial port for the encoder */
		if(priv->serfd < 0){
			priv->serfd = open(priv->serpath, O_RDWR );// | O_NONBLOCK| O_NOCTTY
			if (priv->serfd < 0){
				printf("[%s] open serial port failed: %s\n",priv->devpath, strerror(ret));
				exit(EXIT_FAILURE);
			}
		}

		/*
		 *poll device on 485bus
		 */
		if(abs_encoder_num>=2){
			abs_encoder_num = 0;
		}

		/* write data from buff */
		ret = write(priv->serfd,val[abs_encoder_num++],4);
		if(ret < -1){
			printf("[%s] read serial port failed: %s\n",priv->devpath, strerror(ret));
			exit(EXIT_FAILURE);
		}

		if(ret < 0){
			printf("[ABS_ENCODER] write failed: %s\n", strerror(ret));
		}else{
			abs_encoder_step = 0;
		}
		break;

	default:
		abs_encoder_step = 0;
		abs_encoder_num = 0;
		break;
	}



	/* schedule a cycle call */
	ret = work_queue(HPWORK, &(priv->_work),(worker_t)priv->ops->cycle,priv, USEC2TICK(1000*20));//USEC2TICK(1000)
	if(ret != 0){
		printf("[%s]: Add to workqueue failed:%s\n",priv->devpath,strerror(ret));
		exit(0);
	}

	return ret;
}

/****************************************************************************
 * Name: oricod_abs_encoder_getmagval
 *
 * Description:
 *   This method is called when need open the device serial port and read a char,
 *   and save into buff.
 *
 ****************************************************************************/
static int oricod_abs_encoder_getmagval(FAR struct oricod_abs_encoder_dev_s *priv ,char *buff)
{
	int ret = 0;

	/* open serial port for the magfinder front */
	if(priv->serfd < 0){
		priv->serfd = open(priv->serpath, O_RDWR | O_NONBLOCK | O_NOCTTY);//| O_NONBLOCK | O_NOCTTY
		if (priv->serfd < 0){
			printf("[%s] open serial port failed: %s\n",priv->devpath, strerror(ret));
			exit(EXIT_FAILURE);
		}
	}

	/* read data from serial */
	ret = read(priv->serfd ,buff,ORICOD_ABS_ENCODER_MSG_LEN);
	if(ret < -1){
		printf("[%s] read serial port failed: %s\n",priv->devpath, strerror(ret));
		exit(EXIT_FAILURE);
	}

	return ret;
}

/****************************************************************************
 * Name: oricod_abs_encoder_sendbuff
 *
 * Description:
 *   This method is called when need open the device serial port and send chars
 *   from buff.
 *
 ****************************************************************************/
static int oricod_abs_encoder_sendbuff(FAR struct oricod_abs_encoder_dev_s *priv,char *buff,int len)
{
	int ret = 0;
	return ret;
}

/****************************************************************************
 * Name: oricod_abs_encoder_decode
 *
 * Description:
 *   This method is called when the massage need to be decode.
 *
 *Return:
 *see enum
 ****************************************************************************/
static int oricod_abs_encoder_decode(FAR struct oricod_abs_encoder_dev_s *priv ,char c)
{
	int ret = -1;

	switch(priv->_msg.step){

	//head 58
	case ORC_X:
		if(0x58 == c){

			//earse msg before
			memset(&(priv->_msg),0,sizeof(priv->_msg));

			//set message step
			priv->_msg.step = ORC_DATA;

			//clean count and recive first char
			priv->_msg.lencnt = 0;
			priv->_msg.buff[priv->_msg.lencnt] = c;

		}else{
			priv->_msg.step = ORC_X;
		}
		ret = HEAD;
		break;

	case ORC_DATA:
		//recive data
		priv->_msg.buff[++priv->_msg.lencnt] = c;

		//1.overflow check
		if(priv->_msg.lencnt > (ORICOD_ABS_ENCODER_MSG_LEN - 1)){

			//return error
			ret = DATA_FAILED;

			//reset message step
			priv->_msg.step = ORC_X;

			break;
		}

		//finished char
		if(0x0D == c){

			//parse message
			//2.address check
			if(priv->_msg.buff[1] - 0x30 > 9 || priv->_msg.buff[2] - 0x30 >9 ){

				//return error
				ret = ADDR_FAILED;

				//reset message step
				priv->_msg.step = ORC_X;

				break;
			}

			//3.lenth check
			if(priv->_msg.lencnt != (ORICOD_ABS_ENCODER_MSG_LEN - 1)){

				//return error
				ret = DATA_FAILED;

				//reset message step
				priv->_msg.step = ORC_X;

				break;
			}

//			//oricod protocol has 10bits for data and start at NO.5;
			int val[10],i;
			for(i = 5; i < 15; i++){
				val[i-5] = priv->_msg.buff[i]-0x30;
			}
			priv->_data._addr = (priv->_msg.buff[1]-0x30)*10 + priv->_msg.buff[2]-0x30;
			priv->_data._value = 0;
			priv->_data._value += val[0]*1000000000;
			priv->_data._value += val[1]*100000000;
			priv->_data._value += val[2]*10000000;
			priv->_data._value += val[3]*1000000;
			priv->_data._value += val[4]*100000;
			priv->_data._value += val[5]*10000;
			priv->_data._value += val[6]*1000;
			priv->_data._value += val[7]*100;
			priv->_data._value += val[8]*10;
			priv->_data._value += val[9]*1;

			struct timespec timer;
			clock_gettime(CLOCK_MONOTONIC, &timer);
			priv->_data._time_stamp =timer.tv_sec;

			//return data correct
			ret = DATA_OK;

			//reset message step
			priv->_msg.step = ORC_X;

		}

		ret = CHECK_OK;

	break;

	default:
		ret = INVALID_STEP;
		memset(&(priv->_msg),0,sizeof(priv->_msg));
		break;
	}

	return ret;
}




/****************************************************************************
 * Name: oricod_abs_encoder_register
 *
 * Description:
 *   Register the oricod_abs_encoder magfinder,  character
 *   device as 'devpath'.
 *
 * Input Parameters:NULL
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int oricod_abs_encoder_register(void)
{
  int ret = -1 ,cnt = 0;

  /* Erase device memery */
   for (cnt  = 0 ; cnt < ORICOD_ABS_ENCODER_DEV_MAX ;cnt++){
		/* erase msg buffer */
		memset(&(g_oricod_abs_encoder_dev[cnt]._msg),0,sizeof(g_oricod_abs_encoder_dev[cnt]._msg));
		/* erase data buffer */
		memset(&(g_oricod_abs_encoder_dev[cnt]._data),0,sizeof(g_oricod_abs_encoder_dev[cnt]._data));
   }

  /* Register the character driver */
  for (cnt = 0 ; cnt < ORICOD_ABS_ENCODER_DEV_MAX ; cnt++){
	  ret = register_driver(g_oricod_abs_encoder_dev[cnt].devpath, &g_fops, 0666, &(g_oricod_abs_encoder_dev[cnt]));
	  if (ret < 0){
		  syslog(LOG_ERR,"[%s]: Failed to register driver: %d\n",g_oricod_abs_encoder_dev[cnt].devpath, ret);
		  return ret;
	  }
  }
  return ret;
}
