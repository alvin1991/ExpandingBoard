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
#include <nuttx/sensors/ccfd16_serial.h>


/*
 * Character Driver Methods
 * */
static int     ccfd16_open (FAR struct file *filep);
static int     ccfd16_close(FAR struct file *filep);
static ssize_t ccfd16_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t ccfd16_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int     ccfd16_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);
/****************************************************************************
 * Private Data
 ****************************************************************************/
struct ccfd16_dev_s
{
  FAR const struct ccfd16_ops_s   *ops;
  const char *					  devpath;
  const char *				      serpath;
  int							  serfd;
  struct work_s	    			  _work;
  struct ccfd16_msg_s 			  _msg;
  struct ccfd16_data_s			  _data;
  bool							  _started;
};

/* file operations interface */
static const struct file_operations g_fops =
{
  ccfd16_open,
  ccfd16_close,
  ccfd16_read,
  ccfd16_write,
  NULL,
  ccfd16_ioctl,
#ifndef CONFIG_DISABLE_POLL
  NULL,
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  NULL,
#endif
};

struct ccfd16_ops_s
{
	CODE int (*config)(FAR struct ccfd16_dev_s *priv);
	CODE int (*start)(FAR struct ccfd16_dev_s *priv);
	CODE int (*stop)(FAR struct ccfd16_dev_s *priv);
	CODE int (*cycle)(FAR struct ccfd16_dev_s *priv);
	CODE int (*getmagval)(FAR struct ccfd16_dev_s *priv,char *buff);
	CODE int (*decode)(FAR struct ccfd16_dev_s *priv,char c);
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ccfd16_config(FAR struct ccfd16_dev_s *priv);
static int ccfd16_start(FAR struct ccfd16_dev_s *priv);
static int ccfd16_stop(FAR struct ccfd16_dev_s *priv);
static int ccfd16_cycle(FAR struct ccfd16_dev_s *priv);
static int ccfd16_getmagval(FAR struct ccfd16_dev_s *priv ,char *buff);
static int ccfd16_decode(FAR struct ccfd16_dev_s *priv,char c);


/****************************************************************************
 * Device operations interface
 ****************************************************************************/

static const struct ccfd16_ops_s g_ccfd16_ops = {
   .config = ccfd16_config,
   .start = ccfd16_start,
   .stop = ccfd16_stop,
   .cycle = ccfd16_cycle,
   .getmagval = ccfd16_getmagval,
   .decode = ccfd16_decode,
};


/****************************************************************************
 * Single linked list to store instances of drivers
 ****************************************************************************/
static struct ccfd16_dev_s g_ccfd16_dev[MAGFINDER_DEV_MAX] = {
		{
				.ops = &g_ccfd16_ops,
				.devpath ="/dev/magf",
				.serpath ="/dev/ttyS2",
				.serfd = -1,
		},
		{
				.ops = &g_ccfd16_ops,
				.devpath ="/dev/magb",
				.serpath ="/dev/ttyS3",
				.serfd = -1,
		}
};





/****************************************************************************
 * File operations functions
 ****************************************************************************/

/****************************************************************************
 * Name: ccfd16_open
 *
 * Description:
 *   This method is called when the device is opened.
 *
 ****************************************************************************/
static int ccfd16_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: ccfd16_close
 *
 * Description:
 *   This method is called when the device is closed.
 *
 ****************************************************************************/
static int ccfd16_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: ccfd16_read
 *
 * Description:
 *   This method is called when the device is closed.
 *
 ****************************************************************************/
static ssize_t ccfd16_read(FAR struct file *filep, FAR char *buffer,size_t buflen)
{
	  FAR struct inode        *inode = filep->f_inode;
	  FAR struct ccfd16_dev_s *priv  = inode->i_private;
	  int ret = -1;

	  if (!buffer){
	      snerr("ERROR: Buffer is null\n");
	      return -1;
	  }

	  /* cycle reading data from device */
	  if(!priv->_started){
		  ret = priv->ops->start(priv);
		  if(!(ret < 0)){
			  priv->_started = true;
			  printf("start... \n");
		  }else{
			  printf("[%s]Restart failed :%s \n",priv->devpath,strerror(ret));
		  }

	  }

	  //copy data to buffer
	  memcpy((char *)buffer,(char *)(&(priv->_data)),buflen);

	return sizeof(priv->_data);
}

/****************************************************************************
 * Name: ccfd16_write
 *
 * Description:
 *   This method is called when the device is closed.
 *
 ****************************************************************************/
static ssize_t ccfd16_write(FAR struct file *filep, FAR const char *buffer,size_t buflen)
{
	  FAR struct inode        *inode = filep->f_inode;
	  FAR struct ccfd16_dev_s *priv  = inode->i_private;

	  if (!buffer){
	      snerr("ERROR: Buffer is null\n");
	      return -1;
	  }

	  /* Get the pressure compensated */
	  //ccfd16_start(priv);

	  return OK;
}

/****************************************************************************
 * Name: ccfd16_ioctl
 *
 * Description:
 *   This method is called when the device is closed.
 *
 ****************************************************************************/
static int  ccfd16_ioctl(FAR struct file *filep, int cmd,unsigned long arg)
{
	return 1;
}



/****************************************************************************
 * ccfd16 Operations
 ****************************************************************************/

/****************************************************************************
 * Name: ccfd16_config
 *
 * Description:
 *   Configure the magfinder.
 ****************************************************************************/
static int ccfd16_config(FAR struct ccfd16_dev_s *priv)
{
  /* Sanity check */
  DEBUGASSERT(priv != NULL);

  return OK;
}

/****************************************************************************
 * Name: ccfd16_start
 *
 * Description:
 *   This method is uesd to starting device
 *
 ****************************************************************************/
static int ccfd16_start(FAR struct ccfd16_dev_s *priv)
{
	int ret = -1;

	/* add worker to workqueue */
	ret = work_queue(HPWORK, &(priv->_work),(worker_t)priv->ops->cycle,priv, 1);
	if(ret != 0){
		printf("[%s]: Add to workqueue failed:%s\n",priv->devpath,strerror(ret));
		exit(EXIT_FAILURE);
	}
	return ret;
}

static int ccfd16_stop(FAR struct ccfd16_dev_s *priv)
{
	return OK;
}

/****************************************************************************
 * Name: ccfd16_cycle
 *
 * Description:
 *   This method is uesd for workqueue
 *
 ****************************************************************************/
static int ccfd16_cycle(FAR struct ccfd16_dev_s *priv)
{
	int ret = -1 , nbyte = 0, i = 0;

	static char pool[MAGFINDER_MSG_LEN];

	/* read char from serial */
	nbyte = priv->ops->getmagval(priv,pool);
	if(nbyte > 0){
		for(i = 0; i < nbyte ;i++){

			ret = priv->ops->decode(priv,pool[i]);
			/* update mag data state*/
			if(ret<0){
				priv->_data._state = ret;
				printf("[%s]:Decode error:%d \n",priv->devpath,ret);
			}else if( ret== CHECK_OK){
				priv->_data._state = GOOD;
			}
		}
	}

	/* schedule a cycle call */
	ret = work_queue(HPWORK, &(priv->_work),(worker_t)priv->ops->cycle,priv, USEC2TICK(1000*10));//USEC2TICK(1000)
	if(ret != 0){
		printf("[%s]: Add to workqueue failed:%s\n",priv->devpath,strerror(ret));
		exit(0);
	}

	return ret;
}

/****************************************************************************
 * Name: ccfd16_getmagval
 *
 * Description:
 *   This method is called when need open the device serial port and read a char,
 *   and save into buff.
 *
 ****************************************************************************/
static int ccfd16_getmagval(FAR struct ccfd16_dev_s *priv ,char *buff)

{
	int ret = 0;

	/* open serial port for the magfinder front */
	if(priv->serfd == -1){
		priv->serfd = open(priv->serpath, O_RDWR | O_NONBLOCK | O_NOCTTY);
		if (priv->serfd < 0){
			printf("[%s] open serial port failed: %s\n",priv->devpath, strerror(ret));
			exit(EXIT_FAILURE);
		}
	}

	/* read data from serial */
	ret = read(priv->serfd,buff,MAGFINDER_MSG_LEN);
	if(ret < -1)
	{
		printf("[%s] read serial port failed: %s\n",priv->devpath, strerror(ret));
		exit(EXIT_FAILURE);
	}

	return ret;
}

/****************************************************************************
 * Name: ccfd16_decode
 *
 * Description:
 *   This method is called when the massage need to be decode.
 *
 *Return:
 *see enum
 ****************************************************************************/
static int ccfd16_decode(FAR struct ccfd16_dev_s *priv ,char c)
{
	int ret = -1;

	switch(priv->_msg.step){
	//head 68
	case 0:
		if(0x68 == c){
			priv->_msg.step = 1;
		}else{
			priv->_msg.step = 0;
		}
		ret = HEAD;
		break;

	//len
	case 1:
		// out of mag lenth
		if(0x04 != c){
			priv->_msg.step = 0;
			ret = LEN_FAILED;
		}else{
			//earse msg before
			memset(&(priv->_msg),0,sizeof(priv->_msg));
			priv->_msg.len = c;
			priv->_msg.step = 2;
			ret = LEN_OK;
		}
		break;

    //payload
	case 2:
		//receive a char
		priv->_msg.buff[priv->_msg.lencnt++] = c;
		ret = DATA_OK;
		//receive finished
		if(priv->_msg.lencnt >= priv->_msg.len){

			//calc checksum
			priv->_msg.ck = (char)(priv->_msg.len + priv->_msg.buff[0] + priv->_msg.buff[1] + priv->_msg.buff[2]);

			// check checksum
			if(priv->_msg.ck == priv->_msg.buff[3]){
				struct timespec timer;
				clock_gettime(CLOCK_MONOTONIC, &timer);
				priv->_data._A = priv->_msg.buff[1];
				priv->_data._B = priv->_msg.buff[2];
				priv->_data._time_stamp =timer.tv_sec;
				ret = CHECK_OK;
			}else{
				printf("[%x]:%x %x %x %x %x\n",priv->_msg.ck,priv->_msg.len , priv->_msg.buff[0], priv->_msg.buff[1],priv->_msg.buff[2],priv->_msg.buff[3]);
				ret = CHECK_FAILED;
			}
			//erase buff
			memset(&(priv->_msg),0,sizeof(priv->_msg));
		}
		break;

	default:
		ret = INVALID_STEP;
		memset(&(priv->_msg),0,sizeof(priv->_msg));
		break;
	}

	return ret;
}




/****************************************************************************
 * Name: ccfd16_register
 *
 * Description:
 *   Register the ccfd16 magfinder,  character
 *   device as 'devpath'.
 *
 * Input Parameters:NULL
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ccfd16_register(void)
{
  int ret = -1 ,cnt = 0;

  /* Configure the device */
//  for (cnt  = 0 ; cnt < MAGFINDER_DEV_MAX ;cnt++){
//	  ret = g_ccfd16_dev[cnt].ops->config(&g_ccfd16_dev[cnt]);
//	  if (ret < 0){
//		  syslog(LOG_ERR,"[%s]: Failed to configure device: %d\n",g_ccfd16_dev[cnt].devpath, ret);
//		  return ret;
//	  }
//  }
  /* Erase device memery */
   for (cnt  = 0 ; cnt < MAGFINDER_DEV_MAX ;cnt++){
		/* erase msg buffer */
		memset(&(g_ccfd16_dev[cnt]._msg),0,sizeof(g_ccfd16_dev[cnt]._msg));
		/* erase data buffer */
		memset(&(g_ccfd16_dev[cnt]._data),0,sizeof(g_ccfd16_dev[cnt]._data));
   }

  /* Register the character driver */
  for (cnt = 0 ; cnt < MAGFINDER_DEV_MAX ; cnt++){
	  ret = register_driver(g_ccfd16_dev[cnt].devpath, &g_fops, 0666, &(g_ccfd16_dev[cnt]));
	  if (ret < 0){
		  syslog(LOG_ERR,"[%s]: Failed to register driver: %d\n",g_ccfd16_dev[cnt].devpath, ret);
		  return ret;
	  }
  }
  return ret;
}
