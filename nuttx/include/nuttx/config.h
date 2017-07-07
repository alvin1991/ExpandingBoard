/* config.h -- Autogenerated! Do not edit. */

#ifndef __INCLUDE_NUTTX_CONFIG_H
#define __INCLUDE_NUTTX_CONFIG_H

/* Architecture-specific options *************************/

#define CONFIG_HOST_LINUX 1
#define CONFIG_APPS_DIR "../apps"
#define CONFIG_BUILD_FLAT 1
#define CONFIG_INTELHEX_BINARY 1
#define CONFIG_RAW_BINARY 1
#define CONFIG_DEBUG_ALERT 1
#define CONFIG_ARCH_HAVE_STACKCHECK 1
#define CONFIG_ARCH_HAVE_HEAPCHECK 1
#define CONFIG_DEBUG_SYMBOLS 1
#define CONFIG_ARCH_HAVE_CUSTOMOPT 1
#define CONFIG_DEBUG_NOOPT 1
#define CONFIG_ARCH_ARM 1
#define CONFIG_ARCH "arm"
#define CONFIG_ARCH_CHIP_STM32 1
#define CONFIG_ARCH_CORTEXM4 1
#define CONFIG_ARCH_FAMILY "armv7-m"
#define CONFIG_ARCH_CHIP "stm32"
#define CONFIG_ARM_TOOLCHAIN_GNU 1
#define CONFIG_ARCH_HAVE_CMNVECTOR 1
#define CONFIG_ARCH_HAVE_FPU 1
#define CONFIG_ARM_HAVE_MPU_UNIFIED 1
#define CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIL 1
#define CONFIG_ARMV7M_HAVE_STACKCHECK 1
#define CONFIG_ARCH_CHIP_STM32F429Z 1
#define CONFIG_STM32_FLASH_CONFIG_DEFAULT 1
#define CONFIG_STM32_STM32F40XX 1
#define CONFIG_STM32_STM32F429 1
#define CONFIG_STM32_HAVE_CCM 1
#define CONFIG_STM32_HAVE_OTGFS 1
#define CONFIG_STM32_HAVE_FSMC 1
#define CONFIG_STM32_HAVE_LTDC 1
#define CONFIG_STM32_HAVE_USART3 1
#define CONFIG_STM32_HAVE_UART4 1
#define CONFIG_STM32_HAVE_UART5 1
#define CONFIG_STM32_HAVE_USART6 1
#define CONFIG_STM32_HAVE_UART7 1
#define CONFIG_STM32_HAVE_UART8 1
#define CONFIG_STM32_HAVE_TIM1 1
#define CONFIG_STM32_HAVE_TIM3 1
#define CONFIG_STM32_HAVE_TIM4 1
#define CONFIG_STM32_HAVE_TIM6 1
#define CONFIG_STM32_HAVE_TIM7 1
#define CONFIG_STM32_HAVE_TIM8 1
#define CONFIG_STM32_HAVE_TIM9 1
#define CONFIG_STM32_HAVE_TIM10 1
#define CONFIG_STM32_HAVE_TIM11 1
#define CONFIG_STM32_HAVE_TIM12 1
#define CONFIG_STM32_HAVE_TIM13 1
#define CONFIG_STM32_HAVE_TIM14 1
#define CONFIG_STM32_HAVE_ADC2 1
#define CONFIG_STM32_HAVE_ADC3 1
#define CONFIG_STM32_HAVE_CAN1 1
#define CONFIG_STM32_HAVE_CAN2 1
#define CONFIG_STM32_HAVE_DAC1 1
#define CONFIG_STM32_HAVE_DAC2 1
#define CONFIG_STM32_HAVE_RNG 1
#define CONFIG_STM32_HAVE_ETHMAC 1
#define CONFIG_STM32_HAVE_I2C2 1
#define CONFIG_STM32_HAVE_I2C3 1
#define CONFIG_STM32_HAVE_SPI2 1
#define CONFIG_STM32_HAVE_SPI3 1
#define CONFIG_STM32_HAVE_SPI4 1
#define CONFIG_STM32_HAVE_SPI5 1
#define CONFIG_STM32_HAVE_SPI6 1
#define CONFIG_STM32_FSMC 1
#define CONFIG_STM32_PWR 1
#define CONFIG_STM32_SYSCFG 1
#define CONFIG_STM32_USART1 1
#define CONFIG_STM32_USART2 1
#define CONFIG_STM32_USART3 1
#define CONFIG_STM32_UART4 1
#define CONFIG_STM32_JTAG_SW_ENABLE 1
#define CONFIG_STM32_DISABLE_IDLE_SLEEP_DURING_DEBUG 1
#define CONFIG_STM32_FSMC_SRAM 1
#define CONFIG_STM32_USART 1
#define CONFIG_STM32_SERIALDRIVER 1
#define CONFIG_STM32_USART1_SERIALDRIVER 1
#define CONFIG_STM32_USART2_SERIALDRIVER 1
#define CONFIG_STM32_USART3_SERIALDRIVER 1
#define CONFIG_STM32_UART4_SERIALDRIVER 1
#define CONFIG_ARCH_HAVE_IRQPRIO 1
#define CONFIG_ARCH_HAVE_VFORK 1
#define CONFIG_ARCH_HAVE_MPU 1
#define CONFIG_ARCH_HAVE_RESET 1
#define CONFIG_ARCH_STACKDUMP 1
#define CONFIG_ARCH_HAVE_RAMVECTORS 1
#define CONFIG_BOARD_LOOPSPERMSEC 16717
#define CONFIG_ARCH_HAVE_INTERRUPTSTACK 1
#define CONFIG_ARCH_INTERRUPTSTACK 0
#define CONFIG_ARCH_HAVE_HIPRI_INTERRUPT 1
#define CONFIG_BOOT_RUNFROMFLASH 1
#define CONFIG_RAM_START 0x20000000
#define CONFIG_RAM_SIZE 114688
#define CONFIG_ARCH_BOARD_STM32F429I_DISCO 1
#define CONFIG_ARCH_BOARD "stm32f429i-disco"
#define CONFIG_ARCH_HAVE_LEDS 1
#define CONFIG_ARCH_LEDS 1
#define CONFIG_ARCH_HAVE_BUTTONS 1
#define CONFIG_ARCH_BUTTONS 1
#define CONFIG_ARCH_HAVE_IRQBUTTONS 1
#define CONFIG_DISABLE_OS_API 1
#define CONFIG_ARCH_HAVE_TICKLESS 1
#define CONFIG_USEC_PER_TICK 10000
#define CONFIG_CLOCK_MONOTONIC 1
#define CONFIG_ARCH_HAVE_TIMEKEEPING 1
#define CONFIG_START_YEAR 2011
#define CONFIG_START_MONTH 12
#define CONFIG_START_DAY 6
#define CONFIG_MAX_WDOGPARMS 2
#define CONFIG_PREALLOC_WDOGS 4
#define CONFIG_WDOG_INTRESERVE 0
#define CONFIG_PREALLOC_TIMERS 4
#define CONFIG_INIT_ENTRYPOINT 1
#define CONFIG_USER_ENTRYPOINT nsh_main
#define CONFIG_RR_INTERVAL 200
#define CONFIG_TASK_NAME_SIZE 16
#define CONFIG_MAX_TASKS 16
#define CONFIG_SCHED_WAITPID 1
#define CONFIG_NPTHREAD_KEYS 4
#define CONFIG_DEV_CONSOLE 1
#define CONFIG_SDCLONE_DISABLE 1
#define CONFIG_NFILE_DESCRIPTORS 8
#define CONFIG_NFILE_STREAMS 8
#define CONFIG_NAME_MAX 32
#define CONFIG_BOARD_INITIALIZE 1
#define CONFIG_SIG_SIGUSR1 1
#define CONFIG_SIG_SIGUSR2 2
#define CONFIG_SIG_SIGALARM 3
#define CONFIG_SIG_SIGCONDTIMEDOUT 16
#define CONFIG_SIG_SIGWORK 17
#define CONFIG_PREALLOC_MQ_MSGS 4
#define CONFIG_MQ_MAXMSGSIZE 32
#define CONFIG_SCHED_WORKQUEUE 1
#define CONFIG_SCHED_HPWORK 1
#define CONFIG_SCHED_HPWORKPRIORITY 224
#define CONFIG_SCHED_HPWORKPERIOD 100000
#define CONFIG_SCHED_HPWORKSTACKSIZE 2048
#define CONFIG_SCHED_LPWORK 1
#define CONFIG_SCHED_LPNTHREADS 1
#define CONFIG_SCHED_LPWORKPRIORITY 50
#define CONFIG_SCHED_LPWORKPERIOD 50000
#define CONFIG_SCHED_LPWORKSTACKSIZE 2048
#define CONFIG_IDLETHREAD_STACKSIZE 1024
#define CONFIG_USERMAIN_STACKSIZE 2048
#define CONFIG_PTHREAD_STACK_MIN 256
#define CONFIG_PTHREAD_STACK_DEFAULT 2048
#define CONFIG_DISABLE_POLL 1
#define CONFIG_DEV_NULL 1
#define CONFIG_ARCH_HAVE_I2CRESET 1
#define CONFIG_ARCH_HAVE_SPI_BITORDER 1
#define CONFIG_SPI 1
#define CONFIG_SPI_EXCHANGE 1
#define CONFIG_LCD 1
#define CONFIG_LCD_MAXCONTRAST 63
#define CONFIG_LCD_MAXPOWER 1
#define CONFIG_LCD_LANDSCAPE 1
#define CONFIG_LCD_ILI9341 1
#define CONFIG_LCD_ILI9341_NINTERFACES 1
#define CONFIG_LCD_ILI9341_IFACE0 1
#define CONFIG_LCD_ILI9341_IFACE0_LANDSCAPE 1
#define CONFIG_LCD_ILI9341_IFACE0_RGB565 1
#define CONFIG_USERLED 1
#define CONFIG_SENSORS 1
#define CONFIG_CCF_D16 1
#define CONFIG_MS58XX_VDD 30
#define CONFIG_SERIAL 1
#define CONFIG_SERIAL_CONSOLE 1
#define CONFIG_UART4_SERIALDRIVER 1
#define CONFIG_USART1_SERIALDRIVER 1
#define CONFIG_USART2_SERIALDRIVER 1
#define CONFIG_USART3_SERIALDRIVER 1
#define CONFIG_MCU_SERIAL 1
#define CONFIG_STANDARD_SERIAL 1
#define CONFIG_ARCH_HAVE_SERIAL_TERMIOS 1
#define CONFIG_USART1_SERIAL_CONSOLE 1
#define CONFIG_USART1_RXBUFSIZE 256
#define CONFIG_USART1_TXBUFSIZE 256
#define CONFIG_USART1_BAUD 115200
#define CONFIG_USART1_BITS 8
#define CONFIG_USART1_PARITY 0
#define CONFIG_USART1_2STOP 0
#define CONFIG_USART2_RXBUFSIZE 256
#define CONFIG_USART2_TXBUFSIZE 256
#define CONFIG_USART2_BAUD 115200
#define CONFIG_USART2_BITS 8
#define CONFIG_USART2_PARITY 0
#define CONFIG_USART2_2STOP 1
#define CONFIG_USART3_RXBUFSIZE 256
#define CONFIG_USART3_TXBUFSIZE 256
#define CONFIG_USART3_BAUD 9600
#define CONFIG_USART3_BITS 8
#define CONFIG_USART3_PARITY 0
#define CONFIG_USART3_2STOP 1
#define CONFIG_UART4_RXBUFSIZE 256
#define CONFIG_UART4_TXBUFSIZE 256
#define CONFIG_UART4_BAUD 9600
#define CONFIG_UART4_BITS 8
#define CONFIG_UART4_PARITY 0
#define CONFIG_UART4_2STOP 1
#define CONFIG_SYSLOG_SERIAL_CONSOLE 1
#define CONFIG_SYSLOG_CONSOLE 1
#define CONFIG_FS_MQUEUE_MPATH "/var/mqueue"
#define CONFIG_MM_REGIONS 3
#define CONFIG_ARCH_HAVE_HEAP2 1
#define CONFIG_HEAP2_BASE 0xD0000000
#define CONFIG_HEAP2_SIZE 8388608
#define CONFIG_BUILTIN 1
#define CONFIG_STDIO_BUFFER_SIZE 64
#define CONFIG_STDIO_LINEBUFFER 1
#define CONFIG_NUNGET_CHARS 2
#define CONFIG_LIBC_LONG_LONG 1
#define CONFIG_EOL_IS_EITHER_CRLF 1
#define CONFIG_ARCH_LOWPUTC 1
#define CONFIG_LIB_RAND_ORDER 1
#define CONFIG_LIB_HOMEDIR "/"
#define CONFIG_POSIX_SPAWN_PROXY_STACKSIZE 1024
#define CONFIG_TASK_SPAWN_DEFAULT_STACKSIZE 2048
#define CONFIG_LIBC_STRERROR 1
#define CONFIG_LIBC_STRERROR_SHORT 1
#define CONFIG_ARCH_HAVE_TLS 1
#define CONFIG_LIB_SENDFILE_BUFSIZE 512
#define CONFIG_HAVE_CXX 1
#define CONFIG_HAVE_CXXINITIALIZE 1
#define CONFIG_BUILTIN_PROXY_STACKSIZE 1024
#define CONFIG_EXAMPLES_COMMBRIDGE 1
#define CONFIG_EXAMPLES_COMMBRIDGE_PRIORITY 100
#define CONFIG_EXAMPLES_COMMBRIDGE_STACKSIZE 2048
#define CONFIG_EXAMPLES_NSH 1
#define CONFIG_EXAMPLES_NSH_CXXINITIALIZE 1
#define CONFIG_NSH_LIBRARY 1
#define CONFIG_NSH_READLINE 1
#define CONFIG_NSH_LINELEN 64
#define CONFIG_NSH_CMDPARMS 1
#define CONFIG_NSH_MAXARGUMENTS 6
#define CONFIG_NSH_ARGCAT 1
#define CONFIG_NSH_NESTDEPTH 3
#define CONFIG_NSH_BUILTIN_APPS 1
#define CONFIG_NSH_DISABLE_DATE 1
#define CONFIG_NSH_DISABLE_IFUPDOWN 1
#define CONFIG_NSH_DISABLE_LOSMART 1
#define CONFIG_NSH_DISABLE_PRINTF 1
#define CONFIG_NSH_MMCSDMINOR 0
#define CONFIG_NSH_CMDOPT_DF_H 1
#define CONFIG_NSH_CODECS_BUFSIZE 128
#define CONFIG_NSH_CMDOPT_HEXDUMP 1
#define CONFIG_NSH_FILEIOSIZE 512
#define CONFIG_NSH_CONSOLE 1
#define CONFIG_READLINE_HAVE_EXTMATCH 1
#define CONFIG_SYSTEM_READLINE 1
#define CONFIG_READLINE_ECHO 1

/* Sanity Checks *****************************************/

/* If this is an NXFLAT, external build, then make sure that
 * NXFLAT support is enabled in the base code.
 */

#if defined(__NXFLAT__) && !defined(CONFIG_NXFLAT)
# error "NXFLAT support not enabled in this configuration"
#endif

/* NXFLAT requires PIC support in the TCBs. */

#if defined(CONFIG_NXFLAT)
#  undef  CONFIG_PIC
#  define CONFIG_PIC 1
#endif

/* Binary format support is disabled if no binary formats are
 * configured (at present, NXFLAT is the only supported binary.
 * format).
 */

#if !defined(CONFIG_NXFLAT) && !defined(CONFIG_ELF) && !defined(CONFIG_BUILTIN)
#  undef CONFIG_BINFMT_DISABLE
#  define CONFIG_BINFMT_DISABLE 1
#endif

/* The correct way to disable RR scheduling is to set the
 * timeslice to zero.
 */

#ifndef CONFIG_RR_INTERVAL
#  define CONFIG_RR_INTERVAL 0
#endif

/* The correct way to disable filesystem support is to set the number of
 * file descriptors to zero.
 */

#ifndef CONFIG_NFILE_DESCRIPTORS
#  define CONFIG_NFILE_DESCRIPTORS 0
#endif

/* If no file descriptors are configured, then make certain no streams
 * are configured either.
 */

#if CONFIG_NFILE_DESCRIPTORS == 0
#  undef  CONFIG_NFILE_STREAMS
#  define CONFIG_NFILE_STREAMS 0
#endif

/* The correct way to disable stream support is to set the number of
 * streamd to zero.
 */

#ifndef CONFIG_NFILE_STREAMS
#  define CONFIG_NFILE_STREAMS 0
#endif

/* If a console is selected, then make sure that there are resources for
 * three file descriptors and, if any streams are selected, also for three
 * file streams.
 *
 * CONFIG_DEV_CONSOLE means that a builtin console device exists at /dev/console
 * and can be opened during boot-up.  Other consoles, such as USB consoles, may
 * not exist at boot-upand have to be handled in a different way.  Three file
 * descriptors and three file streams are still needed.
 */

#if defined(CONFIG_DEV_CONSOLE) || defined(CONFIG_CDCACM_CONSOLE) || \
    defined(CONFIG_PL2303_CONSOLE)
# if CONFIG_NFILE_DESCRIPTORS < 3
#   undef CONFIG_NFILE_DESCRIPTORS
#   define CONFIG_NFILE_DESCRIPTORS 3
# endif

# if CONFIG_NFILE_STREAMS > 0 && CONFIG_NFILE_STREAMS < 3
#  undef CONFIG_NFILE_STREAMS
#  define CONFIG_NFILE_STREAMS 3
# endif

/* If no console is selected, then disable all builtin console devices */

#else
#  undef CONFIG_DEV_LOWCONSOLE
#  undef CONFIG_RAMLOG_CONSOLE
#endif

/* If no file streams are configured, then make certain that buffered I/O
 * support is disabled
 */

#if CONFIG_NFILE_STREAMS == 0
#  undef  CONFIG_STDIO_BUFFER_SIZE
#  undef  CONFIG_STDIO_LINEBUFFER
#  undef  CONFIG_STDIO_DISABLE_BUFFERING
#  define CONFIG_STDIO_DISABLE_BUFFERING 1
#endif

/* If priority inheritance is disabled, then do not allocate any
 * associated resources.
 */

#if !defined(CONFIG_PRIORITY_INHERITANCE) || !defined(CONFIG_SEM_PREALLOCHOLDERS)
#  undef CONFIG_SEM_PREALLOCHOLDERS
#  define CONFIG_SEM_PREALLOCHOLDERS 0
#endif

#if !defined(CONFIG_PRIORITY_INHERITANCE) || !defined(CONFIG_SEM_NNESTPRIO)
#  undef  CONFIG_SEM_NNESTPRIO
#  define CONFIG_SEM_NNESTPRIO 0
#endif

/* If the end of RAM is not specified then it is assumed to be the beginning
 * of RAM plus the RAM size.
 */

#ifndef CONFIG_RAM_END
#  define CONFIG_RAM_END (CONFIG_RAM_START+CONFIG_RAM_SIZE)
#endif

#ifndef CONFIG_RAM_VEND
#  define CONFIG_RAM_VEND (CONFIG_RAM_VSTART+CONFIG_RAM_SIZE)
#endif

/* If the end of FLASH is not specified then it is assumed to be the beginning
 * of FLASH plus the FLASH size.
 */

#ifndef CONFIG_FLASH_END
#  define CONFIG_FLASH_END (CONFIG_FLASH_START+CONFIG_FLASH_SIZE)
#endif

/* If the maximum message size is zero, then we assume that message queues
 * support should be disabled
 */

#if !defined(CONFIG_MQ_MAXMSGSIZE) || defined(CONFIG_DISABLE_MQUEUE)
#  undef CONFIG_MQ_MAXMSGSIZE
#  define CONFIG_MQ_MAXMSGSIZE 0
#endif

#if CONFIG_MQ_MAXMSGSIZE <= 0 && !defined(CONFIG_DISABLE_MQUEUE)
#  define CONFIG_DISABLE_MQUEUE 1
#endif

/* The correct way to disable socket support is to set the number of
 * socket descriptors to zero.
 */

#ifndef CONFIG_NSOCKET_DESCRIPTORS
#  define CONFIG_NSOCKET_DESCRIPTORS 0
#endif

/* There can be no network support with no socket descriptors */

#if CONFIG_NSOCKET_DESCRIPTORS <= 0
#  undef CONFIG_NET
#endif

/* Conversely, if there is no network support, there is no need for
 * socket descriptors
 */

#ifndef CONFIG_NET
#  undef  CONFIG_NSOCKET_DESCRIPTORS
#  define CONFIG_NSOCKET_DESCRIPTORS 0
#endif

#endif /* __INCLUDE_NUTTX_CONFIG_H */
