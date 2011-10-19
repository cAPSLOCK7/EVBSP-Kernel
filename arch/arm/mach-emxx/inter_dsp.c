
/*
 * InterDSP Driver
 *
 * Copyright (C) 2010 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/stringify.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/cdev.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/semaphore.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
#include <asm/cacheflush.h>
#include <mach/hardware.h>
#include <mach/pmu.h>
#include <mach/smu.h>
#include <mach/timer.h>
#include <mach/emxx_mem.h>
#ifdef CONFIG_VIDEO_EMXX
#include <mach/emxx_v4l2.h>
#endif
#include <mach/hardware.h>
#ifdef CONFIG_PM
#include <mach/pm.h>
#endif

#include <mach/inter_dsp_ioctl.h>
#include "inter_dsp.h"

#define DSPDEV_MAJOR 0
#define INTERDSP_MINOR_SIZE 4
#define FORCE_INIT 1

#define INTERDSP_DRIVER_VERSION	"0.10 (2009/12/4)"

#ifndef FORCE_INIT
#define FORCE_INIT	0	/* disable */
#endif

#define DSP_MINOR_MAX	32	/* device count */

#define DSP_DEVICE_NAME "emxx_dsp"

/* PD power control ON/OFF.
   when chip ES2.0, always ON. */
#define PD_POWER_AUTO_CONTROL_OFF 0
#define PD_POWER_AUTO_CONTROL_ON  1
#define PD_POWER_AUTO_CONTROL PD_POWER_AUTO_CONTROL_OFF

/* PD power control Mode (Retention/Powerdown) */
#define PD_POWER_CONTROL_RETENTION 0
#define PD_POWER_CONTROL_POWERDOEN 1
#define PD_POWER_CONTROL_MODE PD_POWER_CONTROL_RETENTION

/*********************************************************************
 * Debug macro
 *********************************************************************/

/* #define EMEV_DSP_DEBUG 1 */

#ifndef DSP_DEV_DEBUG
#define DSP_DEV_DEBUG	0	/* disable SMU Register Debug Dump */
#endif

#if 0
#define DPRINT_PREFIX	__FILE__ ":"
#else
#define DPRINT_PREFIX
#endif
#define DEBUG_PRINT(FMT, ARGS...) \
	do {\
		char const *x___fn = __func__;\
		printk(KERN_INFO DPRINT_PREFIX "%s:%d: " FMT, \
			x___fn, __LINE__, ##ARGS);\
	} while (0)

#define TRACE_PRINT(FMT, ARGS...) \
do {\
	printk(KERN_DEBUG "inter_dsp: " FMT, ##ARGS);\
} while (0)

#ifdef EMEV_DSP_DEBUG
#define DBG_INLINE
#define DPRINT DEBUG_PRINT
#define DBG_MEMSET(S, C, N)	({ \
	void *xxx__s = (S); \
	int xxx__c = (C); \
	size_t xxx__n = (N); \
	DPRINT("memset(0x%08lx, %d, %u)\n", \
		(unsigned long)xxx__s, xxx__c, xxx__n); \
	memset(xxx__s, xxx__c, xxx__n); \
})
#define DBG_MEMCPY(D, S, N)	({ \
	void *xxx__d = (D); \
	void const *xxx__s = (S); \
	size_t xxx__n = (N); \
	DPRINT("memcpy(0x%08lx, 0x%08lx, %u)\n", \
		(unsigned long)xxx__d, (unsigned long)xxx__s, xxx__n); \
	memcpy(xxx__d, xxx__s, xxx__n); \
})
#define SQNUM(sq)	((sq) ? (sq) - arm_send_entry : -1)
#else  /* !EMEV_DSP_DEBUG */
#define DBG_INLINE	inline
#define DPRINT(fmt, args...)
#define DBG_MEMSET	memset
#define DBG_MEMCPY	memcpy
#endif	/* EMEV_DSP_DEBUG */

/***********************************************************************
 * ARM-DSP communication driver configuration macro
 **********************************************************************/

#ifndef INTERDSP_ACK_TIMEOUT
#define INTERDSP_ACK_TIMEOUT	1000 /* 1 sec */
#endif

/* download possible area */
#define SDRAM_DL_RANGE_START DOWNLOAD_RANGE_START
#define SDRAM_DL_RANGE_END (SHARED_MEM_ADDRESS + SHARED_MEM_SIZE - 1)

/* transmission queue length */
#ifndef INTERDSP_SQ_LEN
#define INTERDSP_SQ_LEN	8	/* Send Queue length */
#endif /* INTERDSP_SQ_LEN */

/* offset of address each PE*/
#ifndef INTERDSP_PE_INDEX_OFFSET
#define INTERDSP_PE_INDEX_OFFSET	(0x100)	/* 256byte */
#endif /* INTERDSP_PE_INDEX_OFFSET */

/* offset of buffer block area in shared memory */
#ifndef INTERDSP_BLOCK_OFFSET
#define INTERDSP_BLOCK_OFFSET		(INTERDSP_PE_INDEX_OFFSET<<4) /* 4K */
#endif /* INTERDSP_BLOCK_OFFSET */

/* download DMA buffer size */
#ifndef INTERDSP_DMA_SIZE
#define INTERDSP_DMA_SIZE	(PAGE_SIZE*4)
#endif /* INTERDSP_DMA_SIZE */

/* character device */
#define DSPDEV_NAME	"InterDSP"
#ifndef DSPDEV_MAJOR
#define DSPDEV_MAJOR	63	/* XXX */
#endif /* DSPDEV_MAJOR */
#define NEED_SHMEM_INIT(X)	(force_init != 0)

#define DSPDEV_CONTROL_MINOR	16

#define DSPDEV_DATAMGR_MINOR	20 /* DATA MANAGER */

/* minor device num possible extension */
#ifndef INTERDSP_MINOR_SIZE
#define INTERDSP_MINOR_SIZE	32
#endif

/***********************************************************************
 * peripheral register address
 **********************************************************************/

/* INTC register address */
#define EMXX_VA_INTC	IO_ADDRESS(EMXX_INTA_D_BASE)
/* DSP->CPU0/1 Communication between the processor monitor */
#define IT3_IPI0_MON	(EMXX_VA_INTC + 0x003C)
/* DSP->CPU0/1 Communication between the processor clear */
#define IT3_IPI0_CLR	(EMXX_VA_INTC + 0x005C)
/* CPU0->DSP Communication between the processor set */
#define IT0_IPI3_SET	(EMXX_VA_INTC + 0x0030)
/* CPU1->DSP Communication between the processor set */
#define IT1_IPI0_SET	(EMXX_VA_INTC + 0x0034)
/* CPU0->DSP Communication between the processor clear */
#define IT0_IPI3_CLR	(EMXX_VA_INTC + 0x0050)
/* CPU1->DSP Communication between the processor clear */
#define IT1_IPI3_CLR	(EMXX_VA_INTC + 0x0054)
/* DSP interrupt disable set register0 */
#define IT3_IDS0		(EMXX_VA_INTC + 0x0008)
/* DSP interrupt disable set register1 */
#define IT3_IDS1		(EMXX_VA_INTC + 0x000C)
/* DSP interrupt disable set register2 */
#define IT3_IDS2		(EMXX_VA_INTC + 0x0104)
/* DSP interrupt state reset */
#define IT3_IIR			(EMXX_VA_INTC + 0x0024)
/* interrupt output signal clear */
#define IT3_CLR			(EMXX_VA_INTC + 0x0108)

/* Timer register address */
#define TIMER_TI3_SET	(IO_ADDRESS(EMXX_TIMER3_BASE) + 0x0008)
#define TIMER_TI3_OP	(IO_ADDRESS(EMXX_TIMER3_BASE) + 0x0000)
#define TIMER_TI3_CLR	(IO_ADDRESS(EMXX_TIMER3_BASE) + 0x0004)

/* SMU register */
#define SMU_TI3TIN_SEL		SMU_TWI3TIN_SEL
#define SMU_DEBUG_EN_STATUS	(IO_ADDRESS(EMXX_SMU_BASE) + 0x0AB0)

/* SMU_PD_SWON bit */
#define PD_CKRQ_CTREN	(1U<<20)
#define PD_PDON			(1U<<8)
#define PD_SWON			(1U<<0)

/* SMU_SEQ_BUSY bit */
#define PD_SEQ_BUSY		(1U<<5)

/* DCV Support */
#define EMXX_VA_DCV		IO_ADDRESS(EMXX_DCV_BASE)

#define DCV_BANKn_OFFSET(V, N)	((unsigned long)(V)+((N)<<2))
#define DCV_BANKn_SET(V, N)	((unsigned long)(V)+0x40+((N)<<2))

#define DSP_DATAMGR_SIZE		256

/***********************************************************************
 * for driver macro, structure, union definition
 **********************************************************************/

/* PE num */
#define PE_NUM	3

/* setting communication channel and max block num */
#define CH_NUM		INTERDSP_CH_NUM	/* Channel num */
#define BUF_BLOCK_MAX	8	/* block num */

/* channel invalid mark */
#define CH_INVALID	INTERDSP_CH_INVALID

/* macro that conversion from TaskID of Task information to array index */
/* When TaskID starts from 0 */
#define TASKID2INDEX(ID)	((unsigned)(ID))

/* interrupt factor bit */
#define SEND_INT(CH)	(1<<(CH))
#define ACK_INT(CH)	(1<<((CH)+CH_NUM))
#define CH_INT_BITSHIFT	1

#define ELEMENT_SIZE(X)	(sizeof(X) / sizeof(X)[0])

/* transmission queue entry */
struct arm_queue_ent {
	struct list_head list;
	union interdsp_spa_header header;
	int ch;			/* Channel */
	int num;		/* Block number */
	void *buf;		/* buffer block address */
	struct semaphore *blocksem; /* semaphore address */
	int nolock;		/* nolock buffer */
	int locked;		/* Block Locked */
	atomic_t ack;		/* ACK receive state */
	atomic_t discard;	/* destroy */
	atomic_t active;	/* ACK wait state */
	atomic_t sync;		/* have sync */
	atomic_t valid;		/* valid */
	atomic_t timeout;	/* time out */
	struct timer_list timeout_timer; /* timer for time out check */
	wait_queue_head_t ack_waitq;
	void (*ack_callback)(int ch,
				union interdsp_spa_header, int, unsigned long);
	unsigned long cb_data;
};

/* head buffer area [ARM=0, DSP=1] */
typedef union interdsp_spa_header hdr_queue_area[2][CH_NUM];

/* information area for PE of shared memory */
struct dsp_peinfo {
	struct interdsp_chinfo arm_chinfo[CH_NUM];
	struct interdsp_chinfo dsp_chinfo[CH_NUM];
	uint32_t task_chtbl[0];	/* zero length array */
	/* ... */
	/* hdr_queue queue; */
};

/* itnerdsp_chinfo calculation macro */
#define BLOCK_BYTES(CHINFO) \
	((CHINFO)->buffer_blksize * sizeof(uint32_t))
#define CHINFO_BLOCK_BYTES(CHINFO) \
	(BLOCK_BYTES(CHINFO) * (CHINFO)->buffer_blknum)
#define CHINFO_BLOCK_ADDR(CHINFO) \
	BUFADDR(shared_base, (CHINFO)->buffer_offset)

/* task_chtbl[] max index */
#define TASK_INFO_MAX	\
	((INTERDSP_PE_INDEX_OFFSET \
	- (size_t)(((struct dsp_peinfo *)0)->task_chtbl) \
	- sizeof(hdr_queue_area)) \
	/ sizeof((struct dsp_peinfo *)0)->task_chtbl[0])

/* make `struct dsp_peinfo' pointer for PE#N, N=0,1,2 */
#define PEINFO_ADDR(PENUM)	\
	((struct dsp_peinfo *)(((char *)shared_base) \
			+ INTERDSP_PE_INDEX_OFFSET*(PENUM)))

/* header area for PE*/
#define PE_HEADER_AREA(PENUM)	\
	((hdr_queue_area *)((char *)PEINFO_ADDR(PENUM) \
			+ (INTERDSP_PE_INDEX_OFFSET - sizeof(hdr_queue_area))))

/* routine judges it is header area */
#define PE_HEADER_AREA_RANGE(PENUM, ADDR) \
	(((char *)PE_HEADER_AREA(PENUM)) <= (char *)(ADDR) \
	&& (char *)(ADDR) < ((char *)PE_HEADER_AREA(PENUM) \
			+ sizeof(hdr_queue_area)))

/* address<->offset conversion macro */
#define BUFADDR(BASE, OFFSET)	((void *)(((char *)(BASE)) + OFFSET))
#define BUFOFF(BASE, ADDR)	((char *)(ADDR) - (char *)(BASE))

/* address<->offset conversion macro in shared memory*/
#define SHARED_BUFADDR(OFFSET)	BUFADDR(shared_base, OFFSET)
#define SHARED_BUFOFF(ADDR)	BUFOFF(shared_base, ADDR)

/* driver management area in shared memory */
struct dsp_driver_manager {
#define DRIVER_MAGIC	0xb5cc7a92 /* XXX */
	uint32_t magic;		/* MAGIC */
	uint32_t mutex;		/* lock */
	uint32_t block_factor;
	uint32_t nblocks;
};

/* driver management area address in shared memory*/
#define DRIVER_MNG_ADDR	\
((struct dsp_driver_manager *)((char *)shared_base \
				+ INTERDSP_PE_INDEX_OFFSET*(PE_NUM)))

/* Memory Allocation Table(MAT) address */
#define MAT_ADDR \
	((uint8_t *)((char *)shared_base \
	+ INTERDSP_PE_INDEX_OFFSET * (PE_NUM + 1)))

#define MAT_MIN_BFSHIFT	5	/* 2^5 = 32byte */
#define MAT_MAX_BFSHIFT	16	/* 2^16 = 64Kbyte */

/* max value of MAT */
#define MAT_MAX	\
(INTERDSP_BLOCK_OFFSET - ((char *)MAT_ADDR - (char *)shared_base))

/* conversion from MAT to address */
#define MAT2ADDR(BLK) \
((void *)((char *)shared_base + INTERDSP_BLOCK_OFFSET + (BLK) * block_factor))

/* conversion from address to MAT */
#define ADDR2MAT(ADDR) \
(((char *)(ADDR) - (char *)shared_base - INTERDSP_BLOCK_OFFSET) / block_factor)

/* process judges ADDR is in buffer block. */
#define BLOCK_BUFFER_RANGE(ADDR) \
	((char *)MAT2ADDR(0) <= (char *)(ADDR) \
	&& (char *)(ADDR) < (char *)MAT2ADDR(mat_blocks))

/* Value of Memory Allocation Table */
#define MAT_FREE	0x00U
#define MAT_USED	0x01U
#define MAT_RESERVED	0x02U
#define MAT_INVALID	0xffU

/* for dspch[0-2] */
struct dsp_ch_data {
	int ch;			/* channel */
};


/* table entry of extension possible minor device*/
struct minor_device_ent {
	int minor;
	int sticky;
	struct file_operations *fops;
	struct device *cls_dev;
};

#define PROCESS_INFO_NUM	128
#define INVALID_PROCESS_ID (PROCESS_INFO_NUM + 1)

struct process_data {
	int id;
	pid_t pid;
	pid_t ppid;
	int active;
	int ch;
};

struct dspdev_driver {
	int open_cnt;
};

static struct dcv_reg dcv_bank_val[16];

/***********************************************************************
 * external declaration
 **********************************************************************/

/***********************************************************************
 * static function prototype
 **********************************************************************/
static void clear_ackq_entry(void);

static int interdsp_release(struct inode *inode, struct file *file);
static int interdsp_ioctl(struct inode *inode, struct file *file,
		    unsigned int cmd, unsigned long arg);
static unsigned int interdsp_poll(struct file *file, poll_table *wait);
static int interdsp_fsync(struct file *file,
			struct dentry *dentry, int datasync);
static int interdsp_fasync(int fd, struct file *file, int mode);


static int inter_dsp_suspend(struct platform_device *dev, pm_message_t state);
static int inter_dsp_resume(struct platform_device *dev);

static void dsp_start_periperals(unsigned mask);
static void dsp_stop_periperals(unsigned mask);

/***********************************************************************
 *  driver internal data
 **********************************************************************/

/* file discrimination each channel*/
static struct dsp_ch_data const any_ch_private = { .ch = CH_INVALID,};

/* fasync method */
static struct fasync_struct *interdsp_fasync_queue;

/* shared memory top */
static unsigned long shared_mem_address = SHARED_MEM_ADDRESS;
static unsigned long shared_mem_size = SHARED_MEM_SIZE;
static struct dsp_peinfo *shared_base;

/* address and cache of PE information */
static struct dsp_peinfo *my_peinfo;
static union {
	struct dsp_peinfo peinfo;
	char unused[INTERDSP_PE_INDEX_OFFSET];
} cache_info;

/*  driver management area address */
static struct dsp_driver_manager *drv_mngaddr;

/* memory management table */
static uint8_t *mem_alloc_table;

/* memory allocater use Coefficient of making to DFD and
   block num which is allocation possibility */
static uint32_t block_factor;	/* 2^(n) */
static uint32_t mat_blocks;

/* transmission queue entry */
static struct arm_queue_ent arm_send_entry[INTERDSP_SQ_LEN * CH_NUM];

/* non use transmission entry list */
static struct list_head free_sq = LIST_HEAD_INIT(free_sq);

/* lock variable of list of transmission entry */
static DEFINE_SPINLOCK(free_sq_lock);

/* ACK time out */
static unsigned long ack_timeout = INTERDSP_ACK_TIMEOUT;

/* list of ACK processed entry (for temporary shelter) */
static struct list_head free_ackq = LIST_HEAD_INIT(free_ackq);

/* lock variable of list of ACK processed entry */
static DEFINE_SPINLOCK(free_ackq_lock);

/* transmission queue each channel */
static struct list_head arm_ch_sq[CH_NUM];

/* lock variable of transmission queue each channel */
static spinlock_t arm_ch_sq_lock[CH_NUM];
static DEFINE_SPINLOCK(arm_ch_sq_lock_unlock);

/* exclusion semaphore of transmission queue each channel
   (default is INTERDSP_SQ_LEN) */
static struct semaphore arm_ch_sqsem[CH_NUM];

/* counter of transmission queue each channel */
static atomic_t arm_ch_sqcnt[CH_NUM];

/* block semaqhore each channel */
static struct semaphore arm_ch_block[CH_NUM][BUF_BLOCK_MAX];

/* lock state of block bitmap each channel */
static unsigned long arm_ch_block_lock_bitmap[CH_NUM];

/* for select */
static wait_queue_head_t arm_ch_waitq[CH_NUM];

/* for error find */
static unsigned long arm_ch_error;
static unsigned long dsp_ch_error;

/* header area address */
static union interdsp_spa_header *arm_header[CH_NUM];
static union interdsp_spa_header *dsp_header[CH_NUM];

/* data buffer address and size for ARM channel */
static int arm_bufnum[CH_NUM];
static size_t arm_bufsiz[CH_NUM];
static void *arm_buffer[CH_NUM][BUF_BLOCK_MAX];

/* data buffer address and size for DSP channel */
static int dsp_bufnum[CH_NUM];
static void *dsp_buffer[CH_NUM][BUF_BLOCK_MAX];
static size_t dsp_bufsiz[CH_NUM];

/* define tasklet to control transmission queue */
static void dsp_ack_handler_core(int ch, int success);
static void dsp_ack_do_tasklet(unsigned long ch);
static DECLARE_TASKLET_DISABLED(arm_ch0_tasklet, dsp_ack_do_tasklet, 0);
static DECLARE_TASKLET_DISABLED(arm_ch1_tasklet, dsp_ack_do_tasklet, 1);
static DECLARE_TASKLET_DISABLED(arm_ch2_tasklet, dsp_ack_do_tasklet, 2);

/* tasklet array */
static typeof(arm_ch0_tasklet) *arm_ch_tasklet[] = {
	&arm_ch0_tasklet,
	&arm_ch1_tasklet,
	&arm_ch2_tasklet,
};

/* receive header counter */
static int dsp_ch_recv[CH_NUM] = {0,};

/* wait queue for receive */
static wait_queue_head_t dsp_ch_waitq[CH_NUM];

/* tasklet definition for receive */
static void dsp_req_handler(unsigned long ch);
static DECLARE_TASKLET(dsp_ch0_tasklet, dsp_req_handler, 0);
static DECLARE_TASKLET(dsp_ch1_tasklet, dsp_req_handler, 1);
static DECLARE_TASKLET(dsp_ch2_tasklet, dsp_req_handler, 2);

/* tasklet array for receive */
static typeof(arm_ch0_tasklet) *dsp_ch_tasklet[] = {
	&dsp_ch0_tasklet,
	&dsp_ch1_tasklet,
	&dsp_ch2_tasklet,
};

/* flag for "control" */
static atomic_t dsp_rcv_xcount;

/* INTC interrupt control register */
static unsigned int ipi_mon;
static unsigned int ipi_clr;
static unsigned int ipi_set;

/* SDRAM Download range */
static struct {
	unsigned long start;	/* begin */
	unsigned long end;	/* end + 1 */
	int valid;
	int used;
} sdram_dl_range[2];		/* zero cleared */

static int force_init = FORCE_INIT;

static int dspdev_major = DSPDEV_MAJOR;

/* statistics */
static unsigned long send_cmd_count[CH_NUM] = {0,};
static unsigned long recv_ack_count[CH_NUM] = {0,};
static unsigned long recv_resreq_count[CH_NUM] = {0,};
static unsigned long send_ack_count[CH_NUM] = {0,};

/* file_operations for possible extension minor device */
static struct minor_device_ent minor_device_table[INTERDSP_MINOR_SIZE];

/* DCV address */
static unsigned long dcv_va = EMXX_VA_DCV;

static struct process_data process_info[PROCESS_INFO_NUM];

static struct dspdev_driver dsp_datamgr_private;

static char dsp_datamgr_buf[DSP_DATAMGR_SIZE];

/* area management area exclusion access semaphore */
static struct semaphore datamgr_buf_sqsem;

/* variable for cancel INTERDSP_READ */
static wait_queue_head_t dsp_cancel_waitq;
static atomic_t dsp_cancel[PROCESS_INFO_NUM];

/* lock variable for InterDSP initialization */
static DEFINE_SPINLOCK(interdsp_init_lock);

/* lock variable for test_and_clear_data */
static DEFINE_SPINLOCK(test_and_clear_lock);

/* lock variable for dsp_set_tinsel */
static DEFINE_SPINLOCK(dev_tinsel_lock);

/* lock variable for interdsp_open_core */
static DEFINE_SPINLOCK(interdsp_opne_lock);

/* lock variable for interdsp_release_core */
static DEFINE_SPINLOCK(interdsp_release_lock);

/* power auto control enable */
static int power_control_enable = 1;
static int power_control_retention =
	(PD_POWER_CONTROL_MODE == PD_POWER_CONTROL_RETENTION ? 1 : 0);

/***********************************************************************
 * file operation
 **********************************************************************/

/* prototype */
static int interdsp_open(struct inode *inode, struct file *file);
static int interdsp_mmap(struct file *info, struct vm_area_struct *vma);
static int __init interdsp_probe(struct platform_device *dev);

/* /dev/dsp/datamgr */
static int dsp_datamgr_open(struct inode *inode, struct file *file);
static int dsp_datamgr_release(struct inode *inode, struct file *file);
static int dsp_datamgr_ioctl(struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg);

static int dspdev_open(struct inode *inode, struct file *file);
static struct file_operations dspdev_fops = {
	.open = dspdev_open,
};

/* operation structure */
static struct file_operations control_fops = {
	.poll    = interdsp_poll,
	.ioctl   = interdsp_ioctl,
	.open    = interdsp_open,
	.release = interdsp_release,
	.fsync   = interdsp_fsync,
	.fasync  = interdsp_fasync,
	.mmap    = interdsp_mmap,
};


/* /dev/dsp/datamgr */
static struct file_operations dsp_datamgr_fops = {
	.open    = dsp_datamgr_open,
	.release = dsp_datamgr_release,
	.ioctl   = dsp_datamgr_ioctl,
};


/* array of file name and file operation */
static struct {
	char *name;
	struct file_operations *fops;
	int minor;
} const dsp_procfs[] = {
	{ "control", &control_fops,     DSPDEV_CONTROL_MINOR},
	{ "datamgr", &dsp_datamgr_fops, DSPDEV_DATAMGR_MINOR},
};

static struct cdev dspdev_cdev;
static struct class *inter_dsp_class;
static struct device *inter_dsp_device;
static struct platform_driver inter_dsp_driver = {
	.probe = interdsp_probe,
/*	.remove = interdsp_remove, */
#ifdef CONFIG_PM
	.suspend = inter_dsp_suspend,
	.resume = inter_dsp_resume,
#endif
	.driver = {
		.name  = DSP_DEVICE_NAME,
		.owner = THIS_MODULE,
	},
};

#ifdef CONFIG_PM
/* DCV register shelter */
static struct dcv_reg dcv_reg_state[16];
/* DSP Suspend flag (DCV shelter flag) */
static int dcv_store;
#endif

static int driver_open_count;
static int taskid0_processid = INVALID_PROCESS_ID; /* invalid process id */

/***********************************************************************
 * exclusion spin lock between PE
 **********************************************************************/

/* global lock between drivers */
static DBG_INLINE void global_spin_lock(void)
{
	DPRINT("Enter\n");

	while (xchg(&drv_mngaddr->mutex, 1) != 0)
		;		/* EMPTY */

	DPRINT("Exit\n");
}

/* global unlock between drivers */
static DBG_INLINE void global_spin_unlock(void)
{
	DPRINT("Enter\n");

	xchg(&drv_mngaddr->mutex, 0);

	DPRINT("Exit\n");
}

/***********************************************************************
 * memory allocater
 **********************************************************************/

/* conversion from size(byte) to block num */
static DBG_INLINE int size_to_mat_blocks(size_t size)
{
	int ret;

	DPRINT("Enter [size=%u]\n", size);

	ret = (size + block_factor - 1) / block_factor;

	DPRINT("Exit [%d]\n", ret);

	return ret;
}


/* conversion from MAT number to block address */
static DBG_INLINE void *get_block_addr(int mat)
{
	void *addr;

	DPRINT("Enter [mat=%d]\n", mat);

	addr = MAT2ADDR(mat);

	DPRINT("Exit [addr=0x%08lx]\n", (unsigned long)addr);

	return addr;
}


/* conversion from block address to MAT number */
static DBG_INLINE int get_block_mat(void *addr)
{
	int ret;

	DPRINT("Enter [addr=0x%08lx]\n", (unsigned long)addr);

	ret = ADDR2MAT(addr);

	DPRINT("Exit\n");

	return ret;
}


/* reserve a area in MAT */
static int reserve_mat(uint32_t offset, int nblocks)
{
	int beg, end;
	int err = 0;
	int i;
	uint8_t *mp;
	uint8_t m;

	DPRINT("Enter [offset=%d, nblocks=%d]\n", offset, nblocks);

	/* get start num from address */
	beg = get_block_mat(SHARED_BUFADDR(offset));
	if (mat_blocks < beg) {
		err = -ENOMEM;
		goto end;
	}

	/* The terminal is calculated */
	end = beg + nblocks;
	if (mat_blocks < end) {
		err = -ENOMEM;
		goto end;
	}

	mp = &mem_alloc_table[beg];

	DPRINT("[beg=%d, end=%d]\n", beg, end);

	global_spin_lock();

	/* confirm possible setting */
	for (i = beg; i < end; i++) {
		/* non use or reserved is OK, but is NG other than it */
		m = *mp++;
		if (m == MAT_FREE || m == MAT_RESERVED)
			continue;

		DPRINT("Could not reserve: mat[%d]=%02x\n", i, m);

		err = -ENOMEM;
		goto end;
	}

	/* setting MAT */
	DBG_MEMSET(&mem_alloc_table[beg], MAT_RESERVED, nblocks);

end:
	global_spin_unlock();

	DPRINT("Exit [%d]\n", err);

	return err;
}


/* secure a area for block num from MAT */
static int get_mat(int nblocks)
{
	int beg = -1;
	int i, n = 0;
	uint8_t *mp = mem_alloc_table;
	int found = 0;

	DPRINT("Enter [nblocks=%d]\n", nblocks);

	if (nblocks <= 0 || mat_blocks <= nblocks) {
		beg = -ENOMEM;
		goto end;
	}

	global_spin_lock();

	/* search serial area of specified amount in fast fit */
	for (i = 0; i < mat_blocks; i++) {
		int m = *mp++;
		if (m == MAT_FREE) {
			if (beg == -1) {
				/* search start */
				beg = i;
				n = nblocks;
			}
			if (--n == 0) {
				/* Because the empty block was found,
				   it sets it ends. */
				DBG_MEMSET(&mem_alloc_table[beg],
					MAT_USED, nblocks);
				found = 1;
				break;
			}
		} else
			beg = -1; /* next search */
	}

	global_spin_unlock();

	if (found == 0)
		beg = -ENOMEM;

end:
	DPRINT("Exit [%d]\n", beg);

	return beg;
}


/* free MAT */
static DBG_INLINE void put_mat(int beg, int nblocks)
{
	uint8_t *mp;
	int i;

	DPRINT("Enter [beg=%d, nblocks=%d]\n", beg, nblocks);

	mp = &mem_alloc_table[beg];

	global_spin_lock();

	/* Only as for the thing that it did an automatic allotment to free */
	for (i = 0; i < nblocks && i < mat_blocks; i++) {
		uint8_t *xp = mp++;
		if (*xp == MAT_USED) {
			*xp = MAT_FREE;
			continue;
		}
		DPRINT("Could not free: mat[%d]=%02x\n", i, *xp);
	}

	global_spin_unlock();

	DPRINT("Exit\n");
}


/***********************************************************************
 * channel information
 **********************************************************************/

/* free CHINFO */
static void free_chinfo(int ind, int ch)
{
	void *addr;
	struct interdsp_chinfo *cache;
	int beg;
	size_t size;
	size_t nblocks;

	DPRINT("Enter [%s ch=%d]\n", ind ? "DSP" : "ARM", ch);

	if (ind == 0)
		cache = &cache_info.peinfo.arm_chinfo[ch];
	else
		cache = &cache_info.peinfo.dsp_chinfo[ch];

	if (cache->buffer_offset != 0
	    && cache->buffer_blksize != 0
	    && cache->buffer_blknum != 0) {
		addr = CHINFO_BLOCK_ADDR(cache);
		beg = get_block_mat(addr);
		size = CHINFO_BLOCK_BYTES(cache);
		nblocks = size_to_mat_blocks(size);
		put_mat(beg, nblocks);
	}

	DPRINT("Exit\n");
}


/* new channel */
static int new_chinfo(int ind, int ch, struct interdsp_chinfo *chinfo)
{
	int err = 0;
	void *addr;
	int beg;
	size_t size;
	size_t nblocks;
	hdr_queue_area *quehdr = PE_HEADER_AREA(0);
	union interdsp_spa_header *hdrbuf = &(*quehdr)[ind][ch];

	DPRINT("Enter [%s ch=%d, chinfo=0x%08lx]\n",
		ind ? "DSP" : "ARM", ch, (unsigned long)chinfo);

	/* new buffer is get */
	size = CHINFO_BLOCK_BYTES(chinfo);
	nblocks = size_to_mat_blocks(size);
	chinfo->header_offset = SHARED_BUFOFF(hdrbuf);
	if (chinfo->buffer_offset == ~(uint32_t)0) {
		/* block automatic quota */
		beg = get_mat(nblocks);
		if (beg < 0) {
			err = beg;
			goto error_exit;
		}
		addr = get_block_addr(beg);
		chinfo->buffer_offset = SHARED_BUFOFF(addr);
	} else {
		/* block specify */
		err = reserve_mat(chinfo->buffer_offset, nblocks);
		if (err)
			goto error_exit;
	}

error_exit:

	DPRINT("Exit [%d]\n", err);

	return err;
}


/***********************************************************************
 * Task information
 **********************************************************************/

/* Channel corresponding to the task is get. */
static int interdsp_get_taskid_channel(uint32_t taskid)
{
	uint32_t ch;
	int err = -EINVAL;

	DPRINT("Enter [taskid=%d]\n", taskid);

#ifdef MASTER_TASKID
	if (taskid == MASTER_TASKID) {
		err = 0;	/* TaskID=0 is fixed to CH0 */
		goto end;
	}
#endif	/* MASTER_TASKID */

	if (TASKID2INDEX(taskid) < TASK_INFO_MAX) {
		ch = cache_info.peinfo.task_chtbl[TASKID2INDEX(taskid)];
		if (ch == ~(uint32_t)0)
			err = CH_INVALID; /* invalid */
		else if (ch < CH_NUM)
			err = ch;
	}

end: __attribute__ ((unused))

	DPRINT("Exit [%d]\n", err);

	return err;
}


/* Channel corresponding to the task is set. */
static int interdsp_set_taskid_channel(uint32_t taskid, uint32_t ch)
{
	int err = -EINVAL;

	DPRINT("Enter [taskid=%u, ch=0x%08x]\n", taskid, ch);

	/* invalid */
	if (ch == CH_INVALID)
		ch = ~(uint32_t)0;

	if (TASKID2INDEX(taskid) < TASK_INFO_MAX) {
		cache_info.peinfo.task_chtbl[TASKID2INDEX(taskid)] = ch;
		my_peinfo->task_chtbl[TASKID2INDEX(taskid)] = ch;
		err = 0;
	}

	DPRINT("Exit\n");

	return err;
}


/***********************************************************************
 * transmission buffer block management
 **********************************************************************/

/* buffer block do lock */
static DBG_INLINE int arm_lock_buffer(int ch, int num, int nonblock)
{
	int err = 0;

	DPRINT("Enter [ch=%d, num=%d]\n", ch, num);

	if (nonblock) {
		if (down_trylock(&arm_ch_block[ch][num])) {
			err = -EAGAIN;
			goto end;
		}
	} else {
		if (down_interruptible(&arm_ch_block[ch][num])) {
			err = -ERESTARTSYS;
			goto end;
		}
	}
	set_bit(num, &arm_ch_block_lock_bitmap[ch]);

end:
	DPRINT("Exit [%d]\n", err);

	return err;
}


/* buffer block do unlock */
static DBG_INLINE void arm_unlock_buffer(int ch, int num)
{
	DPRINT("Enter [ch=%d, num=%d]\n", ch, num);

	up(&arm_ch_block[ch][num]);
	clear_bit(num, &arm_ch_block_lock_bitmap[ch]);

	DPRINT("Exit\n");
}


/* write to buffer block. however from user space */
static int interdsp_arm_write_buffer(int ch, int num, size_t offset,
				const void *ubuf, size_t sz)
{
	int err = 0;

	DPRINT("Enter [ch=%d, num=%d, offset=%d, ubuf=%08lx, sz=%u]\n",
		ch, num, offset, (unsigned long)ubuf, sz);

	if (sz
	    && (num < 0
		|| arm_bufnum[ch] <= num
		|| arm_bufsiz[ch] < offset + sz
		|| copy_from_user(BUFADDR(arm_buffer[ch][num], offset),
					ubuf, sz)))
		err = -EFAULT;
#ifdef EMEV_DSP_DEBUG
	else
		DPRINT("Write to: 0x%08lx\n",
			(unsigned long)BUFADDR(arm_buffer[ch][num], offset));
#endif	/* EMEV_DSP_DEBUG */

	DPRINT("Exit [%d]\n", err);

	return err;
}


/* read from buffer block. however to user space */
static int interdsp_dsp_read_buffer(int ch, int num, size_t offset,
				void *ubuf, size_t sz)
{
	int err = 0;

	DPRINT("Enter [ch=%d, num=%d, offset=%d, ubuf=0x%08lx, sz=%u]\n",
		ch, num, offset, (unsigned long)ubuf, sz);

	if (sz
	    && (num < 0
		|| dsp_bufnum[ch] <= num
		|| dsp_bufsiz[ch] < offset + sz
		|| copy_to_user(ubuf,
				BUFADDR(dsp_buffer[ch][num], offset), sz)))
		err = -EFAULT;
#ifdef EMEV_DSP_DEBUG
	else {
		DPRINT("Read from: 0x%08lx\n",
			(unsigned long)BUFADDR(dsp_buffer[ch][num], offset));
	}
#endif	/* EMEV_DSP_DEBUG */

	DPRINT("Exit [%d]\n", err);

	return err;
}


/***********************************************************************
 * transmission queue control
 **********************************************************************/

/* transmission queue entry initialization function */
static void init_sq_entry(struct arm_queue_ent *sq)
{
	sq->header.header[0] = 0;
	sq->header.header[1] = 0;
	sq->ch = 0;
	sq->num = 0;
	sq->buf = 0;
	sq->blocksem = 0;
	sq->nolock = 0;
	sq->locked = 0;
	atomic_set(&sq->ack, 0);
	atomic_set(&sq->discard, 1);
	atomic_set(&sq->active, 0);
	atomic_set(&sq->sync, 0);
	atomic_set(&sq->valid, 0);
	atomic_set(&sq->timeout, 0);
	init_timer(&sq->timeout_timer);
	init_waitqueue_head(&sq->ack_waitq);
}


/* block of entry of transmission queue do lock */
static int interdsp_lock_block_sq_entry(struct arm_queue_ent *sq, int nonblock)
{
	int err = 0;

	DPRINT("Enter [sq=0x%08lx:%d, nonblock=%d]\n",
		(unsigned long)sq, SQNUM(sq), nonblock);

	if (sq->nolock == 0 && sq->locked == 0) {
		if (nonblock) {
			if (down_trylock(sq->blocksem)) {
				err = -EAGAIN;
				goto end;
			}
		} else {
			if (down_interruptible(sq->blocksem)) {
				err = -ERESTARTSYS;
				goto end;
			}
		}
		sq->locked = 1;
		set_bit(sq->num, &arm_ch_block_lock_bitmap[sq->ch]);
	}

end:
	DPRINT("Exit [%d]\n", err);
	return err;
}


/* block of entry of transmission queue do unlock */
static void interdsp_unlock_block_sq_entry(struct arm_queue_ent *sq)
{
	DPRINT("Enter [sq=0x%08lx:%d]\n", (unsigned long)sq, SQNUM(sq));

	if (sq->locked) {
		up(sq->blocksem);
		clear_bit(sq->num, &arm_ch_block_lock_bitmap[sq->ch]);
		sq->locked = 0;
	}

	DPRINT("Exit\n");
}


/* get non use transmission queue entry */
static struct arm_queue_ent *get_free_sq_entry(void)
{
	unsigned long flags;
	struct arm_queue_ent *sq;

	DPRINT("Enter\n");

	clear_ackq_entry();	/* Garbage collection */

	spin_lock_irqsave(&free_sq_lock, flags);

	if (list_empty(&free_sq)) {
		printk(KERN_ERR "DSP sendqueue full!!\n");
		sq = 0;
		goto error;
	}

	sq = list_entry(free_sq.next, struct arm_queue_ent, list);
	if (sq)
		list_del(&sq->list);

error:
	spin_unlock_irqrestore(&free_sq_lock, flags);

	DPRINT("Exit [0x%08lx:%d]\n", (unsigned long)sq, SQNUM(sq));

	return sq;
}


/* transmission queue entry to non use list */
static void interdsp_free_sq_entry(struct arm_queue_ent *sq)
{
	unsigned long flags;

	DPRINT("Enter [sq=0x%08lx:%d]\n", (unsigned long)sq, SQNUM(sq));

	atomic_set(&sq->valid, 0);

	spin_lock_irqsave(&free_sq_lock, flags);
	list_add_tail(&sq->list, &free_sq);
	spin_unlock_irqrestore(&free_sq_lock, flags);

	DPRINT("Exit\n");
}


/* entry send to ACK received list */
static void put_free_ackq_entry(struct arm_queue_ent *sq)
{
	unsigned long flags;

	DPRINT("Enter [sq=0x%08lx:%d]\n", (unsigned long)sq, SQNUM(sq));

	spin_lock_irqsave(&free_ackq_lock, flags);
	list_add_tail(&sq->list, &free_ackq);
	spin_unlock_irqrestore(&free_ackq_lock, flags);

	DPRINT("Exit\n");
}


/* do cancel ACK received entry */
static void remove_free_ackq_entry(struct arm_queue_ent *sq)
{
	unsigned long flags;

	DPRINT("Enter [sq=0x%08lx:%d]\n", (unsigned long)sq, SQNUM(sq));

	spin_lock_irqsave(&free_ackq_lock, flags);
	list_del(&sq->list);
	spin_unlock_irqrestore(&free_ackq_lock, flags);

	/* when cancel it, throw  to free list */
	interdsp_free_sq_entry(sq);

	DPRINT("Exit\n");
}


/* do cancel all ACK received entry */
static void clear_ackq_entry(void)
{
	unsigned long flags;
	struct arm_queue_ent *sq;

	DPRINT("Enter\n");

	do {
		spin_lock_irqsave(&free_ackq_lock, flags);

		if (list_empty(&free_ackq)) {
			spin_unlock_irqrestore(&free_ackq_lock, flags);
			break;
		}

		sq = list_entry(free_ackq.next, struct arm_queue_ent, list);
		list_del(&sq->list);
		spin_unlock_irqrestore(&free_ackq_lock, flags);

		DPRINT("Clear ACK [sq=0x%08lx:%d]\n",
			(unsigned long)sq, SQNUM(sq));

		interdsp_free_sq_entry(sq);
	} while (1);

	DPRINT("Exit\n");
}


/* picked out top of transmission queue (now ACTIVE) */
static struct arm_queue_ent *get_sendq_top(int ch)
{
	struct arm_queue_ent *sq;
	unsigned long flags;

	DPRINT("Enter [ch=%d]\n", ch);

	spin_lock_irqsave(&arm_ch_sq_lock[ch], flags);
	if (list_empty(&arm_ch_sq[ch])) {
		sq = 0;
		goto empty;
	}

	sq = list_entry(arm_ch_sq[ch].next, struct arm_queue_ent, list);

	list_del(&sq->list);

	up(&arm_ch_sqsem[ch]);
	atomic_inc(&arm_ch_sqcnt[ch]);

	DPRINT("Remove SendQueue ARM CH%d [sq=0x%08lx:%d], Rest Queue=%d\n",
		ch, (unsigned long)sq, SQNUM(sq),
		atomic_read(&arm_ch_sqcnt[ch]));


#ifdef EMEV_DSP_DEBUG
	{
		struct list_head *pos;
		int i = 0;
		list_for_each(pos, &arm_ch_waitq[ch].task_list) {
			wait_queue_t *wq;
			wq = list_entry(pos, wait_queue_t, task_list);
			DPRINT("Wait[%d]=0x%08lx[0x%08lx]\n",
			i, (unsigned long)wq, (unsigned long)wq->private);
		}
	}
#endif	/* EMEV_DSP_DEBUG */

	wake_up_interruptible(&arm_ch_waitq[ch]); /* for select */

empty:
	spin_unlock_irqrestore(&arm_ch_sq_lock[ch], flags);

	DPRINT("Exit [sq=0x%08lx:%d]\n", (unsigned long)sq, SQNUM(sq));

	return sq;
}


/* ACK time out process function */
static void dsp_ack_timeout(unsigned long data)
{
	struct arm_queue_ent *sq = (struct arm_queue_ent *)data;
	int ch = sq->ch;

	DPRINT("Enter\n");

	/* ACK Timeout */
	atomic_set(&sq->timeout, 1);
	dsp_ack_handler_core(ch, 0); /* ACK Fail */

	DPRINT("Exit\n");
}

/* send top of transmission queue. called from tasklet too */
static void dsp_queue_send(int ch)
{
	struct arm_queue_ent *sq;
	unsigned long flags;

	DPRINT("Enter [ch=%d]\n", ch);

	spin_lock_irqsave(&arm_ch_sq_lock[ch], flags);

	if (list_empty(&arm_ch_sq[ch])) {
		DPRINT("SendQueue Empty\n");
		goto empty;
	}

	sq = list_entry(arm_ch_sq[ch].next, struct arm_queue_ent, list);

	if (atomic_read(&sq->active) == 0) {
		/* active is read only in other case */
		atomic_set(&sq->active, 1);

		DPRINT("Send Header [sq=0x%08lx:%d]/Wait ACK\n",
			(unsigned long)sq, SQNUM(sq));

		/* write to header buffer */
		*arm_header[ch] = sq->header;

		DPRINT("WriteHeader: ARM CH%d[0x%08lx]: 0x%08x 0x%08x\n",
			ch, (unsigned long)arm_header[ch],
			sq->header.header[0], sq->header.header[1]);

		init_timer(&sq->timeout_timer);
		sq->timeout_timer.function = dsp_ack_timeout;
		sq->timeout_timer.data = (unsigned long)sq;
		mod_timer(&sq->timeout_timer,
				jiffies + (ack_timeout * HZ + 999) / 1000);

		/* interrupt send */
		__raw_writel(SEND_INT(ch), ipi_set);
		send_cmd_count[ch]++;
#ifdef CONFIG_PM
		emxx_pm_pdma_suspend_disable();
#endif
	}
#ifdef EMEV_DSP_DEBUG
	else {
		DPRINT("INFO: Header Already Active [sq=0x%08lx:%d]\n",
			(unsigned long)sq, SQNUM(sq));
	}
#endif	/* EMEV_DSP_DEBUG */

empty:
	spin_unlock_irqrestore(&arm_ch_sq_lock[ch], flags);

	DPRINT("Exit\n");
}


/* into tail of transmission queue. non wait */
static DBG_INLINE void put_sendq_tail_core(int ch, struct arm_queue_ent *sq)
{
	unsigned long flags;

	DPRINT("Enter [ch=%d, sq=0x%08lx:%d]\n",
		ch, (unsigned long)sq, SQNUM(sq));

	atomic_set(&sq->valid, 1);

	spin_lock_irqsave(&arm_ch_sq_lock[ch], flags);
	list_add_tail(&sq->list, &arm_ch_sq[ch]);
	spin_unlock_irqrestore(&arm_ch_sq_lock[ch], flags);

	/* kick transmission queue */
	dsp_queue_send(ch);

	DPRINT("Exit\n");
}


/* command send to transmission queue.
   Because there is a case to wait for, this cannot use this from a handler
 */
static int put_sendq_tail(int ch, struct arm_queue_ent *sq, int nonblock)
{
	int err = 0;

	DPRINT("Enter [ch=%d, sq=0x%08lx:%d, nonblock=%d]\n",
		ch, (unsigned long)sq, SQNUM(sq), nonblock);

	if (nonblock) {
		if (down_trylock(&arm_ch_sqsem[ch])) {
			err = -EAGAIN;
			goto end;
		}
	} else {
		if (down_interruptible(&arm_ch_sqsem[ch])) {
			err = -ERESTARTSYS;
			goto end;
		}
	}
	atomic_dec(&arm_ch_sqcnt[ch]);

	put_sendq_tail_core(ch, sq);

	DPRINT("Add SendQueue ARM CH%d [sq=0x%08lx:%d], Rest Queue=%d\n",
		ch, (unsigned long)sq, SQNUM(sq),
		atomic_read(&arm_ch_sqcnt[ch]));

end:
	DPRINT("Exit [%d]\n", err);

	return err;
}


/***********************************************************************
 * ACK process
 **********************************************************************/

/* ACK receive/Timeout process core */
static void dsp_ack_handler_core(int ch, int success)
{
	struct arm_queue_ent *sq;

	DPRINT("Enter [ch=%d, %s]\n", ch, success ? "ACK" : "ACK Timeout");

	/* remove top */
	sq = get_sendq_top(ch);
	if (sq == 0) {
		DPRINT("Null ACK/Timeout handled\n");
		goto end;
	}

	/* any call back run */
	if (sq->ack_callback)
		(*sq->ack_callback)(ch, sq->header, success, sq->cb_data);

	if (success) {
		/* release time out */
		del_timer_sync(&sq->timeout_timer);

		/* ACK handling */
		atomic_inc(&sq->ack);
	} else {
		/* mark of error */
		set_bit(INTERDSP_ERROR_ARMCH_ACK_TIMEOUT(ch), &arm_ch_error);
	}

	/* free had block */
	if (sq->blocksem) {
		up(sq->blocksem);
		clear_bit(sq->num, &arm_ch_block_lock_bitmap[sq->ch]);
		sq->blocksem = 0;
		DPRINT("Unlock block\n");
	}

	/* consistency check */
	if (atomic_read(&sq->active) == 0)
		printk(KERN_ERR "%s: Queue TOP inactive\n", __func__);

	/* if ACK wait, wakeup ACK wait */
	if (atomic_read(&sq->sync)) {
		wake_up_interruptible(&sq->ack_waitq);
		DPRINT("Wakeup SyncTask\n");
	}

	DPRINT("[sq=0x%08lx:%d]\n", (unsigned long)sq, SQNUM(sq));

	/* when cancel it */
	if (atomic_read(&sq->discard))
		interdsp_free_sq_entry(sq);
	else
		put_free_ackq_entry(sq); /* when error */

end:
	/* send top */
	dsp_queue_send(ch);

	DPRINT("Exit\n");
}


/* tasklet substance  that processing ACK */
static void dsp_ack_do_tasklet(unsigned long ch)
{
	DPRINT("Enter [ch=%lu]\n", ch);

	/* disable tasklet run */
	tasklet_disable_nosync(arm_ch_tasklet[ch]);

	dsp_ack_handler_core((int)ch, 1);

	DPRINT("Exit\n");
}


/*********************************************************************
 * IOCTL relation
 *********************************************************************/

static DBG_INLINE int test_and_clear_data(int volatile * ptr)
{
	unsigned long flags;
	int v;

	spin_lock_irqsave(&test_and_clear_lock, flags);
	v = *ptr;
	*ptr = 0;
	spin_unlock_irqrestore(&test_and_clear_lock, flags);

	return v;
}


/* channel is selected by the round robin. internal function */
static DBG_INLINE int round_robin_channel(int *rr)
{
	int i;
	int ch;
	int ret = CH_INVALID;

	DPRINT("Enter [*rr=%d]\n", *rr);

	for (i = 0; i < CH_NUM; i++) {
		ch = (*rr)++;
		if (*rr == CH_NUM)
			*rr = 0;
		if (test_and_clear_data(&dsp_ch_recv[ch])) {
			ret = ch;
			break;
		}
	}

	DPRINT("Exit [ret=%d, *rr=%d]\n", ret, *rr);

	return ret;
}


/* read core */
static int interdsp_read_core(int id, int file_ch, int nonblock,
		   struct dsp_cmd_rw *rwcmd,
		   int (*read_callback)(int, int, size_t, void *buf, size_t))
{
	static int rr_ch;	/* Round Robin channel */
	int err = 0;
	int i;
	int ch;
	int num;
	size_t size;		/* data size (byte) */
	wait_queue_t dsp_w[CH_NUM]; /* for READ */
	wait_queue_t dsp_c; /* for cancel */
	union interdsp_spa_header cmdhdr; /* SPA-SPX Command Headder */
	int cancel_flag = 0;

	DPRINT("Enter [id=%d, file_ch=%d, nonblock=%d]\n",
		id, file_ch, nonblock);

	/* setting wait queue of DSP side all channel */
	for (i = 0; i < CH_NUM; i++) {
		init_waitqueue_entry(&dsp_w[i], current);
		add_wait_queue(&dsp_ch_waitq[i], &dsp_w[i]);
	}
	init_waitqueue_entry(&dsp_c, current);
	add_wait_queue(&dsp_cancel_waitq, &dsp_c);

	/* wait input */
	do {
		ch = CH_INVALID;
		set_current_state(TASK_INTERRUPTIBLE);

		/* read by process discriminationID */
		if (id > 0) {

			/* cancel interdsp_read */
			if (atomic_read(&dsp_cancel[id - 1])) {
				cancel_flag = 1;
				rwcmd->bsize = 0;
				break;
			}

			/* input channel select */
			for (i = 0; i < CH_NUM; i++) {
				if (dsp_ch_recv[i]) {
					cmdhdr = *dsp_header[i];
					if (cmdhdr.spa_res.taskid == 0) {
						/* Does this process
						   receive Taskid==0? */
						if (id == taskid0_processid) {
							ch = i;
							break;
						}
					} else if ((cmdhdr.spa_res.block_num &
							0xff00U) >> 8 == id) {
						ch = i;
						break;
					}
				}
			}
			/* There is input */
			if (ch != CH_INVALID) {
				if (test_and_clear_data(&dsp_ch_recv[ch]))
					break;
			}
		} else {
			/* input channel select */
			if (file_ch == CH_INVALID)
				ch = round_robin_channel(&rr_ch);
			else if (test_and_clear_data(&dsp_ch_recv[file_ch]))
				ch = file_ch;

			/* There is input */
			if (ch != CH_INVALID)
				break;
		}

		/* interrupt SIGNAL */
		if (signal_pending(current)) {
			err = -ERESTARTSYS;
			break;
		}

		/* It finished as EAGAIN in the case of non blocking */
		if (nonblock) {
			err = -EAGAIN;
			break;
		}

		/* sleep */
		schedule();
	} while (1);

	/* remove wait queue of DSP side all channel,
	   and translate state into running */
	for (i = 0; i < CH_NUM; i++)
		remove_wait_queue(&dsp_ch_waitq[i], &dsp_w[i]);
	remove_wait_queue(&dsp_cancel_waitq, &dsp_c);
	set_current_state(TASK_RUNNING);

	/* it ends here at error or cancel. */
	if (err || cancel_flag)
		goto end;

	/* get header */
	cmdhdr = *dsp_header[ch];
	rwcmd->fdata[0] = cmdhdr.header[0];
	rwcmd->fdata[1] = cmdhdr.header[1];

#ifdef EMEV_DSP_DEBUG
	DPRINT("ReadHeader: DSP CH%d[0x%08lx]: 0x%08x 0x%08x\n",
		ch, (unsigned long)dsp_header[ch],
		cmdhdr.header[0], cmdhdr.header[1]);
	if (cmdhdr.spa_res.rrf) {
		DPRINT("Req:RRF=%d, TASKID=%d, CODE=%d, "
			"LEN=%d, BLOCK=%d\n",
			cmdhdr.spa_req.rrf, cmdhdr.spa_req.taskid,
			cmdhdr.spa_req.code, cmdhdr.spa_req.length,
			cmdhdr.spa_req.block_num);
	} else {
		DPRINT("Res:"
			"RRF=%d, EOR=%d, TaskID=%d, ISSUE=%d, "
			"ERROR=%d, CODE=%d, LEN=%d, BLOCK=%d\n",
			cmdhdr.spa_res.rrf, cmdhdr.spa_res.eor,
			cmdhdr.spa_res.taskid, cmdhdr.spa_res.issue,
			cmdhdr.spa_res.error, cmdhdr.spa_res.code,
			cmdhdr.spa_res.length, cmdhdr.spa_res.block_num);
	}
#endif	/* EMEV_DSP_DEBUG */

	/* the number of bytes is calculated from
	   the number of forwarding words */
	size = cmdhdr.spa_res.length * sizeof(uint32_t);

	/* block confirmation */
	num = cmdhdr.spa_res.block_num;
	num &= 0xff;

	/* memory copy.
	   read to the upper limit of the buffer at the maximum here */
	if (rwcmd->bsize > size)
		rwcmd->bsize = size;
	if (read_callback)
		err = (*read_callback)(ch, num, 0,
					rwcmd->bdata, rwcmd->bsize);

	/* send ACK */
	__raw_writel(ACK_INT(ch), ipi_set);
	send_ack_count[ch]++;
#ifdef CONFIG_PM
	emxx_pm_pdma_suspend_enable();
#endif

	DPRINT("send CH%d ACK\n", ch);

end:
	DPRINT("Exit [%d]\n", err);

	return err;
}


/* IOCTL internal function.
   setting return and wakeup process in with interdsp_read_core.
   or, the setting disable.
 */
static void interdsp_read_cancel(int id, int flag)
{
	if (flag) {
		atomic_set(&dsp_cancel[id - 1], 1);
		wake_up_interruptible(&dsp_cancel_waitq);
	} else
		atomic_set(&dsp_cancel[id - 1], 0);
}

/* data from DSP is taken with IOCTL internal function */
static int interdsp_read(int file_ch, int nonblock,
		   struct dsp_cmd_rw *rwcmd,
		   int (*read_callback)(int, int, size_t, void *buf, size_t))
{
	int err;

	DPRINT("Enter [file_ch=%d, nonblock=%d]\n", file_ch, nonblock);

	err = interdsp_read_core(0, file_ch, nonblock, rwcmd, read_callback);

	DPRINT("Exit [%d]\n", err);

	return err;
}

/* data corresponding to process discrimination ID from DSP is taken
   with IOCTL internal function */
static int interdsp_read_process(int id, int nonblock,
		   struct dsp_cmd_rw *rwcmd,
		   int (*read_callback)(int, int, size_t, void *buf, size_t))
{
	int err;

	DPRINT("Enter [id=%d, nonblock=%d]\n", id, nonblock);

	err = interdsp_read_core(id, CH_INVALID,
		nonblock, rwcmd, read_callback);

	DPRINT("Exit [%d]\n", err);

	return err;
}

/* write to buffer with IOCTL internal function */
static int interdsp_writebuf(int ch, int num, int nonblock,
				struct dsp_cmd_rw *rwcmd, size_t size)
{
	int err = 0;

	DPRINT("Enter [ch=%d, num=%d, nonblock=%d, size=%d]\n",
		ch, num, nonblock, size);

	/* When the buffer is not protected, data is written.
	   After writing, the buffer is not protected in this processing. */
	if (size) {
		err = arm_lock_buffer(ch, num, nonblock);
		if (err)
			goto end;
		err = interdsp_arm_write_buffer(ch, num, rwcmd->offset,
						rwcmd->bdata, size);
		arm_unlock_buffer(ch, num);
	}

end:
	DPRINT("Exit [%d]\n", err);

	return err;
}


/* ACK wait with IOCTL internal function */
static int interdsp_wait_ack(int ch, struct arm_queue_ent *sq)
{
	int err = 0;
	DECLARE_WAITQUEUE(wait, current);

	DPRINT("Enter [ch=%d, sq=0x%08lx:%d]\n",
		ch, (unsigned long)sq, SQNUM(sq));

	add_wait_queue(&sq->ack_waitq, &wait);
	do {
		set_current_state(TASK_INTERRUPTIBLE);

		/* it ends at ACK received */
		if (atomic_read(&sq->ack))
			break;

		/* Time out of ACK is detected */
		if (atomic_read(&sq->timeout)) {
			err = -ETIME;
			break;
		}

		/* end when signale is received */
		if (signal_pending(current)) {
			err = -ERESTARTSYS;
			/* this queue cannot be waited for
			   any longer when SIGNAL is received here. */
			atomic_set(&sq->discard, 1);
			break;
		}

		schedule();
	} while (1);

	remove_wait_queue(&sq->ack_waitq, &wait);
	set_current_state(TASK_RUNNING);

	/* remove from ACK received list */
	remove_free_ackq_entry(sq);

	DPRINT("Exit [%d]\n", err);

	return err;
}


/* get new transmission queue entry */
static int interdsp_new_sq_entry(struct arm_queue_ent **sqp,
				int ch, int num, int ack_sync,
				struct dsp_cmd_rw *rwcmd,
				void (*ack_callback)(int ch,
						union interdsp_spa_header,
						int, unsigned long),
				unsigned long cb_data)
{
	int err = 0;
	struct arm_queue_ent *sq = 0;

	DPRINT("Enter [ch=%d, num=%d, ack_sync=%d]\n", ch, num, ack_sync);

	/* get que */
	sq = get_free_sq_entry();
	if (sq == 0) {
		err = -ENOMEM;
		goto error_exit;
	}

	/* make transmission entry */
	sq->header.header[0] = rwcmd->fdata[0];
	sq->header.header[1] = rwcmd->fdata[1];
	sq->ch = ch;
	/* process discrimination ID is set to high rank of block_num 8bit */
	num &= 0xff;
	sq->header.spa_ccf.block_num =
	       (sq->header.spa_ccf.block_num & 0xff00U) | num;
	sq->num = num;
	sq->buf = arm_buffer[ch][num];
	sq->blocksem = &arm_ch_block[ch][num];
	sq->nolock = 0;
	sq->locked = 0;
	atomic_set(&sq->ack, 0);
	atomic_set(&sq->active, 0);
	init_waitqueue_head(&sq->ack_waitq);
	if (ack_sync) {
		atomic_set(&sq->sync, 1); /* ACK wait */
		atomic_set(&sq->discard, 0); /* not automatic cancel */
	} else {
		atomic_set(&sq->sync, 0); /* non ACK wait */
		atomic_set(&sq->discard, 1); /* automatic cancel */
	}
	atomic_set(&sq->timeout, 0);
	sq->ack_callback = ack_callback;
	sq->cb_data = cb_data;

error_exit:

	if (err == 0)
		*sqp = sq;
	else if (sq) {
		interdsp_free_sq_entry(sq);
		sq = 0;
	}

	DPRINT("Exit [%d, sq=0x%08lx:%d]\n",
		err, (unsigned long)sq, SQNUM(sq));

	return err;
}



/* send header int IOCTL internal function */
static int interdsp_sendhdr(int ch, int num, int nonblock, int ack_sync,
				struct dsp_cmd_rw *rwcmd, int iq_num)
{
	int err = 0;
	struct arm_queue_ent *sq = 0;

	DPRINT("Enter [ch=%d, num=%d, nonblock=%d, ack_sync=%d]\n",
		ch, num, nonblock, ack_sync);

	/* get new queue */
	err = interdsp_new_sq_entry(&sq, ch, num, ack_sync, rwcmd, 0, 0);
	if (err)
		goto error_exit;

	/* this judges whether the lock is necessary */
	if (sq->header.spa_tcf.buffer == 0x3) /* TCF && READ */
		sq->nolock = 1;
	else {
		/* buffer protect */
		err = interdsp_lock_block_sq_entry(sq, nonblock);
		if (err)
			goto error_exit;
	}

	/* transmission entry to queue */
	err = put_sendq_tail(ch, sq, nonblock);
	if (err) {
		if (sq->nolock == 0)
			interdsp_unlock_block_sq_entry(sq);
		goto error_exit;
	}

	rwcmd->flags |= INTERDSP_SENDHDR_OK;

	/* ACK wait */
	if (ack_sync)
		err = interdsp_wait_ack(ch, sq);

error_exit:

	if (err != 0 && sq != 0)
		interdsp_free_sq_entry(sq);

	DPRINT("Exit [%d]\n", err);

	return err;
}


static int interdsp_write(int ch, int num, int nonblock, int ack_sync,
				struct dsp_cmd_rw *rwcmd, size_t size)
{
	int err = 0;
	struct arm_queue_ent *sq = 0;

	DPRINT("Enter [ch=%d, num=%d, nonblock=%d, size=%d]\n",
		ch, num, nonblock, size);

	err = arm_lock_buffer(ch, num, nonblock);
	if (err)
		goto end;

	if (size) {
		err = interdsp_arm_write_buffer(ch, num, rwcmd->offset,
						rwcmd->bdata, size);
		if (err) {
			arm_unlock_buffer(ch, num);
			goto end;
		}
		rwcmd->flags |= INTERDSP_WRITEBUF_OK;
	}

	/* get new queue */
	err = interdsp_new_sq_entry(&sq, ch, num, ack_sync, rwcmd, 0, 0);
	if (err) {
		arm_unlock_buffer(ch, num);
		goto end;
	}

	/* this judges whether the lock is necessary */
	if (sq->header.spa_tcf.buffer == 0x3) { /* TCF && READ */
		sq->nolock = 1;
		arm_unlock_buffer(ch, num);
		sq->locked = 0;
	} else if (sq->nolock) {
		arm_unlock_buffer(ch, num);
		sq->locked = 0;
	} else {
		sq->locked = 1;
	}

	/* transmission entry to queue */
	err = put_sendq_tail(ch, sq, nonblock);
	if (err) {
		if (sq->locked) {
			arm_unlock_buffer(ch, num);
			sq->locked = 0;
		}
		goto error_exit;
	}

	rwcmd->flags |= INTERDSP_SENDHDR_OK;

	/* ACK wait */
	if (ack_sync)
		err = interdsp_wait_ack(ch, sq);

error_exit:

	if (err != 0 && sq != 0)
		interdsp_free_sq_entry(sq);

end:
	DPRINT("Exit [%d]\n", err);
	return err;
}


/* CHINFO setting with IOCTL internal function */
static int interdsp_set_chinfo(int ind, int ch, struct interdsp_chinfo *chinfo)
{
	int err = 0;
	void *addr;
	int i;
	struct interdsp_chinfo zero_chinfo;

	DPRINT("Enter [%s ch=%d, chinfo=0x%08lx]\n",
		ind ? "DSP" : "ARM", ch, (unsigned long)chinfo);

#ifdef STRICT_CH_CHECK
	/* Check  channel used */
	if (ind == 0) {
		if (!list_empty(&arm_ch_sq[ch])) {
			DPRINT("ARM CH%d Active!!\n", ch);
			err = -EINVAL;
			goto error_exit;
		}
	} else {
		if (dsp_ch_recv[ch]) {
			DPRINT("DSP CH%d Active!!\n", ch);
			err = -EINVAL;
			goto error_exit;
		}
	}
#endif	/* STRICT_CH_CHECK */

	if (chinfo->header_offset == 0
	    && chinfo->buffer_offset == 0
	    && chinfo->buffer_blknum == 0
	    && chinfo->buffer_blksize == 0) {
		/* free this channel */
		free_chinfo(ind, ch);
	} else {
		/* if channel is finished with acquisition
		   in a buffer, free it */
		free_chinfo(ind, ch);

		/* get new buffer. But you must clear inside data because
		   the buffer is already freed when error rose. Therefore,
		   processing is continued by cleared channel information. */
		err = new_chinfo(ind, ch, chinfo);
		if (err) {
			DBG_MEMSET(&zero_chinfo, 0, sizeof zero_chinfo);
			chinfo = &zero_chinfo;
		}
	}

	/* construct cache and information in shared memory,
	   and develops with driver internal data */
	if (ind == 0) {
		/* ARM */
		cache_info.peinfo.arm_chinfo[ch] = *chinfo;
		my_peinfo->arm_chinfo[ch] = *chinfo;

		/* develops with driver internal data:
		   Is there the thing which is unsafe? -- XXX */
		arm_bufnum[ch] = chinfo->buffer_blknum;
		arm_bufsiz[ch] = BLOCK_BYTES(chinfo);
		if (chinfo->buffer_offset == 0)
			DBG_MEMSET(arm_buffer[ch], 0, sizeof arm_buffer[ch]);
		else {
			addr = CHINFO_BLOCK_ADDR(chinfo);
			for (i = 0; i < chinfo->buffer_blknum; i++) {
				arm_buffer[ch][i] = addr;
				addr = BUFADDR(addr, BLOCK_BYTES(chinfo));
			}
		}
	} else {
		/* DSP */
		cache_info.peinfo.dsp_chinfo[ch] = *chinfo;
		my_peinfo->dsp_chinfo[ch] = *chinfo;

		/* develops with driver internal data:
		   Is there the thing which is unsafe? -- XXX */
		dsp_bufnum[ch] = chinfo->buffer_blknum;
		dsp_bufsiz[ch] = BLOCK_BYTES(chinfo);
		if (chinfo->buffer_offset == 0)
			DBG_MEMSET(dsp_buffer[ch], 0, sizeof dsp_buffer[ch]);
		else {
			addr = CHINFO_BLOCK_ADDR(chinfo);
			for (i = 0; i < chinfo->buffer_blknum; i++) {
				dsp_buffer[ch][i] = addr;
				addr = BUFADDR(addr, BLOCK_BYTES(chinfo));
			}
		}
	}

#ifdef STRICT_CH_CHECK
error_exit:
#endif	/* STRICT_CH_CHECK */
	DPRINT("Exit [%d]\n", err);

	return err;
}


/* get IOCTL internal function in CHINFO*/
static int interdsp_get_chinfo(int ind, int ch, struct interdsp_chinfo *chinfo)
{
	DPRINT("Enter [%s ch=%d, chinfo=0x%08lx]\n",
		ind ? "DSP" : "ARM", ch, (unsigned long)chinfo);

	if (ind == 0)
		*chinfo = cache_info.peinfo.arm_chinfo[ch];
	else
		*chinfo = cache_info.peinfo.dsp_chinfo[ch];

	DPRINT("Exit [0]\n");

	return 0;
}


/* get channel state with IOCTL internal function */
static int interdsp_checkch(struct dsp_cmd_ch_status *cmd)
{
	int ch;
	uint32_t mask;

	cmd->armchmask &= ((1U << CH_NUM) - 1);
	cmd->dspchmask &= ((1U << CH_NUM) - 1);

	DPRINT("Enter [cmd=0x%08lx:ARM=0x%1x, DSP=0x%1x]\n",
		(unsigned long)cmd, cmd->armchmask, cmd->dspchmask);

	mask = 1;
	for (ch = 0; ch < CH_NUM; ch++) {
		if (cmd->armchmask & mask) {
			/* can't send */
			if (atomic_read(&arm_ch_sqcnt[ch]) == 0)
				cmd->armchmask &= ~mask;
		}
		if (cmd->dspchmask & mask) {
			/* no receive */
			if (dsp_ch_recv[ch] == 0)
				cmd->dspchmask &= ~mask;
		}
		mask <<= 1;
	}

	cmd->armch_blockbitmap[0] = arm_ch_block_lock_bitmap[0];
	cmd->armch_blockbitmap[1] = arm_ch_block_lock_bitmap[1];
	cmd->armch_blockbitmap[2] = arm_ch_block_lock_bitmap[2];

	DPRINT("Exit [ARM=0x%1x, DSP=0x%1x]\n",
		cmd->armchmask, cmd->dspchmask);

	return 0;
}


/* channel wait with IOCTL internal function */
static int interdsp_waitch(struct dsp_cmd_ch_status *cmd)
{
	int err = 0;
	int ch;
	uint32_t mask;
	uint32_t arm_mask;
	uint32_t dsp_mask;
	wait_queue_t arm_wait[CH_NUM];
	wait_queue_t dsp_wait[CH_NUM];

	cmd->armchmask &= ((1U << CH_NUM) - 1);
	cmd->dspchmask &= ((1U << CH_NUM) - 1);

	DPRINT("Enter [cmd=0x%08lx:ARM=0x%1x, DSP=0x%1x]\n",
		(unsigned long)cmd, cmd->armchmask, cmd->dspchmask);

	/* if it is empty, it ends at once. */
	mask = (1U << CH_NUM) - 1;
	if ((cmd->armchmask & mask) == 0 && (cmd->dspchmask & mask) == 0) {
		cmd->armchmask = 0;
		cmd->dspchmask = 0;
		DPRINT("Exit [0] -- null\n");
		return 0;
	}

	/* setting of wait queue according to mask */
	mask = 1;
	for (ch = 0; ch < CH_NUM; ch++) {
		if (cmd->armchmask & mask) {
			init_waitqueue_entry(&arm_wait[ch], current);
			add_wait_queue(&arm_ch_waitq[ch], &arm_wait[ch]);
			DPRINT("Add waitqueue entry ARM CH%d\n", ch);
		}
		if (cmd->dspchmask & mask) {
			init_waitqueue_entry(&dsp_wait[ch], current);
			add_wait_queue(&dsp_ch_waitq[ch], &dsp_wait[ch]);
			DPRINT("Add waitqueue entry DSP CH%d\n", ch);
		}
		mask <<= 1;
	}

#ifdef EMEV_DSP_DEBUG
	for (ch = 0; ch < CH_NUM; ch++) {
		struct list_head *pos;
		int i = 0;
		list_for_each(pos, &arm_ch_waitq[ch].task_list) {
			wait_queue_t *wq;
			wq = list_entry(pos, wait_queue_t, task_list);
			DPRINT("ARM CH%d Wait[%d]=0x%08lx[0x%08lx]\n",
				ch, i, (unsigned long)wq,
				(unsigned long)wq->private);
			i++;
		}
	}
	for (ch = 0; ch < CH_NUM; ch++) {
		struct list_head *pos;
		int i = 0;
		list_for_each(pos, &dsp_ch_waitq[ch].task_list) {
			wait_queue_t *wq;
			wq = list_entry(pos, wait_queue_t, task_list);
			DPRINT("DSP CH%d Wait[%d]=0x%08lx[0x%08lx]\n",
				ch, i, (unsigned long)wq,
				(unsigned long)wq->private);
			i++;
		}
	}
#endif	/* EMEV_DSP_DEBUG */

	do {
		set_current_state(TASK_INTERRUPTIBLE);

		arm_mask = cmd->armchmask;
		dsp_mask = cmd->dspchmask;
		mask = 1;
		for (ch = 0; ch < CH_NUM; ch++) {
			if (arm_mask & mask) {
				/* can't transmission */
				if (atomic_read(&arm_ch_sqcnt[ch]) == 0)
					arm_mask &= ~mask;
			}
			if (dsp_mask & mask) {
				/* no receive */
				if (dsp_ch_recv[ch] == 0)
					dsp_mask &= ~mask;
			}
			mask <<= 1;
		}

		/* If either the transmission or
		   the reception is possible, it ends */
		if (arm_mask != 0 || dsp_mask != 0)
			break;

		/* It ends when interrupting with the signal */
		if (signal_pending(current)) {
			err = -ERESTARTSYS;
			break;
		}

		/* sleep */
		schedule();
	} while (1);

	/* remove from wait queue */
	mask = 1;
	for (ch = 0; ch < CH_NUM; ch++) {
		if (cmd->armchmask & mask) {
			remove_wait_queue(&arm_ch_waitq[ch], &arm_wait[ch]);
			DPRINT("Remove waitqueue entry ARM CH%d\n", ch);
		}
		if (cmd->dspchmask & mask) {
			remove_wait_queue(&dsp_ch_waitq[ch], &dsp_wait[ch]);
			DPRINT("Remove waitqueue entry DSP CH%d\n", ch);
		}
		mask <<= 1;
	}

	cmd->armchmask = arm_mask;
	cmd->dspchmask = dsp_mask;

	set_current_state(TASK_RUNNING);

	DPRINT("Exit [ARM=0x%1x, DSP=0x%1x]\n",
		cmd->armchmask, cmd->dspchmask);

	return err;
}


static inline int dcv_set_bank_info(struct dcv_reg *dr)
{
	unsigned int bno = dr->bank_no;

	if (bno >= 16)
		return -EINVAL;

	dcv_bank_val[bno] = *dr;

	return 0;
}

static inline int dcv_get_bank_info(struct dcv_reg *dr)
{
	unsigned int bno = dr->bank_no;

	if (bno >= 16)
		return -EINVAL;

	*dr = dcv_bank_val[bno];

	return 0;
}

static inline int dcv_set_bank_info_reg(struct dcv_reg *dr)
{
	unsigned int bno = dr->bank_no;

	if (bno >= 16)
		return -EINVAL;

	__raw_writel(dr->bank_val.bank_reg.bank_offset & 0x0fff,
		DCV_BANKn_OFFSET(dcv_va, bno));
	__raw_writel(dr->bank_val.bank_reg.bank_set & 0x000f,
		DCV_BANKn_SET(dcv_va, bno));

	return 0;
}

static inline int dcv_get_bank_info_reg(struct dcv_reg *dr)
{
	unsigned int bno = dr->bank_no;

	if (bno >= 16)
		return -EINVAL;

	dr->bank_val.bank_reg.bank_offset =
		__raw_readl(DCV_BANKn_OFFSET(dcv_va, bno)) & 0x0fff;
	dr->bank_val.bank_reg.bank_set =
		__raw_readl(DCV_BANKn_SET(dcv_va, bno)) & 0x000f;

	return 0;
}

static inline void dcv_clock_gate(int set)
{
	if (set) {
		emxx_open_clockgate(EMXX_CLK_DCV);
		emxx_open_clockgate(EMXX_CLK_DCV_P);
	} else {
		emxx_close_clockgate(EMXX_CLK_DCV_P);
		emxx_close_clockgate(EMXX_CLK_DCV);
	}
}

static inline int dcv_clock_gate_status(void)
{
	return (emxx_get_clockgate(EMXX_CLK_DCV)
		&& emxx_get_clockgate(EMXX_CLK_DCV_P))
		? 1 : 0;
}

static inline void dcv_auto_clock(int set)
{
	if (set) {
		emxx_clkctrl_on(EMXX_CLKCTRL_DCV);
		emxx_clkctrl_on(EMXX_CLKCTRL_DCVPCLK);
	} else {
		emxx_clkctrl_off(EMXX_CLKCTRL_DCVPCLK);
		emxx_clkctrl_off(EMXX_CLKCTRL_DCV);
	}
}

static inline int dcv_auto_clock_status(void)
{
	return (emxx_get_clkctrl(EMXX_CLKCTRL_DCV)
		&& emxx_get_clkctrl(EMXX_CLKCTRL_DCVPCLK))
		? 1 : 0;
}


static inline void dcv_reset(int set)
{
	if (set)
		emxx_reset_device(EMXX_RST_DCV);
	else
		emxx_unreset_device(EMXX_RST_DCV);
}


/* SDRAM download area check function */
static inline int
in_dl_range(unsigned long x, unsigned long y, typeof(sdram_dl_range) range)
{
	if (range->valid) {
		/* Is the range of specification in the area? */
		if (range->start <= x && x < range->end
			&& range->start < y && y <= range->end)
			return 1; /* valid */
	} else {
		/* Is even one part of the range of
		   specification piled up in the area? */
		if ((range->start <= x && x < range->end)
			|| (range->start < y && y <= range->end))
			return -1; /* invalid */
	}
	return 0;		/* unknown */
}


/* download from CPU.
   if not copy to SDRAM/SRAM and data unit is non-16bit, copy with care.
 */
static int cpu_download(void *sram, void *udata, size_t size, int k6)
{
	int err = 0;
	unsigned long vbeg, vend;

	DPRINT("Enter [sram=0x%p, udata=0x%p, size=%u]\n", sram, udata, size);

	vbeg = (unsigned long)sram;
	vend = vbeg + size;

	if (k6 == 0
	    || (((unsigned long)sram & 3) == 0
		&& ((unsigned long)udata & 3) == 0
		&& (size & 3) == 0)) {
		/* When you do copy of SDRAM/SRAM or data of
		   the word alignment in to a word unit,
		   it is possible for effective copy */
		DPRINT("Using copy_from_user(0x%p, 0x%p, %u)\n",
			sram, udata, size);
		err = copy_from_user(sram, udata, size);
		if (err)
			goto end;
	} else {
		unsigned char *saddr = udata;
		unsigned short *daddr = sram;
		union {
			unsigned short us;
			unsigned char uc[2];
		} d;

		/* rocessing when beginning from byte alignment */
		if ((unsigned long)daddr & 1) {
			DPRINT("Pre: daddr=0x%p\n", daddr);

			/* 2byte align */
			daddr = (unsigned short *)((unsigned long)daddr
							& ~1UL);
			d.us = *daddr;
			DPRINT("    Old: 0x%p:0x%04x\n", daddr, d.us);
			err = get_user(d.uc[1], saddr);
			if (err)
				goto end;
			DPRINT("    New: 0x%p:0x%04x\n", daddr, d.us);
			*daddr++ = d.us;
			saddr++;
			size--;
		}

		/* copy data to taget, after copy it to
		   buffer of half-word alignment */
		while (size > 1) {
			unsigned short mbuf[256 / sizeof(unsigned short)];
			size_t s = sizeof mbuf;
			unsigned short *x;

			/* 2byte unit */
			if (s > (size & ~1UL))
				s = (size & ~1UL);

			DPRINT("Copying: saddr=0x%p, daddr=0x%p, size=%u\n",
				saddr, daddr, s);

			err = copy_from_user(mbuf, saddr, s);
			if (err)
				goto end;

			saddr += s;
			size -= s;
			x = mbuf;
			while (s) {
				*daddr++ = *x++;
				s -= 2;	/* 2byte copy */
			}
		}

		/* processing when ending in byte alignment */
		if (size) {
			DPRINT("Rest: daddr=0x%p\n", daddr);
			d.us = *daddr;
			DPRINT("    Old: 0x%p:0x%04x\n", daddr, d.us);
			err = get_user(d.uc[0], saddr);
			if (err)
				goto end;
			*daddr = d.us;
			DPRINT("    New: 0x%p:0x%04x\n", daddr, d.us);
		}
	}

end:

	DPRINT("Exit [err=%d]\n", err);

	return err;
}


/* DSP control (ARES) */
static inline void dsp_ares(int set)
{
	if (set)
		emxx_reset_device(EMXX_RST_DSP_A);
	else
		emxx_unreset_device(EMXX_RST_DSP_A);
}


/* DSP control (SRES) */
static inline void dsp_sres(int set)
{
	if (set)
		emxx_reset_device(EMXX_RST_DSP_S);
	else
		emxx_unreset_device(EMXX_RST_DSP_S);
}


/* DSP control (IRES) */
static inline void dsp_ires(int set)
{
	if (set)
		emxx_reset_device(EMXX_RST_DSP_I);
	else
		emxx_unreset_device(EMXX_RST_DSP_I);
}


/* DSP control (clock gete) */
static inline void dsp_clock_gate(int set)
{
	if (set) {
		/* DSP_ACLK_GCK -> DSP_CLK_GCK */
		emxx_open_clockgate(EMXX_CLK_DSP_A);
		emxx_open_clockgate(EMXX_CLK_DSP);
	} else {
		emxx_close_clockgate(EMXX_CLK_DSP);
		emxx_close_clockgate(EMXX_CLK_DSP_A);
	}
}

static inline int dsp_clock_gate_status(void)
{
	return (emxx_get_clockgate(EMXX_CLK_DSP)
		&& emxx_get_clockgate(EMXX_CLK_DSP_A))
		? 1 : 0;
}


static inline void dsp_auto_clock(int set)
{
	if (set) {
		emxx_clkctrl_on(EMXX_CLKCTRL_DSP_A);
		emxx_clkctrl_on(EMXX_CLKCTRL_DSP);
	} else {
		emxx_clkctrl_off(EMXX_CLKCTRL_DSP);
		emxx_clkctrl_off(EMXX_CLKCTRL_DSP_A);
	}
}


static inline int dsp_auto_clock_status(void)
{
	return (emxx_get_clkctrl(EMXX_CLKCTRL_DSP_A)
		&& emxx_get_clkctrl(EMXX_CLKCTRL_DSP))
		? 1 : 0;
}

static int dsp_set_sram_control(int srcclkctrl)
{
	if (srcclkctrl)
		emxx_clkctrl_on(EMXX_CLKCTRL_SRC);
	else
		emxx_clkctrl_off(EMXX_CLKCTRL_SRC);

	return 0;
}

static inline void dsp_power_control(int on)
{
	unsigned int val;

	while (__raw_readl(SMU_SEQ_BUSY) & PD_SEQ_BUSY)
		udelay(5);

	if (on) {
		val = __raw_readl(SMU_PD_SWON);
		val |= PD_SWON;
		val |= PD_PDON; /* Power down mode */
		__raw_writel(val, SMU_PD_SWON);

		if (power_control_enable || power_control_retention) {
			while (__raw_readl(SMU_SEQ_BUSY) & PD_SEQ_BUSY)
				udelay(5);

			/* enable power control sequence with DSP STOP mode */
			/* PD_CKRQ_CTREN should change in
			   the state of power supply On */
			val = __raw_readl(SMU_PD_SWON);
			if (power_control_enable)
				val |= PD_CKRQ_CTREN;
			if (power_control_retention)
				val &= ~PD_PDON;

			__raw_writel(val, SMU_PD_SWON);
		}

	} else {

		if (power_control_enable) {
			/* disable power control sequence with DSP STOP mode */
			val = __raw_readl(SMU_PD_SWON);
			val &= ~PD_CKRQ_CTREN;
			__raw_writel(val, SMU_PD_SWON);

			while (__raw_readl(SMU_SEQ_BUSY) & PD_SEQ_BUSY)
				udelay(5);
		}

		val = __raw_readl(SMU_PD_SWON);
		val &= ~PD_SWON;
		val |= PD_PDON; /* Power down mode */
		__raw_writel(val, SMU_PD_SWON);
	}

	while (__raw_readl(SMU_SEQ_BUSY) & PD_SEQ_BUSY)
		udelay(5);
}

/* DSP program downloader */
static int interdsp_download(struct dsp_cmd_download *dl)
{
	int err = 0;
	unsigned char *sram = 0;
	int save_auto_clock;
	int save_gate_clock;
	int gate_clock = -1;
	int auto_clock = -1;

#define RESET_PREPARE()	\
do {\
	if (gate_clock != 1 && save_gate_clock == 0) { \
		dsp_clock_gate(1); \
		gate_clock = 1; \
	} \
	if (auto_clock != 0 && save_auto_clock != 0) { \
		dsp_auto_clock(0); \
		auto_clock = 0; \
	} \
} while (0)

	DPRINT("Enter\n");

	/* start clock */
	if ((dl->control & INTERDSP_DL_CLOCK) && dl->clock != 0) {
		save_gate_clock = 1;
		gate_clock = 1;
		dsp_clock_gate(1);
	} else
		save_gate_clock = dsp_clock_gate_status();

	/* cancel automatic clock control */
	if ((dl->control & INTERDSP_DL_CLKCTRL) && dl->clkctrl == 0) {
		save_auto_clock = 0;
		auto_clock = 0;
		dsp_auto_clock(0);
	} else
		save_auto_clock = dsp_auto_clock_status();

	if ((dl->control & INTERDSP_DL_RESET)
	    && (dl->reset == 1 || dl->reset == 2)) {
		/* Reset Only */
		if (dl->reset == 1) {
			/* INTC Mask All */
			__raw_writel(~(uint32_t)0, IT3_IDS0);
			__raw_writel(~(uint32_t)0, IT3_IDS1);
			__raw_writel(~(uint32_t)0, IT3_IDS2);
			/* Clear All Interrupt Status */
			__raw_writel(~(uint32_t)0, IT3_IIR);
			/* Clear All IPI Request */
			__raw_writel(0x3fU, IT0_IPI3_CLR);
			__raw_writel(0x3fU, IT1_IPI3_CLR);
			__raw_writel(0x3fU, IT3_IPI0_CLR);
			/* Clear INT_DSP */
			__raw_writel(1, IT3_CLR);

			udelay(100);

			if (auto_clock != 0 && save_auto_clock != 0) {
				dsp_auto_clock(0);
				auto_clock = 0;
			}

			dcv_auto_clock(0);

			/* dsp power off */
			dsp_power_control(0);

			if (!dcv_clock_gate_status())
				dcv_clock_gate(1);

			dcv_reset(1);
			dcv_clock_gate(0);
		}

		/* pre reset control */
		RESET_PREPARE();

		/* IRESET, stop DSP */
		DPRINT("DSP_IRES: Set\n");
		dsp_ires(1);

		/* SRESET SET */
		DPRINT("DSP_SRES: Set\n");
		dsp_sres(1);
		/* ARESET SET */
		DPRINT("DSP_ARES: Set\n");
		dsp_ares(1);
	}

	if ((dl->control & INTERDSP_DL_PROG) != 0 && dl->size > 0) {
		unsigned long paddr;
		size_t size;
		unsigned long offset = dl->offset;

		/* copy cause check */
		if (dl->data == 0) {
			DPRINT("Error: null data\n");
			err = -EFAULT;
			goto end;
		}

		/* memory type */
		DPRINT("Download to SDRAM(BANK2)\n");

		{
			int i;
			int v = 0;
			int r;
			unsigned long s = EMXX_SDRAM_BASE + offset;
			unsigned long e = s + dl->size;

			/* range check */
			for (i = 0; i < ELEMENT_SIZE(sdram_dl_range); i++) {
				r = in_dl_range(s, e, &sdram_dl_range[i]);
				if (r == 1) {
					v = 1;
				} else if (r == -1) {
					v = -1;
					break;
				}
			}

			/* invalid range */
			if (v != 1) {
				DPRINT("Error: Invalid SDRAM Address"
					"[0x%08lx-0x%08lx]\n", s, e);
				err = -EFAULT;
				goto end;
			}

			/* reconfigure paddr, size */
			paddr = s & PAGE_MASK;
			offset = s - paddr;
			size = PAGE_ALIGN(e) - paddr;
		}

		/* mapping */
	#if 0
		sram = ioremap_nocache(paddr, size);
	#else
		sram = ioremap(paddr, size);
	#endif
		if (sram == 0) {
			err = -ENOMEM;
			goto end;
		}

		/* copy */
		err = cpu_download(sram + offset, dl->data,
					dl->size, 0);
		if (err) {
			DEBUG_PRINT("Error: Copy fail [-%d]\n", err);
			goto end;
		}

		DPRINT("Copy 0x%08lx->0x%08lx[%u]\n",
			(unsigned long)dl->data, paddr + offset, dl->size);
	}

	/* reset control */
	if (dl->control & INTERDSP_DL_RESET) {
		if (dl->reset == 0 || dl->reset == 2) {
			/* pre reset control */
			RESET_PREPARE();

			if (dl->reset == 0) {
				int i;

				dcv_auto_clock(0);
				dcv_clock_gate(1);

				dsp_power_control(1);

				dcv_reset(0);
				dcv_auto_clock(1);

				for (i = 0; i < 16; i++)
					dcv_set_bank_info_reg(&dcv_bank_val[i]);
			}

			/* ARESET, SRESET */
			dsp_ares(0);
			DPRINT("DSP_ARES: Clear\n");
			dsp_sres(0);
			DPRINT("DSP_SRES: Clear\n");

			/* IRESET */
			dsp_ires(0);
			DPRINT("DSP_IRES: Clear\n");
		}
	}

	/* stop clock */
	if (((dl->control & INTERDSP_DL_CLOCK) && dl->clock == 0)
	    || (gate_clock == 1 && save_gate_clock == 0)) {
		dsp_clock_gate(0);
	}

	/* start automatic clock control */
	if (((dl->control & INTERDSP_DL_CLKCTRL) && dl->clkctrl != 0)
	    || (auto_clock == 0 && save_auto_clock == 1)) {
		dsp_auto_clock(1);
	}

end:
	if (sram)
		iounmap(sram);

	DPRINT("Exit [%d]\n", err);

	return err;
}


static void dsp_start_periperals(unsigned mask)
{
	DPRINT("Enter: mask=0x%08x\n", mask);

	if (mask & DSPDEV_PERIP_TI3) {
		emxx_open_clockgate(EMXX_CLK_TI3);
		emxx_unreset_device(EMXX_RST_TI3);

	#if 0
		{
			unsigned long val;

			__raw_writel(0x00000000, TIMER_TI3_OP);
			__raw_writel(0x00000002, TIMER_TI3_CLR);
			__raw_writel(0x0000001f, TIMER_TI3_SET);

			/* TI3 setup */
			val = __raw_readl(SMU_TI3TIN_SEL);
			if (val == 0) {
				/* TW3/TI3 is RTC */
				__raw_writel(0x00010001, SMU_TI3TIN_SEL);
			}
		}
	#endif
	}

	DPRINT("Exit\n");
}


static void dsp_stop_periperals(unsigned mask)
{
	DPRINT("Enter: mask=0x%08x\n", mask);

	if (mask & DSPDEV_PERIP_TI3) {
		if (!emxx_get_clockgate(EMXX_CLK_TI3))
			emxx_open_clockgate(EMXX_CLK_TI3);

		emxx_reset_device(EMXX_RST_TI3);
		emxx_close_clockgate(EMXX_CLK_TI3);
	}

	DPRINT("Exit\n");
}

static void dsp_set_tinsel(unsigned mask)
{
	unsigned long flags;
	unsigned int tin_val;

	DPRINT("Enter: mask=0x%08x\n", mask);

	/* Direct SMU register access, because SDK has no pmu_* functions */
	if (mask & DSPDEV_TIN_TI3_BIT) {
		spin_lock_irqsave(&dev_tinsel_lock, flags);

		if (mask & DSPDEV_TIN_TI3_BIT) {
			tin_val = __raw_readl(SMU_TI3TIN_SEL);
			tin_val &= ~MASK_TINTIN_SEL;
			if (mask & DSPDEV_TIN_TI3_VAL)
				tin_val |= (1U << 0);
			__raw_writel(tin_val, SMU_TI3TIN_SEL);
		}
		spin_unlock_irqrestore(&dev_tinsel_lock, flags);
	}
	DPRINT("Exit\n");
}

/* clear channel information */
static void shared_pe_init(void)
{
	int i;

	if (my_peinfo == 0)
		return;

	/* initialization PE information */
	DBG_MEMSET(my_peinfo, 0, INTERDSP_PE_INDEX_OFFSET);
	DBG_MEMSET(&cache_info, 0, sizeof cache_info);

	for (i = 1; i < TASK_INFO_MAX; i++) {
		my_peinfo->task_chtbl[i] = ~0U;
		cache_info.peinfo.task_chtbl[i] = ~0U;
	}
}

/* perfect re-initialization function shared memory
   with IOCTL internal function */
static int interdsp_shmem_init(void)
{
	int error = 0;
	uint32_t size;
	int i;

	DPRINT("Enter\n");

	if (shared_base == 0) {
		error = -ENOMEM;
		goto error_exit;
	}

	/* The data array for channel is clear. */
	DBG_MEMSET(arm_bufnum, 0, sizeof arm_bufnum);
	DBG_MEMSET(arm_bufsiz, 0, sizeof arm_bufsiz);
	DBG_MEMSET(arm_buffer, 0, sizeof arm_buffer);
	DBG_MEMSET(dsp_bufnum, 0, sizeof dsp_bufnum);
	DBG_MEMSET(dsp_bufsiz, 0, sizeof dsp_bufsiz);
	DBG_MEMSET(dsp_buffer, 0, sizeof dsp_buffer);

	/* initialization all PE information */
	DBG_MEMSET(shared_base, 0, INTERDSP_PE_INDEX_OFFSET * PE_NUM);

	drv_mngaddr->mutex = 1;	/* lock */

	/* block factor is calculated */
	block_factor = 0;
	size = shared_mem_size - INTERDSP_BLOCK_OFFSET;
	for (i = MAT_MIN_BFSHIFT; i <= MAT_MAX_BFSHIFT; i++) {
		uint32_t f;
		f = 1U << i;
		if ((size + f - 1) / f < MAT_MAX) {
			block_factor = f;
			break;
		}
	}
	if (block_factor == 0) {
		error = -ENOMEM;
		goto error_exit;
	}

	/* calculate memory allocation table size */
	mat_blocks = (size + block_factor - 1) / block_factor;

	DPRINT("mat_blocks=%d, block_factor=%d\n", mat_blocks, block_factor);

	/* write to management area */
	drv_mngaddr->magic = DRIVER_MAGIC;
	drv_mngaddr->block_factor = block_factor;
	drv_mngaddr->nblocks = mat_blocks;

	/* initialization memory allocation table */
	DBG_MEMSET(mem_alloc_table, MAT_FREE, mat_blocks);
	DBG_MEMSET(mem_alloc_table + mat_blocks,
		MAT_INVALID, MAT_MAX - mat_blocks);

	DPRINT("MAT Setup End\n");

	shared_pe_init();

	/* unlock */
	drv_mngaddr->mutex = 0;

error_exit:

	DPRINT("Exit [error=%d]\n", error);

	return error;
}

static int dsp_set_communication_area(unsigned long addr, unsigned long size)
{
	unsigned long com_addr;
	unsigned long com_size;
	hdr_queue_area *quehdr;
	int i;

	DPRINT("addr=0x%08lx, size=0x%08lx\n", addr, size);

	com_size = size & PAGE_MASK;
	if ((com_size < 0x00001000) || (SHARED_MEM_SIZE < com_size))
		return -EINVAL;

	com_addr = addr & PAGE_MASK;
	if ((com_addr < SDRAM_DL_RANGE_START) ||
	    ((SDRAM_DL_RANGE_END + 1) < (com_addr + com_size)))
		return -EINVAL;

#if 0
	if ((sdram_dl_range[0].start == com_addr) &&
	    (sdram_dl_range[0].end == (com_addr + com_size)))
		return 0;
#endif

	if (shared_base)
		iounmap(shared_base);

	shared_base = ioremap_nocache(com_addr, com_size);
	if (shared_base == 0) {
		printk(KERN_INFO "Couldn't remap shared memory\n");
		return -ENOMEM;
	}

	shared_mem_address = com_addr;
	shared_mem_size = com_size;

	/* Download protect */
	sdram_dl_range[0].start = com_addr;
	sdram_dl_range[0].end = com_addr + com_size;
	sdram_dl_range[0].valid = 0; /* invalid */
	sdram_dl_range[0].used = 1; /* used */

	quehdr = PE_HEADER_AREA(0);
	my_peinfo = PEINFO_ADDR(0);
	drv_mngaddr = DRIVER_MNG_ADDR;
	mem_alloc_table = MAT_ADDR;

	DPRINT("shared_base = 0x%08lx\n", (unsigned long)shared_base);
	DPRINT("my_peinfo = 0x%08lx\n", (unsigned long)my_peinfo);
	DPRINT("drv_mngaddr = 0x%08lx\n", (unsigned long)drv_mngaddr);
	DPRINT("mem_alloc_table = 0x%08lx\n", (unsigned long)mem_alloc_table);

	for (i = 0; i < CH_NUM; i++) {
		arm_header[i] = &(*quehdr)[0][i];
		dsp_header[i] = &(*quehdr)[1][i];
		DPRINT("arm_header(CH %d) = 0x%08lx\n",
			i, (unsigned long)arm_header[i]);
		DPRINT("dsp_header(CH %d) = 0x%08lx\n",
			i, (unsigned long)dsp_header[i]);
	}

	interdsp_shmem_init();

	return 0;
}

/* release communication area */
static void dsp_release_communication_area(void)
{
	int i;

	if (shared_base != 0) {
		iounmap(shared_base);
		shared_base = 0;
		my_peinfo = 0;
		drv_mngaddr = 0;
		mem_alloc_table = 0;
		for (i = 0; i < CH_NUM; i++) {
			arm_header[i] = 0;
			dsp_header[i] = 0;
		}
	}
}

/* get communication area */
static int dsp_get_communication_area(struct dsp_communication_area *area)
{
	if (area == NULL)
		return -EINVAL;

	area->addr = sdram_dl_range[0].start;
	area->size = sdram_dl_range[0].end - sdram_dl_range[0].start;

	return 0;
}

static int dsp_get_download_area(struct dsp_download_area *area)
{
	if (area == NULL)
		return -EINVAL;

	area->addr = SDRAM_DL_RANGE_START;
	area->size = (SDRAM_DL_RANGE_END + 1) - SDRAM_DL_RANGE_START;

	return 0;
}


/* ioctl implementation */
static int interdsp_ioctl(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int i;
	int nb = 0;		/* non blocking */
	int wa = 0;		/* wait ACK */
	int ch;			/* channel number */
	int num;		/* block number */
	int taskid;		/* SubTask ID */
	size_t size;		/* data size (byte) */
	int file_ch;		/* file channel */
	union interdsp_spa_header cmdhdr; /* SPA-SPX Command Headder */
	struct dsp_cmd_rw rwcmd, *urwcmd; /* ioctl cmds */
	struct dsp_cmd_ch chcmd, *uchcmd;
	struct dsp_cmd_task_ch taskcmd, *utaskcmd;
	struct dsp_cmd_ch_status statuscmd, *ustatuscmd;
	struct dsp_cmd_download dlcmd, *udlcmd;
	struct dsp_cmd_dcv_set dcv_set, *udcv_set;
	struct dcv_reg dcv_tmp[16]; /* 16 banks */
	int count;
	unsigned long flags = 0;
	int errnr;
	int do_copy;
	int proc_id = 0;
	struct dsp_download_area dlar, *udlar;
	struct dsp_communication_area comar, *ucomar;

#ifdef EMEV_DSP_DEBUG
	switch (cmd) {
	case INTERDSP_READ:
		DPRINT("Enter cmd=INTERDSP_READ\n");
		break;
	case INTERDSP_WRITE:
		DPRINT("Enter cmd=INTERDSP_WRITE\n");
		break;
	case INTERDSP_READ_CANCEL:
		DPRINT("Enter cmd=INTERDSP_READ_CANCEL\n");
		break;
	case INTERDSP_WRITEBUF:
		DPRINT("Enter cmd=INTERDSP_WRITEBUF\n");
		break;
	case INTERDSP_SENDHDR:
		DPRINT("Enter cmd=INTERDSP_SENDHDR\n");
		break;
	case INTERDSP_GET_CHINFO:
		DPRINT("Enter cmd=INTERDSP_GET_CHINFO\n");
		break;
	case INTERDSP_SET_CHINFO:
		DPRINT("Enter cmd=INTERDSP_SET_CHINFO\n");
		break;
	case INTERDSP_GET_TASKCH:
		DPRINT("Enter cmd=INTERDSP_GET_TASKCH\n");
		break;
	case INTERDSP_SET_TASKCH:
		DPRINT("Enter cmd=INTERDSP_SET_TASKCH\n");
		break;
	case INTERDSP_SHMEM_INIT:
		DPRINT("Enter cmd=INTERDSP_SHMEM_INIT\n");
		break;
	case INTERDSP_CHECKCH:
		DPRINT("Enter cmd=INTERDSP_CHECKCH\n");
		break;
	case INTERDSP_WAITCH:
		DPRINT("Enter cmd=INTERDSP_WAITCH\n");
		break;
	case INTERDSP_DOWNLOAD:
		DPRINT("Enter cmd=INTERDSP_DOWNLOAD\n");
		break;
	case INTERDSP_GET_ERROR:
		DPRINT("Enter cmd=INTERDSP_GET_ERROR\n");
		break;
	case INTERDSP_SET_DCV_REGS:
		DPRINT("Enter cmd=INTERDSP_SET_DCV_REGS\n");
		break;
	case INTERDSP_GET_DCV_REGS:
		DPRINT("Enter cmd=INTERDSP_GET_DCV_REGS\n");
		break;
	case DSPDEV_START_PERIPHERALS:
		DPRINT("Enter cmd=DSPDEV_START_PERIPHERALS\n");
		break;
	case DSPDEV_STOP_PERIPHERALS:
		DPRINT("Enter cmd=DSPDEV_STOP_PERIPHERALS\n");
		break;
	case DSPDEV_TIN_SEL:
		DPRINT("Enter cmd=DSPDEV_TIN_SEL\n");
		break;

	case DSPDEV_SET_COM_AREA:
		DPRINT("Enter cmd=DSPDEV_SET_COM_AREA\n");
		break;
	case DSPDEV_GET_COM_AREA:
		DPRINT("Enter cmd=DSPDEV_GET_COM_AREA\n");
		break;
	case DSPDEV_GET_DOWNLOAD_AREA:
		DPRINT("Enter cmd=DSPDEV_GET_DOWNLOAD_AREA\n");
		break;
	case DSPDEV_SRAMCTRL:
		DPRINT("Enter cmd=DSPDEV_SRAMCTRL\n");
		break;

	default:
		DPRINT("Enter Auxiliary cmd=<%u>\n", cmd);
		break;
	}
#endif	/* EMEV_DSP_DEBUG */

	/* reset error */
	err = 0;

	/* is dspch[012] ? */
	if (file->private_data) {
		file_ch = ((struct process_data *)file->private_data)->ch;
		proc_id = ((struct process_data *)file->private_data)->id;
	} else
		file_ch = CH_INVALID; /* ANY */

	/* area check */
	if ((_IOC_DIR(cmd) & _IOC_READ)
	    && arg != 0
	    && !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd)) != 0) {
		DPRINT("EFAULT\n");
		err = -EFAULT;
		goto error_exit;
	}

	/* file mode confirmation */
	if (file->f_flags & O_SYNC)
		wa = 1;
	if (file->f_flags & (O_NONBLOCK | O_NDELAY))
		nb = 1;

	switch (cmd) {
	case INTERDSP_READ:
		urwcmd = (struct dsp_cmd_rw *)arg;
		if (copy_from_user(&rwcmd, urwcmd, sizeof *urwcmd)) {
			DPRINT("EFAULT\n");
			err = -EFAULT;
			break;
		}

		if (file_ch == CH_INVALID && proc_id > 0)
			err = interdsp_read_process(proc_id, nb, &rwcmd,
						interdsp_dsp_read_buffer);
		else
			err = interdsp_read(file_ch, nb, &rwcmd,
						interdsp_dsp_read_buffer);
		if (err)
			break;

		/* clear process identifier */
		rwcmd.fdata[1] &= 0x00ffffffU;

		/* Update of argument */
		if (copy_to_user(urwcmd, &rwcmd, sizeof *urwcmd)) {
			DPRINT("EFAULT\n");
			err = -EFAULT;
		}

		break;

	case INTERDSP_WRITE:
	case INTERDSP_WRITEBUF:
	case INTERDSP_SENDHDR:
		urwcmd = (struct dsp_cmd_rw *)arg;
		if (copy_from_user(&rwcmd, urwcmd, sizeof *urwcmd)) {
			DPRINT("EFAULT\n");
			err = -EFAULT;
			break;
		}

		/* get header */
		cmdhdr.header[0] = rwcmd.fdata[0];
		cmdhdr.header[1] = rwcmd.fdata[1];

		taskid = cmdhdr.spa_tcf.taskid;
		num = cmdhdr.spa_tcf.block_num & 0xff;
		cmdhdr.spa_tcf.block_num = (proc_id << 8) | num;
		rwcmd.fdata[1] = cmdhdr.header[1]; /* write back */

		ch = interdsp_get_taskid_channel(taskid);
		if (ch < 0 || ch == CH_INVALID) { /* error or invalid */
			DPRINT("Channel Invalid\n");
			err = -EINVAL;
			break;
		}

		do_copy = 1;

		/* forward size */
		if (cmdhdr.spa_tcf.buffer & 0x2) {
			/* TCF */
			if (cmdhdr.spa_tcf.buffer & 1) {
				/* DSP->CPU */
				do_copy = 0;
				size = 0;
			} else {
				/* CPU->DSP */
				/* 2004/03/24 [Revision 3.2] */
				size = cmdhdr.spa_tcf.length;
			}
		} else
			size = cmdhdr.spa_ccf.length; /* CCF */

		/* conversion to Byte because word num */
		size *= sizeof(uint32_t);

		/* progress state clear */
		rwcmd.flags = 0;

		/* write from buffer */
		if (cmd == INTERDSP_WRITE) {
			err = interdsp_write(ch, num, nb, wa, &rwcmd, size);
			if (err)
				break;
		} else if (cmd == INTERDSP_WRITEBUF) {
			if (do_copy) {
				err = interdsp_writebuf(ch, num, nb,
					&rwcmd, size);
				if (err)
					break;
				rwcmd.flags |= INTERDSP_WRITEBUF_OK;
			}
		} else if (cmd == INTERDSP_SENDHDR) {
			err = interdsp_sendhdr(ch, num, nb, wa,
						&rwcmd, -1);
		}

		/* Update of argument */
		if (put_user(rwcmd.flags, &urwcmd->flags)) {
			DPRINT("EFAULT\n");
			err = -EFAULT;
		}

		break;

	case INTERDSP_READ_CANCEL:
		interdsp_read_cancel(proc_id, arg);
		break;

	case INTERDSP_SET_CHINFO:
	case INTERDSP_GET_CHINFO:
		uchcmd = (struct dsp_cmd_ch *)arg;
		if (copy_from_user(&chcmd, uchcmd, sizeof *uchcmd)) {
			DPRINT("EFAULT\n");
			err = -EFAULT;
			break;
		}

		i = chcmd.dsparm ? 1 : 0; /* ARM[0] or DSP[1] ? */
		ch = chcmd.chnum;

		if (ch < 0 || CH_NUM <= ch) {
			DPRINT("Channel Invalid\n");
			err = -EINVAL;
			break;
		}

		if (cmd == INTERDSP_SET_CHINFO)
			err = interdsp_set_chinfo(i, ch, &chcmd.chinfo);
		else
			err = interdsp_get_chinfo(i, ch, &chcmd.chinfo);
		if (err)
			break;

		/* write back argument */
		if (copy_to_user(&uchcmd->chinfo, &chcmd.chinfo,
					sizeof uchcmd->chinfo)) {
			DPRINT("EFAULT\n");
			err = -EFAULT;
		}

		break;

	case INTERDSP_SET_TASKCH: /* set Task information */
		utaskcmd = (struct dsp_cmd_task_ch *)arg;
		if (copy_from_user(&taskcmd, utaskcmd, sizeof *utaskcmd)) {
			DPRINT("EFAULT\n");
			err = -EFAULT;
			break;
		}

		ch = taskcmd.chnum;
		if (ch == -1 || (0 <= ch && ch < CH_NUM)) {
			if (ch == -1)
				ch = CH_INVALID; /* invalid */
			err = interdsp_set_taskid_channel(taskcmd.taskid, ch);
		} else {
			DPRINT("Channel Invalid\n");
			err = -EINVAL;
		}

		break;

	case INTERDSP_GET_TASKCH: /* get Tsk information */
		utaskcmd = (struct dsp_cmd_task_ch *)arg;
		if (copy_from_user(&taskcmd, utaskcmd, sizeof *utaskcmd)) {
			DPRINT("EFAULT\n");
			err = -EFAULT;
			break;
		}

		err = interdsp_get_taskid_channel(taskcmd.taskid);
		if (err < 0)
			break;

		/* invalid channel? */
		if (err == CH_INVALID)
			ch = -1;
		else
			ch = err;

		err = 0;	/* clear error */

		if (put_user(ch, &utaskcmd->chnum)) {
			DPRINT("EFAULT\n");
			err = -EFAULT;
		}

		break;

	case INTERDSP_CHECKCH:	/* get channel state */
	case INTERDSP_WAITCH:	/* channel wait */
		ustatuscmd = (struct dsp_cmd_ch_status *)arg;
		if (copy_from_user(&statuscmd,
					ustatuscmd, sizeof *ustatuscmd)) {
			DPRINT("EFAULT\n");
			err = -EFAULT;
			break;
		}

		if (cmd == INTERDSP_CHECKCH)
			err = interdsp_checkch(&statuscmd);
		else
			err = interdsp_waitch(&statuscmd);

		if (copy_to_user(ustatuscmd,
					&statuscmd, sizeof *ustatuscmd)) {
			DPRINT("EFAULT\n");
			err = -EFAULT;
		}

		break;

	case INTERDSP_SHMEM_INIT:
		err = interdsp_shmem_init();
		break;

	case INTERDSP_DOWNLOAD:
		udlcmd = (struct dsp_cmd_download *)arg;
		if (copy_from_user(&dlcmd, udlcmd, sizeof *udlcmd)) {
			DPRINT("EFAULT\n");
			err = -EFAULT;
			break;
		}

		err = interdsp_download(&dlcmd);

		break;

	case INTERDSP_GET_ERROR:
		flags = 0;
		for (ch = 0; ch < CH_NUM; ch++) {
			errnr = INTERDSP_ERROR_ARMCH_ACK_TIMEOUT(ch);
			if (test_and_clear_bit(errnr, &arm_ch_error))
				flags |= (1UL << errnr);
		}
		if (put_user(flags, (unsigned long *)arg)) {
			DPRINT("EFAULT\n");
			err = -EFAULT;
		}
		break;

	case INTERDSP_SET_DCV_REGS:
	case INTERDSP_GET_DCV_REGS:
		udcv_set = (struct dsp_cmd_dcv_set *)arg;
		if (copy_from_user(&dcv_set, udcv_set, sizeof dcv_set)) {
			DPRINT("EFAULT\n");
			err = -EFAULT;
			break;
		}

		count = dcv_set.count;

		if (count > 16) {
			DPRINT("dcv_set.count(%u)>16\n", count);
			err = -EINVAL;
			break;
		}

		if (count == 0) {
			/* err = -EINVAL; */
			break;
		}

		if (copy_from_user(dcv_tmp, dcv_set.dcv_regs,
					sizeof dcv_tmp[0] * count)) {
			DPRINT("EFAULT\n");
			err = -EFAULT;
			break;
		}

		if (cmd == INTERDSP_SET_DCV_REGS) {
			for (i = 0; i < count; i++)
				dcv_set_bank_info(&dcv_tmp[i]);
		} else {
			/* in the case of all acquisition,
			   bank_no sets it automatically */
			if (count == 16) {
				for (i = 0; i < 16; i++)
					dcv_tmp[i].bank_no = i;
			}

			for (i = 0; i < count; i++)
				dcv_get_bank_info(&dcv_tmp[i]);

			if (copy_to_user(dcv_set.dcv_regs, dcv_tmp,
						sizeof dcv_tmp[0] * count)) {
				DPRINT("EFAULT\n");
				err = -EFAULT;
				break;
			}
		}

		break;

	case DSPDEV_START_PERIPHERALS:
		dsp_start_periperals(arg);
		break;

	case DSPDEV_STOP_PERIPHERALS:
		dsp_stop_periperals(arg);
		break;

	case DSPDEV_TIN_SEL:
		dsp_set_tinsel(arg);
		break;

	case DSPDEV_SET_COM_AREA:
		ucomar = (struct dsp_communication_area *)arg;
		if (copy_from_user(&comar, ucomar, sizeof *ucomar)) {
			DPRINT("EFAULT\n");
			err = -EFAULT;
			break;
		}
		err = dsp_set_communication_area(comar.addr, comar.size);
		break;

	case DSPDEV_GET_COM_AREA:
		ucomar = (struct dsp_communication_area *)arg;
		dsp_get_communication_area(&comar);
		if (copy_to_user(ucomar, &comar, sizeof *ucomar)) {
			DPRINT("EFAULT\n");
			err = -EFAULT;
			break;
		}
		break;

	case DSPDEV_GET_DOWNLOAD_AREA:
		dsp_get_download_area(&dlar);
		udlar = (struct dsp_download_area *)arg;
		if (copy_to_user(udlar, &dlar, sizeof dlar)) {
			DPRINT("EFAULT\n");
			err = -EFAULT;
		}
		break;

	case DSPDEV_SRAMCTRL:
		err = dsp_set_sram_control(arg);
		break;

	default:
		err = -EINVAL;
		break;
	}

error_exit:
	DPRINT("Exit [%d]\n", err);

	return err;
}


/***********************************************************************
 * Open file operationrelation
 **********************************************************************/

/* open core */
static int interdsp_open_core(struct inode *inode, struct file *file)
{
	int ch;
	int i;
	int pinfo_num = -1;
	unsigned long flags;
	int blk;
	int err = 0;

	DPRINT("Enter [driver_open_count=%d]\n", driver_open_count);

	if (driver_open_count == 0) {

		taskid0_processid = INVALID_PROCESS_ID;

		/* initialization error */
		arm_ch_error = 0;
		dsp_ch_error = 0;

		/* initialization each channel */
		for (ch = 0; ch < CH_NUM; ch++) {
			INIT_LIST_HEAD(&arm_ch_sq[ch]);
			arm_ch_sq_lock[ch] = arm_ch_sq_lock_unlock;
			sema_init(&arm_ch_sqsem[ch], INTERDSP_SQ_LEN);
			for (blk = 0; blk < BUF_BLOCK_MAX; blk++)
				sema_init(&arm_ch_block[ch][blk], 1);

			arm_ch_block_lock_bitmap[ch] = 0;
			init_waitqueue_head(&arm_ch_waitq[ch]);
			init_waitqueue_head(&dsp_ch_waitq[ch]);
			dsp_ch_recv[ch] = 0;
		}
		init_waitqueue_head(&dsp_cancel_waitq);

		/* This is unsafe. */
		if (NEED_SHMEM_INIT()
			|| xchg(&drv_mngaddr->magic, DRIVER_MAGIC) !=
				DRIVER_MAGIC) {
			err = interdsp_shmem_init();
			if (err)
				goto error_exit;
		} else {
			global_spin_lock();

			block_factor = drv_mngaddr->block_factor;
			mat_blocks = drv_mngaddr->nblocks;

			global_spin_unlock();

			shared_pe_init();
		}

	} else if (driver_open_count >= PROCESS_INFO_NUM) {
		err = -EBUSY;
		goto error_exit;
	}

	driver_open_count++;

	spin_lock_irqsave(&interdsp_opne_lock, flags);

	/* process identifier information is registered. */
	for (i = 0; i < PROCESS_INFO_NUM; i++) {
		if (process_info[i].active == 0) {
			pinfo_num = i;
			break;
		}
	}

	if (pinfo_num == -1) {
		spin_unlock_irqrestore(&interdsp_opne_lock, flags);
		err = -EBUSY;
		goto error_exit;
	}

	process_info[pinfo_num].pid = current->pid;
	process_info[pinfo_num].ppid = current->parent->pid;

	if (file->private_data)
		process_info[pinfo_num].ch =
			((struct dsp_ch_data *)file->private_data)->ch;
	else
		process_info[pinfo_num].ch = CH_INVALID;

	process_info[pinfo_num].active = 1;
	atomic_set(&dsp_cancel[pinfo_num], 0);

	if (process_info[pinfo_num].id < taskid0_processid)
		taskid0_processid = process_info[pinfo_num].id;

	spin_unlock_irqrestore(&interdsp_opne_lock, flags);

	file->private_data = (void *)&process_info[pinfo_num];

error_exit:

	DPRINT("Exit\n");

	return err;
}

/* open common function */
static int interdsp_open(struct inode *inode, struct file *file)
{
	int err;

	DPRINT("Enter\n");

	file->private_data = (void *)&any_ch_private;
	err = interdsp_open_core(inode, file);

	DPRINT("Exit [%d]\n", err);

	return err;
}


/***********************************************************************
 * Relese file operationrelation
 **********************************************************************/

/* release core */
static int interdsp_release_core(struct inode *inode, struct file *file)
{
	int ch;
	int i;
	struct arm_queue_ent *sq;
	unsigned long flags;
	struct process_data *p;
	int id = INVALID_PROCESS_ID;

	DPRINT("Enter\n");

	if (driver_open_count == 0)
		goto end;

	driver_open_count--;

	if (file->private_data) {
		p = (struct process_data *)file->private_data;
		p->active = 0;
		id = p->id;
		atomic_set(&dsp_cancel[id - 1], 0);
	}

	if (driver_open_count == 0) {

		/* disable interrupt */
		spin_lock_irqsave(&interdsp_release_lock, flags);

		taskid0_processid = INVALID_PROCESS_ID;

		for (ch = 0; ch < CH_NUM; ch++) {
			/* top ACK wait stop of transmission queue */
			sq = get_sendq_top(ch);
			if (sq != 0 && atomic_read(&sq->active))
				del_timer_sync(&sq->timeout_timer);

			/* initialization transmission queue */
			INIT_LIST_HEAD(&arm_ch_sq[ch]);
			arm_ch_sq_lock[ch] = arm_ch_sq_lock_unlock;
			sema_init(&arm_ch_sqsem[ch], INTERDSP_SQ_LEN);
			atomic_set(&arm_ch_sqcnt[ch], INTERDSP_SQ_LEN);

			/* free buffer block */
			for (i = 0; i < BUF_BLOCK_MAX; i++)
				sema_init(&arm_ch_block[ch][i], 1);
			arm_ch_block_lock_bitmap[ch] = 0;

			/* initialization interrupt relation */
			init_waitqueue_head(&arm_ch_waitq[ch]);
			init_waitqueue_head(&dsp_ch_waitq[ch]);
			dsp_ch_recv[ch] = 0;
		}
		init_waitqueue_head(&dsp_cancel_waitq);

		/* initialization error */
		arm_ch_error = 0;
		dsp_ch_error = 0;

		/* initialization transmission queue entry */
		INIT_LIST_HEAD(&free_sq);
		for (i = 0; i < INTERDSP_SQ_LEN * CH_NUM; i++) {
			init_sq_entry(&arm_send_entry[i]);
			list_add_tail(&arm_send_entry[i].list, &free_sq);
		}

		/* free channel */
		for (ch = 0; ch < CH_NUM; ch++) {
			free_chinfo(0, ch); /* ARM */
			free_chinfo(1, ch); /* DSP */
		}

		/* enable interrupt */
		spin_unlock_irqrestore(&interdsp_release_lock, flags);

		dsp_clock_gate(0);
		__raw_writel(__raw_readl(TIMER_TI3_OP) & ~0x3 , TIMER_TI3_OP);
	} else if (taskid0_processid == id) {
		/* disable interrupt */
		spin_lock_irqsave(&interdsp_release_lock, flags);

		for (i = 0; i < PROCESS_INFO_NUM; i++) {
			if (process_info[i].active) {
				taskid0_processid = process_info[i].id;
				DPRINT(" change task0 receive ID : %d \n",
						taskid0_processid);
				break;
			}
		}

		/* enable interrupt */
		spin_unlock_irqrestore(&interdsp_release_lock, flags);
	}

end:
	DPRINT("Exit [driver_open_count=%d]\n", driver_open_count);

	return 0;
}

/* release for armch[012], control */
static int interdsp_release(struct inode *inode, struct file *file)
{
	int err;

	DPRINT("Enter\n");

	if (interdsp_fasync_queue)
		interdsp_fasync(-1, file, 0);
	err = interdsp_release_core(inode, file);

	DPRINT("Exit [%d]\n", err);

	return err;
}


/***********************************************************************
 * mmap file operation
 **********************************************************************/

#define INTERDSP_MMAP_START		SDRAM_DL_RANGE_START
#define INTERDSP_MMAP_END		SDRAM_DL_RANGE_END
#define INTERDSP_MMAP_AREA_SIZE	((INTERDSP_MMAP_END+1)-INTERDSP_MMAP_START)

static int interdsp_mmap(struct file *info, struct vm_area_struct *vma)
{
	unsigned long start;
	unsigned long off;
	u32 len;
	int ret;

	DPRINT("Enter\n");

	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
		return -EINVAL;

	off = vma->vm_pgoff << PAGE_SHIFT;
	if (off < INTERDSP_MMAP_AREA_SIZE) {
		start = (unsigned long)INTERDSP_MMAP_START;
		len = PAGE_ALIGN((start & ~PAGE_MASK) +
			INTERDSP_MMAP_AREA_SIZE);
	} else
		return -EINVAL;

	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;

	off += (start & PAGE_MASK);

	/* Is even one part of the range of specification
	   piled up in the communication area? */
	if (in_dl_range(off, vma->vm_end - vma->vm_start + off,
		&sdram_dl_range[0]) == -1)
		return -ENOMEM;

	vma->vm_pgoff = off >> PAGE_SHIFT;

#ifdef CONFIG_VIDEO_EMXX
	ret = emxx_v4l2_mmap(vma);
	if (ret)
		return ret;
#else
	/* Accessing memory will be done non-cached. */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	/* To stop the swapper from even considering these pages */
	vma->vm_flags |= (VM_IO | VM_RESERVED);

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
		vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		return -EAGAIN;
	}
#endif

	DPRINT("area start=0x%08lx, size=0x%08lx\n",
		off, vma->vm_end - vma->vm_start);

	DPRINT("Exit\n");
	return 0;
}


/***********************************************************************
 * Poll file operationrelation
 **********************************************************************/

/* poll for control */
static unsigned int interdsp_poll(struct file *file, poll_table *wait)
{
	int ch;
	int state = 0;

	DPRINT("Enter\n");

	for (ch = 0; ch < CH_NUM; ch++) {
		poll_wait(file, &arm_ch_waitq[ch], wait);
		poll_wait(file, &dsp_ch_waitq[ch], wait);
		if (atomic_read(&arm_ch_sqcnt[ch]))
			state |= POLLOUT | POLLWRNORM;
		if (dsp_ch_recv[ch])
			state |= POLLIN | POLLRDNORM;
	}

	if (arm_ch_error)
		state |= POLLERR;
	if (dsp_ch_error)
		state |= POLLERR;

	DPRINT("Exit [%d]\n", state);

	return state;
}



/***********************************************************************
 * Fsync file operationrelation
 **********************************************************************/


/* fsync for control */
static int interdsp_fsync(struct file *file,
		struct dentry *dentry, int datasync)
{
	int err = 0;
	int ch;
	wait_queue_t arm_wait[CH_NUM];
	int active = 0;

	DPRINT("Enter\n");

	for (ch = 0; ch < CH_NUM; ch++) {
		init_waitqueue_entry(&arm_wait[ch], current);
		add_wait_queue(&arm_ch_waitq[ch], &arm_wait[ch]);
	}

	do {
		set_current_state(TASK_INTERRUPTIBLE);

		for (ch = 0; ch < CH_NUM; ch++) {
			if (atomic_read(&arm_ch_sqcnt[ch])
				== INTERDSP_SQ_LEN) {
				active = 1;
				break;
			}
		}

		if (active)
			break;

		if (signal_pending(current)) {
			err = -ERESTARTSYS;
			break;
		}

		schedule();
	} while (1);

	for (ch = 0; ch < CH_NUM; ch++)
		remove_wait_queue(&arm_ch_waitq[ch], &arm_wait[ch]);

	set_current_state(TASK_RUNNING);

	DPRINT("Exit [%d]\n", err);

	return err;
}



/***********************************************************************
 * Fasync file operation (asynchronous report)
 **********************************************************************/

/* fasync for control */
static int interdsp_fasync(int fd, struct file *file, int mode)
{
	int err;

	DPRINT("Enter\n");

	err = fasync_helper(fd, file, mode, &interdsp_fasync_queue);

	DPRINT("Exit [%d]\n", err);

	return err;
}


/***********************************************************************
 * for character device
 **********************************************************************/

static int dspdev_open(struct inode *inode, struct file *file)
{
	int minor = MINOR(inode->i_rdev);
	int err = -ENODEV;
	int i;

	DPRINT("Enter\n");

	switch (minor) {
	case DSPDEV_CONTROL_MINOR:
		DPRINT("minor=DSPDEV_CONTROL_MINOR\n");
		file->f_op = &control_fops;
		break;
	default:
		for (i = 0; i < INTERDSP_MINOR_SIZE; i++) {
			if (minor_device_table[i].minor == minor) {
				file->f_op = minor_device_table[i].fops;
				goto try_open;
			}
		}

		printk(KERN_INFO "inter_dsp: unknown minor device(%d)\n",
				minor);
		goto end;
	}

try_open:
	err = (*file->f_op->open)(inode, file);

end:
	DPRINT("Exit [err=%d]\n", err);

	return err;
}



/* /dev/dsp/datamgr */

static pid_t datamgr_access_pid;
static struct semaphore datamgr_access_sqsem;

static int dsp_datamgr_read(struct dspdev_cmd_data *mng_data)
{
	int ret = 0;

	if (mng_data->offset + mng_data->size > DSP_DATAMGR_SIZE)
		return -EINVAL;

	if (down_interruptible(&datamgr_buf_sqsem))
		return -ERESTARTSYS;

	if (datamgr_access_pid != current->pid) {
		ret = -EACCES;
		goto error;
	}

	if (copy_to_user(mng_data->buf,
	      (void *)(dsp_datamgr_buf + mng_data->offset), mng_data->size)) {
		return -EFAULT;
	}

error:
	up(&datamgr_buf_sqsem);

	return ret;
}

static int dsp_datamgr_write(struct dspdev_cmd_data *mng_data)
{
	int ret = 0;

	if (mng_data->offset + mng_data->size > DSP_DATAMGR_SIZE)
		return -EINVAL;

	if (down_interruptible(&datamgr_buf_sqsem))
		return -ERESTARTSYS;

	if (datamgr_access_pid != current->pid) {
		ret = -EACCES;
		goto error;
	}

	if (copy_from_user((void *)(dsp_datamgr_buf + mng_data->offset),
					mng_data->buf, mng_data->size)) {
		return -EFAULT;
	}

error:
	up(&datamgr_buf_sqsem);

	return ret;
}

static int dsp_datamgr_crement(struct dspdev_cmd_data *mng_data, int crement)
{
	int ret = 0;
	unsigned long *buf;

	/* 32bit access */
	if ((mng_data->offset + 4 > DSP_DATAMGR_SIZE) ||
		(mng_data->size < 4))
		return -EINVAL;

	if (down_interruptible(&datamgr_buf_sqsem))
		return -ERESTARTSYS;

	if (datamgr_access_pid != current->pid) {
		ret = -EACCES;
		goto error;
	}

	buf = (unsigned long *)(dsp_datamgr_buf + mng_data->offset);

	if (crement)
		(*buf)++;
	else
		(*buf)--;

	if (copy_to_user(mng_data->buf, (void *)buf, 4))
		ret = -EFAULT;
error:
	up(&datamgr_buf_sqsem);

	return ret;
}

static int dsp_datamgr_lock(void)
{
	if (down_interruptible(&datamgr_access_sqsem))
		return -ERESTARTSYS;

	datamgr_access_pid = current->pid;

	return 0;
}


static int dsp_datamgr_unlock(void)
{
	if (datamgr_access_pid == current->pid) {
		datamgr_access_pid = 0;
		up(&datamgr_access_sqsem);
	} else {
		return -EACCES;
	}

	return 0;
}


static int dsp_datamgr_open(struct inode *inode, struct file *file)
{
	struct dspdev_driver *p = &dsp_datamgr_private;

	if (p->open_cnt++ == 0) {
		datamgr_access_pid = 0;
		sema_init(&datamgr_buf_sqsem, 1);
		sema_init(&datamgr_access_sqsem, 1);
		memset(dsp_datamgr_buf, 0, DSP_DATAMGR_SIZE);
	}

	file->private_data = p;
	return 0;
}

static int dsp_datamgr_release(struct inode *inode, struct file *file)
{
	struct dspdev_driver *p = file->private_data;
	p->open_cnt--;
	return 0;
}

static int dsp_datamgr_ioctl(struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	int err = -EINVAL;
	struct dspdev_cmd_data mng_data;

	switch (cmd) {
	case DSPDEV_DATA_READ:
		if (copy_from_user((void *)&mng_data,
				(void *)arg, sizeof(mng_data))) {
			err = -EFAULT;
			break;
		}
		err = dsp_datamgr_read(&mng_data);
		break;
	case DSPDEV_DATA_WRITE:
		if (copy_from_user((void *)&mng_data,
				(void *)arg, sizeof(mng_data))) {
			err = -EFAULT;
			break;
		}
		err = dsp_datamgr_write(&mng_data);
		break;

	case DSPDEV_DATA_INCREMENT:
		if (copy_from_user((void *)&mng_data,
				(void *)arg, sizeof(mng_data))) {
			err = -EFAULT;
			break;
		}
		err = dsp_datamgr_crement(&mng_data, 1);
		break;
	case DSPDEV_DATA_DECREMENT:
		if (copy_from_user((void *)&mng_data,
				(void *)arg, sizeof(mng_data))) {
			err = -EFAULT;
			break;
		}
		err = dsp_datamgr_crement(&mng_data, 0);
		break;

	case DSPDEV_DATA_LOCK:
		err = dsp_datamgr_lock();
		break;

	case DSPDEV_DATA_UNLOCK:
		err = dsp_datamgr_unlock();
		break;

	default:
		break;
	}

	return err;
}


/***********************************************************************
 * interrupt hander form DSP
 **********************************************************************/

/* Res/Req tasklet */
static void dsp_req_handler(unsigned long ch)
{
	DPRINT("Enter [ch=%lu]\n", ch);

	DPRINT("[CH=%ld] wakeup task\n", ch);

	/* There is res/req from DSP */
	dsp_ch_recv[ch] = 1;

	/* wake up child process */
	wake_up_interruptible(&dsp_ch_waitq[ch]);

	/* for "control". start one-shot in all channel */
	if (atomic_dec_and_test(&dsp_rcv_xcount)) {
		/* for "control" FASYNC support */
		if (interdsp_fasync_queue != 0)
			kill_fasync(&interdsp_fasync_queue, SIGIO, POLL_IN);
	}

	DPRINT("Exit\n");
}


/* interrupt process */
static irqreturn_t interdsp_interrupt(int irq, void *dev_id)
{
	int ch;
	unsigned int ack;
	unsigned int res;
	unsigned int mon;
	int run = 0;

	DPRINT("Enter\n");

	atomic_set(&dsp_rcv_xcount, 1);

	ack = ACK_INT(0);
	res = SEND_INT(0);

	mon = __raw_readl(ipi_mon);

	for (ch = 0; ch < CH_NUM; ch++) {
		if (mon & ack) {
#ifdef CONFIG_PM
			emxx_pm_pdma_suspend_enable();
#endif

			/* scheduling for transmission processing. */
			run = 0;
			spin_lock(&arm_ch_sq_lock[ch]);
			if (!list_empty(&arm_ch_sq[ch])) {
				struct arm_queue_ent *sq;
				sq = list_entry(arm_ch_sq[ch].next,
						struct arm_queue_ent, list);
				if (atomic_read(&sq->active))
					run = 1;
			}
			spin_unlock(&arm_ch_sq_lock[ch]);
			if (run) {
				tasklet_enable(arm_ch_tasklet[ch]);
				tasklet_schedule(arm_ch_tasklet[ch]);
				DPRINT("DSP CH%d ACK scheduled\n", ch);
			}
#ifdef EMEV_DSP_DEBUG
			else
				DPRINT("DSP CH%d ACK Ignore\n", ch);
#endif	/* EMEV_DSP_DEBUG */
			recv_ack_count[ch]++;
		}
		if (mon & res) {
#ifdef CONFIG_PM
			emxx_pm_pdma_suspend_disable();
#endif

			/* handling */
			if (driver_open_count) {
				DPRINT("DSP CH%d handled Res/Req\n", ch);
				tasklet_schedule(dsp_ch_tasklet[ch]);
			} else {
				/* ACK is immediately returned
				   disregarding Res/Req */
				__raw_writel(ack, ipi_set);
				send_ack_count[ch]++;
#ifdef CONFIG_PM
				emxx_pm_pdma_suspend_enable();
#endif

				DPRINT("DSP CH%d ignore Res/Req\n", ch);
			}
			recv_resreq_count[ch]++;
		}
		ack <<= CH_INT_BITSHIFT;
		res <<= CH_INT_BITSHIFT;
	}

	__raw_writel(mon, ipi_clr);

	DPRINT("Exit\n");

	return IRQ_HANDLED;
}


/***********************************************************************
 * DPM
 **********************************************************************/
static int inter_dsp_suspend(struct platform_device *dev, pm_message_t state)
{
	int err = 0;
	int i;

	DPRINT("Enter [state=%d]\n", state.event);

	switch (state.event) {
	case PM_EVENT_SUSPEND:
		if (driver_open_count) {
			if (((__raw_readl(SMU_CLKSTOPSIG_ST) >> 4) & 0x3)
				!= 0x3) {
				printk(KERN_INFO "DSP busy\n");
				err = -EBUSY;
				goto error_exit;
			}
		}

		if (!power_control_retention) {
			for (i = 0; i < 16; i++) {
				dcv_reg_state[i].bank_no = i;
				dcv_get_bank_info_reg(&dcv_reg_state[i]);
			}
			dcv_store = 1;
		}
		break;

	default:
		break;
	}

error_exit:

	DPRINT("Exit [err=%d]\n", err);

	return err;
}

static int inter_dsp_resume(struct platform_device *dev)
{
	int err = 0;
	int i;

	DPRINT("Enter\n");

	if ((dcv_store == 1) && !power_control_retention) {
		for (i = 0; i < 16; i++)
			dcv_set_bank_info_reg(&dcv_reg_state[i]);

		dcv_store = 0;
	}

	DPRINT("Exit [err=%d]\n", err);

	return err;
}


/***********************************************************************
 * initialization
 **********************************************************************/

static void setup_download_ranges(void)
{
	sdram_dl_range[1].start = SDRAM_DL_RANGE_START;
	sdram_dl_range[1].end = SDRAM_DL_RANGE_END + 1;
	sdram_dl_range[1].valid = 1;
	sdram_dl_range[1].used = 1;
	return;
}

static int __init interdsp_probe(struct platform_device *dev)
{
	inter_dsp_device = &dev->dev;

	return 0;
}


static int __init interdsp_init(void)
{
	int i;
	int error = 0;

	unsigned long flags;
	unsigned int val;
	dev_t devno;
#ifdef CONFIG_MACH_EMEV
	int pd_power_auto_control = PD_POWER_AUTO_CONTROL;
#endif

	DPRINT("Enter\n");

	for (i = 0; i < INTERDSP_MINOR_SIZE; i++) {
		minor_device_table[i].minor = -1;
		minor_device_table[i].fops = 0;
		minor_device_table[i].sticky = 0;
		minor_device_table[i].cls_dev = 0;
	}

	/* channel initialize */
	for (i = 0; i < CH_NUM; i++) {
		int blk;
		INIT_LIST_HEAD(&arm_ch_sq[i]);
		arm_ch_sq_lock[i] = arm_ch_sq_lock_unlock;
		sema_init(&arm_ch_sqsem[i], INTERDSP_SQ_LEN);
		for (blk = 0; blk < BUF_BLOCK_MAX; blk++)
			sema_init(&arm_ch_block[i][blk], 1);
		arm_ch_block_lock_bitmap[i] = 0;
		init_waitqueue_head(&arm_ch_waitq[i]);
		init_waitqueue_head(&dsp_ch_waitq[i]);
		dsp_ch_recv[i] = 0;
	}
	init_waitqueue_head(&dsp_cancel_waitq);

	if (dspdev_major) {
		devno = MKDEV(dspdev_major, 0);
		error = register_chrdev_region(devno, DSP_MINOR_MAX,
						DSP_DEVICE_NAME);
	} else {
		error = alloc_chrdev_region(&devno,
						0, DSP_MINOR_MAX,
						DSP_DEVICE_NAME);
		dspdev_major = MAJOR(devno);
	}
	if (error) {
		printk(KERN_ERR
				"interdsp: Can't allocate chrdev=%d, "
				"error=%d\n", dspdev_major, error);
		goto error_exit;
	}

	cdev_init(&dspdev_cdev, &dspdev_fops);
	error = cdev_add(&dspdev_cdev, devno, DSP_MINOR_MAX);
	if (error)
		goto unregist_chrdev;

#if 0
		printk(KERN_NOTICE "InterDSP Driver for EMXX Version "
			INTERDSP_DRIVER_VERSION ": major=%d\n"
			"Target: Evo\n"
			"Options:"
#if FORCE_INIT
			" FORCE_INIT"
#endif /* FORCE_INIT */
			" DCV"
			"\n", dspdev_major);
#endif	/* 0 */

	inter_dsp_class = class_create(THIS_MODULE, DSP_DEVICE_NAME);
	if (IS_ERR(inter_dsp_class)) {
		printk(KERN_ERR "Error creating dsp class.\n");
		inter_dsp_class = 0;
		goto chrdev_del;
	}

	platform_driver_register(&inter_dsp_driver);

	for (i = 0; i < ARRAY_SIZE(dsp_procfs); i++) {
		int devno = MKDEV(dspdev_major, dsp_procfs[i].minor);
		struct device *cd;
		cd = device_create(inter_dsp_class, inter_dsp_device,
				devno, NULL, "%s", dsp_procfs[i].name);

		if (IS_ERR(cd)) {
			goto dev_dest;
		} else {
			minor_device_table[i].cls_dev = cd;
			minor_device_table[i].minor = dsp_procfs[i].minor;
			minor_device_table[i].fops = dsp_procfs[i].fops;
			minor_device_table[i].sticky = 1;
		}
	}

	/* get register address */
	ipi_mon = IT3_IPI0_MON;
	ipi_clr = IT3_IPI0_CLR;
	ipi_set = IT0_IPI3_SET;

	/* initialization transmission queue */
	INIT_LIST_HEAD(&free_sq);
	for (i = 0; i < INTERDSP_SQ_LEN * CH_NUM; i++) {
		init_sq_entry(&arm_send_entry[i]);
		list_add_tail(&arm_send_entry[i].list, &free_sq);
	}

	/* process identifier information initialization */
	for (i = 0; i < PROCESS_INFO_NUM; i++) {
		process_info[i].id = i + 1;		/* 1 - 128 */
		process_info[i].pid = 0;
		process_info[i].ppid = 0;
		process_info[i].active = 0;
		process_info[i].ch = CH_INVALID;
	}

	/* read-cancel-flag initialization */
	for (i = 0; i < PROCESS_INFO_NUM; i++)
		atomic_set(&dsp_cancel[i], 0);

	/* set interrupt */
	error = request_irq(INT_DSP,
				interdsp_interrupt, IRQF_DISABLED,
				"interdsp", (void *)0);
	if (error) {
		DPRINT("Error: request_irq() = %d\n", error);
		goto dev_dest;
	}

	DPRINT("IRQ Setup End\n");

	setup_download_ranges();

	error = dsp_set_communication_area(shared_mem_address, shared_mem_size);
	if (error)
		goto irq_release;

	spin_lock_irqsave(&interdsp_init_lock, flags);

	/* TI3 setup */
	val = __raw_readl(SMU_TI3TIN_SEL);
	if (val == 0) {
		/* TW3/TI3 is RTC */
		__raw_writel(0x00010001, SMU_TI3TIN_SEL);
	}


#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1) {
		if ((__raw_readl(SMU_DEBUG_EN_STATUS) == 1) ||
			(pd_power_auto_control == PD_POWER_AUTO_CONTROL_OFF)) {
			power_control_enable = 0;
		}
	}
#endif

	/*
	 * DSP Power off
	 */
#if 0
	while (__raw_readl(SMU_SEQ_BUSY) & PD_SEQ_BUSY)
		udelay(5);

	/* Power OFF sequence start */
	val = __raw_readl(SMU_PD_SWON);
	val &= ~PD_SWON;
	val |= PD_PDON; /* Power down mode */
	__raw_writel(val, SMU_PD_SWON);

	while (__raw_readl(SMU_SEQ_BUSY) & PD_SEQ_BUSY)
		udelay(5);
#endif

	spin_unlock_irqrestore(&interdsp_init_lock, flags);

	memset(dsp_datamgr_buf, 0, DSP_DATAMGR_SIZE);

	DPRINT("EXIT: Success\n");

	return 0;

irq_release:
	free_irq(INT_DSP, (void *)0);

dev_dest:
	for (i = 0; i < ARRAY_SIZE(dsp_procfs); i++) {
		if (minor_device_table[i].cls_dev)
			device_destroy(inter_dsp_class,
				MKDEV(dspdev_major, dsp_procfs[i].minor));
	}

	platform_driver_unregister(&inter_dsp_driver);
/* class_dest: */
	class_destroy(inter_dsp_class);
	inter_dsp_class = 0;
chrdev_del:
	cdev_del(&dspdev_cdev);

unregist_chrdev:
	unregister_chrdev_region(MKDEV(dspdev_major, 0), DSP_MINOR_MAX);

error_exit:
	if (error)
		DPRINT("EXIT: Error = %d\n", error);

	DPRINT("Exit\n");

	return error;
}

/***********************************************************************
 * finished process
 **********************************************************************/

static void __exit interdsp_exit(void)
{
	int i;
	int ch;
	struct arm_queue_ent *sq;

	DPRINT("Enter\n");

	/* free interrupt */
	free_irq(INT_DSP, (void *)0);

	platform_driver_unregister(&inter_dsp_driver);

	for (i = 0; i < INTERDSP_MINOR_SIZE; i++) {
		if (minor_device_table[i].minor != -1
			&& minor_device_table[i].fops != 0
			&& minor_device_table[i].sticky == 0
			&& minor_device_table[i].cls_dev != 0
			) {
			device_destroy(inter_dsp_class,
				MKDEV(dspdev_major, dsp_procfs[i].minor));
			minor_device_table[i].minor = -1;
			minor_device_table[i].fops = 0;
		}
	}

	for (i = 0; i < ARRAY_SIZE(dsp_procfs); i++) {
		device_destroy(inter_dsp_class,
			MKDEV(dspdev_major, dsp_procfs[i].minor));
		minor_device_table[i].cls_dev = 0;

	}

	class_destroy(inter_dsp_class);
	inter_dsp_class = 0;

	cdev_del(&dspdev_cdev);

	unregister_chrdev_region(MKDEV(dspdev_major, 0), DSP_MINOR_MAX);

	/* stop tasklet */
	for (ch = 0; ch < CH_NUM; ch++)
		tasklet_disable(arm_ch_tasklet[ch]);

	/* timer stop of time out of ACK wait */
	for (ch = 0; ch < CH_NUM; ch++) {
		sq = get_sendq_top(ch);
		if (sq == 0)
			continue;
		if (atomic_read(&sq->active))
			del_timer_sync(&sq->timeout_timer);
	}

	dsp_release_communication_area();

	DPRINT("Exit\n");
}


/***********************************************************************
 * module setting
 **********************************************************************/
module_init(interdsp_init);
module_exit(interdsp_exit);

/* ACK time out setting (milli second ) */
module_param(ack_timeout, ulong, 0);
MODULE_PARM_DESC(ack_timeout, "ACK Timeout[ms]"
		  " [default " __stringify(INTERDSP_ACK_TIMEOUT) "ms]");

/* force initialization */
module_param(force_init, int, 0);
MODULE_PARM_DESC(force_init, "Force initialize"
		  " [default " __stringify(FORCE_INIT) "]");

MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_DESCRIPTION("EMXX SPXK7 DSP Driver");
MODULE_LICENSE("GPL");
