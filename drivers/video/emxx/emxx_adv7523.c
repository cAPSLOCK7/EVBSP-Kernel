/*
 * File Name       : drivers/video/emxx/emxx_adv7523.c
 * Function        : EM/EV HDMI ADV7523 interface
 * Release Version : Ver 1.00
 * Release Date    : 2010/04/01
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

#include <linux/init.h> /* __init */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mm.h>      /* remap_page_range */
#include <linux/slab.h> /* kmalloc */
#include <linux/poll.h> /* POLLIN */
#include <linux/interrupt.h> /* tasklet */
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/io.h>  /* ioremap */
#include <linux/i2c.h>
#include <linux/irq.h>
#include <mach/irqs.h> /* request_irq */
#include <mach/hardware.h> /* HDMI_ADV7523 base address */
#include <mach/pmu.h> /* clock */
#include <mach/smu.h>
#include <mach/gpio.h>
#include <mach/emxx_hdmi_adv7523.h>

/*
 * debug functions
 */
#ifdef	CONFIG_EMXX_HDMI_ADV7523_DEBUG
#include <stdarg.h>

/* debug functions --------------------------------------------------------- */

static int debug_level;
static const char debug_prefix[] = "";
#define debug_printf printk

static int debug_nowstring(char *buf, int len)
{

	buf[0] = '\0';
	return 0;
}


static void __debug_np(int level, const char *format, ...)
{
	char sbuf[128];
	va_list args;
	if (debug_level < level)
		return;

	va_start(args, format);
	vsnprintf(sbuf, sizeof(sbuf), format, args);
	va_end(args);

	debug_printf("%s%s\n", debug_prefix, sbuf);
}

static void __debug(int level, const char *function, const char *format, ...)
{
	char sbuf[128];
	char times[20];
	va_list args;
	if (debug_level < level)
		return;

	va_start(args, format);
	vsnprintf(sbuf, sizeof(sbuf), format, args);
	va_end(args);

	debug_nowstring(times, sizeof(times));
	debug_printf("%s%s%-16s: %s\n", debug_prefix, times, function, sbuf);
}

static void __dump(int level, const char *function, void *ptr, int len)
{
	unsigned char *p = (unsigned char *)ptr;
	char sbuf[128];
	int i;
	if (debug_level < level)
		return;

	__debug(level, function, "length=%d", len);
	if (len > 32)
		len = 32;

	for (i = 0; i < len;) {
		int nl = 16;
		char *sbufp;

		sbufp = sbuf + sprintf(sbuf, " %04X: ", (unsigned int)(i));
		for (; i < len && nl > 0; i++, nl--)
			sbufp += sprintf(sbufp, "%02X ", p[i]);
		debug_printf("%s\n", sbuf);
	}
}
#endif	/* CONFIG_EMXX_HDMI_ADV7523_DEBUG */

#include "emxx_adv7523.h"

#define HDMI_ADV7523_MODNAME     "emxx_hdmi_adv7523"
#define HDMI_ADV7523_DEVNAME     "hdmi"
#define HDMI_ADV7523_MINOR_MAX   1

#define HDMI_ADV752_MAJOR 0

static char *devname = HDMI_ADV7523_DEVNAME; /* !< default device file name */
static int devmajor = HDMI_ADV752_MAJOR;  /* !< default device major number */

static struct mutex hdmi_adv7523_mutex;

/* !< this driver information */
static struct hdmi_adv7523_info_t s_hdmi_adv7523_info;
struct hdmi_adv7523_info_t *hdmi_adv7523_info = &s_hdmi_adv7523_info;

static DECLARE_WAIT_QUEUE_HEAD(intwaitq);

#if 0
static DECLARE_WAIT_QUEUE_HEAD(readq);
#endif

#ifdef CONFIG_PM
static int hdmi_adv7523_pf_suspend(struct platform_device *dev,
 pm_message_t state);
static int hdmi_adv7523_pf_resume(struct platform_device *dev);
#endif
static int hdmi_adv7523_pf_probe(struct platform_device *dev);
static int hdmi_adv7523_pf_remove(struct platform_device *dev);

static void hdmi_adv7523_pf_release(struct device *dev);

static struct class *hdmi_adv7523_class;
static struct cdev hdmi_adv7523_cdev;
static struct device *hdmi_adv7523_class_device;

static struct platform_device hdmi_adv7523_pf_device = {
	.name = HDMI_ADV7523_MODNAME,
	.id = -1,
	.dev = {
		.release = hdmi_adv7523_pf_release,
	},
};

static struct platform_driver hdmi_adv7523_pf_driver = {
	.probe = hdmi_adv7523_pf_probe,
	.remove = hdmi_adv7523_pf_remove,
#ifdef CONFIG_PM
	.suspend = hdmi_adv7523_pf_suspend,
	.resume = hdmi_adv7523_pf_resume,
#endif
	.driver = {
		.name  = HDMI_ADV7523_MODNAME,
		.owner = THIS_MODULE,
	},
};

#define RETRY_COUNT(x)  ((loops_per_jiffy * x)/(5000/HZ))

/* prototypes for I2C client definition */

#define ADV7523_WRITE(a, d, m)    i2c_hdmi_write(a, d, m)
#define ADV7523_READ(a, d)    	  i2c_hdmi_read(a, d)

#define	ADV7523REG_ST	0x42
#define	ADV7523REG_MAX	0xfe

static unsigned char i2c_hdmi_reg[ADV7523REG_MAX];
static int i2c_hdmi_init_done;
static int i2c_hdmi_read(unsigned char reg, unsigned char *data);
static int hdmi_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *id);
static int hdmi_i2c_remove(struct i2c_client *client);

static struct i2c_device_id hdmi_i2c_idtable[] = {
	{ I2C_SLAVE_HDMI_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hdmi_i2c_idtable);

static struct i2c_driver i2c_hdmi_driver = {
	.driver.name    = "i2c for hdmi",
	.id             = I2C_DRIVERID_I2CDEV, /* Fake ID */
	.id_table       = hdmi_i2c_idtable,
	.probe          = hdmi_i2c_probe,
	.remove         = hdmi_i2c_remove,
};
static struct i2c_client *i2c_hdmi_client;

/*
 * i2c functions
 */
static int hdmi_i2c_probe(struct i2c_client *client,
 const struct i2c_device_id *id)
{
	i2c_hdmi_client = client;
	return 0;
}

static int hdmi_i2c_remove(struct i2c_client *client)
{
	i2c_hdmi_client = NULL;
	return 0;
}

static int i2c_hdmi_cleanup(void)
{
	i2c_del_driver(&i2c_hdmi_driver);
	i2c_hdmi_init_done = 0;
	return 0;
}

static int i2c_hdmi_inserted(void)
{
	return i2c_add_driver(&i2c_hdmi_driver);
}

static int i2c_hdmi_init(void)
{
	int res = 0;

	if (i2c_hdmi_init_done != 0)
		return 0;

	res = i2c_hdmi_inserted();
	if (res == 0) {
		if (i2c_hdmi_client == NULL) {
			i2c_hdmi_cleanup();
			printk(KERN_ERR "hdmi adv7523 i2c_init failed\n");
			return -EIO;
		}
	} else {
		printk(KERN_ERR "i2c hdmi adv7523 inserted failed!\n");
	}
	i2c_hdmi_init_done = -1;

	return res;
}

static int i2c_hdmi_write(unsigned char reg, unsigned char data,
			   unsigned char mask)
{
	int res = 0;
	unsigned char buf[2];

	if ((ADV7523REG_MAX <= reg) || ((data & ~(mask)) != 0))
		return -EINVAL;

	data = (i2c_hdmi_reg[reg] & ~(mask)) | (data & mask);

	i2c_hdmi_reg[reg] = data;

	buf[0] = reg;
	buf[1] = data;

	res = i2c_master_send(i2c_hdmi_client, buf, 2);
	if (res > 0)
		res = 0;
	else
		printk(KERN_ERR "i2c hdmi write failed!\n");

	return res;
}

static int i2c_hdmi_read(unsigned char reg, unsigned char *data)
{
	int res = 0;
#if 0
	unsigned char buf = 0;
#endif

	if (i2c_hdmi_client == NULL) {
		printk(KERN_ERR "i2c hdmi not available!\n");
		return -EIO;
	}

	if (ADV7523REG_MAX <= reg)
		return -EINVAL;

#if 0
	res = i2c_master_send(i2c_hdmi_client, &buf, 1);
	if (res <= 0) {
		printk(KERN_ERR "i2c hdmi send failed!\n");
		return res;
	}
#endif

	res = i2c_master_recv(i2c_hdmi_client, i2c_hdmi_reg, ADV7523REG_MAX);
	if (res > 0) {
		*data = i2c_hdmi_reg[reg];
		res = 0;
	} else {
		printk(KERN_ERR "i2c hdmi recv failed!\n");
	}

	return res;
}

/* controling ADV7523 via I2C */

static	int	hdmi_adv7523_reset_seq[] = {
	0xf8, 0x00,
	0xf8, 0x80,
	-1, -1,
	-1, -1,
};

/*!
 * reset HDMI_ADV7523 module
 * @param void
 * @retval 0 successful
 * @retval -1 failed
 */
static int hdmi_adv7523_reset(void)
{
	int res = 0;
	int i;

	mutex_lock(&hdmi_adv7523_mutex);

	res = i2c_hdmi_init();
	if (res != 0) {
		debug1("hdmi adv7523 \'i2c_hdmi_init on reset\' failed\n");
		goto err1;
	}

	/* Reset Sequence */
	i = 0;
	while (hdmi_adv7523_reset_seq[i] >= 0) {
		res = ADV7523_WRITE(hdmi_adv7523_reset_seq[i],
				    hdmi_adv7523_reset_seq[i+1],
				    0xff);
		debug2("hdmi i2c %x %x %x\n", res,
			hdmi_adv7523_reset_seq[i],
			hdmi_adv7523_reset_seq[i+1]);
		i += 2;
		if (res < 0)
			goto err1;
	}

	debug1("hdmi adv7523 reset\n");
	mutex_unlock(&hdmi_adv7523_mutex);

	return res;

err1:
	mutex_unlock(&hdmi_adv7523_mutex);

	return res;
}


static	int	hdmi_adv7523_seq[] = {
	0x41, 0x10,
	0xeb, 0x46,
	0xe6, 0xfe,
	0xe5, 0x80,
	0xe4, 0x6c,
	0xde, 0x82,
	0xd6, 0x80,
	0xba, 0x60,
	0xaf, 0x06,
	0xa3, 0xb0,
	0xa2, 0xb0,
	0x9f, 0x01,
	0x9d, 0x61,
	0x9c, 0x38,
	0x98, 0x03,
	0x56, 0x08,
	0x55, 0x40,
	0x18, 0x06,
	0x17, 0x02,
	0x16, 0x3c,
	0x15, 0x22,
	0x0b, 0x8e,
	0x03, 0x00,
	0x02, 0x18,
	0x01, 0x00,
	0xBA, 0xE0,

	-1, -1,
	-1, -1,
};

#define	WAIT_CONNECTION_IN_OPEN 1

#ifdef	WAIT_CONNECTION_IN_OPEN
/*!
 * kickoff HDMI_ADV7523 module
 * @param void
 */
static int hdmi_adv7523_connect(void)
{
	if (hdmi_adv7523_info->intr == 0)
		return 0;
	else
		return -1;
}
#endif


/*!
 * start HDMI_ADV7523 module
 * @param void
 * @retval 0 successful
 * @retval -1 failed
 */
static int hdmi_adv7523_power_on(void)
{
	int res = 0;
	int i;
	unsigned char state = 0;

	mutex_lock(&hdmi_adv7523_mutex);
	if (hdmi_adv7523_info->poweron_state) {
		mutex_unlock(&hdmi_adv7523_mutex);
		return 0;
	}

#if 0
	outl(SMU_PLLSEL_OSC1 | SMU_DIV(2), SMU_REFCLKDIV);
	emxx_open_clockgate(EMXX_CLK_REF);
#endif

	res = i2c_hdmi_init();
	if (res != 0) {
		debug1("hdmi adv7523 i2c_hdmi_init failed\n");
		goto err1;
	}

#ifdef	WAIT_CONNECTION_IN_OPEN
	debug1("hdmi adv7523 waiting\n");
	while (hdmi_adv7523_info->intr == 0) {
		if (wait_event_interruptible(intwaitq,
					     hdmi_adv7523_connect())) {
			debug1("hdmi adv7523 wakeup sequence on open()\n");
			hdmi_adv7523_info->intr = 0;
			res = -ERESTARTSYS;
			goto err1;
		}
		debug1("hdmi adv7523 wokeup %d\n", hdmi_adv7523_info->intr);
	}
	hdmi_adv7523_info->intr = 0;
#endif

	i = ADV7523_READ(ADV7523REG_ST, &state);
	if (i != 0 || (state & 0x40) == 0) {
		printk(KERN_ERR "HDMI opened before connect!\n");
		/* pass through */
	}
	debug1("hdmi adv7523 state %x\n", state);

	/* Set Up Sequence */
	i = 0;
	while (hdmi_adv7523_seq[i] >= 0) {
		res = ADV7523_WRITE(hdmi_adv7523_seq[i],
				    hdmi_adv7523_seq[i+1],
				    0xff);
		debug2("hdmi i2c %x %x %x\n", res, hdmi_adv7523_seq[i],
		       hdmi_adv7523_seq[i+1]);
		i += 2;
		if (res < 0)
			goto err1;
	}

	hdmi_adv7523_info->poweron_state++;
	debug1("hdmi adv7523 power on %d\n", hdmi_adv7523_info->poweron_state);
	mutex_unlock(&hdmi_adv7523_mutex);

	return res;

err1:
	mutex_unlock(&hdmi_adv7523_mutex);

	return res;
}

static	int	hdmi_adv7523_stop_seq[] = {
	0x41, 0x50,
	-1, -1,
	-1, -1,
};

/*!
 * stop HDMI_ADV7523 module
 * @param void
 * @retval 0 successful
 * @retval -1 failed
 */
static int hdmi_adv7523_power_off(void)
{
	int res = 0;
	int i;

	mutex_lock(&hdmi_adv7523_mutex);
	if (hdmi_adv7523_info->poweron_state == 0) {
		mutex_unlock(&hdmi_adv7523_mutex);
		return 0;
	}

	i = 0;
	while (hdmi_adv7523_stop_seq[i] >= 0) {
		res = ADV7523_WRITE(hdmi_adv7523_stop_seq[i],
			hdmi_adv7523_stop_seq[i+1], 0xff);
		debug2("hdmi i2c %x %x %x\n", res,
			hdmi_adv7523_stop_seq[i],
			hdmi_adv7523_stop_seq[i+1]);
		i += 2;
		if (res < 0)
			goto out;
	}

	hdmi_adv7523_reset();

	i2c_hdmi_cleanup();

#if 0
	emxx_close_clockgate(EMXX_CLK_REF);
#endif

	hdmi_adv7523_info->poweron_state--;
	debug1("hdmi adv7523 power off %d", hdmi_adv7523_info->poweron_state);
out:
	mutex_unlock(&hdmi_adv7523_mutex);

	return res;
}


/* file operations ----------------------------------------------------------*/

/*!
 * read file operation
 * @param[in] filp
 * @param[out] buf to copy struct hdmi_adv7523_interrupt_t
 * @param[in] count buf size
 * @param[in] offp
 * @retval size size of struct hdmi_adv7523_interrupt_t
 * @retval -EINVAL count error
 */
static ssize_t hdmi_adv7523_read(struct file *filp, char *buf,
			size_t count, loff_t *offp)
{
#ifdef	CONFIG_EMXX_HDMI_ADV7523_DEBUG
	int minor = MINOR(filp->f_dentry->d_inode->i_rdev);
#endif

	debug1("minor %d, count %d", minor, count);

	return -EINVAL;
}

/*!
 * select file operation
 * @param[in] filp
 * @param[in] wait
 * @retval mask POLLIN | POLLRDNORM
 */
static unsigned int hdmi_adv7523_poll(struct file *filp,
			     struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	debug2("minor %d", MINOR(filp->f_dentry->d_inode->i_rdev));

#if 0
	poll_wait(filp, &readq, wait);
#endif

	mask |= POLLIN | POLLRDNORM;
#if 0
	if (hdmi_adv7523_info->interrupt.interrupt.l)
		mask |= POLLIN | POLLRDNORM;
#endif

	debug2("mask 0x%X", mask);
	return mask;
}

/*!
 * ioctl file operation
 * @param[in] inode
 * @param[in] filp
 * @param[in] cmd HDMI_ADV7523_IOCQBUFSIZE or HDMI_ADV7523_IOCSREGCMD
 * @param[in] arg struct hdmi_adv7523_command_t with HDMI_ADV7523_IOCSREGCMD
 * @retval size for HDMI_ADV7523_IOCQBUFSIZE
 * @retval 0 command successful
 * @retval -EINVAL command error
 */
static int hdmi_adv7523_ioctl(struct inode *inode, struct file *filp,
		     unsigned int cmd, unsigned long arg)
{
#if 0
	enum EMXX_HDMI_OUTPUT_MODE ra;
#endif
	int ret = 0;

	switch (cmd) {
	case EMXX_HDMI_GET_OUTPUT:
		debug1("EMXX_HDMI_GET_OUTPUT");

		if (copy_to_user((void *)arg, &hdmi_adv7523_info->resolution,
				sizeof(hdmi_adv7523_info->resolution))) {
			debug0("copy_to_user failed");
			ret = -EFAULT;
			break;
		}
		break;

	case EMXX_HDMI_SET_OUTPUT:
		debug1("EMXX_HDMI_SET_OUTPUT");
		ret = -EIO; /* ENOTTY */
		if (arg == 0) {
			ret = -EINVAL;
			break;
		}
#if 0
		if (copy_from_user(&ra, (void *)arg, sizeof(ra))) {
			debug0("copy_from_user failed");
			ret = -EFAULT;
			break;
		}
		hdmi_adv7523_power_off();
		hdmi_adv7523_info->resolution = (enum EMXX_HDMI_OUTPUT_MODE)ra;
		hdmi_adv7523_power_on();
		ret = 0;
#endif
		break;


	default:
		debug1("unknown cmd 0x%08X, arg 0x%08lX", cmd, arg);
		ret = -EINVAL; /* ENOTTY */
		break;
	}

	debug1("ret %d", ret);
	return ret;
}

/*!
 * mmap file operation
 * @param[in] filp
 * @param[in] vma
 * @retval 0 successful
 * @retval -EINVAL size error
 */
static int hdmi_adv7523_mmap(struct file *filp, struct vm_area_struct *vma)
{
#if 0
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long info_size = PAGE_ALIGN(sizeof(
	 struct hdmi_adv7523_common_info_t));
	unsigned long info_pa;

	info_pa = (unsigned long)hdmi_adv7523_info->pa_common_info;

	debug1("vm_start 0x%08lX", vma->vm_start);
	debug1("vm_end   0x%08lX", vma->vm_end);
	debug1("size     %lu", size);
	debug1("vm_pgoff 0x%08lX", vma->vm_pgoff);
	debug1("vm_page_prot 0x%08lX", pgprot_val(vma->vm_page_prot));

	if (size > info_size) {
		debug1("error request size %lu > allocated size %lu",
		       size, info_size);
		return -EINVAL;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start, info_pa >> PAGE_SHIFT, size,
			     vma->vm_page_prot)) {
		debug1("remap_pfn_range failed");
		return -EAGAIN;
	}

	initialize_common_info(hdmi_adv7523_info->common_info);
	debug1("mapped to 0x%08lX", info_pa);
	return 0;
#else
	return -EINVAL;
#endif
}


/*!
 * open file operation
 * @param[in] inode
 * @param[in] filp
 * @retval 0 successful
 * @retval -EBUSY double request
 * @retval -FAULT power setting failure etc
 */
static int hdmi_adv7523_open(struct inode *inode, struct file *filp)
{
	if (hdmi_adv7523_info->open_state != 0) {
		debug1("already opened");
		return -EBUSY;
	}

	debug1("hdmi_adv7523 open");

	/* power on */
	if (hdmi_adv7523_power_on() < 0) {
		debug0("hdmi_adv7523_power_on failed");
		return -EFAULT;
	}

	hdmi_adv7523_info->open_state = 1;

	return 0;
}

/*!
 * close file operation
 * @param[in] inode
 * @param[in] filp
 * @retval 0 successful
 */
static int hdmi_adv7523_release(struct inode *inode, struct file *filp)
{
#ifdef	CONFIG_EMXX_HDMI_ADV7523_DEBUG
	int minor = MINOR(inode->i_rdev);
#endif

	debug1("minor %d", minor);

	hdmi_adv7523_power_off();

	hdmi_adv7523_info->open_state = 0;

	return 0;
}

/* ! file operations for this driver */
static const struct file_operations hdmi_adv7523_fops = {
	.owner      = THIS_MODULE,
	.llseek     = no_llseek,
	.open       = hdmi_adv7523_open,
	.release    = hdmi_adv7523_release,
	.read       = hdmi_adv7523_read,
	.poll       = hdmi_adv7523_poll,
	.ioctl      = hdmi_adv7523_ioctl,
	.mmap       = hdmi_adv7523_mmap,
};


/* interruption -------------------------------------------------------------*/

/*!
 * HDMI_ADV7523 interruption handler about stream buffer full
 * @param irq
 * @param dev_id
 * @param regs
 */
static irqreturn_t hdmi_adv7523_interrupt(int irq, void *dev_id)
{
	hdmi_adv7523_info->intr++;
	debug1("irq %d\n", hdmi_adv7523_info->intr);
	wake_up_interruptible(&intwaitq);

	return IRQ_HANDLED;
}

#if 0
/*!
 * tasklet for interruption
 * @param[in] value unused
 * @return void
 */
void hdmi_adv7523_tasklet_handler(unsigned long value)
{
	unsigned int factor;
	unsigned int int_a, int_c;

	wake_up_interruptible(&readq);
}
#endif

/* module functions ---------------------------------------------------------*/

#ifdef CONFIG_PM
static int hdmi_adv7523_pf_suspend(struct platform_device *dev,
 pm_message_t state)
{
	if (hdmi_adv7523_info->open_state)
		hdmi_adv7523_power_off();

	return 0;
}

static int hdmi_adv7523_pf_resume(struct platform_device *dev)
{
	if (hdmi_adv7523_info->open_state)
		hdmi_adv7523_power_on();

	return 0;
}
#endif

static int hdmi_adv7523_pf_probe(struct platform_device *dev)
{
	int ret;
#if 0
	int i;
	void *virt;
	unsigned short data;
	unsigned short *code_addr;
#endif
	dev_t devno;

	mutex_init(&hdmi_adv7523_mutex);
	hdmi_adv7523_info->intr = 0;
	hdmi_adv7523_info->open_state = 0;

	init_waitqueue_head(&intwaitq);
	if (hdmi_adv7523_reset() < 0) {
		debug0("hdmi_reset failed");
		ret = -EFAULT;
		goto error_return;
	}

	debug1("requesting irq %d\n", INT_HDMI);
	/* request_irq returns ENOMEM/EINVAL */
	set_irq_type(INT_HDMI, IRQ_TYPE_EDGE_FALLING);
	debug1("set irq type done%d\n", INT_HDMI);

	ret = request_irq(INT_HDMI, hdmi_adv7523_interrupt,
			  IRQF_SHARED, HDMI_ADV7523_MODNAME,
			  hdmi_adv7523_info);

	if (ret < 0) {
		debug0("request_irq(%d) failed %d", INT_HDMI, ret);
		goto error_return;
	}

	/* register chrdev */
	debug1("register_chrdev %d, %s", devmajor, HDMI_ADV7523_MODNAME);
	if (devmajor) {
		devno = MKDEV(devmajor, 0);
		ret = register_chrdev_region(devno, HDMI_ADV7523_MINOR_MAX,
						HDMI_ADV7523_MODNAME);
	} else {
		ret = alloc_chrdev_region(&devno,
						0, HDMI_ADV7523_MINOR_MAX,
						HDMI_ADV7523_MODNAME);
		devmajor = MAJOR(devno);
	}
	if (ret) {
		debug0("register_chrdev %d, %s failed %d",
				devmajor, HDMI_ADV7523_MODNAME, ret);
		goto free_irq_int;
	}

	cdev_init(&hdmi_adv7523_cdev, &hdmi_adv7523_fops);
	ret = cdev_add(&hdmi_adv7523_cdev, devno, HDMI_ADV7523_MINOR_MAX);
	if (ret) {
		debug0("cdev_add %s failed %d", HDMI_ADV7523_MODNAME, ret);
		goto free_chrdev;
	}

	hdmi_adv7523_class = class_create(THIS_MODULE, HDMI_ADV7523_MODNAME);
	if (IS_ERR(hdmi_adv7523_class)) {
		debug0("class_create failed %d", ret);
		ret = PTR_ERR(hdmi_adv7523_class);
		goto free_cdev;
	}

	hdmi_adv7523_class_device = device_create(hdmi_adv7523_class,
		&dev->dev, MKDEV(devmajor, 0),  NULL, "%s",
		HDMI_ADV7523_DEVNAME);
	if (IS_ERR(hdmi_adv7523_class_device)) {
		debug0("class_device_create failed %s %d",
			HDMI_ADV7523_DEVNAME, ret);
		ret = PTR_ERR(hdmi_adv7523_class_device);
		goto free_class;
	}


	hdmi_adv7523_info->open_state = 0;
	/* default value */
	hdmi_adv7523_info->resolution = EMXX_HDMI_OUTPUT_MODE_HDMI_720P_60fps;

	debug1("success");
	return 0;

free_class:
	class_destroy(hdmi_adv7523_class);

free_cdev:
	cdev_del(&hdmi_adv7523_cdev);

free_chrdev:
	unregister_chrdev_region(MKDEV(devmajor, 0), HDMI_ADV7523_MINOR_MAX);

free_irq_int:
	free_irq(INT_HDMI, hdmi_adv7523_info);

error_return:
	return ret;
}

static int hdmi_adv7523_pf_remove(struct platform_device *dev)
{
	device_destroy(hdmi_adv7523_class, MKDEV(devmajor, 0));
	class_destroy(hdmi_adv7523_class);
	debug1("unregister_chrdev %d, %s", devmajor, HDMI_ADV7523_MODNAME);
	unregister_chrdev_region(MKDEV(devmajor, 0), HDMI_ADV7523_MINOR_MAX);
	free_irq(INT_HDMI, hdmi_adv7523_info);

	return 0;
}

static void hdmi_adv7523_pf_release(struct device *dev)
{
	/* none */
}

/*!
 * initialize for insmod
 * @param void
 * @return void
 */
static int __init hdmi_adv7523_init(void)
{
	int ret;

	ret = platform_device_register(&hdmi_adv7523_pf_device);
	if (ret)
		return ret;

	return platform_driver_register(&hdmi_adv7523_pf_driver);
}

/*!
 * finalize for rmmod
 * @param void
 * @return void
 */
static void __exit hdmi_adv7523_exit(void)
{
	(void)platform_driver_unregister(&hdmi_adv7523_pf_driver);
	(void)platform_device_unregister(&hdmi_adv7523_pf_device);
}

module_init(hdmi_adv7523_init);
module_exit(hdmi_adv7523_exit);
module_param(devname, charp, 0444);
module_param(devmajor, int, 0444);
#ifdef	CONFIG_EMXX_HDMI_ADV7523_DEBUG
module_param(debug_level, int, 0444);
#endif
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("EM/EV HDMI ADV7523 Driver");
MODULE_AUTHOR("Renesas Electronics Corporation");
