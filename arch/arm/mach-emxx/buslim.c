/*
* arch/arm/mach-emxx/buslim.c
* This file is bus width limit
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
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/buslim.h>

#define BUS_LIMIT_MAJOR 0
#define BUS_LIMIT_MINOR_MAX 1
#define BUS_LIMIT_MODNAME "emxx_bus_limit"
#define BUS_LIMIT_DEVNAME "bw"


/***********************************************************************
 * macro
 **********************************************************************/
#define ERRPRINT(FMT, ARGS...) printk(KERN_ERR "%s : "FMT, __func__, ##ARGS);

#define BUS_LIMIT_VAL_MASK 0x3F3F

/***********************************************************************
 * register address
 **********************************************************************/
 /* BUS1 */
#define BUS1_BASE_ADDR	0xe1120000
#define BUS1_VA_ADDR	IO_ADDRESS(BUS1_BASE_ADDR)

#define BUS1_CPU0_CONF	(BUS1_VA_ADDR + 0x0)
#define BUS1_CPU1_CONF	(BUS1_VA_ADDR + 0x4)
#define BUS1_DSPD_CONF	(BUS1_VA_ADDR + 0x8)
#define BUS1_AHB_CONF	(BUS1_VA_ADDR + 0xc)
#define BUS1_AVE0_CONF	(BUS1_VA_ADDR + 0x10)
#define BUS1_AVE1_CONF	(BUS1_VA_ADDR + 0x14)
#define BUS1_M2M_CONF	(BUS1_VA_ADDR + 0x18)
#define BUS1_A3D_CONF	(BUS1_VA_ADDR + 0x1c)
#define BUS1_IMC_CONF	(BUS1_VA_ADDR + 0x20)
#define BUS1_IMCW_CONF	(BUS1_VA_ADDR + 0x24)
#define BUS1_M2P_CONF	(BUS1_VA_ADDR + 0x2c)
#define BUS1_DSPI_CONF	(BUS1_VA_ADDR + 0x30)
#define BUS1_P2M_CONF	(BUS1_VA_ADDR + 0x34)
#define BUS1_SIZ_CONF	(BUS1_VA_ADDR + 0x38)
#define BUS1_ROT_CONF	(BUS1_VA_ADDR + 0x3c)

/* MEMC */
#define MEMC_VA_ADDR	IO_ADDRESS(EMXX_MEMC_BASE)
#define MEMC_DDR_CONFIGR1	(MEMC_VA_ADDR + 0x2014)
#define MEMC_DDR_CONFIGR2	(MEMC_VA_ADDR + 0x2018)

/***********************************************************************
 * function
 **********************************************************************/

static int buslim_open(struct inode *inode, struct file *file);
static int buslim_release(struct inode *inode, struct file *file);
static int buslim_ioctl(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg);

/***********************************************************************
 * file operation
 **********************************************************************/

static int devmajor = BUS_LIMIT_MAJOR;

static struct class *buslim_class;
static struct cdev buslim_cdev;
static struct device *buslim_class_device;

static const struct file_operations buslim_fops = {
	.ioctl   = buslim_ioctl,
	.open    = buslim_open,
	.release = buslim_release,
};

static int buslim_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int buslim_release(struct inode *inode, struct file *file)
{
	return 0;
}

static unsigned long cmmand_to_address(unsigned int cmd)
{
	switch (cmd) {
	case BUSLIMIT_SET_CPU0:
	case BUSLIMIT_GET_CPU0:
		return BUS1_CPU0_CONF;

	case BUSLIMIT_SET_CPU1:
	case BUSLIMIT_GET_CPU1:
		return BUS1_CPU1_CONF;

	case BUSLIMIT_SET_DSPD:
	case BUSLIMIT_GET_DSPD:
		return BUS1_DSPD_CONF;

	case BUSLIMIT_SET_DSPI:
	case BUSLIMIT_GET_DSPI:
		return BUS1_DSPI_CONF;

	case BUSLIMIT_SET_AHB:
	case BUSLIMIT_GET_AHB:
		return BUS1_AHB_CONF;

	case BUSLIMIT_SET_AVE0:
	case BUSLIMIT_GET_AVE0:
		return BUS1_AVE0_CONF;

	case BUSLIMIT_SET_AVE1:
	case BUSLIMIT_GET_AVE1:
		return BUS1_AVE1_CONF;

	case BUSLIMIT_SET_M2M:
	case BUSLIMIT_GET_M2M:
		return BUS1_M2M_CONF;

	case BUSLIMIT_SET_A3D:
	case BUSLIMIT_GET_A3D:
		return BUS1_A3D_CONF;

	case BUSLIMIT_SET_IMC:
	case BUSLIMIT_GET_IMC:
		return BUS1_IMC_CONF;

	case BUSLIMIT_SET_IMCW:
	case BUSLIMIT_GET_IMCW:
		return BUS1_IMCW_CONF;

	case BUSLIMIT_SET_M2P:
	case BUSLIMIT_GET_M2P:
		return BUS1_M2P_CONF;

	case BUSLIMIT_SET_P2M:
	case BUSLIMIT_GET_P2M:
		return BUS1_P2M_CONF;

	case BUSLIMIT_SET_SIZ:
	case BUSLIMIT_GET_SIZ:
		return BUS1_SIZ_CONF;

	case BUSLIMIT_SET_ROT:
	case BUSLIMIT_GET_ROT:
		return BUS1_ROT_CONF;


	case REFRESH_SET_CONFIGR1:
	case REFRESH_GET_CONFIGR1:
		return MEMC_DDR_CONFIGR1;

	case REFRESH_SET_CONFIGR2:
	case REFRESH_GET_CONFIGR2:
		return MEMC_DDR_CONFIGR2;

	}

	return 0;
}

#define buslimit_write(mask) \
do { \
	if (copy_from_user(&val, (void *)arg, sizeof(unsigned long))) { \
		ERRPRINT("copy_from_user failed \n"); \
		ret = -EFAULT; \
		break; \
	} \
	addr = cmmand_to_address(cmd); \
	addr_val = readl(addr); \
	val = (addr_val & ~mask) | (val & mask); \
	writel(val, addr); \
} while (0)

#define buslimit_read(mask) \
do { \
	addr = cmmand_to_address(cmd); \
	addr_val = readl(addr); \
	val = addr_val & mask; \
	if (copy_to_user((void *)arg, &val, sizeof(unsigned long))) { \
		ERRPRINT("copy_to_user failed \n"); \
		ret = -EFAULT; \
		break; \
	} \
} while (0)

static int buslim_ioctl(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	unsigned long val;
	unsigned long addr;
	unsigned long addr_val;

	switch (cmd) {
	case BUSLIMIT_SET_CPU0:
	case BUSLIMIT_SET_CPU1:
	case BUSLIMIT_SET_DSPD:
	case BUSLIMIT_SET_DSPI:
	case BUSLIMIT_SET_AHB:
	case BUSLIMIT_SET_AVE0:
	case BUSLIMIT_SET_AVE1:
	case BUSLIMIT_SET_M2M:
	case BUSLIMIT_SET_A3D:
	case BUSLIMIT_SET_IMC:
	case BUSLIMIT_SET_IMCW:
	case BUSLIMIT_SET_M2P:
	case BUSLIMIT_SET_P2M:
	case BUSLIMIT_SET_SIZ:
	case BUSLIMIT_SET_ROT:
		buslimit_write(BUS_LIMIT_VAL_MASK);
		break;

	case BUSLIMIT_GET_CPU0:
	case BUSLIMIT_GET_CPU1:
	case BUSLIMIT_GET_DSPD:
	case BUSLIMIT_GET_DSPI:
	case BUSLIMIT_GET_AHB:
	case BUSLIMIT_GET_AVE0:
	case BUSLIMIT_GET_AVE1:
	case BUSLIMIT_GET_M2M:
	case BUSLIMIT_GET_A3D:
	case BUSLIMIT_GET_IMC:
	case BUSLIMIT_GET_IMCW:
	case BUSLIMIT_GET_M2P:
	case BUSLIMIT_GET_P2M:
	case BUSLIMIT_GET_SIZ:
	case BUSLIMIT_GET_ROT:
		buslimit_read(BUS_LIMIT_VAL_MASK);
		break;

	case REFRESH_SET_CONFIGR1:
	case REFRESH_SET_CONFIGR2:
		buslimit_write(~0);
		break;

	case REFRESH_GET_CONFIGR1:
	case REFRESH_GET_CONFIGR2:
		buslimit_read(~0);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}


static __init int buslim_init(void)
{
	int ret;
	dev_t devno;

	if (devmajor) {
		devno = MKDEV(devmajor, 0);
		ret = register_chrdev_region(devno, BUS_LIMIT_MINOR_MAX,
						BUS_LIMIT_MODNAME);
	} else {
		ret = alloc_chrdev_region(&devno,
					0, BUS_LIMIT_MINOR_MAX,
					BUS_LIMIT_MODNAME);
		devmajor = MAJOR(devno);
	}
	if (ret) {
		ERRPRINT("register_chrdev %d, %s failed %d\n",
				devmajor, BUS_LIMIT_MODNAME, ret);
		goto error_chrdev;
	}

	cdev_init(&buslim_cdev, &buslim_fops);
	ret = cdev_add(&buslim_cdev, devno, BUS_LIMIT_MINOR_MAX);
	if (ret) {
		ERRPRINT("cdev_add %s failed %d\n", BUS_LIMIT_MODNAME, ret);
		goto error_cdev;
	}

	buslim_class = class_create(THIS_MODULE, BUS_LIMIT_MODNAME);
	if (IS_ERR(buslim_class)) {
		ERRPRINT("class_create failed %d\n", ret);
		ret = PTR_ERR(buslim_class);
		goto error_class;
	}

	buslim_class_device = device_create(buslim_class, NULL,
					 devno, NULL,
					 "%s", BUS_LIMIT_DEVNAME);
	if (IS_ERR(buslim_class_device)) {
		ERRPRINT("class_device_create failed %s %d\n",
			BUS_LIMIT_DEVNAME, ret);
		ret = PTR_ERR(buslim_class_device);
		goto error_device;
	}

	return 0;

error_device:
	class_destroy(buslim_class);
error_class:
	cdev_del(&buslim_cdev);
error_cdev:
	unregister_chrdev_region(MKDEV(devmajor, 0), BUS_LIMIT_MINOR_MAX);
error_chrdev:
	return ret;
}


module_init(buslim_init);

