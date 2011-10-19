/*
 *  File Name       : drivers/sizrot2/sizrot2.c
 *  Function        : SIZ/ROT Driver
 *  Release Version : Ver 1.04
 *  Release Date    : 2010.09.15
 *
 *  Copyright (C) 2010 Renesas Electronics Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA.
 */


#define _DEBUG_SIZROT2 0x00 /* 0x01: debug function in
			     * 0x02: debug function out
			     * 0x04: report wait time
			     * 0x08: debug dividing DMA process
			     */

#if _DEBUG_SIZROT2
#define dbg_printk(level, fmt, arg...) \
	if (level > 0)                 \
		printk(KERN_DEBUG " @sizrot2: " fmt, ## arg);
#else
#define dbg_printk(level, fmt, arg...) \
	;
#endif


#define ERR_PRINT_D(func, param, data) printk(KERN_INFO \
	" @sizrot2: %s: Error! %s is incorrect. %s = %ld\n", \
	func, param, param, data);
#define ERR_PRINT_X(func, param, data) printk(KERN_INFO \
	" @sizrot2: %s: Error! %s is incorrect. %s = 0x%lx\n", \
	func, param, param, data);


#include "sizrot2.h"


/****************************************************************************/
/* Prototype declarations                                                   */
/****************************************************************************/
/* ------------------------------------------- */
/*   Device Method                             */
/* ------------------------------------------- */
static int sizrot2_open(struct inode *inode, struct file *file);
static int sizrot2_close(struct inode *inode, struct file *file);
static int sizrot2_ioctl(struct inode *inode, struct file *file,
 unsigned int request, unsigned long arg);
#ifdef CONFIG_VIDEO_EMXX
static int sizrot2_mmap(struct file *file, struct vm_area_struct *vma);
#endif

static const struct file_operations sizrot2_fops = {
	.owner   = THIS_MODULE,
	.ioctl   = sizrot2_ioctl,
	.open    = sizrot2_open,
	.release = sizrot2_close,
#ifdef CONFIG_VIDEO_EMXX
	.mmap    = sizrot2_mmap,
#endif
};

static struct cdev siz_cdev = {
	.owner = THIS_MODULE,
	.ops   = &sizrot2_fops,
};

static struct cdev rot0_cdev = {
	.owner = THIS_MODULE,
	.ops   = &sizrot2_fops,
};

static struct class  *sizrot2_class;
static struct device *class_dev_siz;
static struct device *class_dev_rot0;
static dev_t dev_t_siz;
static dev_t dev_t_rot0;


/* ------------------------------------------- */
/*   Initialize function                       */
/* ------------------------------------------- */
static int sizrot2_init_module(void);


/* ------------------------------------------- */
/*   DPM functions                             */
/* ------------------------------------------- */
static int sizrot2_suspend(struct platform_device *dev, pm_message_t state);
static int sizrot2_resume(struct platform_device *dev);

static struct platform_driver sizrot2_driver = {
	.driver.name  = "emxx_image",
	.driver.bus   = &platform_bus_type,
	.suspend      = sizrot2_suspend,
	.resume       = sizrot2_resume,
};

static struct platform_device sizrot2_device = {
	.name        = "SIZROT2",
	.dev.release = NULL,
	.id          = 0,
};

#define STAT_ON  1
#define STAT_OFF 0
static int status_siz = STAT_OFF;
static int status_rot = STAT_OFF;
#define DRV_SIZ status_siz
#define DRV_ROT status_rot
#define status_ctrl_func(flag, data) \
	do {                         \
		flag = data;         \
	} while (0)


#ifdef CONFIG_MACH_EMEV
#include "rotate_rgb888_sw.h"
#endif
#include "siz_drv.h"
#include "rot2_drv.h"


/* ------------------------------------------- */
/*   Device Method                             */
/* ------------------------------------------- */
/*****************************************************************************
* MODULE   : sizrot2_open
* FUNCTION : open SIZ/ROT device
* RETURN   :       0 : success
*          : -ENODEV : fail
* NOTE     : none
******************************************************************************/
int sizrot2_open(struct inode *inode, struct file *file)
{
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 " @sizrot2: sizrot2_open() <start>");
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "           MINOR(inode->i_rdev): %d\n", MINOR(inode->i_rdev));
	return 0;
}


/*****************************************************************************
* MODULE   : sizrot2_close
* FUNCTION : close SIZ/ROT device
* RETURN   : 0 :
* NOTE     : none
******************************************************************************/
int sizrot2_close(struct inode *inode, struct file *file)
{
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 " @sizrot2: sizrot2_close() <start>");
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "           MINOR(inode->i_rdev): %d\n", MINOR(inode->i_rdev));
	return 0;
}


/*****************************************************************************
* MODULE   : sizrot2_ioctl
* FUNCTION : IOCTL main function
* RETURN   :       0  : success
*          : negative : fail
* NOTE     : none
******************************************************************************/
int sizrot2_ioctl(struct inode *inode, struct file *file, unsigned int request,
 unsigned long arg)
{
	int ret = 0;
	unsigned int minor = MINOR(inode->i_rdev);

	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 " @sizrot2: sizrot2_ioctl() <requset = 0x%x>\n", request);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "           MINOR(inode->i_rdev): %d\n", MINOR(inode->i_rdev));

	if (minor == DEV_MINOR_SIZ)
		ret = siz_ioctl(inode, file, request, arg);
	else if (minor == DEV_MINOR_ROT0)
		ret = rot2_ioctl(inode, file, request, arg);
	else
		ret = -EINVAL;

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "sizrot2_ioctl() <end> return (%d)\n", ret);
	return ret;
}


#ifdef CONFIG_VIDEO_EMXX
/*****************************************************************************
* MODULE   : sizrot2_mmap
* FUNCTION : mmap file operation
* RETURN   :
* NOTE     : none
******************************************************************************/
static int sizrot2_mmap(struct file *file, struct vm_area_struct *vma)
{
	return emxx_v4l2_mmap(vma);
}
#endif


/* ------------------------------------------- */
/*   DPM functions                             */
/* ------------------------------------------- */
/*****************************************************************************
* MODULE   : sizrot2_suspend
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static int sizrot2_suspend(struct platform_device *dev, pm_message_t state)
{
	if (status_siz || status_rot)
		return -EBUSY;
	else
		return 0;
}


/*****************************************************************************
* MODULE   : sizrot2_resume
* FUNCTION : callback function when to resume
* RETURN   : 0
* NOTE     : none
******************************************************************************/
static int sizrot2_resume(struct platform_device *dev)
{
	return 0;
}


/* ------------------------------------------- */
/*   Initialize function                       */
/* ------------------------------------------- */
/*****************************************************************************
* MODULE   : sizrot2_init_module
* FUNCTION : initialize module
* RETURN   :        0 : success
*          : negative : fail
* NOTE     : none
******************************************************************************/
static int sizrot2_init_module(void)
{
	int ret = 0;

	dbg_printk((_DEBUG_SIZROT2 & 0x01), "sizrot2_init_module() <start>\n");

	/* register_chrdev_region */
	dev_t_siz  = MKDEV(DEV_MAJOR, DEV_MINOR_SIZ);
	ret        = register_chrdev_region(dev_t_siz,  1, "siz");
	if (ret < 0) {
		printk(KERN_ERR " @sizrot2: Fail to regist device (SIZ).\n");
		goto fail_register_chrdev_siz;
	}

	dev_t_rot0 = MKDEV(DEV_MAJOR, DEV_MINOR_ROT0);
	ret        = register_chrdev_region(dev_t_rot0, 1, "rot0");
	if (ret < 0) {
		printk(KERN_ERR " @sizrot2: Fail to regist device (ROT0).\n");
		goto fail_register_chrdev_rot0;
	}

	/* cdev_init */
	cdev_init(&siz_cdev, &sizrot2_fops);
	siz_cdev.owner = THIS_MODULE;

	cdev_init(&rot0_cdev, &sizrot2_fops);
	rot0_cdev.owner = THIS_MODULE;

	/* cdev_add */
	ret = cdev_add(&siz_cdev, dev_t_siz, 1);
	if (ret) {
		printk(KERN_ERR " @sizrot2: Fail to add cdev (SIZ).\n");
		goto fail_cdev_add_siz;
	}

	ret = cdev_add(&rot0_cdev, dev_t_rot0, 1);
	if (ret) {
		printk(KERN_ERR " @sizrot2: Fail to add cdev (ROT0).\n");
		goto fail_cdev_add_rot0;
	}

	/* class_create */
	sizrot2_class = class_create(THIS_MODULE, sizrot2_dev_name);
	if (IS_ERR(sizrot2_class)) {
		printk(KERN_ERR " @sizrot2: Fail to create class.\n");
		ret = PTR_ERR(sizrot2_class);
		goto fail_class_create;
	}

	/* device_create */
	class_dev_siz =
	 device_create(sizrot2_class, NULL, dev_t_siz, NULL, "siz");
	if (IS_ERR(class_dev_siz)) {
		printk(KERN_ERR
		 " @sizrot2: Fail to create class device (SIZ).\n");
		ret = PTR_ERR(class_dev_siz);
		goto fail_class_device_create_siz;
	}

	class_dev_rot0 =
	 device_create(sizrot2_class, NULL, dev_t_rot0, NULL, "rot0");
	if (IS_ERR(class_dev_rot0)) {
		printk(KERN_ERR
		 " @sizrot2: Fail to create class device (ROT0).\n");
		ret = PTR_ERR(class_dev_rot0);
		goto fail_class_device_create_rot0;
	}

	/* platform_device_register */
	dev_set_drvdata(&sizrot2_device.dev, NULL);
	if (platform_device_register(&sizrot2_device) < 0) {
		printk(KERN_ERR
		 " @sizrot2: Fail to register platform_device\n");
		goto fail_platform_device;
	}

	/* platform_driver_register */
	if (platform_driver_register(&sizrot2_driver) < 0) {
		printk(KERN_ERR
		 " @sizrot2: Fail to register platform_driver\n");
		goto fail_platform_driver;
	}

	spin_lock_init(&sizrot2_lock);

	/* init SIZ */
	ret = init_siz();
	if (ret < 0) {
		printk(KERN_INFO " @sizrot2: SIZ driver initialize <failed>\n");
		goto fail_init_hw;
	}

	/* init ROT */
	ret = init_rot();
	if (ret < 0) {
		printk(KERN_INFO " @sizrot2: ROT driver initialize <failed>\n");
		goto fail_init_hw;
	}

	dbg_printk(_DEBUG_SIZROT2, "register_chrdev %d\n", DEV_MAJOR);
	dbg_printk(_DEBUG_SIZROT2, "sizrot2 driver initialize <success>\n");
	goto success;

fail_init_hw:
	platform_driver_unregister(&sizrot2_driver);
fail_platform_driver:
	platform_device_unregister(&sizrot2_device);
fail_platform_device:
	device_destroy(sizrot2_class, MKDEV(DEV_MAJOR, DEV_MINOR_ROT0));
fail_class_device_create_rot0:
	device_destroy(sizrot2_class, MKDEV(DEV_MAJOR, DEV_MINOR_SIZ));
fail_class_device_create_siz:
	class_destroy(sizrot2_class);
fail_class_create:
	cdev_del(&rot0_cdev);
fail_cdev_add_rot0:
	cdev_del(&siz_cdev);
fail_cdev_add_siz:
	unregister_chrdev_region(dev_t_rot0, 1);
fail_register_chrdev_rot0:
	unregister_chrdev_region(dev_t_siz, 1);
fail_register_chrdev_siz:
success:
	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "sizrot2_init_module() <end> return (%d)\n", ret);
	return ret;
}


device_initcall(sizrot2_init_module);


