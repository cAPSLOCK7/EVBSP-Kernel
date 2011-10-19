/*
 *  File Name	    : linux/arch/arm/mach-emxx/emev_pdma.c
 *  Function	    : EM/EV PCM Direct interface used SW interrupt
 *  Release Version : Ver 1.02
 *  Release Date    : 2010/10/25
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

/*============================================================*/
/* include header                                             */
/*============================================================*/
#include <linux/module.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#ifdef CONFIG_EMXX_ANDROID
#include <linux/wakelock.h>
#endif
#include <asm/irq.h>
#include <asm/dma.h>

#include <linux/io.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <mach/pmu.h>
#include <mach/pm.h>
#include <mach/pcm_irq.h>
#include "pm_pmu.h"

/*============================================================*/
/* constant definition                                        */
/*============================================================*/
#define DRIVER_NAME	"emev_pdma"
#define DEVICE_NAME	DRIVER_NAME

#define	INTC_IT0_BASE	IO_ADDRESS(EMXX_INTA_D_BASE)
#define	INTC_IT_LIIS	(INTC_IT0_BASE + 0x0320)	/* set */
#define	INTC_IT_LIIR	(INTC_IT0_BASE + 0x0324)	/* clear */

#define	INT_PCM0	INT_SIO1

/* #define DEBUG_PRINT */
#ifdef DEBUG_PRINT
/* #define PDMA_DEBUG_LOG */
#define FUNC_PRINT(FMT, ARGS...)	\
		printk(KERN_INFO "%s(): " FMT, __func__, ##ARGS)
#else
#define FUNC_PRINT(FMT, ARGS...)
#endif

/*============================================================*/
/* static function definition                                 */
/*============================================================*/
static int emev_pdma_open(struct inode *inode, struct file *filp);
static int emev_pdma_release(struct inode *inode, struct file *filp);
static irqreturn_t emev_pdma_interrupt(int irq, void *dev_id);
static irqreturn_t emev_pcm0_interrupt(int irq, void *dev_id);
static irqreturn_t emev_sw_interrupt(int irq, void *dev_id);

/*============================================================*/
/* static variable definition                                 */
/*============================================================*/
static int pdma_major;
static struct cdev pdma_cdev;
static const struct file_operations pdma_fops = {
	.owner   = THIS_MODULE,
	.open    = emev_pdma_open,
	.release = emev_pdma_release,
};

static struct class *pdma_class;
static struct device *pdma_class_device;

static char *pdma_id = "PDMA_interrupt";
static char *pcm_id = "PCM_interrupt";

#ifdef PDMA_DEBUG_LOG
static int pdma_int_ct;
static int pcm0_int_ct;
static int sw_int_ct;
#endif
static int emev_pdma_drv_open;
#ifdef CONFIG_EMXX_ANDROID
static struct wake_lock pdma_wake_lock;
#endif

/*============================================================*/
/* interrupt Handler                                          */
/*============================================================*/
static irqreturn_t emev_pdma_interrupt(int irq, void *dev_id)
{
	/* disable PDMA/PCM0 interrupt */
	disable_irq(INT_PCM0);
	disable_irq(INT_PDMA);

#ifdef CONFIG_PM
	emxx_pm_pdma_suspend_disable();
#endif
#ifdef PDMA_DEBUG_LOG
	pdma_int_ct++;
	FUNC_PRINT("** PDMA_INTERRUPT %d **\n", pdma_int_ct);
#endif

	/* clear SW interrupt 2 */
	__raw_writel(0x00000004, INTC_IT_LIIR);
	/* set SW interrupt 0 */
	__raw_writel(0x00000001, INTC_IT_LIIS);

	return IRQ_HANDLED;
}

static irqreturn_t emev_pcm0_interrupt(int irq, void *dev_id)
{
	/* disable PDMA/PCM0 interrupt */
	disable_irq(INT_PDMA);
	disable_irq(INT_PCM0);

#ifdef CONFIG_PM
	emxx_pm_pdma_suspend_disable();
#endif
#ifdef PDMA_DEBUG_LOG
	pcm0_int_ct++;
	FUNC_PRINT("** PCM0_INTERRUPT %d **\n", pcm0_int_ct);
#endif

	/* clear SW interrupt 3 */
	__raw_writel(0x00000008, INTC_IT_LIIR);
	/* set SW interrupt 1 */
	__raw_writel(0x00000002, INTC_IT_LIIS);

	return IRQ_HANDLED;
}

static irqreturn_t emev_sw_interrupt(int irq, void *dev_id)
{
#ifdef PDMA_DEBUG_LOG
	sw_int_ct++;
	FUNC_PRINT("** SW_INTERRUPT%d %d **\n", (irq - INT_LIIS0), sw_int_ct);
#endif
	/* clear SW interrupt 2 or 3 */
	__raw_writel((0x00000001 << (irq - INT_LIIS0)), INTC_IT_LIIR);

#ifdef CONFIG_PM
	emxx_pm_pdma_suspend_enable();
#endif

	enable_irq(INT_PDMA);
	enable_irq(INT_PCM0);

	return IRQ_HANDLED;
}

/*============================================================*/
/*  open/close function                                       */
/*============================================================*/
/* ----------------------------------------------------------
 *  FUNCTION : emev_pdma_open
 *  IMPORT   : inode, filp
 *  RETURN   : 0      : success
 *             -EBUSY : false
 *  NOTE     : none
 *  UPDATE   : 2010.10.25
 * ---------------------------------------------------------- */
static int emev_pdma_open(struct inode *inode, struct file *filp)
{
	int ret = 0, err = 0;

	FUNC_PRINT("\n");

	if (emev_pdma_drv_open) {
		FUNC_PRINT("emev_pdma is being opened.\n");
		return -EBUSY;
	}

	/* sw interrupt clear */
	__raw_writel(0x0000000f, INTC_IT_LIIR);

	/* request irq */
	/* PDMA */
	ret = emev_pdma_request_irq(emev_pdma_interrupt, IRQF_DISABLED,
					pdma_id, NULL, EMEV_PCMMODE_DSP);
	FUNC_PRINT("-- request_irq(INT_PDMA) ret = 0x%08x\n", ret);
	if (!ret) {
		/* PCM0 */
		ret = emev_pcm0_request_irq(emev_pcm0_interrupt,
			IRQF_DISABLED, pcm_id, NULL, EMEV_PCMMODE_DSP);
		FUNC_PRINT("-- request_irq(INT_PCM0) ret = 0x%08x\n", ret);
	} else {
		err = 1;
	}
	if (!ret) {
		/*  */
		ret = request_irq(INT_LIIS2, &emev_sw_interrupt, 0,
						"SW_INT2_interrupt", NULL);
		FUNC_PRINT("-- request_irq(INT_LIIS2) ret = 0x%08x\n", ret);
	} else if (!err) {
		err = 2;
	}
	if (!ret) {
		ret = request_irq(INT_LIIS3, &emev_sw_interrupt, 0,
						"SW_INT3_interrupt", NULL);
		FUNC_PRINT("-- request_irq(INT_LIIS3) ret = 0x%08x\n", ret);
	} else if (!err) {
		err = 3;
	}
	if (ret && !err)
		err = 4;

	switch (err) {
	case 4:
			free_irq(INT_LIIS2, NULL);
	case 3:
			emev_pcm0_free_irq(NULL, EMEV_PCMMODE_DSP);
	case 2:
			emev_pdma_free_irq(NULL, EMEV_PCMMODE_DSP);
	case 1:
			ret = -EBUSY;
			break;
	default:
			emev_pdma_drv_open = 1;
#ifdef CONFIG_EMXX_ANDROID
			wake_lock(&pdma_wake_lock);
#endif
#ifdef PDMA_DEBUG_LOG
			pdma_int_ct = 0;
			pcm0_int_ct = 0;
			sw_int_ct = 0;
#endif
			break;
	}

	return ret;
}


/* ----------------------------------------------------------
 *  FUNCTION : emev_pdma_release
 *  IMPORT   : inode, filp
 *  RETURN   : 0 : success
 *  NOTE     : none
 *  UPDATE   : 2010.10.25
 * ---------------------------------------------------------- */
static int emev_pdma_release(struct inode *inode, struct file *filp)
{
	FUNC_PRINT("\n");
#ifdef PDMA_DEBUG_LOG
	FUNC_PRINT("pdma_int = %d, pcm0_int = %d, sw_int = %d\n",
				pdma_int_ct, pcm0_int_ct, sw_int_ct);
#endif

	if (emev_pdma_drv_open) {
		/* sw interrupt clear */
		__raw_writel(0x0000000f, INTC_IT_LIIR);
		/* free irq */
		emev_pdma_free_irq(NULL, EMEV_PCMMODE_DSP);
		emev_pcm0_free_irq(NULL, EMEV_PCMMODE_DSP);
		free_irq(INT_LIIS2, NULL);
		free_irq(INT_LIIS3, NULL);
#ifdef CONFIG_EMXX_ANDROID
		wake_unlock(&pdma_wake_lock);
#endif
		emev_pdma_drv_open = 0;
	}

	return 0;
}


/*============================================================*/
/*  init/end function                                         */
/*============================================================*/
/* ----------------------------------------------------------
 *  FUNCTION : emev_pdma_init
 *  IMPORT   : none
 *  RETURN   : 0 : success
 *  NOTE     : none
 *  UPDATE   : 2010.10.25
 * ---------------------------------------------------------- */
static int __init emev_pdma_init(void)
{
	dev_t devno;

	pdma_major = 0;
	if (alloc_chrdev_region(&devno, 0, 1, DRIVER_NAME) < 0) {
		printk(KERN_ERR "alloc_chrdev_region failed\n");
		goto fail_chrdev_region;
	}
	pdma_major = MAJOR(devno);
	FUNC_PRINT("major = %d\n", pdma_major);

	cdev_init(&pdma_cdev, &pdma_fops);
	if (cdev_add(&pdma_cdev, MKDEV(pdma_major, 0), 1)) {
		printk(KERN_ERR "cdev_add failed\n");
		goto fail_class_create;
	}

	pdma_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(pdma_class)) {
		printk(KERN_ERR "class_create failed\n");
		goto fail_class_create;
	}

	pdma_class_device = device_create(pdma_class, NULL,
					 devno, NULL,
					 "%s", DEVICE_NAME);
	if (IS_ERR(pdma_class_device)) {
		printk(KERN_ERR "device_create failed\n");
		goto fail_device_create;
	}

#ifdef CONFIG_EMXX_ANDROID
	wake_lock_init(&pdma_wake_lock, WAKE_LOCK_SUSPEND, DRIVER_NAME);
#endif

#ifdef PDMA_DEBUG_LOG
	pdma_int_ct = 0;
	pcm0_int_ct = 0;
	sw_int_ct = 0;
#endif
	emev_pdma_drv_open = 0;
	FUNC_PRINT("loaded  into kernel\n");

	return 0;

fail_device_create:
	class_destroy(pdma_class);
fail_class_create:
	cdev_del(&pdma_cdev);
	unregister_chrdev_region(MKDEV(pdma_major, 0), 1);
fail_chrdev_region:
	return -EBUSY;
}


/* ----------------------------------------------------------
 *  FUNCTION : emev_pdma_exit
 *  IMPORT   : none
 *  RETURN   : none
 *  NOTE     : none
 *  UPDATE   : 2010.10.25
 * ---------------------------------------------------------- */
static void __exit emev_pdma_exit(void)
{
#ifdef CONFIG_EMXX_ANDROID
	wake_lock_destroy(&pdma_wake_lock);
#endif

	device_destroy(pdma_class, MKDEV(pdma_major, 0));
	class_destroy(pdma_class);

	cdev_del(&pdma_cdev);
	unregister_chrdev_region(MKDEV(pdma_major, 0), 1);

	FUNC_PRINT("removed from kernel\n");
}


/*============================================================*/
/*  module definition                                         */
/*============================================================*/
module_init(emev_pdma_init);
module_exit(emev_pdma_exit);

MODULE_DESCRIPTION("EMEV PDMA driver for PCM Direct");
MODULE_LICENSE("GPL");
MODULE_ALIAS(DRIVER_NAME);
MODULE_AUTHOR("Renesas Electronics Corporation");
