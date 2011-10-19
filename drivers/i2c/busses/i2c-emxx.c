/*
 *  File Name		: drivers/i2c/busses/i2c-emxx.c
 *  Function		: i2c
 *  Release Version	: Ver 1.02
 *  Release Date	: 2010/08/09
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
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/sched.h>

#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/platform_device.h>

#include <mach/pmu.h>
#include <mach/smu.h>
#include <mach/pm.h>

#include "i2c-emxx.h"

/* #define I2C_DEBUG 1 */

#ifdef I2C_DEBUG
#define DBG(level, format, arg...) printk(level format, ## arg)
#else
#define DBG(level, format, arg...) do { } while (0)
#endif

static int i2c_pending[I2C_NR];
static spinlock_t i2c_lock;
static spinlock_t i2c_irq_lock;

static int irq = 1;	/* polling mode flag(default=1 is interrupt mode) */

static int emxx_i2c_xfer(struct i2c_adapter *, struct i2c_msg[], int);
static int emxx_i2c_xbytes(struct i2c_adapter *, struct i2c_msg *, int);
static int emxx_i2c_suspend(struct platform_device *dev, pm_message_t state);
static int emxx_i2c_resume(struct platform_device *dev);


struct emxx_i2c_device_ {
	struct i2c_adapter	adap;
	wait_queue_head_t	i2c_wait;
	char			*membase;
	int			irq;
	unsigned int		ch;
	int			rstdev;
	int			clkdev;
	int			sclkdev;
	int			mst;
	int			iir;
};
#define emxx_i2c_device	struct emxx_i2c_device_


#ifndef CONFIG_I2C_EMXX_DFC
#define CONFIG_I2C_EMXX_DFC I2C_DFC_ON
#endif


static i2c_ctrl_t emxx_i2c_ctrl = {
	.smc = CONFIG_I2C_EMXX_SMC,	/* mode */
	.dfc = CONFIG_I2C_EMXX_DFC,	/* digital filter */
};

static u32 emxx_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}


static struct i2c_algorithm emxx_i2c_algo = {
	.master_xfer = emxx_i2c_xfer,
	.smbus_xfer = NULL,
	.functionality = emxx_i2c_func,
};

static emxx_i2c_device emxx_i2c_devs[I2C_NR] = {
	[0] = {
		.adap = {
			.owner = THIS_MODULE,
			.name = "emxx I2C1 adapter",
			.id = I2C_HW_EMXX,
			.algo = &emxx_i2c_algo,
			.algo_data = &emxx_i2c_ctrl,
			.client_register = NULL,
			.client_unregister = NULL,
			.timeout = 100,		/* 1sec */
			.retries = 3,		/* 3times */
		},
		.membase = (void *)I2C_ADDR,
		.irq     = INT_IIC0,
		.ch      = EMXX_I2C_CH1,
		.rstdev  = EMXX_RST_IIC0,
		.clkdev  = EMXX_CLK_IIC0,
		.sclkdev = EMXX_CLK_IIC0_S,
		.mst     = EMXX_I2C_INTC_MST,
		.iir     = EMXX_I2C_INTC_IIR,
	},
#ifdef CONFIG_I2C_EMXX_ENABLE_CH2
	[1] = {
		.adap = {
			.owner = THIS_MODULE,
			.name = "emxx I2C2 adapter",
			.id = I2C_HW_EMXX,
			.algo = &emxx_i2c_algo,
			.algo_data = &emxx_i2c_ctrl,
			.client_register = NULL,
			.client_unregister = NULL,
			.timeout = 100,		/* 1sec */
			.retries = 3,		/* 3times */
		},
		.membase = (void *)I2C2_ADDR,
		.irq     = INT_IIC1,
		.ch      = EMXX_I2C_CH2,
		.rstdev  = EMXX_RST_IIC1,
		.clkdev  = EMXX_CLK_IIC1,
		.sclkdev = EMXX_CLK_IIC1_S,
		.mst     = EMXX_I2C2_INTC_MST,
		.iir     = EMXX_I2C2_INTC_IIR,
	},
#endif
};

static void emxx_i2c_enable_clock(emxx_i2c_device *i2c_dev)
{
	emxx_open_clockgate(i2c_dev->sclkdev);
	emxx_open_clockgate(i2c_dev->clkdev);
}

static void emxx_i2c_disable_clock(emxx_i2c_device *i2c_dev)
{
	emxx_close_clockgate(i2c_dev->clkdev);
	emxx_close_clockgate(i2c_dev->sclkdev);
}

static int emxx_i2c_get_clock_status(struct platform_device *dev)
{
	emxx_i2c_device *i2c_dev = platform_get_drvdata(dev);

	if (emxx_get_clockgate(i2c_dev->sclkdev) ||
		emxx_get_clockgate(i2c_dev->clkdev)) {
		return 1;
	}

	return 0;
}

static void emxx_i2c_setup_mode(struct i2c_adapter *adap)
{
	u16 bit = 0x0;
	i2c_ctrl_t *ctrl = (i2c_ctrl_t *) adap->algo_data;
	emxx_i2c_device *i2c_dev;

	i2c_dev = (emxx_i2c_device *)(i2c_get_adapdata(adap));

	DBG(KERN_INFO, "%s(): IICCL0: %04x (reg:0x%08x)\n",
		__func__,
		readl(i2c_dev->membase + I2C_OFS_IICCL0),
		(unsigned int)i2c_dev->membase+I2C_OFS_IICCL0);

	/* mode */
	if (ctrl->smc == I2C_SMC_HIGH_SPEED) {
		/* High-speed mode */
		bit |= I2C_BIT_SMC0;
	}

	/* digital filter */
	if (ctrl->dfc == I2C_DFC_ON) {
		/* digital filter ON */
		bit |= I2C_BIT_DFC0;
	}

	writel(bit, i2c_dev->membase + I2C_OFS_IICCL0);

	DBG(KERN_INFO, "%s(): IICCL0: %04x (reg:0x%08x)\n",
		__func__,
		readl(i2c_dev->membase + I2C_OFS_IICCL0),
		(unsigned int)i2c_dev->membase + I2C_OFS_IICCL0);
}

static int emxx_i2c_wait_sp(emxx_i2c_device *i2c_dev)
{
	int timeout = 200;

	/* Stop condition */
	spin_lock(&i2c_lock);
	writel((readl(i2c_dev->membase + I2C_OFS_IICC0) | I2C_BIT_SPT0),
		i2c_dev->membase + I2C_OFS_IICC0);
	spin_unlock(&i2c_lock);

	while ((readl(i2c_dev->membase + I2C_OFS_IICSE0) & I2C_BIT_SPD0) == 0) {
		if (0 == timeout--) {
			DBG(KERN_INFO,
				"%s(): could not detect stop condition\n",
				__func__);
			return -EBUSY;
		}
		udelay(1);
	}
	DBG(KERN_INFO, "%s(): stop:: con: %04x, stat: %04x\n",
		__func__,
		readl(i2c_dev->membase + I2C_OFS_IICC0),
		readl(i2c_dev->membase + I2C_OFS_IICSE0));

	return 0;
}

static void emxx_i2c_reset(struct i2c_adapter *adap)
{
	emxx_i2c_device *i2c_dev;

	i2c_dev = (emxx_i2c_device *)(i2c_get_adapdata(adap));
	/* I2C disable */
	if (readl(i2c_dev->membase + I2C_OFS_IICACT0) & I2C_BIT_IICE0) {
		if ((readl(i2c_dev->membase + I2C_OFS_IICCL0)
				& (I2C_BIT_CLD0 | I2C_BIT_DAD0))
				!= (I2C_BIT_CLD0 | I2C_BIT_DAD0)) {

			emxx_i2c_wait_sp(i2c_dev);
			/* what about case error returned. */
		}
		spin_lock(&i2c_lock);
		writel(0, i2c_dev->membase + I2C_OFS_IICACT0);
		while (readl(i2c_dev->membase + I2C_OFS_IICACT0) == 1)
			;
		spin_unlock(&i2c_lock);
	}
	/* transfer mode set */
	emxx_i2c_setup_mode(adap);

	/* Refuse transaction */
	/* Can write STCEN at not trns other i2c.*/
	writel((I2C_BIT_STCEN | I2C_BIT_IICRSV),
		i2c_dev->membase + I2C_OFS_IICF0);

	/* I2C enable, 9bit interrupt mode */
	spin_lock(&i2c_lock);
	writel(I2C_BIT_WTIM0, i2c_dev->membase + I2C_OFS_IICC0);
	writel(I2C_BIT_IICE0, i2c_dev->membase + I2C_OFS_IICACT0);
	while (readl(i2c_dev->membase + I2C_OFS_IICACT0) == 0)
		;
	spin_unlock(&i2c_lock);

	DBG(KERN_INFO, "%s()_: con: %04x, stat: %04x (reg:0x%08x,0x%08X)\n",
		__func__,
		readl(i2c_dev->membase + I2C_OFS_IICC0),
		readl(i2c_dev->membase + I2C_OFS_IICSE0),
		(unsigned int)i2c_dev->membase + I2C_OFS_IICC0,
		(unsigned int)i2c_dev->membase + I2C_OFS_IICSE0);
}

/*
 * Wait until I2C bus is free
 */
static int emxx_i2c_wait_for_bb(struct i2c_adapter *adap)
{
	emxx_i2c_device *i2c_dev;
	int status;
	int timeout = adap->timeout;

	i2c_dev = (emxx_i2c_device *)(i2c_get_adapdata(adap));

	/* wait until I2C bus free */
	while ((readl(i2c_dev->membase + I2C_OFS_IICF0) & I2C_BIT_IICBSY)
			&& timeout--) {
		schedule_timeout_uninterruptible(1);
	}

	status = (timeout <= 0);		/* set 0 or 1 */
	if (status)
		DBG(KERN_INFO, "%s(): Timeout, I2C bus is busy\n", __func__);

	return status;
}

/*
 * Wait until INT_I2C handler will be interrupted
 */
static int emxx_i2c_wait_for_pin(struct i2c_adapter *adap, u16 *status)
{
	emxx_i2c_device *i2c_dev;
	int retries = adap->retries;
	unsigned long timeout;
	int pending = 0;
	sigset_t oldset;
	sigset_t newset;

	DECLARE_WAITQUEUE(wait, current);

	i2c_dev = (emxx_i2c_device *)(i2c_get_adapdata(adap));

	if (irq) {
		add_wait_queue(&i2c_dev->i2c_wait, &wait);
		spin_lock_irq(&current->sighand->siglock);
		oldset = current->blocked;
		newset = current->blocked;
		spin_unlock_irq(&current->sighand->siglock);
		sigaddset(&newset, SIGINT);
		sigaddset(&newset, SIGKILL);
		sigprocmask(SIG_BLOCK, &newset, NULL);
	}

	do {
		/* interrupt mode */
		if (irq) {
			timeout = adap->timeout;
			current->state = TASK_UNINTERRUPTIBLE;

			spin_lock_irq(&i2c_irq_lock);
			pending = i2c_pending[i2c_dev->ch];
			i2c_pending[i2c_dev->ch] = 0;
			spin_unlock_irq(&i2c_irq_lock);

			*status = readl(i2c_dev->membase + I2C_OFS_IICSE0);
			DBG(KERN_INFO,
			 "%s(): con: %04x, stat: %04x (reg:0x%08x,0x%08x)\n",
			 __func__,
			 readl(i2c_dev->membase + I2C_OFS_IICC0),
			 *status,
			 (unsigned int)i2c_dev->membase + I2C_OFS_IICC0,
			 (unsigned int)i2c_dev->membase + I2C_OFS_IICC0);

			if (pending)
				break;

			schedule_timeout(timeout);
		}
		/* polling mode */
		else {
			timeout = adap->timeout * 10000;
			do {
				udelay(10);

				/* Check INT_I2C/INT_I2C2 bit */
				if (i2c_dev->mst
					& readl(EMXX_I2C_INT_RAW)) {
					pending = 1;
					outl(i2c_dev->iir,
						EMXX_I2C_INT_IIR);
					break;
				}
			} while (--timeout);

			*status = readl(i2c_dev->membase + I2C_OFS_IICSE0);
			DBG(KERN_INFO, "%s(): con: %04x, stat: %04x\n",
			  __func__,
			  readl(i2c_dev->membase + I2C_OFS_IICC0), *status);

			if (pending)
				break;

		}
	} while ((*status & I2C_BIT_MSTS0) && retries--);

	/* interrupt mode */
	if (irq) {
		sigprocmask(SIG_SETMASK, &oldset, NULL);
		current->state = TASK_RUNNING;
		remove_wait_queue(&i2c_dev->i2c_wait, &wait);
	}

	/* retry counter is expired */
	if (retries <= 0)
		DBG(KERN_INFO, "%s(): Timeout\n", __func__);


	return (retries <= 0) ? -1 : 0;
}

/*
 * master_xfer Interface
 */
static int emxx_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
	int num)
{
	int i;
	int status = 0;
	emxx_i2c_device *i2c_dev;

	DBG(KERN_INFO, "%s(): msgs: %d\n", __func__, num);

	i2c_dev = (emxx_i2c_device *)(i2c_get_adapdata(adap));

	/* Check invalid param in i2c_msg */
	if (msgs == NULL)
		return -EINVAL;

	for (i = 0; i < num; i++) {
		if (msgs[i].buf == NULL)
			return -EINVAL;

	}

	/* I2C macro Enable */
	emxx_i2c_enable_clock(i2c_dev);
	/* I2C transfer mode set */
	emxx_i2c_reset(adap);

	status = emxx_i2c_wait_for_bb(adap);
	if (status) {
		DBG(KERN_INFO, "%s(): %s: bus busy\n", __func__, adap->name);

		if (!(readl(i2c_dev->membase + I2C_OFS_IICSE0)
				& I2C_BIT_MSTS0)) {
			/* Slave mode -> Error */
			emxx_i2c_disable_clock(i2c_dev);
			return -EBUSY;
		}
		DBG(KERN_INFO,
			"%s(): %s is already active. Stop Condition...\n",
			__func__, adap->name);
		/* Stop condition */
		spin_lock(&i2c_lock);
		writel((readl(i2c_dev->membase + I2C_OFS_IICC0) | I2C_BIT_SPT0),
			i2c_dev->membase + I2C_OFS_IICC0);
		spin_unlock(&i2c_lock);

		status = emxx_i2c_wait_for_bb(adap);
		if (status) {
			DBG(KERN_INFO,
				"%s(): %s is already active. Reset...\n",
				__func__, adap->name);
			/* I2C Reset */
			emxx_i2c_reset(adap);

			if (emxx_i2c_wait_for_bb(adap)) {
				DBG(KERN_INFO,
				"%s(): %s is already active. Reset failed...\n",
				__func__, adap->name);

				emxx_i2c_disable_clock(i2c_dev);
				return -EREMOTEIO;
			}
		}
	}

	for (i = 0; i < num; i++) {
		DBG(KERN_INFO,
			"%s(): msg: %d, addr: 0x%04x, len: %d, flags: 0x%x\n",
			__func__, i, msgs[i].addr, msgs[i].len, msgs[i].flags);

		/* Transfer msgs[] */
		status = emxx_i2c_xbytes(adap, &msgs[i], (i == (num - 1)));

		DBG(KERN_INFO, "%s(): status : %d\n", __func__, status);
		if (status < 0) {
			/* transfer error */
			break;
		}
	}

	/* I2C transfer completed */
	if (status >= 0) {
		/* the number of i2c_msg transfered */
		status = i;
	}

	DBG(KERN_INFO, "%s(): status : %d\n", __func__, status);

	/* I2C macro Disable */
	emxx_i2c_disable_clock(i2c_dev);
	/* if after stop clock then register all zero. */

	return status;
}

/*
 * master_xfer transaction(Low level)
 */
static int emxx_i2c_xbytes(struct i2c_adapter *adap, struct i2c_msg *msg,
	int stop)
{
	u16 status;
	u8 data = 0;
	int count = 0;
	int timeout;
	emxx_i2c_device *i2c_dev = (emxx_i2c_device *)i2c_get_adapdata(adap);

	DBG(KERN_INFO, "%s(): addr: 0x%04x, len: %d, flags: 0x%x, stop: %d\n",
		__func__, msg->addr, msg->len, msg->flags, stop);

	/* interrupt mode */
	if (irq) {
		spin_lock_irq(&i2c_irq_lock);
		i2c_pending[i2c_dev->ch] = 0;
		spin_unlock_irq(&i2c_irq_lock);
	}

	/* Start condition */
	spin_lock(&i2c_lock);
	writel((readl(i2c_dev->membase + I2C_OFS_IICC0) & ~I2C_BIT_ACKE0)
		| I2C_BIT_WTIM0, i2c_dev->membase + I2C_OFS_IICC0);
	writel((readl(i2c_dev->membase + I2C_OFS_IICC0) | I2C_BIT_STT0),
		i2c_dev->membase + I2C_OFS_IICC0);
	spin_unlock(&i2c_lock);

	/* wait for completion of Start condition */
	timeout = 10;
	do {
		udelay(EMXX_I2C_WAIT); /* 1us wait */
		DBG(KERN_INFO, "%s(): con: %04x, flg: %04x\n", __func__,
			readl(i2c_dev->membase + I2C_OFS_IICC0),
			readl(i2c_dev->membase + I2C_OFS_IICF0));
		if (((readl(i2c_dev->membase + I2C_OFS_IICF0)
			& I2C_BIT_STCF) == 0)
			&& (readl(i2c_dev->membase + I2C_OFS_IICSE0)
			& I2C_BIT_MSTS0)) {
			break;
		}
		if (--timeout == 0)
			return -EBUSY;

	} while (1);
	DBG(KERN_INFO, "%s(): addr: %04x, flags: %04x\n", __func__, msg->addr,
		msg->flags);
	DBG(KERN_INFO, "%s(): addrdir: %04x\n", __func__,
		(msg->addr << I2C_DIR_SHIFT)
		| ((msg->flags & I2C_M_RD) ? 1 : 0));

	/* Send slave address and R/W type */
	writel((msg->addr << I2C_DIR_SHIFT)
		| ((msg->flags & I2C_M_RD) ? 1 : 0),
		i2c_dev->membase + I2C_OFS_IIC0);

	/* Wait for transaction */
	if (emxx_i2c_wait_for_pin(adap, &status)) {
		emxx_i2c_reset(adap);
		return -EREMOTEIO;
	}

	/* Arbitration */
	if (status & I2C_BIT_ALD0) {
		emxx_i2c_reset(adap);
		return -EREMOTEIO;
	}
	/* Extension mode or Slave mode */
	else if (status & (I2C_BIT_EXC0 | I2C_BIT_COI0)
			|| !(status & I2C_BIT_MSTS0)) {
		emxx_i2c_reset(adap);
		return -EREMOTEIO;
	}
	/* Read transaction */
	else if (!(status & I2C_BIT_TRC0)) {
		/* msg->flags is Write type */
		if (!(msg->flags & I2C_M_RD)) {
			DBG(KERN_INFO, "%s(): %s r/w hardware fault.\n",
				__func__, adap->name);
			emxx_i2c_reset(adap);
			return -EREMOTEIO;
		}

		/* Recieved No ACK (result of setting slave address and R/W) */
		if (!(status & I2C_BIT_ACKD0)) {
			DBG(KERN_INFO, "%s(): %s recieved No ACK.\n",
				__func__, adap->name);
			/* Stop condition */
			emxx_i2c_wait_sp(i2c_dev);
			return -EREMOTEIO;
		}

		/* 8bit interrupt mode */
		spin_lock(&i2c_lock);
		writel((readl(i2c_dev->membase + I2C_OFS_IICC0)
				& ~I2C_BIT_WTIM0) | I2C_BIT_ACKE0,
				i2c_dev->membase + I2C_OFS_IICC0);
		writel((readl(i2c_dev->membase + I2C_OFS_IICC0)
				& ~I2C_BIT_WTIM0) | I2C_BIT_WREL0,
				i2c_dev->membase + I2C_OFS_IICC0);
		spin_unlock(&i2c_lock);

		/* Wait for transaction */
		if (emxx_i2c_wait_for_pin(adap, &status)) {
			emxx_i2c_reset(adap);
			return -EREMOTEIO;
		}
	}

	do {
		/* Arbitration */
		if (status & I2C_BIT_ALD0) {
			emxx_i2c_reset(adap);
			return -EREMOTEIO;
		}
		/* Extension mode or Slave mode */
		else if (status & (I2C_BIT_EXC0 | I2C_BIT_COI0)
				|| !(status & I2C_BIT_MSTS0)) {
			emxx_i2c_reset(adap);
			return -EREMOTEIO;
		}
		/* Read transaction */
		else if (!(status & I2C_BIT_TRC0)) {
			/* msg->flags is Write type */
			if (!(msg->flags & I2C_M_RD)) {
				DBG(KERN_INFO, "%s(): %s r/w hardware fault.\n",
					__func__, adap->name);
				emxx_i2c_reset(adap);
				return -EREMOTEIO;
			}

			/* Read transaction is completed without any errors */
			if (count == msg->len)
				break;

			/* read data */
			data = readl(i2c_dev->membase + I2C_OFS_IIC0);
			DBG(KERN_INFO, "%s(): read: data: %04x\n", __func__,
				data);
			if (count < msg->len) {
				msg->buf[count++] = data;
			} else if (msg->len != 0) {
				/* recieve error */
				DBG(KERN_INFO, "%s(): %s rcv count mismatch.\n",
					__func__, adap->name);
				break;
			}

			if (count < msg->len) {
				spin_lock(&i2c_lock);
				writel((readl(i2c_dev->membase + I2C_OFS_IICC0)
					| I2C_BIT_WREL0),
					i2c_dev->membase + I2C_OFS_IICC0);
				spin_unlock(&i2c_lock);
			} else {
				spin_lock(&i2c_lock);
				writel((readl(i2c_dev->membase + I2C_OFS_IICC0)
					& ~I2C_BIT_ACKE0) | I2C_BIT_WTIM0,
					i2c_dev->membase + I2C_OFS_IICC0);
				writel(readl(i2c_dev->membase + I2C_OFS_IICC0)
					| I2C_BIT_WREL0,
					i2c_dev->membase + I2C_OFS_IICC0);
				spin_unlock(&i2c_lock);
			}

		/* Write transaction */
		} else if (status & I2C_BIT_TRC0) {
			/* msg->flags is Read type */
			if ((msg->flags & I2C_M_RD)) {
				DBG(KERN_INFO, "%s(): %s r/w hardware fault.\n",
					__func__, adap->name);
				emxx_i2c_reset(adap);
				return -EREMOTEIO;
			}

			/* Recieved No ACK (result of setting slave address
			 * and R/W)
			 */
			if (!(status & I2C_BIT_ACKD0)) {
				DBG(KERN_INFO, "%s(): %s recieved No ACK.\n",
					__func__, adap->name);
				/* Stop condition */
				emxx_i2c_wait_sp(i2c_dev);
				return -EREMOTEIO;
			}

			if (count < msg->len) {
				data = msg->buf[count++];
			} else if (count == msg->len) {
				/* send transaction is completed without any
				 * errors
				 */
				break;
			} else if (msg->len != 0) {
				/* send error */
				DBG(KERN_INFO, "%s(): %s xmt count mismatch.\n",
					__func__, adap->name);
				break;
			}

			/* write data */
			DBG(KERN_INFO, "%s(): write: data: %04x\n", __func__,
				data);
			writel(data, i2c_dev->membase + I2C_OFS_IIC0);

		/* not reached ? */
		} else {
			DBG(KERN_INFO, "%s(): con: %04x, stat: %04x\n",
			  __func__, readl(i2c_dev->membase + I2C_OFS_IICC0),
			  readl(i2c_dev->membase + I2C_OFS_IICSE0));
		}

		/* Wait for R/W transaction */
		if (emxx_i2c_wait_for_pin(adap, &status)) {
			emxx_i2c_reset(adap);
			return -EREMOTEIO;
		}
	} while (count <= msg->len);

	DBG(KERN_INFO, "%s(): count: %d, len: %d, stop %d\n", __func__, count,
		msg->len, stop);

	/* Stop condition */
	if ((count >= msg->len) && stop) {
		spin_lock(&i2c_lock);
		writel((readl(i2c_dev->membase + I2C_OFS_IICC0) | I2C_BIT_SPT0),
			i2c_dev->membase + I2C_OFS_IICC0);
		spin_unlock(&i2c_lock);
		timeout = 200;
		while ((readl(i2c_dev->membase + I2C_OFS_IICSE0)
				& I2C_BIT_SPD0) == 0) {
			if (0 == timeout--) {
				DBG(KERN_INFO,
				"%s(): could not detect stop condition\n",
				__func__);
				return -EBUSY;
			}
			udelay(1);
		}
		DBG(KERN_INFO, "%s(): stop:: con: %04x, stat: %04x\n",
			__func__, readl(i2c_dev->membase + I2C_OFS_IICC0),
			readl(i2c_dev->membase + I2C_OFS_IICSE0));
	}

	return (count > 0) ? (msg->len ? count : 0) : count;
}

static int emxx_i2c_add_bus(struct i2c_adapter *adap)
{
	emxx_i2c_device *i2c_dev =
		(emxx_i2c_device *)(i2c_get_adapdata(adap));

	DBG(KERN_INFO, "%s(): hw routines for %s registered.\n", __func__,
		adap->name);

	emxx_i2c_enable_clock(i2c_dev);
	emxx_unreset_device(i2c_dev->rstdev);
	emxx_i2c_reset(adap);
	emxx_i2c_disable_clock(i2c_dev);

	i2c_add_numbered_adapter(adap);

	return 0;
}

static int emxx_i2c_del_bus(struct i2c_adapter *adap)
{
	int res;

	res = i2c_del_adapter(adap);
	if (res < 0)
		return res;

	DBG(KERN_INFO, "%s(): adapter unregistered: %s\n", __func__,
		adap->name);

	return 0;
}


static irqreturn_t emxx_i2c_handler(int this_irq, void *dev_id,
	struct pt_regs *regs)
{
	emxx_i2c_device *i2c_dev = dev_id;

	spin_lock(&i2c_irq_lock);
	i2c_pending[i2c_dev->ch] = 1;
	spin_unlock(&i2c_irq_lock);

	DBG(KERN_INFO, "%s(): in interrupt handler\n", __func__);

	wake_up(&i2c_dev->i2c_wait);

	return IRQ_HANDLED;
}


static int emxx_i2c_hw_resrc_init(struct platform_device *dev)
{
	emxx_i2c_device *i2c_dev = platform_get_drvdata(dev);
	DBG(KERN_INFO, "%s(): resrc Initializing.\n", __func__);

	if (i2c_dev->ch == EMXX_I2C_CH1) {
		outl((inl(SMU_IICSCLKDIV) & 0xffff0000) | I2C_DIVIIC_VAL1,
			SMU_IICSCLKDIV);
		if (irq) {
			if (request_irq(i2c_dev->irq, (void *)emxx_i2c_handler,
					0, "emxx_i2c",
					&emxx_i2c_devs[0]) < 0) {
				return -ENODEV;
			}
		}
	}
#ifdef CONFIG_I2C_EMXX_ENABLE_CH2
	else
	if (i2c_dev->ch == EMXX_I2C_CH2) {
		outl((inl(SMU_IICSCLKDIV) & 0x0000ffff) | I2C_DIVIIC_VAL2,
			SMU_IICSCLKDIV);
		if (irq) {
			if (request_irq(i2c_dev->irq, (void *)emxx_i2c_handler,
					0, "emxx_i2c",
					&emxx_i2c_devs[1]) < 0) {
				return -ENODEV;
			}
		}
	}
#endif

	return 0;
}

static void emxx_i2c_release(struct platform_device *dev)
{
	emxx_i2c_device *i2c_dev = platform_get_drvdata(dev);

	if (irq)
		free_irq(i2c_dev->irq, &emxx_i2c_devs[i2c_dev->ch]);
}


static int emxx_i2c_suspend(struct platform_device *dev, pm_message_t state)
{
	switch (state.event) {
	case PM_EVENT_SUSPEND:
		if (emxx_i2c_get_clock_status(dev))
			return -EBUSY;
		break;
	default:
		break;
	}

	return 0;
}

static int emxx_i2c_resume(struct platform_device *dev)
{
	return 0;
}


/*
 * this function is called when this module is loaded.
 */
static int emxx_i2c_probe(struct platform_device *dev)
{
	DBG(KERN_INFO, "Starting emxx_i2c.\n");

	if (dev->id > I2C_NR)
		return -ENODEV;

	platform_set_drvdata(dev, &emxx_i2c_devs[dev->id]);
	i2c_set_adapdata(&emxx_i2c_devs[dev->id].adap,
				&emxx_i2c_devs[dev->id]);

	if (emxx_i2c_hw_resrc_init(dev) < 0)
		return -ENODEV;

	init_waitqueue_head(&emxx_i2c_devs[dev->id].i2c_wait);

	emxx_i2c_devs[dev->id].adap.nr = dev->id;
	if (emxx_i2c_add_bus(&emxx_i2c_devs[dev->id].adap) < 0)
		return -ENODEV;

	return 0;
}

static int emxx_i2c_remove(struct platform_device *dev)
{
	emxx_i2c_device *i2c_dev = platform_get_drvdata(dev);

	DBG(KERN_INFO, "%s(): ch=%d.\n", __func__, i2c_dev->ch);

	emxx_i2c_del_bus(&emxx_i2c_devs[i2c_dev->ch].adap);
	emxx_i2c_release(dev);

	return 0;
}

static struct platform_driver emxx_i2c_driver = {
	.probe = emxx_i2c_probe,
	.remove = emxx_i2c_remove,
	.suspend = emxx_i2c_suspend,
	.resume = emxx_i2c_resume,
	.driver = {
		.name = "i2c",
		.owner = THIS_MODULE
	}
};


static int __init emxx_i2c_init(void)
{
	spin_lock_init(&i2c_lock);
	spin_lock_init(&i2c_irq_lock);

	return platform_driver_register(&emxx_i2c_driver);
}

static void __exit emxx_i2c_exit(void)
{
	platform_driver_unregister(&emxx_i2c_driver);
}

module_param(irq, int, 0);

MODULE_LICENSE("GPL");

module_init(emxx_i2c_init);
module_exit(emxx_i2c_exit);
