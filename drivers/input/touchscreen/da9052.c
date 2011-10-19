/*
 *  File Name       : drivers/input/touchscreen/da9052.c
 *  Function        : button Interface
 *  Release Version : Ver 1.12
 *  Release Date    : 2010/12/24
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/platform_device.h>

#include <asm/irq.h>
#include <mach/pwc.h>

static u32 debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "DA9052 touch debug level");

#define DS9052_TS_SCAN_TIME	10
static unsigned int scan_interval = DS9052_TS_SCAN_TIME;
module_param(scan_interval, uint, 0644);
MODULE_PARM_DESC(scan_interval, "DA9052 touch scan interval time [ms]");

#define DA9052_NAME	"da9052-ts"

#define DA9052_TS_DOWN_IRQ	INT_PWC_E_PEN_DOWN
#define DA9052_TS_READY_IRQ	INT_PWC_E_TSI_READY

#define	MAX_10BIT	((1 << 10) - 1)

#ifdef CONFIG_EMXX_ANDROID
#define X_MIN	20
#define X_MAX	1000
#define Y_MIN	40
#define Y_MAX	980
#else
#define X_MIN	0
#define X_MAX	MAX_10BIT
#define Y_MIN	0
#define Y_MAX	MAX_10BIT
#endif

struct da9052_ts {
	struct input_dev *dev;

	struct timer_list	timer;

	int down_irq;
	int ready_irq;
};

static u8 da9052_pen_down;

static void da9052_ts_pwr_up(struct da9052_ts *ts)
{
	pwc_reg_write(DA9052_LDO9_REG, 0x59);
	pwc_reg_write(DA9052_TSICONTA_REG, 0xe3);
	pwc_write(DA9052_ADCCONT_REG, 0x40, 0x40);
}

static void da9052_ts_pwr_down(struct da9052_ts *ts)
{
	pwc_reg_write(DA9052_LDO9_REG, 0x19);
	pwc_reg_write(DA9052_TSICONTA_REG, 0xe2);
	pwc_write(DA9052_ADCCONT_REG, 0x00, 0x40);
}

static int da9052_get_data(struct da9052_ts *ts)
{
	unsigned int x, y, z, tmp;
	int pressure = 0;

	pwc_read(DA9052_TSIXMSB_REG, &x);
	pwc_read(DA9052_TSIYMSB_REG, &y);
	pwc_read(DA9052_TSIZMSB_REG, &z);
	pwc_read(DA9052_TSILSB_REG, &tmp);
	pwc_write(DA9052_EVENTB_REG, 0xC0, 0xC0);

	x = (x << 2) | ((tmp & 0x03) >> 0);
	y = (y << 2) | ((tmp & 0x0c) >> 2);
	z = (z << 2) | ((tmp & 0x30) >> 4);

	if (z != 0) {
		pressure = 685*x*(1024 - z) - (159*z*(1024-y));
		pressure /= (z*1024);
	}

	if (debug)
		printk(KERN_INFO "%s: x=%d y=%d z=%d p=%d (%s)\n",
			__func__, x, y, z, pressure,
			(tmp & 0x40) ? "P" : "R");

#ifdef CONFIG_EMXX_ANDROID
		{
			int y2 = (Y_MIN + Y_MAX) - y;
			if (y2 < 0)
				y = Y_MIN;
			else
				y = y2;
		}
#endif

	if (tmp & 0x40) {
		input_report_abs(ts->dev, ABS_X, x);
		input_report_abs(ts->dev, ABS_Y, y);
		input_report_abs(ts->dev, ABS_PRESSURE, pressure);
		if (da9052_pen_down == 0) {
			input_report_key(ts->dev, BTN_TOUCH, 1);
			da9052_pen_down = 1;
		}
		input_sync(ts->dev);
		return 0;
	} else {
		input_report_abs(ts->dev, ABS_PRESSURE, 0);
		if (da9052_pen_down) {
			input_report_key(ts->dev, BTN_TOUCH, 0);
			input_sync(ts->dev);
			da9052_pen_down = 0;
		}
		return -1;
	}
}

static void da9052_timer_handler(unsigned long data)
{
	struct da9052_ts *ts = (struct da9052_ts *)data;
	int ret;

	ret = da9052_get_data(ts);
	if (ret == 0) {
		mod_timer(&ts->timer,
			jiffies + msecs_to_jiffies(scan_interval));
	} else {
		/* Change Low-power mode */
		da9052_ts_pwr_down(ts);
		enable_irq(ts->down_irq);
		enable_irq(ts->ready_irq);
	}
}

#include <linux/delay.h>
static irqreturn_t da9052_irq_handler(int irq, void *dev_id)
{
	struct da9052_ts *ts = dev_id;
	int ret;

	if (irq == ts->down_irq) {
		/* Change Active mode */
		disable_irq_nosync(ts->down_irq);
		da9052_ts_pwr_up(ts);
	} else {
		ret = da9052_get_data(ts);
		if (ret == 0) {
			disable_irq_nosync(ts->ready_irq);
			mod_timer(&ts->timer,
				jiffies + msecs_to_jiffies(scan_interval*2));
		} else {
			da9052_ts_pwr_down(ts);
			enable_irq(ts->down_irq);
		}
	}
	return IRQ_HANDLED;
}


int emxx_touch_pm_state(void)
{
	if (da9052_pen_down)
		return -EBUSY;
	return 0;
}
EXPORT_SYMBOL(emxx_touch_pm_state);

static int __init da9052_ts_probe(struct platform_device *pdev)
{
	int ret;
	struct input_dev *dev;
	struct da9052_ts *ts;

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL)
		return -ENOMEM;

	dev = input_allocate_device();
	if (!dev) {
		printk(KERN_INFO
			"%s: error allocating memory for input structure\n",
			__func__);
		ret = -ENOMEM;
		goto err_alloc_dev;
	}

	dev->name = DA9052_NAME;
	dev->dev.parent = &pdev->dev;
	dev->id.bustype = BUS_HOST;
	dev->id.vendor = 0x10b7;
	dev->id.product = 0x0001;
	dev->id.version = 0x0001;

	dev->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
	dev->keybit[BIT_WORD(BTN_TOUCH)] |= BIT_MASK(BTN_TOUCH);
	dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);

	/* value based on board measurement */
	input_set_abs_params(dev, ABS_X,    X_MIN,     X_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Y,    Y_MIN,     Y_MAX, 0, 0);
	input_set_abs_params(dev, ABS_PRESSURE, 0, MAX_10BIT, 0, 0);

	init_timer(&ts->timer);
	setup_timer(&ts->timer, da9052_timer_handler, (unsigned long)ts);

	ts->dev = dev;
	ts->down_irq = DA9052_TS_DOWN_IRQ;
	ts->ready_irq = DA9052_TS_READY_IRQ;

	da9052_pen_down = 0;

	pwc_reg_write(DA9052_TSICONTB_REG, 0x80);
	da9052_ts_pwr_down(ts);

	ret = request_irq(ts->ready_irq, da9052_irq_handler,
			IRQF_DISABLED, DA9052_NAME, ts);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to register the irq (%d)\n",
				DA9052_NAME, DA9052_TS_READY_IRQ);
		goto err_ready_irq;
	}
	ret = request_irq(ts->down_irq, da9052_irq_handler,
			IRQF_DISABLED, DA9052_NAME, ts);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to register the irq (%d)\n",
				DA9052_NAME, DA9052_TS_READY_IRQ);
		goto err_down_irq;
	}

	ret = input_register_device(dev);
	if (ret != 0) {
		printk(KERN_ERR "%s: error registering with input system\n",
				DA9052_NAME);
		goto err_register;
	}

	platform_set_drvdata(pdev, ts);

	return 0;

err_register:
	free_irq(ts->down_irq, ts);
err_down_irq:
	free_irq(ts->ready_irq, ts);
err_ready_irq:
	input_free_device(dev);
err_alloc_dev:
	kfree(ts);

	return ret;
}

static int __devexit da9052_ts_remove(struct platform_device *pdev)
{
	struct da9052_ts *ts = platform_get_drvdata(pdev);

	free_irq(ts->ready_irq, ts);
	free_irq(ts->down_irq, ts);

	input_unregister_device(ts->dev);
	input_free_device(ts->dev);

	kfree(ts);
	return 0;
}

static struct platform_driver da9052_ts_driver = {
	.probe		= da9052_ts_probe,
	.remove		= __devexit_p(da9052_ts_remove),
	.driver		= {
		.name	= DA9052_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init da9052_ts_init(void)
{
	return platform_driver_register(&da9052_ts_driver);
}

static void __exit da9052_ts_exit(void)
{
	platform_driver_unregister(&da9052_ts_driver);
}

module_init(da9052_ts_init);
module_exit(da9052_ts_exit);

MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_DESCRIPTION("Driver for the DS9052 Touch Screen Controller");
MODULE_LICENSE("GPL");

