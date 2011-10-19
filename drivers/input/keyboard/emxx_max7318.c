/*
 *  File Name       : drivers/input/keyboard/emev_max7318.c
 *  Function        : button Interface
 *  Release Version : Ver 1.12
 *  Release Date    : 2010/08/09
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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>

#include <asm/mach/irq.h>

#include "emxx_max7318.h"

/* #define BUTTON_DEBUG */

static const int irq_key_array[] = {
	INT_KEY_DATA0,
	INT_KEY_DATA1,
	INT_KEY_DATA2,
	INT_KEY_DATA3
};

/**** Global parameters ****/
static unsigned int scan_delay = BTN_CHAT_GUARDTIME;
static unsigned int scan_interval = BTN_SCAN_INTERVAL;

static struct input_dev *emxx_max7318_dev;

/* work queue */
static struct workqueue_struct *emxx_max7318_workqueue;
static struct delayed_work emxx_max7318_work;

static int emxx_max7318_initialized;

static unsigned long last_bcode;	/* store last button matrix state */
static unsigned long black_bcode;	/* the button should be abandoned */

static int emxx_max7318_push;

/* Translation table from the got data to keycode */
static struct emxx_btn_table emxx_max7318_code_tbl[] = {
/*
		3
	4	5	6
		7
	8	9	10
	11	12	13
	14	15	16
	17	18	19
-------------------
	     Up
	Left Enter  Right
	     Down
	Home Menu   Back
	Call Powor  End
	 -   Search V_up
	Cam    -    V_down
*/
	/* KET_OUT0 */
	{.scancode = KEY_UP,         .btn_id_bit = (1 << 0),},	/* SW3 */
	{.scancode = KEY_HOME,       .btn_id_bit = (1 << 1),},	/* SW8 */
	{.scancode = KEY_END,        .btn_id_bit = (1 << 2),},	/* SW13 */
	{.scancode = KEY_F4,         .btn_id_bit = (1 << 3),},	/* SW18 */

	/* KET_OUT1 */
	{.scancode = KEY_LEFT,       .btn_id_bit = (1 << 4),},	/* SW4 */
	{.scancode = KEY_F1,         .btn_id_bit = (1 << 5),},	/* SW9 */
	{.scancode = KEY_F2,         .btn_id_bit = (1 << 6),},	/* SW14 */
	{.scancode = KEY_VOLUMEDOWN, .btn_id_bit = (1 << 7),},	/* SW19 */

	/* KET_OUT2 */
	{.scancode = KEY_ENTER,      .btn_id_bit = (1 << 8),},	/* SW5 */
	{.scancode = KEY_BACK,       .btn_id_bit = (1 << 9),},	/* SW10 */
	{.scancode = KEY_SEARCH,     .btn_id_bit = (1 << 10),},	/* SW15 */

	/* KET_OUT3 */
	{.scancode = KEY_RIGHT,      .btn_id_bit = (1 << 12),},	/* SW6 */
	{.scancode = KEY_SEND,       .btn_id_bit = (1 << 13),},	/* SW11 */
	{.scancode = KEY_VOLUMEUP,   .btn_id_bit = (1 << 14),},	/* SW16 */

	/* KET_OUT4 */
	{.scancode = KEY_DOWN,       .btn_id_bit = (1 << 16),},	/* SW7 */
	{.scancode = KEY_POWER,      .btn_id_bit = (1 << 17),},	/* SW12 */
	{.scancode = KEY_CAMERA,     .btn_id_bit = (1 << 18),},	/* SW17 */
	{.scancode = 0, .btn_id_bit = 0,},
};


#ifdef CONFIG_EMXX_ANDROID
#include <linux/proc_fs.h>
#include <linux/wakelock.h>
#include <linux/uaccess.h>
static int dummy_code[] = {
	KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9, KEY_0,
	KEY_A, KEY_B, KEY_C, KEY_D, KEY_E, KEY_F, KEY_G, KEY_H, KEY_I, KEY_J,
	KEY_K, KEY_L, KEY_M, KEY_N, KEY_O, KEY_P, KEY_Q, KEY_R, KEY_S, KEY_T,
	KEY_U, KEY_V, KEY_W, KEY_X, KEY_Y, KEY_Z,
	KEY_EQUAL, KEY_MINUS, KEY_TAB, KEY_SLASH, KEY_SPACE, KEY_KPPLUS,
	KEY_F1, KEY_BACKSPACE, KEY_DOT, KEY_VOLUMEDOWN, KEY_VOLUMEUP, KEY_HOME,
	KEY_CAMERA,
};

static struct wake_lock key_lock;

#define MAX_BUF 255

static void repo_dummy(int code)
{
	struct input_dev *btn_dev = emxx_max7318_dev;

	input_report_key(btn_dev, code, 1);
	schedule_timeout(1);
	input_report_key(btn_dev, code, 0);
}

static int key_read_proc(char *buf, char **start, off_t offset,
	int count, int *eof, void *data)
{
	return 0;
}

static int key_write_proc(struct file *file, const char *buf,
	u_long count, void *data)
{
	char mybuf[MAX_BUF];
	int i, ret;

	if (count >= MAX_BUF)
		return -EINVAL;

	ret = copy_from_user(mybuf, buf, count);

	for (i = 0; i < count; i++) {
		if (mybuf[i] == '\n' || mybuf[i] == '\0')
			break;

		switch (mybuf[i]) {
		case '1':
			repo_dummy(KEY_1);
			break;
		case '2':
			repo_dummy(KEY_2);
			break;
		case '3':
			repo_dummy(KEY_3);
			break;
		case '4':
			repo_dummy(KEY_4);
			break;
		case '5':
			repo_dummy(KEY_5);
			break;
		case '6':
			repo_dummy(KEY_6);
			break;
		case '7':
			repo_dummy(KEY_7);
			break;
		case '8':
			repo_dummy(KEY_8);
			break;
		case '9':
			repo_dummy(KEY_9);
			break;
		case '0':
			repo_dummy(KEY_0);
			break;
		case 'a':
			repo_dummy(KEY_A);
			break;
		case 'b':
			repo_dummy(KEY_B);
			break;
		case 'c':
			repo_dummy(KEY_C);
			break;
		case 'd':
			repo_dummy(KEY_D);
			break;
		case 'e':
			repo_dummy(KEY_E);
			break;
		case 'f':
			repo_dummy(KEY_F);
			break;
		case 'g':
			repo_dummy(KEY_G);
			break;
		case 'h':
			repo_dummy(KEY_H);
			break;
		case 'i':
			repo_dummy(KEY_I);
			break;
		case 'j':
			repo_dummy(KEY_J);
			break;
		case 'k':
			repo_dummy(KEY_K);
			break;
		case 'l':
			repo_dummy(KEY_L);
			break;
		case 'm':
			repo_dummy(KEY_M);
			break;
		case 'n':
			repo_dummy(KEY_N);
			break;
		case 'o':
			repo_dummy(KEY_O);
			break;
		case 'p':
			repo_dummy(KEY_P);
			break;
		case 'q':
			repo_dummy(KEY_Q);
			break;
		case 'r':
			repo_dummy(KEY_R);
			break;
		case 's':
			repo_dummy(KEY_S);
			break;
		case 't':
			repo_dummy(KEY_T);
			break;
		case 'u':
			repo_dummy(KEY_U);
			break;
		case 'v':
			repo_dummy(KEY_V);
			break;
		case 'w':
			repo_dummy(KEY_W);
			break;
		case 'x':
			repo_dummy(KEY_X);
			break;
		case 'y':
			repo_dummy(KEY_Y);
			break;
		case 'z':
			repo_dummy(KEY_Z);
			break;
		case '=':
			repo_dummy(KEY_EQUAL);
			break;
		case '-':
			repo_dummy(KEY_MINUS);
			break;
		case '\t':
			repo_dummy(KEY_TAB);
			break;
		case '/':
			repo_dummy(KEY_SLASH);
			break;
		case ' ':
			repo_dummy(KEY_SPACE);
			break;
		case '+':
			repo_dummy(KEY_F1);
			break;
		case ',':
			repo_dummy(KEY_BACKSPACE);
			break;
		case '.':
			repo_dummy(KEY_DOT);
			break;
		case '%':
			repo_dummy(KEY_VOLUMEUP);
			break;
		case '&':
			repo_dummy(KEY_VOLUMEDOWN);
			break;
		case '(':
			repo_dummy(KEY_HOME);
			break;
		case ')':
			repo_dummy(KEY_CAMERA);
			break;
		default:
			return count;
		}
	}
	return count;
}
#endif


/*
 * emxx_max7318_scan_core
 *    scan button matrix, and judge valid pressing
 */
static void emxx_max7318_scan_core(struct work_struct *work)
{
	int i, irq_count;
	int btn_push_cnt = 0;
	unsigned long chg_bcode;
	unsigned long tmp_bcode;
	unsigned long btncode = 0;
	unsigned char val1 = 0, val2 = 0, val3 = 0, val4 = 0, val5 = 0;
	struct input_dev *btn_dev = emxx_max7318_dev;

	struct emxx_btn_table *bst;

#ifdef BUTTON_DEBUG
	printk(KERN_INFO "%s(): time=%ld\n", __func__, jiffies);
#endif

	extio_write(EXTIO_OUTPUT0, 0x01, 0x1f);
	extio_read(EXTIO_INPUT1, &val1);
	val1 &= 0xf;

	extio_write(EXTIO_OUTPUT0, 0x02, 0x1f);
	extio_read(EXTIO_INPUT1, &val2);
	val2 &= 0xf;

	extio_write(EXTIO_OUTPUT0, 0x04, 0x1f);
	extio_read(EXTIO_INPUT1, &val3);
	val3 &= 0x7;

	extio_write(EXTIO_OUTPUT0, 0x08, 0x1f);
	extio_read(EXTIO_INPUT1, &val4);
	val4 &= 0x7;

	extio_write(EXTIO_OUTPUT0, 0x10, 0x1f);
	extio_read(EXTIO_INPUT1, &val5);
	val5 &= 0x7;

	btncode = ((val5 << 16) | (val4 << 12) |
			(val3 << 8) | (val2 << 4) | val1);

#ifdef BUTTON_DEBUG
	printk(KERN_INFO "%s(): button bit =0x%08lx\n", __func__, btncode);
#endif

	/* check out pressed button number */
	bst = emxx_max7318_code_tbl;
	tmp_bcode = btncode;

	while ((bst->btn_id_bit != 0) && (tmp_bcode != 0)) {
		if ((btncode & bst->btn_id_bit) != 0) {
			btn_push_cnt++;
			tmp_bcode &= ~(bst->btn_id_bit);
		}
		bst++;
	}

	chg_bcode = last_bcode ^ btncode;

	/* if total pressed button number is invalid, ignore it */
	if (btn_push_cnt > 1) {
		tmp_bcode = (chg_bcode & btncode);
		black_bcode |= (tmp_bcode & ~last_bcode);
	}

	/* which button released ? */
	tmp_bcode = (chg_bcode & last_bcode);
	if ((tmp_bcode & ~black_bcode) != 0) {
		bst = emxx_max7318_code_tbl;
		while (bst->btn_id_bit != 0) {
			/* check out which button */
			if ((tmp_bcode & bst->btn_id_bit) != 0) {
				/* button release */
				input_report_key(btn_dev, bst->scancode, 0);
#ifdef BUTTON_DEBUG
				printk(KERN_INFO "%s(): RELEASE %d time=%ld\n",
				       __func__, bst->scancode, jiffies);
#endif
				break;
			}
			bst++;
		}
	}

	/* which button pressed ? */
	tmp_bcode = (chg_bcode & btncode);
	if ((tmp_bcode & ~black_bcode) != 0) {
		bst = emxx_max7318_code_tbl;
		while (bst->btn_id_bit != 0) {
			/* check out which button */
			if ((tmp_bcode & bst->btn_id_bit) != 0) {
				/* button press */
				input_report_key(btn_dev, bst->scancode, 1);
#ifdef BUTTON_DEBUG
				printk(KERN_INFO "%s(): PRESS %d time=%ld\n",
				       __func__, bst->scancode, jiffies);
#endif
				break;
			}
			bst++;
		}
	}

	/* make a record for legal button */
	last_bcode = btncode & ~black_bcode;

	/* try to clear black bit */
	black_bcode &= btncode;

	if (btncode != 0) {
		queue_delayed_work(emxx_max7318_workqueue,
				&emxx_max7318_work, scan_interval);
	} else {
		/* button scan all on */
		extio_write(EXTIO_OUTPUT0, 0x1f, 0x1f);

		emxx_max7318_push = 0;
#ifdef CONFIG_EMXX_ANDROID
		wake_unlock(&key_lock);
#endif

		irq_count = sizeof(irq_key_array) / sizeof(irq_key_array[0]);
		for (i = 0; i < irq_count; i++)
			enable_irq(irq_key_array[i]);
	}

	return;
}

/*
 * emxx_max7318_interrupt
 *     button interrupt handler for all buttons
 */
static irqreturn_t
emxx_max7318_interrupt(int irq, void *dev_id)
{
	int i, irq_count;

#ifdef BUTTON_DEBUG
	printk(KERN_INFO "%s(): irq %d, time=%ld\n",
					__func__, irq - INT_PWC_BASE,  jiffies);
#endif

	/* button int mask */
	irq_count = sizeof(irq_key_array) / sizeof(irq_key_array[0]);
	for (i = 0; i < irq_count; i++)
		disable_irq_nosync(irq_key_array[i]);

	emxx_max7318_push = 1;
#ifdef CONFIG_EMXX_ANDROID
	wake_lock(&key_lock);
#endif

	queue_delayed_work(emxx_max7318_workqueue,
			&emxx_max7318_work, scan_delay);

	return IRQ_HANDLED;
}


static int emxx_max7318_start(void)
{
	int err;
	struct emxx_btn_table *bst;
	int irq = 0;
#ifdef CONFIG_EMXX_ANDROID
	int i;
	struct proc_dir_entry *key_proc;
#endif
	int irq_count;

	/* input device & irq init */
	emxx_max7318_dev = input_allocate_device();
	if (!emxx_max7318_dev) {
		printk(KERN_ERR "emxx_max7318.c: Not enough memory for input device\n");
		return -ENOMEM;
	}

	/* work queue */
	INIT_DELAYED_WORK(&emxx_max7318_work, emxx_max7318_scan_core);

	/* setup input device */
	set_bit(EV_KEY, emxx_max7318_dev->evbit);
#ifndef CONFIG_EMXX_ANDROID
	set_bit(EV_REP, emxx_max7318_dev->evbit);
#endif

	bst = emxx_max7318_code_tbl;
	while (bst->btn_id_bit != 0) {
		set_bit(bst->scancode, emxx_max7318_dev->keybit);
		bst++;
	}
#ifdef CONFIG_EMXX_ANDROID
	for (i = 0; i < ARRAY_SIZE(dummy_code); i++)
		__set_bit(dummy_code[i], emxx_max7318_dev->keybit);

	key_proc = create_proc_read_entry("key", 0, NULL, key_read_proc, NULL);
	key_proc->write_proc = key_write_proc;

	wake_lock_init(&key_lock, WAKE_LOCK_SUSPEND, "key");
#endif

	emxx_max7318_dev->name = EMXX_BTN_NAME;

	err = input_register_device(emxx_max7318_dev);
	if (err < 0) {
		printk(KERN_ERR "Unable to register emxx_max7318 input device\n");
		goto out_input_free_device;
	}

	/* button scan all on */
	extio_write(EXTIO_OUTPUT0, 0x1f, 0x1f);

	irq_count = sizeof(irq_key_array) / sizeof(irq_key_array[0]);
	for (; irq < irq_count; irq++) {
		/* set interrupt high level */
		set_irq_type(irq_key_array[irq], IRQ_TYPE_LEVEL_HIGH);

		/* request interrupt handler */
		err = request_irq(irq_key_array[irq],
				emxx_max7318_interrupt,
				IRQF_DISABLED, EMXX_BTN_NAME, NULL);
		if (err != 0) {
			printk(KERN_INFO
			       "%s(): reuest_irq(%#x, %p, IRQF_DISABLED, %s, 0)"
			       "failed (%d)\n",
			       __func__, irq_key_array[irq],
			       emxx_max7318_interrupt, EMXX_BTN_NAME, err);
			goto out_free_irq;
		}
	}

	emxx_max7318_initialized = 1;

	return 0;

out_free_irq:
#ifdef CONFIG_EMXX_ANDROID
	wake_lock_destroy(&key_lock);
#endif
	for (irq--; irq >= 0; irq--)
		free_irq(irq_key_array[irq], 0);

	input_unregister_device(emxx_max7318_dev);

out_input_free_device:
	input_free_device(emxx_max7318_dev);

	return err;
}

static int wait_count;
static void emxx_max7318_start_wait(struct work_struct *work)
{
	int ret;

	if (emxx_extio_initialized == 0) {
		if (wait_count++ < 10)
			queue_delayed_work(emxx_max7318_workqueue,
			&emxx_max7318_work, 10);
	} else {
		ret = emxx_max7318_start();
	}
}


int emxx_key_pm_state(void)
{
	if (emxx_max7318_push)
		return -EBUSY;
	return 0;
}
EXPORT_SYMBOL(emxx_key_pm_state);

static int emxx_max7318_suspend(struct platform_device *pdev, pm_message_t state)
{
	switch (state.event) {
	case PM_EVENT_SUSPEND:
		if (emxx_max7318_push)
			return -EBUSY;
		break;
	default:
		break;
	}
	return 0;
}

static int emxx_max7318_resume(struct platform_device *dev)
{
	return 0;
}

/*
 * emxx_max7318_probe
 */
static int __init emxx_max7318_probe(struct platform_device *pdev)
{
	int err;

	emxx_max7318_push = 0;

	/* work queue */
	emxx_max7318_workqueue = create_singlethread_workqueue(EMXX_BTN_NAME);

	if (emxx_extio_initialized == 0) {
		INIT_DELAYED_WORK(&emxx_max7318_work,
				emxx_max7318_start_wait);
		queue_delayed_work(emxx_max7318_workqueue,
				&emxx_max7318_work, 10);
	} else {
		err = emxx_max7318_start();
		if (err != 0)
			return err;
	}
	/* set scan delay */
	scan_delay = scan_delay * HZ / 1000;
	if (scan_delay < 1)
		scan_delay = BTN_CHAT_GUARDTIME * HZ / 1000;

	/* set scan interval */
	scan_interval = scan_interval * HZ / 1000;
	if (scan_interval < 1)
		scan_interval = BTN_SCAN_INTERVAL * HZ / 1000;

	return 0;
}

/*
 * emxx_max7318_remove
 */
static int __devexit emxx_max7318_remove(struct platform_device *pdev)
{
	int irq;
	int irq_count;

	/* button scan all off */
	extio_write(EXTIO_OUTPUT0, 0, 0x1f);

	if (emxx_max7318_initialized) {
		irq_count = sizeof(irq_key_array) /
				sizeof(irq_key_array[0]);
		for (irq = 0; irq < irq_count; irq++)
			free_irq(irq_key_array[irq], 0);

#ifdef CONFIG_EMXX_ANDROID
		wake_lock_destroy(&key_lock);
		remove_proc_entry("key", NULL);
#endif
		input_unregister_device(emxx_max7318_dev);
		input_free_device(emxx_max7318_dev);
	}

	destroy_workqueue(emxx_max7318_workqueue);
	return 0;
}

static struct platform_driver emxx_max7318_driver = {
	.probe		= emxx_max7318_probe,
	.remove		= __devexit_p(emxx_max7318_remove),
	.suspend	= emxx_max7318_suspend,
	.resume		= emxx_max7318_resume,
	.driver		= {
		.name	= EMXX_BTN_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init emxx_max7318_init(void)
{
	return platform_driver_register(&emxx_max7318_driver);
}

static void __exit emxx_max7318_exit(void)
{
	platform_driver_unregister(&emxx_max7318_driver);
}

module_init(emxx_max7318_init);
module_exit(emxx_max7318_exit);

module_param(scan_delay, uint, 0644);
MODULE_PARM_DESC(scan_delay, "emxx button chattering guard time [ms]");

module_param(scan_interval, uint, 0644);
MODULE_PARM_DESC(scan_interval, "emxx button scan interval time [ms]");

MODULE_LICENSE("GPL");


