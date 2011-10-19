/*
 * File Name       : drivers/nts/emxx_adv7179.c
 * Function        : adv7179 video encoder for NTSC I/F Driver
 * Release Version : Ver 1.01
 * Release Date    : 2011.02.17
 *
 * Copyright (C) 2010-2011 Renesas Electronics Corporation
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

/***************************************************************************/
/*    Include Header Files                                                 */
/***************************************************************************/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/pci.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/device.h>
/* #include <linux/videodev.h>*/
#include <linux/video_encoder.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>

#include <linux/io.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/segment.h>
#include <linux/uaccess.h>

#include "../video/emxx/emxx_common.h"
#include "adv7179.h"
#include "emxx_nts_common.h"
#include "emxx_nts.h"


/***************************************************************************/
/*    Definitions                                                          */
/***************************************************************************/
#define I2C_ADV7179_CLIENT_INIT NULL


/***************************************************************************/
/*    structure                                                            */
/***************************************************************************/
struct adv7179 {
	int norm;
	int enable;
};


/***************************************************************************/
/*    function                                                             */
/***************************************************************************/
static int i2c_adv7179_probe(struct i2c_client *client,
 const struct i2c_device_id *id);
static int i2c_adv7179_remove(struct i2c_client *client);

/***************************************************************************/
/*    variable                                                             */
/***************************************************************************/
static char			adv7179_name[] = "ADV7179 video encoder";
static struct encoder_reg	*encoder_adv7179;

static struct i2c_device_id i2c_adv7179_idtable[] = {
	{ "adv7179", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, i2c_adv7179_idtable);
static struct i2c_driver i2c_adv7179_driver = {
	.driver = {
		.name	= "i2c for adv7179",
		.owner	= THIS_MODULE,
	},
	.id_table	= i2c_adv7179_idtable,
	.probe		= i2c_adv7179_probe,
	.remove		= i2c_adv7179_remove,
};

static struct i2c_client *i2c_adv7179_client = I2C_ADV7179_CLIENT_INIT;

/* i2c functions */
static int i2c_adv7179_probe(struct i2c_client *client,
 const struct i2c_device_id *id)
{
	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_WRITE_BYTE_DATA | I2C_FUNC_SMBUS_BYTE)) {
		printk(KERN_INFO
			"[Error] %s: i2c_check_functionality() error.\n",
			__func__);
		return -1;
	}

	i2c_adv7179_client = client;
	return 0;
}
static int i2c_adv7179_remove(struct i2c_client *client)
{
	i2c_adv7179_client = NULL;
	return 0;
}

static int i2c_adv7179_cleanup(void)
{
	i2c_del_driver(&i2c_adv7179_driver);
	return 0;
}

static int i2c_adv7179_init(void)
{
	int ret;
	ret = i2c_add_driver(&i2c_adv7179_driver);
	if (ret != 0) {
		printk(KERN_INFO "[Error] %s: i2c_add_driver() error.\n",
		 __func__);
	}
	return ret;
}

static int i2c_adv7179_write(unsigned char reg, unsigned char data)
{
	unsigned char i2c_data[2];

	i2c_data[0] = reg;
	i2c_data[1] = data;

	return i2c_master_send(i2c_adv7179_client, i2c_data, sizeof(i2c_data));
}


static int emxx_adv7179_command(unsigned int cmd, void *arg)
{
	int ret = 0;
	struct adv7179 *encoder;

	encoder = (struct adv7179 *)encoder_adv7179->private_data;

	if (encoder == NULL)
		return -1;

	switch (cmd) {
	case ENCODER_SET_NORM:
	{
		int iarg = *(int *) arg;
		int i;
		unsigned char reg[6];
		unsigned char data[6];

		switch (iarg) {
		case NTS_OUTPUT_NTSC:
			reg[0]  = ADV7179_MODE0;
			data[0] = MODE0_CHROMA_FILTER_SELECT_1_3MHZ_LOWPASS
			 | MODE0_LUMA_FILTER_SELECT_LOWPASS_NTSC
			 | MODE0_OUTPUT_VIDEO_SELECT_NTSC;
			reg[1]  = ADV7179_MODE4;
			data[1] = MODE4_PEDESTAL_CONTROL_ON;
			reg[2]  = ADV7179_SUBCAREER_FREQUENCY0;
			data[2] = 0x16;
			reg[3]  = ADV7179_SUBCAREER_FREQUENCY1;
			data[3] = 0x7C;
			reg[4]  = ADV7179_SUBCAREER_FREQUENCY2;
			data[4] = 0xF0;
			reg[5]  = ADV7179_SUBCAREER_FREQUENCY3;
			data[5] = 0x21;
			break;
		case NTS_OUTPUT_PAL:
#ifdef CONFIG_EMXX_NTS_PAL /* PAL B/D/G/H/I/N */
			reg[0]  = ADV7179_MODE0;
			data[0] = MODE0_CHROMA_FILTER_SELECT_1_3MHZ_LOWPASS
			 | MODE0_LUMA_FILTER_SELECT_LOWPASS_PAL
			 | MODE0_OUTPUT_VIDEO_SELECT_PAL;
			reg[1]  = ADV7179_MODE4;
			data[1] = MODE4_PEDESTAL_CONTROL_OFF;
			reg[2]  = ADV7179_SUBCAREER_FREQUENCY0;
			data[2] = 0xCB;
			reg[3]  = ADV7179_SUBCAREER_FREQUENCY1;
			data[3] = 0x8A;
			reg[4]  = ADV7179_SUBCAREER_FREQUENCY2;
			data[4] = 0x09;
			reg[5]  = ADV7179_SUBCAREER_FREQUENCY3;
			data[5] = 0x2A;
#endif
#ifdef CONFIG_EMXX_NTS_PAL60 /* PAL-60 */
			reg[0]  = ADV7179_MODE0;
			data[0] = MODE0_CHROMA_FILTER_SELECT_1_3MHZ_LOWPASS
			 | MODE0_LUMA_FILTER_SELECT_LOWPASS_PAL;
			reg[1]  = ADV7179_MODE4;
			data[1] = MODE4_PEDESTAL_CONTROL_OFF;
			reg[2]  = ADV7179_SUBCAREER_FREQUENCY0;
			data[2] = 0xCB;
			reg[3]  = ADV7179_SUBCAREER_FREQUENCY1;
			data[3] = 0x8A;
			reg[4]  = ADV7179_SUBCAREER_FREQUENCY2;
			data[4] = 0x09;
			reg[5]  = ADV7179_SUBCAREER_FREQUENCY3;
			data[5] = 0x2A;
#endif
			break;
		default:
			return -EINVAL;
		}

		for (i = 0; i < 6; i++)
			ret = i2c_adv7179_write(reg[i], data[i]);

		break;
	}
	case ENCODER_SET_INPUT:
	{
		int *iarg = arg;
		/* not much choice of inputs */
		if (*iarg != 0)
			return -EINVAL;
		break;
	}
	case ENCODER_SET_OUTPUT:
	{
		int *iarg = arg;
		/* not much choice of outputs */
		if (*iarg != 0)
			return -EINVAL;
		break;
	}
	case ENCODER_ENABLE_OUTPUT:
	{
		int *iarg = arg;
		encoder->enable = !!*iarg;
		break;
	}
	default:
		return -EINVAL;
	}

	return ret;
}

static void emxx_adv7179_shutdown(void *dummy)
{
	struct adv7179 *encoder;

	encoder = (struct adv7179 *)encoder_adv7179->private_data;

	kfree(encoder);		/* NULL check is inside. */
	encoder_adv7179->private_data = (void *)NULL;

	i2c_adv7179_cleanup();

	return;
}


#include <mach/pwc.h>

static int emxx_adv7179_init(void *dummy)
{
	int ret = 0;
	struct adv7179 *encoder;

	if (i2c_adv7179_init() != 0) {
		printk(KERN_INFO "[Error] %s: i2c initialize error.\n",
		 __func__);
		return -ENODEV;
	}

	encoder = kmalloc(sizeof(struct adv7179), GFP_KERNEL);
	if (encoder == NULL) {
		printk(KERN_INFO "[Error] %s: memory allocate error.\n",
		 __func__);
		i2c_adv7179_cleanup();
		return -ENOMEM;
	}
	memset(encoder, 0, sizeof(struct adv7179));
	encoder->norm = NTS_OUTPUT_NTSC;
	encoder->enable = 1;

	ret = i2c_adv7179_write(ADV7179_MODE1, 0x10);
	if (ret <= 0) {
		printk(KERN_INFO "[Error] %s: i2c_master_send() error.\n",
		 __func__);
		i2c_adv7179_cleanup();
		kfree(encoder);
		return ret;
	}

	encoder_adv7179->private_data = (void *)encoder;
	return 0;
}

int ntsc_encoder_init(struct emxx_nts_dev *ntsc)
{
	encoder_adv7179 = kmalloc(sizeof(struct encoder_reg), GFP_KERNEL);
	if (encoder_adv7179 == NULL)
		return -ENOMEM;

	memset(encoder_adv7179, 0, sizeof(struct encoder_reg));
	strcpy(encoder_adv7179->name, adv7179_name);
	encoder_adv7179->hw_init	= emxx_adv7179_init;
	encoder_adv7179->hw_shutdown	= emxx_adv7179_shutdown;
	encoder_adv7179->hw_command	= emxx_adv7179_command;

	ntsc->encoder = encoder_adv7179;
	return 0;
}

void ntsc_encoder_exit(struct emxx_nts_dev *ntsc)
{
	if (encoder_adv7179) {
		if (encoder_adv7179->private_data) {
			void *dummy = 0;
			emxx_adv7179_shutdown(dummy);
		}
		kfree(encoder_adv7179);
		encoder_adv7179 = NULL;
	}

	ntsc->encoder = NULL;
}
