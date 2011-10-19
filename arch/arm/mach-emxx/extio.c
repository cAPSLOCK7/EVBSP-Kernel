/*
 *  File Name       : arch/arm/mach-emxx/extio.c
 *  Function        : External GPIO
 *  Release Version : Ver 1.01
 *  Release Date    : 2011/02/08
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
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>

#include <asm/mach/irq.h>

#include <mach/extio.h>

#define EXTIO1_OUT_INITDATA	0x001f
#define EXTIO2_OUT_INITDATA	0x0008

#define EXTIO1_INIT_INOUT	0x2fe0
#define EXTIO2_INIT_INOUT	0x0c00

#define EXTIO1_INIT_IRQ_TYPE	0x00C0

/* #define EXTIO_DEBUG */
#ifdef EXTIO_DEBUG
#define DPRINT(FMT, ARGS...) \
	printk(KERN_INFO "%s(): " FMT, __func__, ##ARGS)
#else
#define DPRINT(FMT, ARGS...)
#endif

int emxx_extio_initialized;
EXPORT_SYMBOL(emxx_extio_initialized);

static struct mutex extio_lock;
static struct i2c_client *emxx_extio1_client;
static struct i2c_client *emxx_extio2_client;
static u32 extio1_out_data;
static u32 extio2_out_data;
static u32 extio1_direction;
static u32 extio2_direction;

static u32 extio1_irq_type;
static u32 extio1_mask;
static struct irq_desc *extio1_desc;
static struct work_struct extio1_workqueue;

int extio_reg_read(int addr, unsigned char *data)
{
	int ret, is_ext2;
	u8 offset;

	if (emxx_extio_initialized == 0)
		return -EBUSY;

	offset = addr & 0xf;
	is_ext2 = (addr & 0x10);

	switch (offset) {
	case EXTIO_INPUT0:
	case EXTIO_INPUT1:
	case EXTIO_POLRITY0:
	case EXTIO_POLRITY1:
		ret = i2c_smbus_read_byte_data(
		  is_ext2 ? emxx_extio2_client : emxx_extio1_client, offset);
		break;
	case EXTIO_OUTPUT0:
		if (is_ext2)
			ret = extio2_out_data & 0xff;
		else
			ret = extio1_out_data & 0xff;
		break;
	case EXTIO_OUTPUT1:
		if (is_ext2)
			ret = (extio2_out_data >> 8) & 0xff;
		else
			ret = (extio1_out_data >> 8) & 0xff;
		break;
	case EXTIO_CONF0:
		if (is_ext2)
			ret = extio2_direction & 0xff;
		else
			ret = extio1_direction & 0xff;
		break;
	case EXTIO_CONF1:
		if (is_ext2)
			ret = (extio2_direction >> 8) & 0xff;
		else
			ret = (extio1_direction >> 8) & 0xff;
		break;
	default:
		return -EINVAL;
	}
	if (ret < 0)
		return ret;

	*data = (unsigned char)ret;
	return 0;
}
EXPORT_SYMBOL(extio_reg_read);

int extio_reg_write(int addr, unsigned char data)
{
	int ret, is_ext2;
	u8 offset;
	struct i2c_client *client;

	if (emxx_extio_initialized == 0)
		return -EBUSY;

	offset = addr & 0xf;
	is_ext2 = (addr & 0x10);
	client = is_ext2 ? emxx_extio2_client : emxx_extio1_client;

	switch (offset) {
	case EXTIO_POLRITY0:
	case EXTIO_POLRITY1:
		ret = i2c_smbus_write_byte_data(client, offset, data);
		break;
	case EXTIO_OUTPUT0:
		ret = i2c_smbus_write_byte_data(client, offset, data);
		if (ret >= 0) {
			if (is_ext2) {
				extio2_out_data &= ~0xff;
				extio2_out_data |= data;
			} else {
				extio1_out_data &= ~0xff;
				extio1_out_data |= data;
			}
		}
		break;
	case EXTIO_OUTPUT1:
		ret = i2c_smbus_write_byte_data(client, offset, data);
		if (ret >= 0) {
			if (is_ext2) {
				extio2_out_data &= ~0xff00;
				extio2_out_data |= (data << 8);
			} else {
				extio1_out_data &= ~0xff00;
				extio1_out_data |= (data << 8);
			}
		}
		break;
	case EXTIO_CONF0:
		ret = i2c_smbus_write_byte_data(client, offset, data);
		if (ret >= 0) {
			if (is_ext2) {
				extio2_direction &= ~0xff;
				extio2_direction |= data;
			} else {
				extio1_direction &= ~0xff;
				extio1_direction |= data;
			}
		}
		break;
	case EXTIO_CONF1:
		ret = i2c_smbus_write_byte_data(client, offset, data);
		if (ret >= 0) {
			if (is_ext2) {
				extio2_direction &= ~0xff00;
				extio2_direction |= (data << 8);
			} else {
				extio1_direction &= ~0xff00;
				extio1_direction |= (data << 8);
			}
		}
		break;
	default:
		return -EINVAL;
	}
	return ret;
}
EXPORT_SYMBOL(extio_reg_write);

int extio_read(int addr, unsigned char *data)
{
	return extio_reg_read(addr, data);
}
EXPORT_SYMBOL(extio_read);

int extio_write(int addr, unsigned char data, unsigned char mask)
{
	int ret;
	u8 tmp_data;

	if (emxx_extio_initialized == 0)
		return -EBUSY;

	mutex_lock(&extio_lock);
	ret = extio_reg_read(addr, &tmp_data);
	if (ret < 0)
		goto err;
	tmp_data = (tmp_data & (~mask)) | (data & mask);
	ret = extio_reg_write(addr, tmp_data);

err:
	mutex_unlock(&extio_lock);
	return 0;
}
EXPORT_SYMBOL(extio_write);

int extio_set_direction(unsigned gpio, int is_input)
{
	u8 mask, val, offset;

	if ((gpio < GPIO_EXT1_P0) || (gpio > GPIO_EXT1_P31))
		return -EINVAL;

	gpio -= GPIO_EXT1_P0;
	mask = 1U << (gpio & 0x7);
	val = (is_input) ? mask : 0;

	if (gpio < 8)
		offset = EXTIO_CONF0;
	else if (gpio < 16)
		offset = EXTIO_CONF1;
	else if (gpio < 24)
		offset = EXTIO_CONF2;
	else
		offset = EXTIO_CONF3;

	return extio_write(offset, val, mask);
}
EXPORT_SYMBOL(extio_set_direction);

int extio_get_value(unsigned int gpio)
{
	int ret = 0;
	u8 mask, val, input;

	if ((gpio < GPIO_EXT1_P0) || (gpio > GPIO_EXT1_P31))
		return -EINVAL;

	gpio -= GPIO_EXT1_P0;
	mask = 1U << (gpio & 0x7);

	if (gpio < 8) {
		input = (extio1_direction & 0xff) & mask;
		if (input)
			ret = extio_read(EXTIO_INPUT0, &val);
		else
			val = extio1_out_data & 0xff;
	} else if (gpio < 16) {
		input = ((extio1_direction >> 8) & 0xff) & mask;
		if (input)
			ret = extio_read(EXTIO_INPUT1, &val);
		else
			val = (extio1_out_data >> 8) & 0xff;
	} else if (gpio < 24) {
		input = (extio2_direction & 0xff) & mask;
		if (input)
			ret = extio_read(EXTIO_INPUT2, &val);
		else
			val = extio2_out_data & 0xff;
	} else {
		input = ((extio2_direction >> 8) & 0xff) & mask;
		if (input)
			ret = extio_read(EXTIO_INPUT3, &val);
		else
			val = (extio2_out_data >> 8) & 0xff;
	}
	return ret ? ret : ((val & mask) ? 1 : 0);
}
EXPORT_SYMBOL(extio_get_value);

void extio_set_value(unsigned int gpio, int value)
{
	u8 mask, val, offset;

	if ((gpio < GPIO_EXT1_P0) || (gpio > GPIO_EXT1_P31))
		return;

	gpio -= GPIO_EXT1_P0;
	mask = 1U << (gpio & 0x7);
	val = (value) ? mask : 0;

	if (gpio < 8)
		offset = EXTIO_OUTPUT0;
	else if (gpio < 16)
		offset = EXTIO_OUTPUT1;
	else if (gpio < 24)
		offset = EXTIO_OUTPUT2;
	else
		offset = EXTIO_OUTPUT3;

	extio_write(offset, val, mask);
}
EXPORT_SYMBOL(extio_set_value);

static int emxx_extio1_set_irq_type(unsigned int irq, unsigned int type)
{
	u32 mask;

	mask = 1 << (irq - INT_EXTIO1_BASE);

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_LEVEL_HIGH:
		extio1_irq_type &= ~mask;
		break;
	case IRQ_TYPE_EDGE_FALLING:
	case IRQ_TYPE_LEVEL_LOW:
		extio1_irq_type |= mask;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
static void emxx_extio1_mask(unsigned int irq)
{
	u32 mask;

	mask = 1 << (irq - INT_EXTIO1_BASE);
	extio1_mask &= ~mask;
}
static void emxx_extio1_unmask(unsigned int irq)
{
	u32 mask;

	mask = 1 << (irq - INT_EXTIO1_BASE);
	extio1_mask |= mask;
}

static struct irq_chip emxx_extio1_chip = {
	.name     = "EXTIO1",
	.mask     = emxx_extio1_mask,
	.unmask   = emxx_extio1_unmask,
	.disable  = emxx_extio1_mask,
	.set_type = emxx_extio1_set_irq_type,
};

static void emxx_extio1_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	if (irq != INT_EXTIO1_INT)
		return;

	DPRINT("entry\n");

	extio1_desc = desc;
	extio1_desc->chip->mask(irq);
	extio1_desc->chip->ack(irq);
	schedule_work(&extio1_workqueue);
}

static void emxx_extio1_work(struct work_struct *work)
{
	u16 extio1_irq_state;
	u32 state, i, irq;
	struct irq_desc *d;

	i2c_smbus_read_i2c_block_data(emxx_extio1_client,
			EXTIO_INPUT0, 2, (u8 *)&extio1_irq_state);

	state = (extio1_direction & extio1_irq_state) ^ extio1_irq_type;
	DPRINT("indata = 0x%x state = 0x%x\n", extio1_irq_state, state);

	for (i = 0; i < 16; i++) {
		state &= extio1_mask;
		if (state & (1 << i)) {
			irq = INT_EXTIO1_BASE + i;
			d = irq_desc + irq;
			d->handle_irq(irq, d);
		}
	}
	extio1_desc->chip->unmask(INT_EXTIO1_INT);
}

static int
emxx_extio_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i;
	u16 tmp;

	if ((client->addr & 0x01) == 0) {
		if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE_DATA))
			return -EIO;

		emxx_extio1_client = client;

		extio1_out_data = EXTIO1_OUT_INITDATA;
		i2c_smbus_write_i2c_block_data(emxx_extio1_client,
				EXTIO_OUTPUT0, 2, (u8 *)&extio1_out_data);

		extio1_direction = EXTIO1_INIT_INOUT;
		i2c_smbus_write_i2c_block_data(emxx_extio1_client,
				EXTIO_CONF0, 2, (u8 *)&extio1_direction);
	} else {
		if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE_DATA))
			return -EIO;

		emxx_extio2_client = client;

		extio2_out_data = EXTIO2_OUT_INITDATA;
		i2c_smbus_write_i2c_block_data(emxx_extio2_client,
				EXTIO_OUTPUT0, 2, (u8 *)&extio2_out_data);

		extio2_direction = EXTIO2_INIT_INOUT;
		i2c_smbus_write_i2c_block_data(emxx_extio2_client,
				EXTIO_CONF0, 2, (u8 *)&extio2_direction);
	}

	if (emxx_extio1_client != NULL && emxx_extio2_client != NULL) {
		emxx_extio_initialized = 1;
		mutex_init(&extio_lock);

		INIT_WORK(&extio1_workqueue, emxx_extio1_work);

		/* interrupt clear */
		i2c_smbus_read_i2c_block_data(emxx_extio1_client,
				EXTIO_INPUT0, 2, (u8 *)&tmp);

		extio1_mask = 0;
		extio1_irq_type = EXTIO1_INIT_IRQ_TYPE;
		for (i = INT_EXTIO1_BASE; i <= INT_EXTIO1_LAST; i++) {
			set_irq_chip(i, &emxx_extio1_chip);
			set_irq_handler(i, handle_simple_irq);
			set_irq_flags(i, IRQF_VALID);
		}
		set_irq_type(INT_EXTIO1_INT, IRQ_TYPE_LEVEL_LOW);
		set_irq_chained_handler(INT_EXTIO1_INT,
				emxx_extio1_irq_handler);
	}

	return 0;
}

static int emxx_extio_remove(struct i2c_client *client)
{
	emxx_extio_initialized = 0;
	return 0;
}

static struct i2c_device_id emxx_extio_idtable[] = {
	{I2C_SLAVE_EXTIO1_NAME, 0},
	{I2C_SLAVE_EXTIO2_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, emxx_extio_idtable);

static struct i2c_driver emxx_extio_driver = {
	.driver = {
		.name	= "emxx-extio",
		.owner	= THIS_MODULE,
	},
	.id_table	= emxx_extio_idtable,
	.probe		= emxx_extio_probe,
	.remove		= __devexit_p(emxx_extio_remove),
};

static int __init emxx_extio_init(void)
{
	return i2c_add_driver(&emxx_extio_driver);
}

device_initcall(emxx_extio_init);
