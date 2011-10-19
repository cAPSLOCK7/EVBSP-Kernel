/*
 *  File Name	    : linux/arch/arm/mach-emxx/pcm_irq.c
 *  Function	    : PCM Interrupt Control
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

#include <linux/module.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <mach/pcm_irq.h>

/* #define DEBUG_PDMA */
#ifdef DEBUG_PDMA
#define DPRINT(fmt, args...) \
	printk(KERN_INFO "####%s:%d: " fmt , __func__ , __LINE__ , ## args);
#else
#define DPRINT(fmt, args...)
#endif

#define DRIVER_NAME	"pcm_irq"

#define PCMMODE_NUM	2
#define EMEV_INT_PCM0	INT_SIO1
#define EMEV_INT_PDMA	INT_PDMA

struct irq_info {
	unsigned int pcmmode;
	irq_handler_t handler;
	unsigned long irqflags;
	const char *devname;
	void *dev_id;
	unsigned int mask;
};

#define VALID_PCMMODE(x)	\
	((x == EMEV_PCMMODE_ALSA) || (x == EMEV_PCMMODE_DSP))
#define MODE_TO_IDX(x)		(x - 1)
#define GET_MODEINFO(x, y)	\
	((x == EMEV_INT_PCM0) ? &pcm0_info[y] : &pdma_info[y])

static struct irq_info pcm0_info[PCMMODE_NUM];
static struct irq_info pdma_info[PCMMODE_NUM];

static unsigned int current_mode = EMEV_PCMMODE_NONE;

static int do_request_irq(unsigned int irq, irq_handler_t handler,
	unsigned long irqflags, const char *devname, void *dev_id,
	unsigned int pcmmode)
{
	int err = 0;
	struct irq_info *info;
	struct irq_info *info2;
	struct irq_desc *desc;

	DPRINT("irq = %d, dev_id = %x, pcmmode = %d\n", irq,
				(unsigned int)dev_id, pcmmode);
	if (!VALID_PCMMODE(pcmmode))
		return -EINVAL;

	info = GET_MODEINFO(irq, MODE_TO_IDX(pcmmode));

	if (info->handler)
		return -EBUSY;

	if (can_request_irq(irq, IRQF_DISABLED) == 0) {
		info2 = GET_MODEINFO(irq, !MODE_TO_IDX(pcmmode));
		/* check irq status */
		desc = irq_to_desc(irq);
		if (desc)
			info2->mask = desc->status;
		if (info2->handler)
			free_irq(irq, info2->dev_id);
		else
			return -EBUSY;
	}
	err = request_irq(irq, handler, irqflags, devname, dev_id);

	if (err != 0) {
		printk(KERN_INFO "%s(): unable to request IRQ %d (%s)\n",
			  __func__, irq, devname);
		return err;
	}

	info->devname = devname;
	info->irqflags = irqflags;
	info->dev_id = dev_id;
	info->handler = handler;
	info->pcmmode = pcmmode;

	DPRINT("info->devname = %x\n", (u32)info->devname);
	DPRINT("info->irqflags = %x\n", (u32)info->irqflags);
	DPRINT("info->dev_id = %x\n", (u32)info->dev_id);
	DPRINT("info->handler = %x\n", (u32)info->handler);
	DPRINT("info->pcmmode = %d\n", (u32)info->pcmmode);

	if (irq == EMEV_INT_PCM0)
		current_mode = pcmmode;

	DPRINT("current_mode = %d\n", current_mode);
	return 0;
}

static void do_free_irq(unsigned int irq, void *dev_id, unsigned int pcmmode)
{
	int err = 0;
	struct irq_info *info;
	struct irq_info *info2;

	if (!VALID_PCMMODE(pcmmode))
		return;

	info = GET_MODEINFO(irq, MODE_TO_IDX(pcmmode));
	if (info->handler == 0)
		return;

	if ((irq != EMEV_INT_PCM0) || (current_mode == pcmmode)) {
		info2 = GET_MODEINFO(irq, !MODE_TO_IDX(pcmmode));

		free_irq(irq, info->dev_id);
		if (info2->handler) {
			err = request_irq(irq, info2->handler,
			    info2->irqflags, info2->devname, info2->dev_id);
			if (err != 0)
				printk(KERN_INFO
				"%s(): unable to request IRQ %d (%s)\n",
					__func__, irq, info2->devname);
			else {
				/* check irq status */
				if (info2->mask & IRQ_DISABLED)
					disable_irq(irq);
				info2->mask = 0;
			}
		}
		if (irq == EMEV_INT_PCM0)
			current_mode = info2->pcmmode;
	}

	info->pcmmode = EMEV_PCMMODE_NONE;
	info->handler = 0;
	info->dev_id = 0;
	info->devname = 0;
	info->irqflags = 0;

	return;
}

int emev_pcm0_request_irq(irq_handler_t handler, unsigned long irqflags,
	const char *devname, void *dev_id, unsigned int pcmmode)
{
	return do_request_irq(EMEV_INT_PCM0, handler, irqflags,
			devname, dev_id, pcmmode);
}
EXPORT_SYMBOL(emev_pcm0_request_irq);

void emev_pcm0_free_irq(void *dev_id, unsigned int pcmmode)
{
	do_free_irq(EMEV_INT_PCM0, dev_id, pcmmode);
}
EXPORT_SYMBOL(emev_pcm0_free_irq);

int emev_pdma_request_irq(irq_handler_t handler, unsigned long irqflags,
	const char *devname, void *dev_id, unsigned int pcmmode)
{
	return do_request_irq(EMEV_INT_PDMA, handler, irqflags,
			devname, dev_id, pcmmode);
}
EXPORT_SYMBOL(emev_pdma_request_irq);

void emev_pdma_free_irq(void *dev_id, unsigned int pcmmode)
{
	do_free_irq(EMEV_INT_PDMA, dev_id, pcmmode);
}
EXPORT_SYMBOL(emev_pdma_free_irq);

unsigned int emev_pcm0_get_pcmmode(void)
{
	return current_mode;
}
EXPORT_SYMBOL(emev_pcm0_get_pcmmode);

MODULE_DESCRIPTION("pcm_irq_driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS(DRIVER_NAME);
MODULE_AUTHOR("Renesas Electronics Corporation");
