/*
 *  File Name	    : linux/arch/arm/mach-emxx/include/mach/pcm_irq.h
 *  Function	    : PCM Interrupt Control
 *  Release Version : Ver 1.00
 *  Release Date    : 2010/10/05
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

#ifndef __ASM_ARCH_EMXX_PCM_IRQ_H
#define __ASM_ARCH_EMXX_PCM_IRQ_H

#define EMEV_PCMMODE_NONE	0
#define EMEV_PCMMODE_ALSA	1
#define EMEV_PCMMODE_DSP	2

extern int emev_pcm0_request_irq(irq_handler_t handler, unsigned long irqflags,
	const char *devname, void *dev_id, unsigned int pcmmode);
extern void emev_pcm0_free_irq(void *dev_id, unsigned int pcmmode);
extern int emev_pdma_request_irq(irq_handler_t handler, unsigned long irqflags,
	const char *devname, void *dev_id, unsigned int pcmmode);
extern void emev_pdma_free_irq(void *dev_id, unsigned int pcmmode);

extern unsigned int emev_pcm0_get_pcmmode(void);
#endif /* __ASM_ARCH_EMXX_PCM_IRQ_H */
