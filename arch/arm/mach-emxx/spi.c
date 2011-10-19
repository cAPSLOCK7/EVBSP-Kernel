/*
 *  File Name	    : arch/arm/mach-emxx/spi.c
 *  Function	    : SPI1 interface
 *  Release Version : Ver 1.02
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>

#include <linux/io.h>
#include <mach/dma.h>
#include <mach/smu.h>
#include <mach/pmu.h>

#include "spi.h"

static void spi_interrupt_dma_rx(void *, int, int);
static void spi_interrupt_dma_tx(void *, int, int);

static int spi_open_chrdev(struct inode *inode, struct file *file);

static int spi_major;
static int spi_sleep_flag;

static struct class        *spi_class;
static dev_t t_dev;

static const struct file_operations spi_fops = {
	.owner   = THIS_MODULE,
	.open    = spi_open_chrdev,
};
static struct cdev spi_cdev;

static unsigned int spi_clock_table[] = {
	1433,           /* (1434KHz) */
	2867,           /* (2.87MHz) */
	5734,           /* (5.73MHz) */
	11468,          /* (11.47MHz) */
	22937,          /* (22.94MHz) */
	38229,          /* (38.23MHz) */
};

static unsigned int spi_plldiv_table[] = {
	SMU_DIV(160),     /* 1500KHz(1434KHz) */
	SMU_DIV(80),      /* 3MHz   (2.87MHz) */
	SMU_DIV(40),      /* 6MHz   (5.73MHz) */
	SMU_DIV(20),      /* 12MHz  (11.47MHz) */
	SMU_DIV(10),      /* 24MHz  (22.94MHz) */
	SMU_DIV(6),       /* 48MHz  (38.23MHz) */
};

/* spi mode info (SPI1) */
static struct spi_mode_info mode_spi1 = {
	.nb = SPI_NB_32BIT,
	.cs_sel = SPI_CS_SEL_CS0,
	.m_s = SPI_M_S_MASTER,
	.dma = SPI_DMA_ON,
	.sclk = SPI_SCLK_3MHZ,
	.blksize = SPI_BLKSIZE,
};

/* transfer info (SPI1) */
static struct spi_trans_t trans_spi1 = {
	.dma_regs = NULL,
	.rx_lch = EMXX_DMAC_P2M_SIO2,
	.tx_lch = EMXX_DMAC_M2P_SIO2,
	.rx_data = SPx_RX_DATA_PHYS(EMXX_SIO2_BASE),
	.tx_data = SPx_TX_DATA_PHYS(EMXX_SIO2_BASE),
	.int_spi = INT_SIO2,
	.dma_err = 0,
	.spi_err = 0,
	.state = SPI_STOP,
	.spinlock = __SPIN_LOCK_UNLOCKED(trans_spi1.spinlock),
};

/* smu info (SPI1) */
static struct spi_smu_t smu_spi1 = {
	.pclk = EMXX_CLK_USIB_S2_P | EMXX_CLK_USIB_S2_H,
	.sclk = EMXX_CLK_USIB_S2_S,
	.pclk_ctrl = EMXX_CLKCTRL_USIBS2PCLK,
	.sclk_ctrl = EMXX_CLKCTRL_USIBS2,
	.reset = EMXX_RST_USIB_S2_A | EMXX_RST_USIB_S2_S,
	.div_sclk = SMU_USIB0SCLKDIV,
};

/* spi data info */
static struct spi_data_t spi_private[] = {
	{
		/* SP0 is Unsupported (Dummy) */
	},
	{
		/* SP1 */
		.regs = (struct spi_regs *)(SP1_ADDR),
		.mode = &mode_spi1,
		.pol = SPx_POL_VAL_SPI1,
		.smu = &smu_spi1,
		.trans = &trans_spi1,
		.opened = 0,
		.k_flag = 0,
	},
};

/*
 * software reset
 */
static void spi_sft_reset(struct spi_data_t *spi)
{
	spi->regs->control |= SPx_CONTROL_RST;
	udelay((4000 / spi_clock_table[spi->mode->sclk]) + 1);
	spi->regs->control &= ~SPx_CONTROL_RST;
}

/*
 * reset on/off
 */
static void spi_reset_ctrl(struct spi_data_t *spi, unsigned char onoff)
{
	if ((onoff != SPI_OFF) && (onoff != SPI_ON) && (onoff != SPI_RESET))
		return;

	/* auto clock off */
	emxx_clkctrl_off(spi->smu->sclk_ctrl);
	emxx_clkctrl_off(spi->smu->pclk_ctrl);

	/* clock on */
	emxx_open_clockgate(spi->smu->pclk);
	emxx_open_clockgate(spi->smu->sclk);

	switch (onoff) {
	case SPI_OFF:
		/* reset off */
		emxx_unreset_device(spi->smu->reset);
		break;

	case SPI_ON:
		/* reset on */
		/* clock off */
		emxx_close_clockgate(spi->smu->sclk);
		emxx_close_clockgate(spi->smu->pclk);
		break;

	case SPI_RESET:
		/* reset on -> off */
		emxx_unreset_device(spi->smu->reset);
		break;

	default:
		break;
	}

	/* auto clock on */
	emxx_clkctrl_on(spi->smu->pclk_ctrl);
	emxx_clkctrl_on(spi->smu->sclk_ctrl);
}

static unsigned int spi_unit(struct spi_data_t *spi, unsigned int bit)
{
	if (bit == 0)
		return 0;
	else if (bit <= SPI_NB_8BIT)
		return 1;
	else if (bit <= SPI_NB_16BIT)
		return 2;
	else if ((bit <= SPI_NB_24BIT) && (spi->mode->dma == SPI_DMA_OFF))
		return 3;
	else
		return 4;
}

static int spi_set_sclk_div(struct spi_data_t *spi, unsigned int sclk)
{
	unsigned int val;

	switch (sclk) {
	case SPI_SCLK_1500KHZ:
	case SPI_SCLK_3MHZ:
	case SPI_SCLK_6MHZ:
	case SPI_SCLK_12MHZ:
	case SPI_SCLK_24MHZ:
	case SPI_SCLK_48MHZ:
		break;
	default:
		return -EINVAL;
	}

	val = inl(spi->smu->div_sclk) & 0xffff;
	outl(val | spi_plldiv_table[sclk], spi->smu->div_sclk);

	return 0;
}

/*
 * initialize data
 */
static void spi_init_data(struct spi_data_t *spi)
{
	spi->trans->buf.dma_ptr = 0;
	spi->trans->buf.usr_ptr = 0;
	spi->trans->buf.state = SPI_BUFF_EMPTY;
	spi->trans->spi_err = 0;
	spi->trans->dma_err = 0;
}

/*
 * allocate DMA buffer
 */
static int spi_allocate_buffer(struct spi_data_t *spi, unsigned int blksize)
{
	dma_addr_t phys;
	unsigned int blknum;
	void *virt;

	if (spi->trans->buf.dma_addr == 0) {
		blknum = SPI_BUFSIZE_MAX / blksize;
		if (blknum > SPI_BLKNUM_MAX)
			blknum = SPI_BLKNUM_MAX;

		virt = dma_alloc_coherent(NULL, (blksize * blknum), &phys, 0);
		if (virt == NULL) {
			printk(KERN_INFO "%s: memory allocation error\n",
			       SPI_NAME);
			return -ENOMEM;
		}
		spi->trans->buf.blknum = blknum;
		spi->trans->buf.dma_addr = phys;
		spi->trans->buf.top = (unsigned long)virt;
		spi->trans->buf.end = (unsigned long)virt + (blksize * blknum);
		spi_init_data(spi);
	}

	return 0;
}

/*
 * free DMA buffer
 */
static void spi_free_buffer(struct spi_data_t *spi)
{
	if (spi->trans->buf.dma_addr != 0) {
		dma_free_coherent(NULL,
				  (spi->mode->blksize * spi->trans->buf.blknum),
				  (void *)(spi->trans->buf.top),
				  spi->trans->buf.dma_addr);
		spi->trans->buf.blknum = 0;
		spi->trans->buf.dma_addr = 0;
		spi->trans->buf.top = 0;
		spi->trans->buf.end = 0;
		spi_init_data(spi);
	}
}

/*
 * request DMA interrupt
 */
static int spi_request_dma(struct spi_data_t *spi)
{
	int ret = 0;

	if (spi->trans->dma_regs == NULL) {
		if (spi->opened & FMODE_READ) {
			ret =
				emxx_request_dma(spi->trans->rx_lch,
						  SPI_NAME,
						  spi_interrupt_dma_rx,
						  (void *)spi,
						  &spi->trans->dma_regs);
		} else {
			ret =
				emxx_request_dma(spi->trans->tx_lch,
						  SPI_NAME,
						  spi_interrupt_dma_tx,
						  (void *)spi,
						  &spi->trans->dma_regs);
		}
	}
#ifdef SPI_DEBUG
	if (ret < 0)
		printk(KERN_INFO "%s(): error\n", __func__);
#endif

	return ret;
}

/*
 * free DMA interrupt
 */
static void spi_free_dma(struct spi_data_t *spi)
{
	if (spi->trans->dma_regs != NULL) {
		if (spi->opened & FMODE_READ)
			emxx_free_dma(spi->trans->rx_lch);
		else
			emxx_free_dma(spi->trans->tx_lch);
		spi->trans->dma_regs = NULL;
	}
}

/*
 * copy from user
 */
static int spi_copy_from_user(char *dst, const char *src, unsigned int size)
{
	int ret = 0;

	ret = copy_from_user(dst, src, size);
	if (ret < 0) {
#ifdef SPI_DEBUG
		printk(KERN_INFO "%s(): copy error\n", __func__);
#endif
		return -EFAULT;
	}

	return 0;
}

/*
 * copy to user
 */
static int spi_copy_to_user(char *dst, char *src, unsigned int size)
{
	int ret = 0;

	ret = copy_to_user(dst, src, size);
	if (ret < 0) {
#ifdef SPI_DEBUG
		printk(KERN_INFO "%s(): copy error\n", __func__);
#endif
		return -EFAULT;
	}

	return 0;
}

/*
 * set mode
 */
static int spi_set_mode(struct spi_data_t *spi, struct spi_mode_info *mode)
{
	unsigned int mode_val = 0;
	unsigned int control2 = 0;

	int ret = 0;

	/* bit length */
	if ((SPI_NB_8BIT <= mode->nb) && (mode->nb <= SPI_NB_32BIT))
		mode_val |= ((mode->nb - 1) << 8);
	else
		return -EINVAL;

	if (mode->dma == SPI_DMA_ON) {
		switch (mode->nb) {
		case SPI_NB_8BIT:
		case SPI_NB_16BIT:
		case SPI_NB_32BIT:
			break;
		default:
			return -EINVAL;
		}
	}

	/* chip select */
	switch (mode->cs_sel) {
	case SPI_CS_SEL_CS0:
	case SPI_CS_SEL_CS1:
	case SPI_CS_SEL_CS2:
	case SPI_CS_SEL_CS3:
	case SPI_CS_SEL_CS4:
	case SPI_CS_SEL_CS5:
	case SPI_CS_SEL_CS6:
	case SPI_CS_SEL_CS7:
		mode_val |= (mode->cs_sel << 4);
		break;
	default:
		return -EINVAL;
	}

	mode_val |= (mode->m_s << 1);

	/* dma on/off */
	switch (mode->dma) {
	case SPI_DMA_OFF:
		if ((spi->opened & FMODE_READ)
		    || ((spi->opened & FMODE_WRITE)
			&& (mode->m_s == SPI_M_S_SLAVE))) {
			mode->dma = SPI_DMA_ON;
		}
	/* through */
	case SPI_DMA_ON:
		mode_val |= (mode->dma << 0);
		break;
	default:
		return -EINVAL;
	}

	if ((mode->dma == SPI_DMA_ON) && (mode->m_s == SPI_M_S_MASTER))
		control2 = SPx_CONTROL2_TX_STOP_MODE;

	/* block size */
	if ((mode->blksize == 0) || (mode->blksize > SPI_BLKSIZE_MAX) ||
	    ((mode->blksize % 4) != 0)) {
		return -EINVAL;
	}

	/* sclk */
	switch (mode->sclk) {
	case SPI_SCLK_1500KHZ:
	case SPI_SCLK_3MHZ:
	case SPI_SCLK_6MHZ:
	case SPI_SCLK_12MHZ:
	case SPI_SCLK_24MHZ:
	case SPI_SCLK_48MHZ:
		break;
	default:
		return -EINVAL;
	}

	/* set sclk */
	if (spi_set_sclk_div(spi, mode->sclk) < 0)
		return -EINVAL;

	/* set SPx_MODE */
	spi->regs->mode = mode_val;

	/* set SPx_POL */
	spi->regs->pol = spi->pol;

	/* set SPx_CONTROL2 */
	spi->regs->control2 = control2;

	/* software reset */
	spi_sft_reset(spi);

	if (mode->blksize != spi->mode->blksize)
		spi_free_buffer(spi);
	ret = spi_allocate_buffer(spi, mode->blksize);
	if (ret < 0)
		return ret;

	if (mode->dma == SPI_DMA_ON)
		ret = spi_request_dma(spi);
	else
		spi_free_dma(spi);

	return ret;
}

/*
 * SPI stop
 */
static void spi_stop(struct spi_data_t *spi)
{
	if (spi->trans->state == SPI_STOP)
		return;

	if (spi->mode->dma == SPI_DMA_ON) {
		if (spi->mode->m_s == SPI_M_S_MASTER) {
			if (spi->regs->control & SPx_CONTROL_START) {
				spi->regs->control |= SPx_CONTROL_STOP;
				udelay(((1000 * 32) /
					spi_clock_table[spi->mode->sclk]) + 1);
			}
		}
	}

	/* interrupt disable */
	spi->regs->enclr = SPx_ENCLR_ALL_MASK;

	/* status to stop */
	spi->trans->state = SPI_STOP;
}

/*
 * RX stop
 */
static void spi_rx_stop(struct spi_data_t *spi)
{
	if (spi->mode->dma == SPI_DMA_ON) {
		/* SPI stop */
		spi_stop(spi);
		/* DMA stop */
		emxx_stop_dma(spi->trans->rx_lch);

		if (spi->mode->m_s == SPI_M_S_MASTER) {
			spi->regs->control2 &=  ~(SPx_CONTROL2_RX_STOP_MODE
					  | SPx_CONTROL2_RX_FIFO_FULL_MASK);
		}
	} else {
		/* SPI stop */
		spi_stop(spi);
	}
}

/*
 * TX stop
 */
static void spi_tx_stop(struct spi_data_t *spi)
{
	if (spi->mode->dma == SPI_DMA_ON) {
		/* SPI stop */
		spi_stop(spi);

		/* DMA stop */
		emxx_stop_dma(spi->trans->tx_lch);

		if (spi->mode->m_s == SPI_M_S_MASTER)
			spi->regs->control2 &= ~SPx_CONTROL2_TX_STOP_MODE;
	} else {
		/* SPI stop */
		spi_stop(spi);
	}
}

/*
 * RX start
 */
static int spi_rx_start(struct spi_data_t *spi, int size)
{
	int intmask = 0;
	int ret = 0;

	/* status to active */
	spi->trans->state = SPI_ACTIVE;

	/* initialize error status */
	spi->regs->ffclr = SPx_FFCLR_ALL_CLR;

	/* interrupt enable (TERR/TX_UDR/RX_OVR) */
	spi->regs->enset |= SPx_ENSET_ALLERR_EN;

	if (spi->mode->dma == SPI_DMA_ON) {
		unsigned int mode = 0;
		switch (spi->mode->nb) {
		case SPI_NB_8BIT:
			mode = EMXX_DMAC_DEFMODE_8BIT;
			break;
		case SPI_NB_16BIT:
			mode = EMXX_DMAC_DEFMODE_16BIT;
			break;
		case SPI_NB_32BIT:
			mode = EMXX_DMAC_DEFMODE_32BIT;
			break;
		default:
			return -EINVAL;
		}

		if (spi->mode->m_s == SPI_M_S_MASTER) {
			/* DMA start (length) */
			spi->trans->dma_regs->mode = mode;
			spi->trans->dma_regs->boff = 0;
			spi->trans->dma_regs->leng = size;
			spi->trans->dma_regs->bsize =
				(size > SPI_DMA_BLOCK_SIZE_MAX) ?
					SPI_DMA_BLOCK_SIZE_MAX : size;
			spi->trans->dma_regs->bsize_count = 0;
			intmask = EMXX_DMAC_INT_ERROR_EN |
					EMXX_DMAC_INT_LENG_EN;
		} else {
			/* DMA start (infinity length) */
			spi->trans->dma_regs->mode =
				EMXX_DMAC_BMODE_REPEAT | mode;
			spi->trans->dma_regs->boff = 0;
			spi->trans->dma_regs->leng = 0;
			spi->trans->dma_regs->bsize = spi->mode->blksize;
			spi->trans->dma_regs->bsize_count =
				spi->trans->buf.blknum - 1;
			intmask = EMXX_DMAC_INT_ERROR_EN |
					EMXX_DMAC_INT_BLOCK_EN;
		}
		ret = emxx_start_dma(spi->trans->rx_lch, spi->trans->rx_data,
				0, spi->trans->buf.dma_addr, intmask);
		if (ret < 0) {
			spi_rx_stop(spi);
#ifdef SPI_DEBUG
			printk(KERN_INFO "%s(): dma error\n", __func__);
#endif
			return ret;
		}

		if (spi->mode->m_s == SPI_M_S_MASTER) {
			unsigned int full =
				((size / spi_unit(spi, spi->mode->nb)) - 1);
			if (full < 0x10000) {
				spi->regs->control2 &=
					~SPx_CONTROL2_RX_FIFO_FULL_MASK;

				spi->regs->control2 |=
					(((full & 0xff00) << 8) | (full & 0xff)
					 | SPx_CONTROL2_RX_STOP_MODE);
			}
		}
	}

	/* RX start */
	spi->regs->control |= (SPx_CONTROL_RD | SPx_CONTROL_START);

	return 0;
}

/*
 * TX start
 */
static int spi_tx_start(struct spi_data_t *spi)
{
	unsigned int addr = 0;
	unsigned int bufsize = 0;
	int intmask = 0;
	int size = 0;
	int ret = 0;

	/* buffer is empty */
	if (spi->trans->buf.state & SPI_BUFF_EMPTY)
		return 0;

	/* status to active */
	spi->trans->state = SPI_ACTIVE;

	bufsize = spi->mode->blksize * spi->trans->buf.blknum;
	size = spi->trans->buf.usr_ptr - spi->trans->buf.dma_ptr;
	if (size <= 0)
		size = bufsize - spi->trans->buf.dma_ptr;

	/* initialize error status */
	spi->regs->ffclr = SPx_FFCLR_ALL_CLR;

	/* interrupt enable (TERR/TX_UDR/RX_OVR) */
	spi->regs->enset |= SPx_ENSET_ALLERR_EN;

	if (spi->mode->dma == SPI_DMA_ON) {
		unsigned int mode = 0;
		switch (spi->mode->nb) {
		case SPI_NB_8BIT:
			mode = EMXX_DMAC_DEFMODE_8BIT;
			break;
		case SPI_NB_16BIT:
			mode = EMXX_DMAC_DEFMODE_16BIT;
			break;
		case SPI_NB_32BIT:
			mode = EMXX_DMAC_DEFMODE_32BIT;
			break;
		default:
			return -EINVAL;
		}
		spi->trans->dma_regs->mode = mode;
		spi->trans->dma_regs->aoff = 0;
		spi->trans->dma_regs->leng = size;
		spi->trans->dma_regs->asize =
			(size > SPI_DMA_BLOCK_SIZE_MAX) ?
			SPI_DMA_BLOCK_SIZE_MAX : size;
		spi->trans->dma_regs->asize_count = 0;
		intmask = EMXX_DMAC_INT_ERROR_EN | EMXX_DMAC_INT_LENG_EN;

		if (spi->mode->m_s == SPI_M_S_MASTER) {
			/* interrupt enable (TX_STOP) */
			spi->regs->enset |= SPx_ENSET_TX_STOP_EN;
		}
		ret =
			emxx_start_dma(spi->trans->tx_lch,
					spi->trans->buf.dma_addr +
					spi->trans->buf.dma_ptr, 0,
					spi->trans->tx_data, intmask);
		if (ret < 0) {
			spi_tx_stop(spi);
#ifdef SPI_DEBUG
			printk(KERN_INFO "%s(): dma error\n", __func__);
#endif
			return ret;
		}
	} else {
		/* interrupt enable (END) */
		spi->regs->enset |= SPx_ENSET_END_EN;

		/* set data */
		addr = spi->trans->buf.top + spi->trans->buf.dma_ptr;
		spi->regs->tx_data = *(unsigned int *)addr;
	}

	/* TX start */
	spi->regs->control |= (SPx_CONTROL_WRT | SPx_CONTROL_START);

	return 0;
}

/*
 * SPI interrupt callback
 */
static irqreturn_t spi_interrupt(int irq, void *dev_id)
{
	struct spi_data_t *spi;
	unsigned int status = 0;
	unsigned int bufsize = 0;
	int ret = 0;

	if (dev_id == NULL)
		return IRQ_NONE;
	spi = (struct spi_data_t *) dev_id;

	/* interrupt status */
	status = spi->regs->status;

	/* initialize interrupt status */
	spi->regs->ffclr |= status;

	/* error check (TERR/TX_UDR/RX_OVR) */
	if (status & SPx_STATUS_ALLERR) {
		spi->trans->spi_err = (status & SPx_STATUS_ALLERR);
		if (status & SPx_STATUS_TX_UDR) {
			if (spi->mode->m_s == SPI_M_S_MASTER) {
				spi_tx_stop(spi);
			} else {
				if (spi->trans->dma_regs->wcount == 0) {
					spi->trans->spi_err &=
						~(SPx_STATUS_TX_UDR);
					if (spi->trans->buf.state
					    & SPI_BUFF_EMPTY) {
						spi_tx_stop(spi);
					} else {
						ret = spi_tx_start(spi);
						if (ret < 0)
							spi_tx_stop(spi);
					}
				} else {
					spi_tx_stop(spi);
				}
			}
		}
		if (status & SPx_STATUS_RX_OVR) {
			if (spi->mode->m_s == SPI_M_S_MASTER) {
				if (spi->trans->dma_regs->rcount == 0) {
					spi->trans->spi_err &=
						~(SPx_STATUS_RX_OVR);
					spi_stop(spi);
					return IRQ_HANDLED;
				} else {
					spi_rx_stop(spi);
				}
			} else {
				spi_rx_stop(spi);
			}
		}
		if (status & SPx_STATUS_TERR)
			spi_rx_stop(spi);
		if (waitqueue_active(&spi->trans->wait))
			wake_up_interruptible(&spi->trans->wait);
#ifdef SPI_DEBUG
		printk(KERN_INFO "%s(): status = %08x\n", __func__, status);
#endif
		return IRQ_HANDLED;
	}
#ifdef SPI_DEBUG
	if (status & SPx_STATUS_RDV) {
		printk(KERN_INFO "%s(): RDV is not supported\n", __func__);
		return IRQ_NONE;
	}
#endif

	if (status & SPx_STATUS_TX_STOP) { /* master only */
		if (spi->regs->control & SPx_CONTROL_TX_EMP) {
			if (spi->trans->buf.state & SPI_BUFF_EMPTY) {
				spi_tx_stop(spi);
			} else {
				ret = spi_tx_start(spi);
				if (ret < 0)
					spi_tx_stop(spi);
			}
		} else { /* error */
			spi->trans->spi_err = SPx_STATUS_TX_STOP;
			spi_tx_stop(spi);
		}
		if (waitqueue_active(&spi->trans->wait))
			wake_up_interruptible(&spi->trans->wait);
#ifdef SPI_DEBUG
		printk(KERN_INFO "%s(): status = %08x\n", __func__, status);
#endif
		return IRQ_HANDLED;
	}

	/* CPU write */
	if (status & SPx_STATUS_END) {
		/* update dma pointer */
		bufsize = spi->mode->blksize * spi->trans->buf.blknum;
		spi->trans->buf.dma_ptr = spi->trans->buf.dma_ptr + 4;
		if (spi->trans->buf.dma_ptr == bufsize)
			spi->trans->buf.dma_ptr = 0;

		/* update buffer status */
		if (spi->trans->buf.dma_ptr == spi->trans->buf.usr_ptr)
			spi->trans->buf.state |= SPI_BUFF_EMPTY;
		if (spi->trans->buf.state & SPI_BUFF_FULL)
			spi->trans->buf.state &= ~(SPI_BUFF_FULL);

		if (spi->trans->buf.state & SPI_BUFF_EMPTY) {
			spi_tx_stop(spi);
			if (waitqueue_active(&spi->trans->wait))
				wake_up_interruptible(&spi->trans->wait);
		} else {
			ret = spi_tx_start(spi);
			if (ret < 0) {
				spi_tx_stop(spi);
				if (waitqueue_active(&spi->trans->wait)) {
					wake_up_interruptible(&spi->trans->
							      wait);
				}
				return IRQ_HANDLED;
			}
		}
	}

	return IRQ_HANDLED;
}

/*
 * DMA interrupt callback (RX)
 */
static void spi_interrupt_dma_rx(void *data, int intsts, int intrawsts)
{
	struct spi_data_t *spi;
	unsigned int dma_pos = 0;
	unsigned int bufsize = 0;
	int size = 0;
	int blanksize = 0;

	if (data == NULL)
		return;
	spi = (struct spi_data_t *) data;

	/* error check */
	if (intsts & (EMXX_DMAC_INT_ERROR_WR | EMXX_DMAC_INT_ERROR_RD)) {
		spi->trans->dma_err =
			(intsts &
			 (EMXX_DMAC_INT_ERROR_WR | EMXX_DMAC_INT_ERROR_RD));
		spi_rx_stop(spi);
		if (waitqueue_active(&spi->trans->wait))
			wake_up_interruptible(&spi->trans->wait);
#ifdef SPI_DEBUG
		printk(KERN_INFO "%s(): intsts = %08x\n", __func__, intsts);
#endif
		return;
	}

	if (((spi->mode->m_s == SPI_M_S_MASTER)
	     && (intsts & EMXX_DMAC_INT_LENG_WR))
	    || ((spi->mode->m_s == SPI_M_S_SLAVE)
		&& (intsts & EMXX_DMAC_INT_BLOCK_WR))) {
		bufsize = spi->mode->blksize * spi->trans->buf.blknum;
		dma_pos = emxx_get_dma_pos(spi->trans->rx_lch);
		dma_pos = dma_pos - spi->trans->buf.dma_addr;
		size = dma_pos - spi->trans->buf.dma_ptr;
		if (size <= 0)
			size = size + bufsize;

		/* calculate blank size */
		if (spi->trans->buf.state & SPI_BUFF_FULL) {
			blanksize = 0;
		} else {
			blanksize = spi->trans->buf.usr_ptr -
					spi->trans->buf.dma_ptr;
			if (blanksize <= 0)
				blanksize = blanksize + bufsize;
		}

		/* update dma counter */
		spi->trans->buf.dma_ptr = spi->trans->buf.dma_ptr + size;
		if (spi->trans->buf.dma_ptr >= bufsize) {
			spi->trans->buf.dma_ptr =
				spi->trans->buf.dma_ptr - bufsize;
		}

		/* update buffer status */
		if (blanksize < size) {
			spi->trans->buf.state |=
				(SPI_BUFF_FULL | SPI_BUFF_OVER_ERR);
			spi_rx_stop(spi);
#ifdef SPI_DEBUG
			printk(KERN_INFO "%s(): buffer overwrited\n",
			       __func__);
#endif
		} else {
			if (spi->trans->buf.dma_ptr == spi->trans->buf.usr_ptr)
				spi->trans->buf.state |= SPI_BUFF_FULL;
			if (spi->trans->buf.state & SPI_BUFF_EMPTY)
				spi->trans->buf.state &= ~(SPI_BUFF_EMPTY);
		}

		if (spi->mode->m_s == SPI_M_S_MASTER)
			spi_rx_stop(spi);

		if (waitqueue_active(&spi->trans->wait))
			wake_up_interruptible(&spi->trans->wait);
	}
}

/*
 * DMA interrupt callback (TX)
 */
static void spi_interrupt_dma_tx(void *data, int intsts, int intrawsts)
{
	struct spi_data_t *spi;
	if (data == NULL)
		return;
	spi = (struct spi_data_t *) data;

	/* error check */
	if (intsts & (EMXX_DMAC_INT_ERROR_WR | EMXX_DMAC_INT_ERROR_RD)) {
		spi->trans->dma_err =
			(intsts &
			 (EMXX_DMAC_INT_ERROR_WR | EMXX_DMAC_INT_ERROR_RD));
		spi_tx_stop(spi);
		if (waitqueue_active(&spi->trans->wait))
			wake_up_interruptible(&spi->trans->wait);
#ifdef SPI_DEBUG
		printk(KERN_INFO "%s(): intsts = %08x\n", __func__, intsts);
#endif
		return;
	}

	if (intsts & EMXX_DMAC_INT_LENG_WR) {
		spi->trans->buf.dma_ptr =
			emxx_get_dma_pos(spi->trans->tx_lch)
			- spi->trans->buf.dma_addr;
		if (spi->trans->buf.dma_ptr ==
		    (spi->mode->blksize * spi->trans->buf.blknum)) {
			spi->trans->buf.dma_ptr = 0;
		}
#ifdef SPI_DEBUG
		printk(KERN_INFO "%s(): dma ptr = %d\n", __func__,
		       (spi->trans->buf.dma_ptr));
#endif
		/* update buffer status */
		if (spi->trans->buf.dma_ptr == spi->trans->buf.usr_ptr)
			spi->trans->buf.state |= SPI_BUFF_EMPTY;
		if (spi->trans->buf.state & SPI_BUFF_FULL)
			spi->trans->buf.state &= ~(SPI_BUFF_FULL);

		if (waitqueue_active(&spi->trans->wait))
			wake_up_interruptible(&spi->trans->wait);
	}
}

/*
 * read
 */
static ssize_t spi_read_func(unsigned char dev, struct file *file,
		char __user *buf, size_t count, loff_t *ppos)
{
	DECLARE_WAITQUEUE(wait, current);
	struct spi_data_t *spi;
	unsigned int errsts;
	unsigned int dma_ptr;
	unsigned int usr_ptr;
	unsigned int end;
	unsigned int cpsize;
	unsigned int bufsize;
	unsigned int len = 0;
	int size = 0;
	int ret = 0;

#ifdef SPI_DEBUG
	printk(KERN_INFO "%s(): count = %d\n", __func__, count);
#endif

	/* device is not found */
	if (DEVNO_IS_INVALID(dev))
		return -ENODEV;

	spi = &spi_private[dev];

	/* seek is not supported */
	if (*ppos != file->f_pos)
		return -ESPIPE;

	/* size check */
	if (count == 0)
		return 0;
	if (count < 4)
		return -EINVAL;
	if ((count % 4) != 0)
		count = (count / 4) * 4;

	/* RX start */
	bufsize = spi->mode->blksize * spi->trans->buf.blknum;
	add_wait_queue(&spi->trans->wait, &wait);
	while ((count - len) > 0) {
		set_current_state(TASK_INTERRUPTIBLE);

		spin_lock_irq(&spi->trans->spinlock);
		errsts = spi->trans->spi_err;
		spi->trans->spi_err = 0;
		spin_unlock_irq(&spi->trans->spinlock);
		if (errsts != 0) {
			ret = -EIO;
			break;
		}
		/* DMA error check */
		spin_lock_irq(&spi->trans->spinlock);
		errsts = spi->trans->dma_err;
		spi->trans->dma_err = 0;
		spin_unlock_irq(&spi->trans->spinlock);
		if (errsts != 0) {
			ret = -EIO;
			break;
		}
		/* buffer error check */
		spin_lock_irq(&spi->trans->spinlock);
		errsts = spi->trans->buf.state & SPI_BUFF_OVER_ERR;
		spi->trans->buf.state &= ~SPI_BUFF_OVER_ERR;
		spin_unlock_irq(&spi->trans->spinlock);
		if (errsts != 0) {
			ret = -EIO;
			break;
		}

		if (!(spi->trans->state == SPI_ACTIVE)
		    && (spi->trans->buf.state & SPI_BUFF_EMPTY)) {
			ret = spi_allocate_buffer(spi, spi->mode->blksize);
			if (ret < 0)
				return ret;
			if (spi->mode->dma == SPI_DMA_ON) {
				ret = spi_request_dma(spi);
				if (ret < 0)
					return ret;
			}
			spi_init_data(spi);

			if (spi->mode->m_s == SPI_M_S_MASTER) {
				size =
					((count - len) > bufsize) ?
					bufsize : (count - len);
			} else {
				size = 0;
			}
			ret = spi_rx_start(spi, size);
			if (ret < 0)
				return ret;
		}

		/* readable */
		if (!(spi->trans->buf.state & SPI_BUFF_EMPTY)) {
			dma_ptr = spi->trans->buf.top + spi->trans->buf.dma_ptr;
			usr_ptr = spi->trans->buf.top + spi->trans->buf.usr_ptr;
			end = spi->trans->buf.end;
			cpsize = (dma_ptr > usr_ptr) ?
					(dma_ptr - usr_ptr) : (end - usr_ptr);
			if (cpsize > (count - len))
				cpsize = count - len;
			ret =
				spi_copy_to_user((char *)(buf + len),
						 (char *)usr_ptr, cpsize);
			if (ret < 0)
				break;
			len = len + cpsize;

			/* update user pointer */

			spin_lock_irq(&spi->trans->spinlock);
			spi->trans->buf.usr_ptr =
				spi->trans->buf.usr_ptr + cpsize;
			if (spi->trans->buf.usr_ptr == bufsize)
				spi->trans->buf.usr_ptr = 0;

			/* update buffer status */
			if (spi->trans->buf.usr_ptr == spi->trans->buf.dma_ptr)
				spi->trans->buf.state |= SPI_BUFF_EMPTY;
			if (spi->trans->buf.state & SPI_BUFF_FULL)
				spi->trans->buf.state &= ~(SPI_BUFF_FULL);
			spin_unlock_irq(&spi->trans->spinlock);
			if (spi->mode->m_s == SPI_M_S_MASTER) {
				size =
					((count - len) >
					 bufsize) ? bufsize : (count - len);
				if (size > 0) {
					spi_init_data(spi);
					ret = spi_rx_start(spi, size);
					if (ret < 0)
						break;
				}
			}
		} else {
			if (file->f_flags & O_NONBLOCK) {
				if (len == 0)
					ret = -EAGAIN;
				break;
			}
			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
				break;
			}
			schedule();
		}
	}
	remove_wait_queue(&spi->trans->wait, &wait);
	set_current_state(TASK_RUNNING);

	if (ret < 0) {
		if (ret != -EAGAIN) {
			if (spi->trans->state == SPI_ACTIVE)
				spi_rx_stop(spi);
		}
		return ret;
	}

	/* return copy size */
	return len;
}

int
emxx_spi_read(unsigned char dev, char *buf, unsigned int count,
		unsigned char block_mode)
{
	DECLARE_WAITQUEUE(wait, current);
	struct spi_data_t *spi;
	unsigned int errsts;
	unsigned int dma_ptr;
	unsigned int usr_ptr;
	unsigned int end;
	unsigned int cpsize;
	unsigned int bufsize;
	unsigned int len = 0;
	int size = 0;
	int ret = 0;

#ifdef SPI_DEBUG
	printk(KERN_INFO "%s(): count = %d\n", __func__, count);
#endif

	if (DEVNO_IS_INVALID(dev))
		return -EACCES;

	spi = &spi_private[dev];

	if (spi->k_flag != SPI_READ_MODE)
		return -EACCES;

	/* size check */
	if (count == 0)
		return 0;
	if (count < 4)
		return -EINVAL;
	if ((count % 4) != 0)
		count = (count / 4) * 4;

	if (down_interruptible(&spi->sem_rw))
		return -ERESTARTSYS;

	/* RX start */
	bufsize = spi->mode->blksize * spi->trans->buf.blknum;
	add_wait_queue(&spi->trans->wait, &wait);
	while ((count - len) > 0) {
		set_current_state(TASK_INTERRUPTIBLE);

		spin_lock_irq(&spi->trans->spinlock);
		errsts = spi->trans->spi_err;
		spi->trans->spi_err = 0;
		spin_unlock_irq(&spi->trans->spinlock);
		if (errsts != 0) {
			ret = -EIO;
			break;
		}
		/* DMA error check */
		spin_lock_irq(&spi->trans->spinlock);
		errsts = spi->trans->dma_err;
		spi->trans->dma_err = 0;
		spin_unlock_irq(&spi->trans->spinlock);
		if (errsts != 0) {
			ret = -EIO;
			break;
		}
		/* buffer error check */
		spin_lock_irq(&spi->trans->spinlock);
		errsts = spi->trans->buf.state & SPI_BUFF_OVER_ERR;
		spi->trans->buf.state &= ~SPI_BUFF_OVER_ERR;
		spin_unlock_irq(&spi->trans->spinlock);
		if (errsts != 0) {
			ret = -EIO;
			break;
		}

		if (!(spi->trans->state == SPI_ACTIVE)
		    && (spi->trans->buf.state & SPI_BUFF_EMPTY)) {
			ret = spi_allocate_buffer(spi, spi->mode->blksize);
			if (ret < 0) {
				up(&spi->sem_rw);
				return ret;
			}
			if (spi->mode->dma == SPI_DMA_ON) {
				ret = spi_request_dma(spi);
				if (ret < 0) {
					up(&spi->sem_rw);
					return ret;
				}
			}
			spi_init_data(spi);

			if (spi->mode->m_s == SPI_M_S_MASTER) {
				size =
					((count - len) > bufsize) ?
					bufsize : (count - len);
			} else {
				size = 0;
			}
			ret = spi_rx_start(spi, size);
			if (ret < 0) {
				up(&spi->sem_rw);
				return ret;
			}
		}

		/* readable */
		if (!(spi->trans->buf.state & SPI_BUFF_EMPTY)) {
			dma_ptr = spi->trans->buf.top + spi->trans->buf.dma_ptr;
			usr_ptr = spi->trans->buf.top + spi->trans->buf.usr_ptr;
			end = spi->trans->buf.end;
			cpsize = (dma_ptr > usr_ptr) ?
					(dma_ptr - usr_ptr) : (end - usr_ptr);
			if (cpsize > (count - len))
				cpsize = count - len;
			memcpy((char *)(buf + len),
			       (char *)usr_ptr, cpsize);

			len = len + cpsize;

			/* update user pointer */
			spin_lock_irq(&spi->trans->spinlock);

			spi->trans->buf.usr_ptr =
				spi->trans->buf.usr_ptr + cpsize;
			if (spi->trans->buf.usr_ptr == bufsize)
				spi->trans->buf.usr_ptr = 0;

			/* update buffer status */
			if (spi->trans->buf.usr_ptr == spi->trans->buf.dma_ptr)
				spi->trans->buf.state |= SPI_BUFF_EMPTY;
			if (spi->trans->buf.state & SPI_BUFF_FULL)
				spi->trans->buf.state &= ~(SPI_BUFF_FULL);
			spin_unlock_irq(&spi->trans->spinlock);
			if (spi->mode->m_s == SPI_M_S_MASTER) {
				size =
					((count - len) >
					 bufsize) ? bufsize : (count - len);
				if (size > 0) {
					spi_init_data(spi);
					ret = spi_rx_start(spi, size);
					if (ret < 0)
						break;
				}
			}
		} else {
			if (block_mode & SPI_NONBLOCK) {
				if (len == 0)
					ret = -EAGAIN;
				break;
			}
			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
				break;
			}
			schedule();
		}
	}
	remove_wait_queue(&spi->trans->wait, &wait);
	set_current_state(TASK_RUNNING);

	if (ret < 0) {
		if (ret != -EAGAIN) {
			if (spi->trans->state == SPI_ACTIVE)
				spi_rx_stop(spi);
		}
		up(&spi->sem_rw);
		return ret;
	}

	/* return copy size */
	up(&spi->sem_rw);
	return len;
}
EXPORT_SYMBOL(emxx_spi_read);


/*
 * write
 */
static ssize_t spi_write_func(unsigned char dev, struct file *file,
		const char __user *buf, size_t count, loff_t *ppos)
{
	DECLARE_WAITQUEUE(wait, current);
	struct spi_data_t *spi;
	unsigned int errsts;
	unsigned int dma_ptr;
	unsigned int usr_ptr;
	unsigned int end;
	unsigned int cpsize;
	unsigned int bufsize;
	unsigned int len = 0;
	int ret = 0;

#ifdef SPI_DEBUG
	printk(KERN_INFO "%s(): count = %d\n", __func__, count);
#endif

	/* device is not found */
	if (DEVNO_IS_INVALID(dev))
		return -ENODEV;

	spi = &spi_private[dev];

	/* seek is not supported */
	if (*ppos != file->f_pos)
		return -ESPIPE;

	/* size check */
	if (count == 0)
		return 0;
	if (count < 4)
		return -EINVAL;
	if ((count % 4) != 0)
		count = (count / 4) * 4;

	add_wait_queue(&spi->trans->wait, &wait);
	while ((count - len) > 0) {
		set_current_state(TASK_INTERRUPTIBLE);

		spin_lock_irq(&spi->trans->spinlock);
		errsts = spi->trans->spi_err;
		spi->trans->spi_err = 0;
		spin_unlock_irq(&spi->trans->spinlock);
		if (errsts != 0) {
			ret = -EIO;
			break;
		}
		/* DMA error check */
		spin_lock_irq(&spi->trans->spinlock);
		errsts = spi->trans->dma_err;
		spi->trans->dma_err = 0;
		spin_unlock_irq(&spi->trans->spinlock);
		if (errsts != 0) {
			ret = -EIO;
			break;
		}

		if (!(spi->trans->state == SPI_ACTIVE)
		    && (spi->trans->buf.state & SPI_BUFF_EMPTY)) {
			ret = spi_allocate_buffer(spi, spi->mode->blksize);
			if (ret < 0)
				return ret;
			if (spi->mode->dma == SPI_DMA_ON) {
				ret = spi_request_dma(spi);
				if (ret < 0)
					return ret;
			}
			spi_init_data(spi);
		}

		/* writable */
		if (!(spi->trans->buf.state & SPI_BUFF_FULL)) {
			dma_ptr = spi->trans->buf.top + spi->trans->buf.dma_ptr;
			usr_ptr = spi->trans->buf.top + spi->trans->buf.usr_ptr;
			end = spi->trans->buf.end;
			cpsize = (dma_ptr > usr_ptr) ?
					(dma_ptr - usr_ptr) : (end - usr_ptr);
			if (cpsize > (count - len))
				cpsize = count - len;
			ret = spi_copy_from_user((char *)usr_ptr,
					(const char *)(buf + len), cpsize);
			if (ret < 0)
				break;
			len = len + cpsize;

			/* update user pointer */
			spin_lock_irq(&spi->trans->spinlock);
			spi->trans->buf.usr_ptr =
				spi->trans->buf.usr_ptr + cpsize;
			bufsize = spi->mode->blksize * spi->trans->buf.blknum;
			if (spi->trans->buf.usr_ptr == bufsize)
				spi->trans->buf.usr_ptr = 0;

			/* update buffer status */
			if (spi->trans->buf.usr_ptr == spi->trans->buf.dma_ptr)
				spi->trans->buf.state |= SPI_BUFF_FULL;
			if (spi->trans->buf.state & SPI_BUFF_EMPTY)
				spi->trans->buf.state &= ~(SPI_BUFF_EMPTY);
			spin_unlock_irq(&spi->trans->spinlock);

			/* TX start */
			if (!(spi->trans->state == SPI_ACTIVE)) {
				ret = spi_tx_start(spi);
				if (ret < 0)
					break;
			}
		} else {
			if (file->f_flags & O_NONBLOCK) {
				if (len == 0)
					ret = -EAGAIN;
				break;
			}
			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
				break;
			}
			schedule();
		}
	}
	remove_wait_queue(&spi->trans->wait, &wait);
	set_current_state(TASK_RUNNING);

	if (ret < 0) {
		if (ret != -EAGAIN) {
			if (spi->trans->state == SPI_ACTIVE)
				spi_tx_stop(spi);
		}
		return ret;
	}

	/* return copy size */
	return len;
}

int
emxx_spi_write(unsigned char dev, const char *buf,  unsigned int count,
		unsigned char block_mode)
{
	DECLARE_WAITQUEUE(wait, current);
	struct spi_data_t *spi;
	unsigned int errsts;
	unsigned int dma_ptr;
	unsigned int usr_ptr;
	unsigned int end;
	unsigned int cpsize;
	unsigned int bufsize;
	unsigned int len = 0;
	int ret = 0;

#ifdef SPI_DEBUG
	printk(KERN_INFO "%s(): count = %d\n", __func__, count);
#endif

	/* device is not found */
	if (DEVNO_IS_INVALID(dev))
		return -EACCES;

	spi = &spi_private[dev];

	if (spi->k_flag != SPI_WRITE_MODE)
		return -EACCES;

	/* size check */
	if (count == 0)
		return 0;
	if (count < 4)
		return -EINVAL;
	if ((count % 4) != 0)
		count = (count / 4) * 4;

	if (down_interruptible(&spi->sem_rw))
		return -ERESTARTSYS;
	add_wait_queue(&spi->trans->wait, &wait);
	while ((count - len) > 0) {
		set_current_state(TASK_INTERRUPTIBLE);

		spin_lock_irq(&spi->trans->spinlock);
		errsts = spi->trans->spi_err;
		spi->trans->spi_err = 0;
		spin_unlock_irq(&spi->trans->spinlock);
		if (errsts != 0) {
			ret = -EIO;
			break;
		}
		/* DMA error check */
		spin_lock_irq(&spi->trans->spinlock);
		errsts = spi->trans->dma_err;
		spi->trans->dma_err = 0;
		spin_unlock_irq(&spi->trans->spinlock);
		if (errsts != 0) {
			ret = -EIO;
			break;
		}

		if (!(spi->trans->state == SPI_ACTIVE)
		    && (spi->trans->buf.state & SPI_BUFF_EMPTY)) {
			ret = spi_allocate_buffer(spi, spi->mode->blksize);
			if (ret < 0) {
				up(&spi->sem_rw);
				return ret;
			}
			if (spi->mode->dma == SPI_DMA_ON) {
				ret = spi_request_dma(spi);
				if (ret < 0) {
					up(&spi->sem_rw);
					return ret;
				}
			}
			spi_init_data(spi);
		}

		/* writable */
		if (!(spi->trans->buf.state & SPI_BUFF_FULL)) {
			dma_ptr = spi->trans->buf.top + spi->trans->buf.dma_ptr;
			usr_ptr = spi->trans->buf.top + spi->trans->buf.usr_ptr;
			end = spi->trans->buf.end;
			cpsize = (dma_ptr > usr_ptr) ?
					(dma_ptr - usr_ptr) : (end - usr_ptr);
			if (cpsize > (count - len))
				cpsize = count - len;
			memcpy((char *)usr_ptr,
			       (char *)(buf + len), cpsize);

			len = len + cpsize;

			/* update user pointer */
			spin_lock_irq(&spi->trans->spinlock);
			spi->trans->buf.usr_ptr =
				spi->trans->buf.usr_ptr + cpsize;
			bufsize = spi->mode->blksize * spi->trans->buf.blknum;
			if (spi->trans->buf.usr_ptr == bufsize)
				spi->trans->buf.usr_ptr = 0;

			/* update buffer status */
			if (spi->trans->buf.usr_ptr == spi->trans->buf.dma_ptr)
				spi->trans->buf.state |= SPI_BUFF_FULL;
			if (spi->trans->buf.state & SPI_BUFF_EMPTY)
				spi->trans->buf.state &= ~(SPI_BUFF_EMPTY);
			spin_unlock_irq(&spi->trans->spinlock);

			/* TX start */
			if (!(spi->trans->state == SPI_ACTIVE)) {
				ret = spi_tx_start(spi);
				if (ret < 0)
					break;
			}
		} else {
			if (block_mode & SPI_NONBLOCK) {
				if (len == 0)
					ret = -EAGAIN;
				break;
			}
			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
				break;
			}
			schedule();
		}
	}
	remove_wait_queue(&spi->trans->wait, &wait);
	set_current_state(TASK_RUNNING);

	if (ret < 0) {
		if (ret != -EAGAIN) {
			if (spi->trans->state == SPI_ACTIVE)
				spi_tx_stop(spi);
		}
		up(&spi->sem_rw);
		return ret;
	}
	up(&spi->sem_rw);
	/* return copy size */
	return len;
}
EXPORT_SYMBOL(emxx_spi_write);

/*
 * poll
 */
static unsigned int spi_poll(unsigned char dev, struct file *file,
			     poll_table *wait)
{
	struct spi_data_t *spi;
	unsigned int mask = 0;

	/* device is not found */
	if (DEVNO_IS_INVALID(dev)) {
		/* error */
		return POLLERR;
	}

	spi = &spi_private[dev];

	poll_wait(file, &spi->trans->wait, wait);

	/* readable */
	if (spi->opened & FMODE_READ) {
		if (!(spi->trans->buf.state & SPI_BUFF_EMPTY))
			mask |= (POLLIN | POLLRDNORM);
	}

	/* writable */
	if (spi->opened & FMODE_WRITE) {
		if (!(spi->trans->buf.state & SPI_BUFF_FULL))
			mask |= (POLLOUT | POLLWRNORM);
	}

	/* error check */
	if ((spi->trans->spi_err != 0) || (spi->trans->dma_err != 0) ||
	    (spi->trans->buf.state & SPI_BUFF_OVER_ERR)) {
		mask |= POLLERR;
	}
#ifdef SPI_DEBUG
	printk(KERN_INFO "%s(): mask = %s%s\n", __func__,
	       (mask & POLLIN) ? "r" : "", (mask & POLLOUT) ? "w" : "");
#endif

	return mask;
}

/*
 * ioctl
 */
static int spi_ioctl(unsigned char dev, struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	struct spi_data_t *spi;
	struct spi_mode_info mode;
	int ret = 0;

	/* device is not found */
	if (DEVNO_IS_INVALID(dev))
		return -ENODEV;

	spi = &spi_private[dev];

	if (_IOC_DIR(cmd) == _IOC_WRITE) {
		if (spi->trans->state == SPI_ACTIVE)
			return -EBUSY;
	}

	switch (cmd) {
	case SPI_CMD_RX_START:
		if ((file->f_mode & FMODE_READ)
		    && (spi->mode->m_s == SPI_M_S_SLAVE)) {
			if (!(spi->trans->state == SPI_ACTIVE)) {
				ret =
					spi_allocate_buffer(spi,
							    spi->mode->blksize);
				if (ret < 0)
					break;
				if (spi->mode->dma == SPI_DMA_ON) {
					ret = spi_request_dma(spi);
					if (ret < 0)
						return ret;
				}
				spi_init_data(spi);
				ret = spi_rx_start(spi, 0);
			}
		}
		break;

	case SPI_CMD_RX_STOP:
		if ((file->f_mode & FMODE_READ)
		    && (spi->mode->m_s == SPI_M_S_SLAVE)) {
			if (spi->trans->state == SPI_ACTIVE)
				spi_rx_stop(spi);
		}
		break;

	case SPI_CMD_GET_MODE:
		if (copy_to_user((char *)arg, (char *)spi->mode,
					sizeof(struct spi_mode_info))) {
#ifdef SPI_DEBUG
			printk(KERN_INFO "%s(): copy error\n", __func__);
#endif
			ret = -EFAULT;
		}
		break;

	case SPI_CMD_SET_MODE:
		if (copy_from_user((char *)&mode, (char *)arg,
					sizeof(struct spi_mode_info))) {
#ifdef SPI_DEBUG
			printk(KERN_INFO "%s(): copy error\n", __func__);
#endif
			ret = -EFAULT;
			break;
		}

		ret = spi_set_mode(spi, &mode);
		if (ret < 0)
			break;
		memcpy((char *)spi->mode, (char *)&mode,
				sizeof(struct spi_mode_info));
		break;

	default:
		/* error */
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*
 * open
 */
static int spi_open(unsigned char dev, struct inode *inode, struct file *file)
{
	struct spi_data_t *spi;
	int ret;

	/* device is not found */
	if (DEVNO_IS_INVALID(dev))
		return -ENODEV;

	spi = &spi_private[dev];

	if (down_interruptible(&spi->sem_open))
		return -ERESTARTSYS;

	/* device is busy */
	if (spi->opened != 0) {
#ifdef SPI_DEBUG
		printk(KERN_INFO "%s(): device is busy\n", __func__);
#endif
		ret = -EBUSY;
		goto err;
	}

	/* R/W mode is not supported */
	if ((file->f_mode & FMODE_READ) && (file->f_mode & FMODE_WRITE)) {
		ret = -EACCES;
		goto err;
	}

	spi->opened = file->f_mode;

	memset(&spi->trans->buf, 0x00, sizeof(struct spi_buf_t));
	spi->trans->dma_regs = NULL;

	/* initialize data */
	spi_init_data(spi);

	/* sleep disable */
	spi_sleep_flag++;

	/* reset off */
	spi_reset_ctrl(spi, SPI_OFF);

	/* request SPI interrupt */
	ret =
		request_irq(spi->trans->int_spi, spi_interrupt, IRQF_DISABLED,
			    SPI_NAME, (void *)spi);
	if (ret < 0) {
#ifdef SPI_DEBUG
		printk(KERN_INFO "%s(): request irq error\n", __func__);
#endif
		goto err1;
	}

	/* set mode */
	ret = spi_set_mode(spi, spi->mode);
	if (ret < 0)
		goto err2;

	init_waitqueue_head(&spi->trans->wait);

	up(&spi->sem_open);

	return 0;

err2:
	spi_free_buffer(spi);
	free_irq(spi->trans->int_spi, (void *)spi);
err1:
	spi_reset_ctrl(spi, SPI_ON);
	spi_sleep_flag--;
	spi->opened = 0;
err:
	up(&spi->sem_open);

	return ret;
}

/*
 * same open
 */
int
emxx_spi_init(unsigned char dev, unsigned char rw_mode)
{
	struct spi_data_t *spi;
	int ret;

	/* device is not found */
	if (DEVNO_IS_INVALID(dev))
		return -EACCES;
	/* R/W mode is not supported */
	if (rw_mode != SPI_READ_MODE && rw_mode != SPI_WRITE_MODE)
		return -EACCES;

	spi = &spi_private[dev];

	if (down_interruptible(&spi->sem_open))
		return -ERESTARTSYS;

	/* device is busy */
	if (spi->opened != 0) {
#ifdef SPI_DEBUG
		printk(KERN_INFO "%s(): device is busy\n", __func__);
#endif
		ret = -EBUSY;
		goto err;
	}

	if (rw_mode == SPI_READ_MODE) {
		spi->opened = FMODE_READ;
		spi->k_flag = SPI_READ_MODE;
	} else{
		spi->opened = FMODE_WRITE;
		spi->k_flag = SPI_WRITE_MODE;
	}

	memset(&spi->trans->buf, 0x00, sizeof(struct spi_buf_t));
	spi->trans->dma_regs = NULL;

	/* initialize data */
	spi_init_data(spi);

	/* sleep disable */
	spi_sleep_flag++;

	/* reset off */
	spi_reset_ctrl(spi, SPI_OFF);

	/* request SPI interrupt */
	ret =
		request_irq(spi->trans->int_spi, spi_interrupt, IRQF_DISABLED,
			    SPI_NAME, (void *)spi);
	if (ret < 0) {
#ifdef SPI_DEBUG
		printk(KERN_INFO "%s(): request irq error\n", __func__);
#endif
		goto err1;
	}

	/* set mode */
	ret = spi_set_mode(spi, spi->mode);
	if (ret < 0)
		goto err2;

	init_waitqueue_head(&spi->trans->wait);

	up(&spi->sem_open);

	return 0;

err2:
	spi_free_buffer(spi);
	free_irq(spi->trans->int_spi, (void *)spi);
err1:
	spi_reset_ctrl(spi, SPI_ON);
	spi_sleep_flag--;
	spi->opened = 0;
	spi->k_flag = 0;
err:
	up(&spi->sem_open);

	return ret;
}
EXPORT_SYMBOL(emxx_spi_init);

/*
 * release
 */
static int spi_release(unsigned char dev, struct inode *inode,
		       struct file *file)
{
	struct spi_data_t *spi;

	/* device is not found */
	if (DEVNO_IS_INVALID(dev))
		return -ENODEV;

	spi = &spi_private[dev];

	if (down_interruptible(&spi->sem_open))
		return -ERESTARTSYS;

	if (file->f_mode & FMODE_READ) {
		if (spi->trans->state == SPI_ACTIVE) {
			/* RX stop */
			spi_rx_stop(spi);
		}
	}
	if (file->f_mode & FMODE_WRITE) {
		if (spi->trans->state == SPI_ACTIVE) {
			/* sync of TX buffer */
			DECLARE_WAITQUEUE(wait, current);
			add_wait_queue(&spi->trans->wait, &wait);
			while (spi->trans->state == SPI_ACTIVE) {
				set_current_state(TASK_INTERRUPTIBLE);
				if (signal_pending(current) != 0) {
					spi_tx_stop(spi);
					break;
				}
				schedule();
			}
			remove_wait_queue(&spi->trans->wait, &wait);
			set_current_state(TASK_RUNNING);
		}
	}

	spi_free_dma(spi);

	spi_free_buffer(spi);

	free_irq(spi->trans->int_spi, (void *)spi);

	/* reset on */
	spi_reset_ctrl(spi, SPI_ON);

	/* sleep enable */
	spi_sleep_flag--;

	/* initialize data */
	spi_init_data(spi);

	spi->opened = 0;

	up(&spi->sem_open);

	return 0;
}

/*
 * same release
 */
int
emxx_spi_end(unsigned char dev)
{
	struct spi_data_t *spi;

	/* device is not found */
	if (DEVNO_IS_INVALID(dev))
		return -EACCES;
	spi = &spi_private[dev];

	if (spi->k_flag != SPI_READ_MODE && spi->k_flag != SPI_WRITE_MODE)
		return -EACCES;

	if (down_interruptible(&spi->sem_open))
		return -ERESTARTSYS;

	if (spi->opened & FMODE_READ) {
		if (spi->trans->state == SPI_ACTIVE) {
			/* RX stop */
			spi_rx_stop(spi);
		}
	}
	if (spi->opened & FMODE_WRITE) {
		if (spi->trans->state == SPI_ACTIVE) {
			/* sync of TX buffer */
			DECLARE_WAITQUEUE(wait, current);
			add_wait_queue(&spi->trans->wait, &wait);
			while (spi->trans->state == SPI_ACTIVE) {
				set_current_state(TASK_INTERRUPTIBLE);
				if (signal_pending(current) != 0) {
					spi_tx_stop(spi);
					break;
				}
				schedule();
			}
			remove_wait_queue(&spi->trans->wait, &wait);
			set_current_state(TASK_RUNNING);
		}
	}

	spi_free_dma(spi);

	spi_free_buffer(spi);

	free_irq(spi->trans->int_spi, (void *)spi);

	/* reset on */
	spi_reset_ctrl(spi, SPI_ON);

	/* sleep enable */
	spi_sleep_flag--;

	/* initialize data */
	spi_init_data(spi);

	spi->opened = 0;
	spi->k_flag = 0;

	up(&spi->sem_open);

	return 0;
}
EXPORT_SYMBOL(emxx_spi_end);

int
emxx_spi_setmode(unsigned char dev, struct spi_mode_info *mode)
{
	struct spi_data_t *spi;
	int ret;

	/* device is not found */
	if (DEVNO_IS_INVALID(dev))
		return -EACCES;

	spi = &spi_private[dev];

	if (spi->k_flag != SPI_READ_MODE && spi->k_flag != SPI_WRITE_MODE)
		return -EACCES;
	ret = spi_set_mode(spi, mode);
	if (ret < 0)
		return -EFAULT;

	memcpy((char *)spi->mode, (char *)mode, sizeof(struct spi_mode_info));

	return 0;
}
EXPORT_SYMBOL(emxx_spi_setmode);

int
emxx_spi_getmode(unsigned char dev, struct spi_mode_info *mode)
{
	struct spi_data_t *spi;

	/* device is not found */
	if (DEVNO_IS_INVALID(dev))
		return -EACCES;

	if (mode == NULL)
		return -EACCES;

	spi = &spi_private[dev];
	memcpy((char *)mode, (char *)spi->mode, sizeof(struct spi_mode_info));

	return 0;
}
EXPORT_SYMBOL(emxx_spi_getmode);

/*
 * read (SPI1)
 */
static ssize_t spi_read_spi1(struct file *file, char *buf, size_t count,
			     loff_t *ppos)
{
	return spi_read_func(SPI_DEV_SP1, file, buf, count, ppos);
}

/*
 * write (SPI1)
 */
static ssize_t spi_write_spi1(struct file *file, const char *buf, size_t count,
			      loff_t *ppos)
{
	return spi_write_func(SPI_DEV_SP1, file, buf, count, ppos);
}

/*
 * poll (SPI1)
 */
static unsigned int spi_poll_spi1(struct file *file, poll_table *wait)
{
	return spi_poll(SPI_DEV_SP1, file, wait);
}

/*
 * ioctl (SPI1)
 */
static int spi_ioctl_spi1(struct inode *inode, struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	return spi_ioctl(SPI_DEV_SP1, inode, file, cmd, arg);
}

/*
 * open (SPI1)
 */
static int spi_open_spi1(struct inode *inode, struct file *file)
{
	return spi_open(SPI_DEV_SP1, inode, file);
}

/*
 * release (SPI1)
 */
static int spi_release_spi1(struct inode *inode, struct file *file)
{
	return spi_release(SPI_DEV_SP1, inode, file);
}

/* file operations info (SPI1) */
static const struct file_operations spi_fops_spi1 = {
	.read = spi_read_spi1,
	.write = spi_write_spi1,
	.poll = spi_poll_spi1,
	.ioctl = spi_ioctl_spi1,
	.open = spi_open_spi1,
	.release = spi_release_spi1,
	.owner = THIS_MODULE,
};

/*
 * open
 */
static int spi_open_chrdev(struct inode *inode, struct file *file)
{
	int minor = MINOR(inode->i_rdev);

	switch (minor) {
	/* SPI1 */
	case SPI_MINOR_SPI1:
		file->f_op = &spi_fops_spi1;
		break;
	default:
		/* error */
		return -ENODEV;
	}

	if (file->f_op->open != 0)
		return (*file->f_op->open)(inode, file);

	return 0;
}

/*
 * register of device
 */
static int spi_probe(struct platform_device *dev)
{
	int result = 0;
	struct device *spi_class_dev;
	dev_t t_dev_temp;

	if (DEVNO_IS_INVALID(dev->id)) {
		/* Unsupported */
		return result;
	}

	sema_init(&spi_private[dev->id].sem_rw, 1);
	sema_init(&spi_private[dev->id].sem_open, 1);

	t_dev_temp = MKDEV(MAJOR(t_dev), dev->id);
	spi_class_dev = device_create(spi_class, NULL, t_dev_temp, NULL,
			"spi%d", dev->id);
	if (IS_ERR(spi_class_dev)) {
		printk(KERN_ERR "spi: Unable to create class_device.\n");
		result = PTR_ERR(spi_class_dev);
	}
	return result;
}

/*
 * unregister of device
 */
static int spi_remove(struct platform_device *dev)
{
	if (DEVNO_IS_INVALID(dev->id))
		return -ENODEV;

	device_destroy(spi_class, MKDEV(spi_major, dev->id));

	return 0;
}

static int  spi_suspend(struct platform_device *dev, pm_message_t state)
{
#ifdef SPI_DEBUG
	printk(KERN_INFO "%s(): id=%d state.event=%d\n",
	       __func__, dev->id, state.event);
#endif

	if (DEVNO_IS_INVALID(dev->id))
		return 0;

	switch (state.event) {
	case PM_EVENT_SUSPEND:
		if (spi_sleep_flag != 0)
			return -EBUSY;
		break;
	default:
		break;
	}

	return 0;
}

static int  spi_resume(struct platform_device *dev)
{
#ifdef SPI_DEBUG
	printk(KERN_INFO "%s(): id=%d\n",
	       __func__, dev->id);
#endif

	if (DEVNO_IS_INVALID(dev->id))
		return 0;

	return 0;
}

static struct platform_driver spi_drv = {
	.driver.name  = SPI_NAME,
	.driver.bus   = &platform_bus_type,
	.probe        = spi_probe,
	.remove       = spi_remove,
	.suspend      = spi_suspend,
	.resume       = spi_resume,
};

/*
 * initialize
 */
static int __init spi_init(void)
{
	int result = 0;
	int minor_num_start = 1;
	int dev_count = 1; /* SPI1 */

	if (spi_major) {
		t_dev  = MKDEV(spi_major, minor_num_start);
		result = register_chrdev_region(t_dev, dev_count, SPI_NAME);
	} else{
		result = alloc_chrdev_region(&t_dev, minor_num_start,
				dev_count, SPI_NAME);
		spi_major  = MAJOR(t_dev);
	}
	if (result < 0) {
		printk(KERN_ERR "spi: can't get major number.\n");
		goto fail_get_major;
	}

	cdev_init(&spi_cdev, &spi_fops);
	spi_cdev.owner = THIS_MODULE;
	result = cdev_add(&spi_cdev, t_dev, dev_count);
	if (result)
		goto fail_cdev_add;

	spi_class = class_create(THIS_MODULE, SPI_NAME);
	if (IS_ERR(spi_class)) {
		printk(KERN_ERR "spi: Unable to create spi class.\n");
		result = PTR_ERR(spi_class);
		goto fail_class_create;
	}

	if (platform_driver_register(&spi_drv) < 0) {
		result = -1;
		printk(KERN_ERR " @spi: could not register platform_driver\n");
		goto fail_platform_driver;
	}

	printk(KERN_INFO "spi: registered device spi [spi]\n");

	goto success;

fail_platform_driver:
	class_destroy(spi_class);
fail_class_create:
	cdev_del(&spi_cdev);
fail_cdev_add:
	unregister_chrdev(spi_major, SPI_NAME);
fail_get_major:
success:

	return result;
}

/*
 * exit
 */
static void __exit spi_exit(void)
{
	platform_driver_unregister(&spi_drv);

	class_destroy(spi_class);
	cdev_del(&spi_cdev);
	unregister_chrdev(spi_major, SPI_NAME);
}

#ifdef SPI_DEBUG
unsigned int get_spi_k_flag(unsigned char dev)
{
	return spi_private[dev].k_flag;
}
EXPORT_SYMBOL(get_spi_k_flag);

void set_spi_k_flag(unsigned char dev, unsigned int k_flag)
{
	spi_private[dev].k_flag = k_flag;
}
EXPORT_SYMBOL(set_spi_k_flag);
#endif

module_init(spi_init);
module_exit(spi_exit);
MODULE_LICENSE("GPL");
