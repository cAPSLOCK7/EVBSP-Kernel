/*
 *  File Name	    : arch/arm/mach-emxx/spi0.c
 *  Function	    : SPI0 interface
 *  Release Version : Ver 1.01
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
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <asm/irq.h>
#include <mach/dma.h>
#include <mach/smu.h>
#include <mach/pmu.h>

#include "spi0.h"

#if defined(SPI_DEBUG)
#define DEB(fmt, args...) \
	printk(KERN_DEBUG "%s: " fmt, __func__, ## args);
#else
#define DEB(fmt, args...)
#endif

static void spi_interrupt_dma_rx(void *, int, int);
static void spi_interrupt_dma_tx(void *, int, int);

unsigned int sp0_excl = CONFIG_EMXX_SPI_EXCL_SP0;

/* transfer info (SP0) */
static struct spi_trans trans_sp0 = {
	.rx_lch = EMXX_DMAC_P2M_SIO0,
	.tx_lch = EMXX_DMAC_M2P_SIO0,
	.rx_data = SPx_RX_DATA_PHYS(EMXX_SIO0_BASE),
	.tx_data = SPx_TX_DATA_PHYS(EMXX_SIO0_BASE),
	.int_spi = INT_SIO0,
	.state = SPI_UNUSED,
	.spinlock = __SPIN_LOCK_UNLOCKED(trans_sp0.spinlock),
};

/* spi config info (SP0) */
static SPI_CONFIG config_sp0 = {
	.dev = SPI_DEV_SP0,
	.nbr = SPI_NB_16BIT,
	.nbw = SPI_NB_16BIT,
	.cs_sel = SPI_CS_SEL_CS0,
	.m_s = SPI_M_S_MASTER,
	.dma = SPI_DMA_OFF,
	.pol = SPI_POL_SP0_CS0,
	.sclk = SPI_SCLK_3MHZ,
	.tiecs = SPI_TIECS_NORMAL
};

/* smu info (SP0) */
static struct spi_smu smu_sp0 = {
	.pclk = EMXX_CLK_USIA_S0_P | EMXX_CLK_USIA_S0_H,
	.sclk = EMXX_CLK_USIA_S0_S,
	.pclk_ctrl = EMXX_CLKCTRL_USIAS0PCLK,
	.sclk_ctrl = EMXX_CLKCTRL_USIAS0,
	.reset = EMXX_RST_USIA_S0_S | EMXX_RST_USIA_S0_A,
	.sclk_div = SMU_USIASCLKDIV,
};

/* spi private data info */
static struct spi_data spi_private[] = {
	{
		.regs = (struct spi_regs *)(SP0_ADDR),
		.trans = &trans_sp0,
		.config = &config_sp0,
		.smu = &smu_sp0,
		.probe = 0,
		.pol = SPI_POL_SP0,
	},
};

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

static void spi_sft_reset(struct spi_data *spi)
{
	spi->regs->control |= SPx_CONTROL_RST;
	udelay((1000 * 4 / spi_clock_table[spi->config->sclk]) + 1);
	spi->regs->control &= ~(SPx_CONTROL_RST);
}

static void spi_power_on(struct spi_data *spi)
{
	emxx_open_clockgate(spi->smu->pclk);
	emxx_open_clockgate(spi->smu->sclk);
	emxx_unreset_device(spi->smu->reset);
	emxx_clkctrl_on(spi->smu->pclk_ctrl);
	emxx_clkctrl_on(spi->smu->sclk_ctrl);
}

static void spi_power_off(struct spi_data *spi)
{
	emxx_clkctrl_off(spi->smu->sclk_ctrl);
	emxx_clkctrl_off(spi->smu->pclk_ctrl);
	emxx_reset_device(spi->smu->reset);
	emxx_close_clockgate(spi->smu->sclk);
	emxx_close_clockgate(spi->smu->pclk);
}

static int spi_set_sclk_div(struct spi_data *spi, unsigned int sclk)
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

	val = readl(spi->smu->sclk_div) & ~0x0000ffff;
	val |= spi_plldiv_table[sclk];
	writel(val, spi->smu->sclk_div);

	DEB("sclk = %d[KHz]\n", spi_clock_table[sclk]);

	return 0;
}

static unsigned int spi_unit(struct spi_data *spi, unsigned int bit)
{
	if (bit == 0)
		return 0;
	else if (bit <= SPI_NB_8BIT)
		return 1;
	else if (bit <= SPI_NB_16BIT)
		return 2;
	else if ((bit <= SPI_NB_24BIT) && (spi->config->dma == SPI_DMA_OFF))
		return 3;
	else
		return 4;
}

static void spi_init_data(struct spi_data *spi)
{
	spi->trans->dma_err = 0;
	spi->trans->spi_err = 0;
	spi->trans->size = 0;
}

static int spi_request_dma(struct spi_data *spi)
{
	int ret;

	if (spi->excl)
		return 0;

	ret = emxx_request_dma(spi->trans->rx_lch, SPI_NAME,
				spi_interrupt_dma_rx, (void *)spi,
				&spi->trans->dma_rx_regs);
	if (ret < 0) {
		DEB("error1 %d\n", ret);
		return ret;
	}

	ret = emxx_request_dma(spi->trans->tx_lch, SPI_NAME,
				spi_interrupt_dma_tx, (void *)spi,
				&spi->trans->dma_tx_regs);
	if (ret < 0) {
		DEB("error2 %d\n", ret);
		emxx_free_dma(spi->trans->rx_lch);
		return ret;
	}

	return 0;
}

static void spi_free_dma(struct spi_data *spi)
{
	if (spi->excl)
		return;

	emxx_free_dma(spi->trans->tx_lch);
	emxx_free_dma(spi->trans->rx_lch);
}

static int spi_config(struct spi_data *spi, SPI_CONFIG *config)
{
	unsigned int mode = 0;
	unsigned int pol = 0;
	unsigned int pol_bit = 0;
	unsigned int csw = 0;
	unsigned int csw_pol = 0;
	unsigned int nb = 0;
	unsigned int tiecs = 0;

	/* bit length */
	if (spi->trans->state == SPI_READ)
		nb = config->nbr;
	else if (spi->trans->state == SPI_WRITE)
		nb = config->nbw;
	else if (spi->trans->state == SPI_RW)
		nb = config->nbr + config->nbw;
	else
		nb = SPI_NB_16BIT;
	if ((nb < SPI_NB_8BIT) || (SPI_NB_32BIT < nb))
		return -EINVAL;
	mode |= ((nb - 1) << 8);

	/* chip select */
	switch (config->cs_sel) {
	case SPI_CS_SEL_CS0:
	case SPI_CS_SEL_CS1:
	case SPI_CS_SEL_CS2:
	case SPI_CS_SEL_CS3:
	case SPI_CS_SEL_CS4:
	case SPI_CS_SEL_CS5:
	case SPI_CS_SEL_CS6:
	case SPI_CS_SEL_CS7:
		mode |= (config->cs_sel << 4);
		break;
	default:
		return -EINVAL;
	}

	/* master/slave */
	if ((config->m_s == SPI_M_S_MASTER)
	    || (config->m_s == SPI_M_S_SLAVE)) {
		mode |= (config->m_s << 1);
	} else {
		return -EINVAL;
	}

	/* dma on/off */
	if ((config->dma == SPI_DMA_OFF) || (config->dma == SPI_DMA_ON))
		mode |= (config->dma  << 0);
	else
		return -EINVAL;

	/* tiecs */
	if ((config->tiecs == SPI_TIECS_NORMAL)
	    || (config->tiecs == SPI_TIECS_FIXED)) {
		tiecs = config->tiecs << config->cs_sel;
	} else {
		return -EINVAL;
	}

	/* pol */
	csw = (config->pol & SPI_CSW_MASK);
	if (SPI_CSW_16CLK < csw)
		return -EINVAL;

	switch (config->cs_sel) {
	case SPI_CS_SEL_CS0:
	case SPI_CS_SEL_CS1:
	case SPI_CS_SEL_CS2:
	case SPI_CS_SEL_CS3:
		pol_bit = config->cs_sel * 3;
		break;
	case SPI_CS_SEL_CS4:
	case SPI_CS_SEL_CS5:
	case SPI_CS_SEL_CS6:
	case SPI_CS_SEL_CS7:
		pol_bit = 16 + ((config->cs_sel - 4) * 3);
		break;
	default:
		return -EINVAL;
	}

	pol = (config->pol & SPI_POL_MASK);

	if (config->dev == SPI_DEV_SP0) {
		csw_pol = spi->pol;
		csw_pol &= ~(SPI_CSW_MASK | (SPI_POL_MASK << pol_bit));
		csw_pol |= (csw | (pol << pol_bit));
	} else {
		return -EINVAL;
	}

	/* set sclk */
	if (spi_set_sclk_div(spi, config->sclk) < 0)
		return -EINVAL;

	/* set SPx_MODE */
	spi->regs->mode = mode;
	/* set SPx_TIECS */
	spi->regs->tiecs = tiecs;
	/* set SPx_POL */
	spi->regs->pol = csw_pol;

	spi->pol = csw_pol;

	/* software reset */
	spi_sft_reset(spi);

	memcpy(spi->config, config, sizeof(SPI_CONFIG));

	return 0;
}

static void spi_config_release(struct spi_data *spi)
{
	if (spi->config->tiecs == SPI_TIECS_FIXED) {
		if (spi->config->dev == SPI_DEV_SP0) {
			spi->regs->tiecs = 0x00000000; /* all normal */
		} else {
			/* Unsupported */
			return;
		}
	}
}

static int spi_xferbytes(struct spi_data *spi, char *buf, unsigned int count,
		unsigned int flags)
{
	unsigned int data = 0;
	int ret = 0;

	if (count == 0)
		return 0;

	if (flags == SPI_READ) {
		spi->regs->control = (SPx_CONTROL_RD | SPx_CONTROL_START);
		while ((spi->regs->
			control & (SPx_CONTROL_START | SPx_CONTROL_RX_EMP)) !=
		       0) {
			if ((spi->regs->
			     raw_status & SPx_RAW_STATUS_RX_ALLERR_RAW) != 0) {
				ret = -EIO;
				break;
			}
		}
		data = spi->regs->rx_data;
		memcpy(buf, (char *)&data, count);
	} else if (flags == SPI_WRITE) {
		memcpy((char *)&data, buf, count);
		spi->regs->tx_data = data;
		spi->regs->control = (SPx_CONTROL_WRT | SPx_CONTROL_START);
		while ((spi->regs->control & SPx_CONTROL_START) != 0) {
			if ((spi->regs->
			     raw_status & SPx_RAW_STATUS_TX_ALLERR_RAW) != 0) {
				ret = -EIO;
				break;
			}
		}
	} else if (flags == SPI_RW) {
		memcpy((char *)&data, buf, count);
		spi->regs->tx_data = data;
		spi->regs->control =
			(SPx_CONTROL_RD | SPx_CONTROL_WRT | SPx_CONTROL_START);
		while ((spi->regs->control &
			  (SPx_CONTROL_START | SPx_CONTROL_RX_EMP)) != 0) {
			if ((spi->regs->
			     raw_status & SPx_RAW_STATUS_ALLERR_RAW) != 0) {
				ret = -EIO;
				break;
			}
		}
		data = spi->regs->rx_data;
		memcpy(buf, (char *)&data, count);
	}

	return ret;
}

static void spi_rx_stop(struct spi_data *spi)
{
	if (spi->config->dma == SPI_DMA_ON) {
		if (spi->config->m_s == SPI_M_S_MASTER) {
			if ((spi->regs->control & SPx_CONTROL_START) != 0) {
				spi->regs->control |= SPx_CONTROL_STOP;
				udelay((1000 * spi->config->nbr /
				    spi_clock_table[spi->config->sclk]) + 1);
			}
			spi->regs->control2 &=  ~(SPx_CONTROL2_RX_STOP_MODE
					|  SPx_CONTROL2_RX_FIFO_FULL_MASK);
		}
		emxx_stop_dma(spi->trans->rx_lch);
		spi->regs->enclr = SPx_ENCLR_RX_ALL_MASK;
	}
	spi_sft_reset(spi);
}

static void spi_tx_stop(struct spi_data *spi)
{
	if (spi->config->dma == SPI_DMA_ON) {
		if (spi->config->m_s == SPI_M_S_MASTER) {
			if ((spi->regs->control & SPx_CONTROL_START) != 0) {
				spi->regs->control |= SPx_CONTROL_STOP;
				udelay((1000 * spi->config->nbw /
				    spi_clock_table[spi->config->sclk]) + 1);
			}
			spi->regs->control2 &= ~(SPx_CONTROL2_TX_STOP_MODE);
		}
		emxx_stop_dma(spi->trans->tx_lch);
		spi->regs->enclr = SPx_ENCLR_TX_ALL_MASK;
	}

	spi_sft_reset(spi);
}

static int spi_rx_start(struct spi_data *spi, unsigned int size)
{
	int ret;

	if (size == 0)
		return 0;

	if (spi->config->dma == SPI_DMA_ON) {
		int intmask;

		if (spi->config->nbr <= SPI_NB_8BIT) {
			spi->trans->dma_rx_regs->mode =
				EMXX_DMAC_DEFMODE_8BIT;
		} else if (spi->config->nbr <= SPI_NB_16BIT) {
			spi->trans->dma_rx_regs->mode =
				EMXX_DMAC_DEFMODE_16BIT;
		} else {
			spi->trans->dma_rx_regs->mode =
				EMXX_DMAC_DEFMODE_32BIT;
		}
		spi->trans->dma_rx_regs->boff = 0;
		spi->trans->dma_rx_regs->bsize =
			(size > SPI_DMA_BLOCK_MAXSIZE) ?
			SPI_DMA_BLOCK_MAXSIZE : size;
		spi->trans->dma_rx_regs->bsize_count = 0;
		spi->trans->dma_rx_regs->leng = size;
		intmask = (EMXX_DMAC_INT_ERROR_EN | EMXX_DMAC_INT_LENG_EN);

		DEB("addr=0x%08x mode=0x%08x bsize=0x%08x leng=0x%08x "
		    "intmask=0x%08x \n", spi->trans->buf.dma_addr,
		    spi->trans->dma_rx_regs->mode,
		    spi->trans->dma_rx_regs->bsize,
		    spi->trans->dma_rx_regs->leng, intmask);

		ret = emxx_start_dma(spi->trans->rx_lch, spi->trans->rx_data,
				0, spi->trans->buf.dma_addr, intmask);
		if (ret < 0) {
			DEB("dma error\n");
			return ret;
		}
		if (spi->config->m_s == SPI_M_S_MASTER) {
			unsigned int full =
				((size / spi_unit(spi, spi->config->nbr)) - 1);
			if (full < 0x10000) {
				spi->regs->control2 &=
					~SPx_CONTROL2_RX_FIFO_FULL_MASK;
				spi->regs->control2 |=
					(((full & 0xff00) << 8) | (full & 0xff)
					 | SPx_CONTROL2_RX_STOP_MODE);
			}
		}
		spi->regs->enset = SPx_ENSET_RX_ALLERR_EN;
		spi->regs->control = (SPx_CONTROL_RD | SPx_CONTROL_START);
	} else {
		int i;
		unsigned int unit;
		char *buf;

		unit = spi_unit(spi, spi->config->nbr);
		for (i = 0; i < size; i += unit) {
			buf = (char *)(spi->trans->buf.addr + i);
			ret = spi_xferbytes(spi, buf, unit, SPI_READ);
			if (ret < 0) {
				spi_sft_reset(spi);
				return ret;
			}
		}
	}

	return 0;
}

static int spi_tx_start(struct spi_data *spi, unsigned int size)
{
	int ret;

	if (size == 0)
		return 0;

	if (spi->config->dma == SPI_DMA_ON) {
		int intmask;

		if (spi->config->nbw <= SPI_NB_8BIT) {
			spi->trans->dma_tx_regs->mode =
				EMXX_DMAC_DEFMODE_8BIT;
		} else if (spi->config->nbw <= SPI_NB_16BIT) {
			spi->trans->dma_tx_regs->mode =
				EMXX_DMAC_DEFMODE_16BIT;
		} else {
			spi->trans->dma_tx_regs->mode =
				EMXX_DMAC_DEFMODE_32BIT;
		}
		spi->trans->dma_tx_regs->aoff = 0;
		spi->trans->dma_tx_regs->asize =
			(size > SPI_DMA_BLOCK_MAXSIZE) ?
			SPI_DMA_BLOCK_MAXSIZE : size;
		spi->trans->dma_tx_regs->asize_count = 0;
		spi->trans->dma_tx_regs->leng = size;
		intmask = (EMXX_DMAC_INT_ERROR_EN | EMXX_DMAC_INT_LENG_EN);

		DEB("addr=0x%08x mode=0x%08x asize=0x%08x leng=0x%08x "
		    "intmask=0x%08x \n", spi->trans->buf.dma_addr,
		    spi->trans->dma_tx_regs->mode,
		    spi->trans->dma_tx_regs->asize,
		    spi->trans->dma_tx_regs->leng, intmask);

		ret = emxx_start_dma(spi->trans->tx_lch,
				      spi->trans->buf.dma_addr, 0,
				      spi->trans->tx_data, intmask);
		if (ret < 0) {
			DEB("dma error\n");
			spi->trans->size = 0;
			return ret;
		}

		if (spi->config->m_s == SPI_M_S_MASTER)
			spi->regs->control2 |= SPx_CONTROL2_TX_STOP_MODE;
		spi->regs->enset =
			(SPx_ENSET_TX_ALLERR_EN | SPx_ENSET_TX_STOP_EN);
		spi->regs->control = (SPx_CONTROL_WRT | SPx_CONTROL_START);
	} else {
		int i;
		unsigned int unit;
		char *buf;

		unit = spi_unit(spi, spi->config->nbw);
		for (i = 0; i < size; i += unit) {
			buf = (char *)(spi->trans->buf.addr + i);
			ret = spi_xferbytes(spi, buf, unit, SPI_WRITE);
			if (ret < 0) {
				spi_sft_reset(spi);
				return ret;
			}
		}
	}

	return 0;
}

static irqreturn_t spi_interrupt(int irq, void *dev_id)
{
	struct spi_data *spi;
	unsigned int status;

	if (dev_id == NULL)
		return IRQ_NONE;

	spi = (struct spi_data *) dev_id;

	status = spi->regs->status;
	spi->regs->ffclr = status;

	DEB("status = %08x\n", status);

	/* error */
	if ((status & SPx_STATUS_ALLERR) != 0) {
		spi->trans->spi_err = (status & SPx_STATUS_ALLERR);
		if ((status & (SPx_STATUS_TERR | SPx_STATUS_RX_OVR)) != 0)
			spi_rx_stop(spi);
		if ((status & SPx_STATUS_TX_UDR) != 0) {
			/* slave */
			if (spi->trans->dma_tx_regs->wcount == 0)
				spi->trans->spi_err &= ~(SPx_STATUS_TX_UDR);
			spi_tx_stop(spi);
		}
		if (waitqueue_active(&spi->trans->wait))
			wake_up_interruptible(&spi->trans->wait);
		return IRQ_HANDLED;
	}

	/* tx stop (dma master only) */
	if ((status & SPx_STATUS_TX_STOP) != 0) {
		if ((spi->trans->dma_tx_regs->wcount != 0) ||
		    ((spi->regs->control & SPx_CONTROL_TX_EMP) == 0)) {
			/* error */
			spi->trans->spi_err |= SPx_STATUS_TX_STOP;
		}
		spi_tx_stop(spi);
		if (waitqueue_active(&spi->trans->wait))
			wake_up_interruptible(&spi->trans->wait);
		return IRQ_HANDLED;
	}

	return IRQ_HANDLED;
}

static void spi_interrupt_dma_rx(void *data, int intsts, int intrawsts)
{
	struct spi_data *spi;

	if (data == NULL)
		return;

	DEB("intsts = %08x\n", intsts);

	spi = (struct spi_data *) data;

	if (intsts & (EMXX_DMAC_INT_ERROR_WR | EMXX_DMAC_INT_ERROR_RD)) {
		spi->trans->dma_err =
			(intsts &
			 (EMXX_DMAC_INT_ERROR_WR | EMXX_DMAC_INT_ERROR_RD));
	} else if (intsts & EMXX_DMAC_INT_LENG_WR) {
		spi->trans->size =
			emxx_get_dma_pos(spi->trans->rx_lch) -
			spi->trans->buf.dma_addr;
		DEB("size = %d\n", spi->trans->size);
	} else {
		return;
	}

	spi_rx_stop(spi);

	if (waitqueue_active(&spi->trans->wait))
		wake_up_interruptible(&spi->trans->wait);
}

static void spi_interrupt_dma_tx(void *data, int intsts, int intrawsts)
{
	struct spi_data *spi;

	if (data == NULL)
		return;

	DEB("intsts = %08x\n", intsts);

	spi = (struct spi_data *) data;

	if (intsts & (EMXX_DMAC_INT_ERROR_WR | EMXX_DMAC_INT_ERROR_RD)) {
		spi->trans->dma_err =
			(intsts &
			 (EMXX_DMAC_INT_ERROR_WR | EMXX_DMAC_INT_ERROR_RD));
	} else if (intsts & EMXX_DMAC_INT_LENG_WR) {
		spi->trans->size =
			emxx_get_dma_pos(spi->trans->tx_lch) -
			spi->trans->buf.dma_addr;

		DEB("size = %d\n", spi->trans->size);
		return;
	} else {
		return;
	}

	spi_tx_stop(spi);

	if (waitqueue_active(&spi->trans->wait))
		wake_up_interruptible(&spi->trans->wait);
}

int spi_read(SPI_CONFIG *config, char *buf, unsigned long phys,
	     unsigned int count, unsigned int flags)
{
	struct spi_data *spi;
	unsigned int len = 0;
	unsigned int unit;
	int ret = 0;
	unsigned long lock_flags = 0;

	if ((config == NULL) ||
	    ((buf == NULL) && (config->dma == SPI_DMA_OFF)) ||
	    ((phys == 0) && (config->dma == SPI_DMA_ON))) {
		return -EINVAL;
	}

	if (config->dev != SPI_DEV_SP0)
		return -ENODEV;

	spi = &spi_private[config->dev];

	if (spi->probe == 0)
		return -EPERM;

	if (spi->excl) {
		/* dma/slave is unsupported */
		if (config->dma == SPI_DMA_ON
		    || config->m_s == SPI_M_S_SLAVE) {
			return -EINVAL;
		}
		spin_lock_irqsave(&spi->trans->spinlock, lock_flags);
	} else {
		/* cpu slave is unsupported */
		if (config->dma == SPI_DMA_OFF
		    && config->m_s == SPI_M_S_SLAVE) {
			return -EINVAL;
		}
		if ((flags & SPI_NONBLOCK) != 0) {
			if (down_trylock(&spi->sem) != 0)
				return -EAGAIN;
		} else {
			if (down_interruptible(&spi->sem) != 0)
				return -ERESTARTSYS;
		}
	}
	spi->trans->state = SPI_READ;
	ret = spi_config(spi, config);
	if (ret < 0)
		goto out;

	unit = spi_unit(spi, config->nbr);
	count = (count / unit) * unit;

	DEB("count = %d\n", count);

	if (spi->config->dma == SPI_DMA_OFF) {
		spi->trans->buf.addr = (unsigned int)buf;
		ret = spi_rx_start(spi, count);
		if (ret == 0)
			ret = count;
	} else {
		DECLARE_WAITQUEUE(wait, current);
		unsigned int spi_err;
		unsigned int dma_err;
		int size;

		spi->trans->size = 0;

		add_wait_queue(&spi->trans->wait, &wait);
		while ((count - len) > 0) {
			set_current_state(TASK_INTERRUPTIBLE);
			spin_lock_irq(&spi->trans->spinlock);
			spi_err = spi->trans->spi_err;
			dma_err = spi->trans->dma_err;
			size = spi->trans->size;
			spi->trans->size = 0;
			spin_unlock_irq(&spi->trans->spinlock);
			if ((spi_err != 0) || (dma_err != 0)) {
				spi_init_data(spi);
				ret = -EIO;
				break;
			}

			if (size > 0) {
				if ((count - len) < size)
					size = count - len;
				len = len + size;
				ret = len;
			} else {
				if ((count - len) > SPI_DMA_MAXSIZE)
					size = SPI_DMA_MAXSIZE;
				else
					size = count - len;

				spi->trans->buf.dma_addr = phys + len;
				ret = spi_rx_start(spi, size);
				if (ret < 0)
					break;

				if (signal_pending(current)) {
					spi_rx_stop(spi);
					ret = -ERESTARTSYS;
					break;
				}
				schedule();
			}
		}
		remove_wait_queue(&spi->trans->wait, &wait);
		set_current_state(TASK_RUNNING);
	}

	spi_config_release(spi);

out:
	spi->trans->state = SPI_UNUSED;

	if (spi->excl)
		spin_unlock_irqrestore(&spi->trans->spinlock, lock_flags);
	else
		up(&spi->sem);

	return ret;
}
EXPORT_SYMBOL(spi_read);

int spi_write(SPI_CONFIG *config, char *buf, unsigned long phys,
	      unsigned int count, unsigned int flags)
{
	struct spi_data *spi;
	unsigned int len = 0;
	unsigned int unit;
	int ret = 0;
	unsigned long lock_flags = 0;

	if ((config == NULL) ||
	    ((buf == NULL) && (config->dma == SPI_DMA_OFF)) ||
	    ((phys == 0) && (config->dma == SPI_DMA_ON))) {
		return -EINVAL;
	}

	if (config->dev != SPI_DEV_SP0)
		return -ENODEV;

	spi = &spi_private[config->dev];

	if (spi->probe == 0)
		return -EPERM;

	if (spi->excl) {
		/* dma/slave is unsupported */
		if (config->dma == SPI_DMA_ON
		    || config->m_s == SPI_M_S_SLAVE) {
			return -EINVAL;
		}

		spin_lock_irqsave(&spi->trans->spinlock, lock_flags);
	} else {
		/* cpu slave is unsupported */
		if (config->dma == SPI_DMA_OFF
		    && config->m_s == SPI_M_S_SLAVE) {
			return -EINVAL;
		}
		if ((flags & SPI_NONBLOCK) != 0) {
			if (down_trylock(&spi->sem) != 0)
				return -EAGAIN;
		} else {
			if (down_interruptible(&spi->sem) != 0)
				return -ERESTARTSYS;
		}
	}

	spi->trans->state = SPI_WRITE;

	ret = spi_config(spi, config);
	if (ret < 0)
		goto out;

	unit = spi_unit(spi, config->nbw);
	count = (count / unit) * unit;

	DEB("count = %d\n", count);

	spi->trans->size = 0;

	if (spi->config->dma == SPI_DMA_OFF) {
		spi->trans->buf.addr = (unsigned int)buf;
		ret = spi_tx_start(spi, count);
		if (ret == 0)
			ret = count;
	} else {
		DECLARE_WAITQUEUE(wait, current);
		unsigned int spi_err;
		unsigned int dma_err;
		int size;

		add_wait_queue(&spi->trans->wait, &wait);
		while ((count - len) > 0) {
			set_current_state(TASK_INTERRUPTIBLE);
			spin_lock_irq(&spi->trans->spinlock);
			spi_err = spi->trans->spi_err;
			dma_err = spi->trans->dma_err;
			size = spi->trans->size;
			spi->trans->size = 0;
			spin_unlock_irq(&spi->trans->spinlock);
			if ((spi_err != 0) || (dma_err != 0)) {
				spi_init_data(spi);
				ret = -EIO;
				break;
			}

			if (size > 0) {
				if ((count - len) < size)
					size = count - len;
				len = len + size;
				ret = len;
			} else {
				if ((count - len) > SPI_DMA_MAXSIZE)
					size = SPI_DMA_MAXSIZE;
				else
					size = count - len;

				spi->trans->buf.dma_addr = phys + len;
				ret = spi_tx_start(spi, size);
				if (ret < 0)
					break;

				if (signal_pending(current)) {
					spi_tx_stop(spi);
					ret = -ERESTARTSYS;
					break;
				}
				schedule();
			}
		}
		remove_wait_queue(&spi->trans->wait, &wait);
		set_current_state(TASK_RUNNING);
	}
	spi_config_release(spi);
	spi->trans->state = SPI_UNUSED;

out:
	if (spi->excl)
		spin_unlock_irqrestore(&spi->trans->spinlock, lock_flags);
	else {
		up(&spi->sem);
	}

	return ret;
}
EXPORT_SYMBOL(spi_write);

int spi_cmd_read(SPI_CONFIG *config, char *cmd, char *buf, unsigned int flags)
{
	struct spi_data *spi;
	unsigned int unit;
	unsigned int data = 0;
	unsigned long lock_flags = 0;
	int ret = 0;

	if ((config == NULL) || (cmd == NULL) || (buf == NULL))
		return -EINVAL;

	if (config->dev != SPI_DEV_SP0)
		return -ENODEV;

	spi = &spi_private[config->dev];

	if (spi->probe == 0)
		return -EPERM;

	DEB("cmd = %08x\n", *(unsigned int *)cmd);

	/* dma/slave is unsupported */
	if (config->dma == SPI_DMA_ON || config->m_s == SPI_M_S_SLAVE)
		return -EINVAL;

	if (spi->excl) {
		spin_lock_irqsave(&spi->trans->spinlock, lock_flags);
	} else {
		if ((flags & SPI_NONBLOCK) != 0) {
			if (down_trylock(&spi->sem) != 0)
				return -EAGAIN;
		} else {
			if (down_interruptible(&spi->sem) != 0)
				return -ERESTARTSYS;
		}
	}

	while (1) {
		if ((flags & SPI_RW_2CYCLE) != 0) {
			spi->trans->state = SPI_WRITE;
			ret = spi_config(spi, config);
			if (ret < 0)
				break;
			unit = spi_unit(spi, spi->config->nbw);
			ret = spi_xferbytes(spi, cmd, unit, SPI_WRITE);
			if (ret < 0)
				break;

			spi->trans->state = SPI_READ;
			ret = spi_config(spi, config);
			if (ret < 0)
				break;
			unit = spi_unit(spi, spi->config->nbr);
			ret = spi_xferbytes(spi, buf, unit, SPI_READ);
			if (ret < 0)
				break;
		} else {
			spi->trans->state = SPI_RW;
			ret = spi_config(spi, config);
			if (ret < 0)
				break;
			memcpy((char *)&data, cmd,
					spi_unit(spi, spi->config->nbw));
			data = (data << spi->config->nbr);
			unit = spi_unit(spi,
					spi->config->nbr + spi->config->nbw);
			ret = spi_xferbytes(spi, (char *)&data, unit, SPI_RW);
			if (ret < 0)
				break;
			memcpy(buf, (char *)&data,
					spi_unit(spi, spi->config->nbr));
		}
		break;
	}

	if (ret < 0)
		spi_sft_reset(spi);

	spi_config_release(spi);
	spi->trans->state = SPI_UNUSED;

	if (spi->excl)
		spin_unlock_irqrestore(&spi->trans->spinlock, lock_flags);
	else
		up(&spi->sem);

	return ret;
}
EXPORT_SYMBOL(spi_cmd_read);

static int spi_probe(struct platform_device *dev)
{
	struct spi_data *spi;
	int ret;

	if (dev->id != SPI_DEV_SP0)
		return -ENODEV;

	spi = &spi_private[dev->id];

	if (spi->probe == 0) {
		spi_power_on(spi);

		spi->excl = sp0_excl;

		ret = spi_request_dma(spi);
		if (ret < 0) {
			spi_power_off(spi);
			return ret;
		}

		ret = request_irq(spi->trans->int_spi, spi_interrupt,
				    IRQF_DISABLED, SPI_NAME, (void *)spi);
		if (ret < 0) {
			spi_free_dma(spi);
			spi_power_off(spi);
			return ret;
		}

		ret = spi_config(spi, spi->config);
		if (ret < 0) {
			free_irq(spi->trans->int_spi, (void *)spi);
			spi_free_dma(spi);
			spi_power_off(spi);
			return ret;
		}

		init_waitqueue_head(&spi->trans->wait);
		sema_init(&spi->sem, 1);
		spi_init_data(spi);

		spi->probe = 1;
	}

	return 0;
}

static int spi_remove(struct platform_device *dev)
{
	struct spi_data *spi;

	if (dev->id != SPI_DEV_SP0)
		return -ENODEV;
	spi = &spi_private[dev->id];

	if (spi->probe != 0) {
		spi->probe = 0;
		free_irq(spi->trans->int_spi, (void *)spi);
		spi_free_dma(spi);
		spi_power_off(spi);
	}

	return 0;
}

static int spi_suspend(struct platform_device *dev, pm_message_t state)
{
	struct spi_data *spi;

	switch (state.event) {
	case PM_EVENT_SUSPEND:
		if (dev->id != SPI_DEV_SP0)
			return -ENODEV;
		spi = &spi_private[dev->id];
		if (spi->trans->state != SPI_UNUSED)
			return -EBUSY;
		break;
	default:
		break;
	}

	return 0;
}

static int spi_resume(struct platform_device *dev)
{
	return 0;
}

static struct platform_driver spi_drv = {
	.driver = {
		.name = SPI_NAME,
		.owner = THIS_MODULE,
	},
	.probe = spi_probe,
	.remove = spi_remove,
	.suspend = spi_suspend,
	.resume = spi_resume,
};

static int __init spi_init(void)
{
	return platform_driver_register(&spi_drv);
}

static void __exit spi_exit(void)
{
	platform_driver_unregister(&spi_drv);

}

module_init(spi_init);
module_exit(spi_exit);
MODULE_LICENSE("GPL");
