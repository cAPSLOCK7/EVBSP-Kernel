/*
 *  linux/drivers/mmc/core/mmc.c
 *
 *  Copyright (C) 2010 Renesas Electronics Corporation
 *  Copyright (C) 2003-2004 Russell King, All Rights Reserved.
 *  Copyright (C) 2005-2007 Pierre Ossman, All Rights Reserved.
 *  MMCv4 support Copyright (C) 2006 Philip Langdale, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>

#include "core.h"
#include "bus.h"
#include "mmc_ops.h"

static const unsigned int tran_exp[] = {
	10000,		100000,		1000000,	10000000,
	0,		0,		0,		0
};

static const unsigned char tran_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

static const unsigned int tacc_exp[] = {
	1,	10,	100,	1000,	10000,	100000,	1000000, 10000000,
};

static const unsigned int tacc_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

#define UNSTUFF_BITS(resp,start,size)					\
	({								\
		const int __size = size;				\
		const u32 __mask = (__size < 32 ? 1 << __size : 0) - 1;	\
		const int __off = 3 - ((start) / 32);			\
		const int __shft = (start) & 31;			\
		u32 __res;						\
									\
		__res = resp[__off] >> __shft;				\
		if (__size + __shft > 32)				\
			__res |= resp[__off-1] << ((32 - __shft) % 32);	\
		__res & __mask;						\
	})

/*
 * Given the decoded CSD structure, decode the raw CID to our CID structure.
 */
static int mmc_decode_cid(struct mmc_card *card)
{
	u32 *resp = card->raw_cid;

	/*
	 * The selection of the format here is based upon published
	 * specs from sandisk and from what people have reported.
	 */
	switch (card->csd.mmca_vsn) {
	case 0: /* MMC v1.0 - v1.2 */
	case 1: /* MMC v1.4 */
		card->cid.manfid	= UNSTUFF_BITS(resp, 104, 24);
		card->cid.prod_name[0]	= UNSTUFF_BITS(resp, 96, 8);
		card->cid.prod_name[1]	= UNSTUFF_BITS(resp, 88, 8);
		card->cid.prod_name[2]	= UNSTUFF_BITS(resp, 80, 8);
		card->cid.prod_name[3]	= UNSTUFF_BITS(resp, 72, 8);
		card->cid.prod_name[4]	= UNSTUFF_BITS(resp, 64, 8);
		card->cid.prod_name[5]	= UNSTUFF_BITS(resp, 56, 8);
		card->cid.prod_name[6]	= UNSTUFF_BITS(resp, 48, 8);
		card->cid.hwrev		= UNSTUFF_BITS(resp, 44, 4);
		card->cid.fwrev		= UNSTUFF_BITS(resp, 40, 4);
		card->cid.serial	= UNSTUFF_BITS(resp, 16, 24);
		card->cid.month		= UNSTUFF_BITS(resp, 12, 4);
		card->cid.year		= UNSTUFF_BITS(resp, 8, 4) + 1997;
		break;

	case 2: /* MMC v2.0 - v2.2 */
	case 3: /* MMC v3.1 - v3.3 */
	case 4: /* MMC v4 */
		card->cid.manfid	= UNSTUFF_BITS(resp, 120, 8);
		card->cid.oemid		= UNSTUFF_BITS(resp, 104, 16);
		card->cid.prod_name[0]	= UNSTUFF_BITS(resp, 96, 8);
		card->cid.prod_name[1]	= UNSTUFF_BITS(resp, 88, 8);
		card->cid.prod_name[2]	= UNSTUFF_BITS(resp, 80, 8);
		card->cid.prod_name[3]	= UNSTUFF_BITS(resp, 72, 8);
		card->cid.prod_name[4]	= UNSTUFF_BITS(resp, 64, 8);
		card->cid.prod_name[5]	= UNSTUFF_BITS(resp, 56, 8);
		card->cid.serial	= UNSTUFF_BITS(resp, 16, 32);
		card->cid.month		= UNSTUFF_BITS(resp, 12, 4);
		card->cid.year		= UNSTUFF_BITS(resp, 8, 4) + 1997;
		break;

	default:
		printk(KERN_ERR "%s: card has unknown MMCA version %d\n",
			mmc_hostname(card->host), card->csd.mmca_vsn);
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_ARCH_EMXX
static unsigned int s_csd_struct = -1;
#endif
/*
 * Given a 128-bit response, decode to our card CSD structure.
 */
static int mmc_decode_csd(struct mmc_card *card)
{
	struct mmc_csd *csd = &card->csd;
	unsigned int e, m, csd_struct;
	u32 *resp = card->raw_csd;

	/*
	 * We only understand CSD structure v1.1 and v1.2.
	 * v1.2 has extra information in bits 15, 11 and 10.
	 */
	csd_struct = UNSTUFF_BITS(resp, 126, 2);
#ifdef CONFIG_ARCH_EMXX
	s_csd_struct = csd_struct;
#endif

	if (csd_struct != 1 && csd_struct != 2) {
#ifdef CONFIG_ARCH_EMXX
		s_csd_struct = -1;
		if (csd_struct != 3) {
			printk(KERN_ERR "%s: unrecognised CSD structure version %d\n",
				mmc_hostname(card->host), csd_struct);
			return -EINVAL;
		}
		/* check csd_struct in ext_csd */
#else
		printk(KERN_ERR "%s: unrecognised CSD structure version %d\n",
			mmc_hostname(card->host), csd_struct);
		return -EINVAL;
#endif
	}

	csd->mmca_vsn	 = UNSTUFF_BITS(resp, 122, 4);
	m = UNSTUFF_BITS(resp, 115, 4);
	e = UNSTUFF_BITS(resp, 112, 3);
	csd->tacc_ns	 = (tacc_exp[e] * tacc_mant[m] + 9) / 10;
	csd->tacc_clks	 = UNSTUFF_BITS(resp, 104, 8) * 100;

	m = UNSTUFF_BITS(resp, 99, 4);
	e = UNSTUFF_BITS(resp, 96, 3);
	csd->max_dtr	  = tran_exp[e] * tran_mant[m];
	csd->cmdclass	  = UNSTUFF_BITS(resp, 84, 12);

	e = UNSTUFF_BITS(resp, 47, 3);
	m = UNSTUFF_BITS(resp, 62, 12);
	csd->capacity	  = (1 + m) << (e + 2);

	csd->read_blkbits = UNSTUFF_BITS(resp, 80, 4);
	csd->read_partial = UNSTUFF_BITS(resp, 79, 1);
	csd->write_misalign = UNSTUFF_BITS(resp, 78, 1);
	csd->read_misalign = UNSTUFF_BITS(resp, 77, 1);
	csd->r2w_factor = UNSTUFF_BITS(resp, 26, 3);
	csd->write_blkbits = UNSTUFF_BITS(resp, 22, 4);
	csd->write_partial = UNSTUFF_BITS(resp, 21, 1);

	return 0;
}

/*
 * Read and decode extended CSD.
 */
#ifdef CONFIG_ARCH_EMXX
static int mmc_read_ext_csd(struct mmc_card *card, u32 ocr)
#else
static int mmc_read_ext_csd(struct mmc_card *card)
#endif
{
	int err;
	u8 *ext_csd;
	unsigned int ext_csd_struct;

	BUG_ON(!card);

#ifdef CONFIG_ARCH_EMXX
	if (s_csd_struct != -1 && card->csd.mmca_vsn < CSD_SPEC_VER_4)
		return 0;
#else
	if (card->csd.mmca_vsn < CSD_SPEC_VER_4)
		return 0;
#endif

	/*
	 * As the ext_csd is so large and mostly unused, we don't store the
	 * raw block in mmc_card.
	 */
	ext_csd = kmalloc(512, GFP_KERNEL);
	if (!ext_csd) {
		printk(KERN_ERR "%s: could not allocate a buffer to "
			"receive the ext_csd.\n", mmc_hostname(card->host));
		return -ENOMEM;
	}

	err = mmc_send_ext_csd(card, ext_csd);
	if (err) {
		/*
		 * We all hosts that cannot perform the command
		 * to fail more gracefully
		 */
		if (err != -EINVAL)
			goto out;

		/*
		 * High capacity cards should have this "magic" size
		 * stored in their CSD.
		 */
		if (card->csd.capacity == (4096 * 512)) {
			printk(KERN_ERR "%s: unable to read EXT_CSD "
				"on a possible high capacity card. "
				"Card will be ignored.\n",
				mmc_hostname(card->host));
		} else {
			printk(KERN_WARNING "%s: unable to read "
				"EXT_CSD, performance might "
				"suffer.\n",
				mmc_hostname(card->host));
			err = 0;
		}

		goto out;
	}

#ifdef CONFIG_ARCH_EMXX
	ext_csd_struct = ext_csd[194];
	if (s_csd_struct == -1) {
		/* check csd_struct */
		s_csd_struct = ext_csd_struct;
		if (s_csd_struct != 1 && s_csd_struct != 2) {
			printk(KERN_ERR "%s: unrecognised CSD structure version %d in ext_csd\n",
				mmc_hostname(card->host), s_csd_struct);
			return -EINVAL;
		}
	}
#else
	ext_csd_struct = ext_csd[EXT_CSD_REV];
#endif
	if (ext_csd_struct > 2) {
		printk(KERN_ERR "%s: unrecognised EXT_CSD structure "
			"version %d\n", mmc_hostname(card->host),
			ext_csd_struct);
		err = -EINVAL;
		goto out;
	}

	if (ext_csd_struct >= 2) {
		card->ext_csd.sectors =
			ext_csd[EXT_CSD_SEC_CNT + 0] << 0 |
			ext_csd[EXT_CSD_SEC_CNT + 1] << 8 |
			ext_csd[EXT_CSD_SEC_CNT + 2] << 16 |
			ext_csd[EXT_CSD_SEC_CNT + 3] << 24;
#ifdef CONFIG_ARCH_EMXX
		if (card->host->card_num > 1) {
			if (card->ext_csd.sectors > 0x400000)
				mmc_card_set_blockaddr(card);
		} else {
			if ((card->ext_csd.sectors) && (ocr & (1 << 30)))
				mmc_card_set_blockaddr(card);
		}
#else
		if (card->ext_csd.sectors)
			mmc_card_set_blockaddr(card);
#endif
	}

	switch (ext_csd[EXT_CSD_CARD_TYPE]) {
	case EXT_CSD_CARD_TYPE_52 | EXT_CSD_CARD_TYPE_26:
		card->ext_csd.hs_max_dtr = 52000000;
		break;
	case EXT_CSD_CARD_TYPE_26:
		card->ext_csd.hs_max_dtr = 26000000;
		break;
	default:
		/* MMC v4 spec says this cannot happen */
		printk(KERN_WARNING "%s: card is mmc v4 but doesn't "
			"support any high-speed modes.\n",
			mmc_hostname(card->host));
		goto out;
	}

out:
	kfree(ext_csd);

	return err;
}

MMC_DEV_ATTR(cid, "%08x%08x%08x%08x\n", card->raw_cid[0], card->raw_cid[1],
	card->raw_cid[2], card->raw_cid[3]);
MMC_DEV_ATTR(csd, "%08x%08x%08x%08x\n", card->raw_csd[0], card->raw_csd[1],
	card->raw_csd[2], card->raw_csd[3]);
MMC_DEV_ATTR(date, "%02d/%04d\n", card->cid.month, card->cid.year);
MMC_DEV_ATTR(fwrev, "0x%x\n", card->cid.fwrev);
MMC_DEV_ATTR(hwrev, "0x%x\n", card->cid.hwrev);
MMC_DEV_ATTR(manfid, "0x%06x\n", card->cid.manfid);
MMC_DEV_ATTR(name, "%s\n", card->cid.prod_name);
MMC_DEV_ATTR(oemid, "0x%04x\n", card->cid.oemid);
MMC_DEV_ATTR(serial, "0x%08x\n", card->cid.serial);

static struct attribute *mmc_std_attrs[] = {
	&dev_attr_cid.attr,
	&dev_attr_csd.attr,
	&dev_attr_date.attr,
	&dev_attr_fwrev.attr,
	&dev_attr_hwrev.attr,
	&dev_attr_manfid.attr,
	&dev_attr_name.attr,
	&dev_attr_oemid.attr,
	&dev_attr_serial.attr,
	NULL,
};

static struct attribute_group mmc_std_attr_group = {
	.attrs = mmc_std_attrs,
};

static struct attribute_group *mmc_attr_groups[] = {
	&mmc_std_attr_group,
	NULL,
};

static struct device_type mmc_type = {
	.groups = mmc_attr_groups,
};

/*
 * Handle the detection and initialisation of a card.
 *
 * In the case of a resume, "oldcard" will contain the card
 * we're trying to reinitialise.
 */
static int mmc_init_card(struct mmc_host *host, u32 ocr,
#ifdef CONFIG_ARCH_EMXX
	struct mmc_card **oldcard)
#else
	struct mmc_card *oldcard)
#endif
{
	struct mmc_card *card;
	int err;
	u32 cid[4];
	unsigned int max_dtr;
#ifdef CONFIG_ARCH_EMXX
	int i;
	u32 rocr;
#endif

	BUG_ON(!host);
	WARN_ON(!host->claimed);
#ifdef CONFIG_ARCH_EMXX
	BUG_ON(host->card_num > MMC_CARD_MAX_NUM);
	BUG_ON(oldcard && !oldcard[0]);

	/* initialize select state */
	host->select = 0xffffffff;
#endif

	/*
	 * Since we're changing the OCR value, we seem to
	 * need to tell some cards to go back to the idle
	 * state.  We wait 1ms to give cards time to
	 * respond.
	 */
	mmc_go_idle(host);

	/* The extra bit indicates that we support high capacity */
#ifdef CONFIG_ARCH_EMXX
	err = mmc_send_op_cond(host, ocr | (1 << 30), &rocr);
#else
	err = mmc_send_op_cond(host, ocr | (1 << 30), NULL);
#endif
	if (err)
		goto err;

#ifdef CONFIG_ARCH_EMXX
	for (i = 0; i < host->card_num; i++) {
		err = mmc_all_send_cid(host, cid);
		if (err) {
			if (i == 0) {
				printk(KERN_ERR "%s:%d: Error \n",
							__func__, __LINE__);
				goto err;
			} else {
				host->card_num = i;
				break;
			}
		}

		if (oldcard) {
			if (memcmp(cid, oldcard[i]->raw_cid,
							sizeof(cid)) != 0) {
				err = -ENOENT;
				printk(KERN_ERR "%s:%d: Error \n",
							__func__, __LINE__);
				goto err;
			}
			host->card[i] = oldcard[i];
		} else {
			/*
			 * Allocate card structure.
			 */
			host->card[i] = mmc_alloc_card(host, &mmc_type);
			if (IS_ERR(host->card[i])) {
				err = PTR_ERR(host->card[i]);
				goto err;
			}

			host->card[i]->type = MMC_TYPE_MMC;
			host->card[i]->rca = i+1;
			memcpy(host->card[i]->raw_cid, cid, sizeof(cid));
		}
		/*
		 * Set card RCA.
		 */
		err = mmc_set_relative_addr(host->card[i]);
		if (err)
			goto free_card;
	}
#else
	/*
	 * For SPI, enable CRC as appropriate.
	 */
	if (mmc_host_is_spi(host)) {
		err = mmc_spi_set_crc(host, use_spi_crc);
		if (err)
			goto err;
	}

	/*
	 * Fetch CID from card.
	 */
	if (mmc_host_is_spi(host))
		err = mmc_send_cid(host, cid);
	else
		err = mmc_all_send_cid(host, cid);
	if (err)
		goto err;

	if (oldcard) {
		if (memcmp(cid, oldcard->raw_cid, sizeof(cid)) != 0) {
			err = -ENOENT;
			goto err;
		}

		card = oldcard;
	} else {
		/*
		 * Allocate card structure.
		 */
		card = mmc_alloc_card(host, &mmc_type);
		if (IS_ERR(card)) {
			err = PTR_ERR(card);
			goto err;
		}

		card->type = MMC_TYPE_MMC;
		card->rca = 1;
		memcpy(card->raw_cid, cid, sizeof(card->raw_cid));
	}

	/*
	 * For native busses:  set card RCA and quit open drain mode.
	 */
	if (!mmc_host_is_spi(host)) {
		err = mmc_set_relative_addr(card);
		if (err)
			goto free_card;
#endif
#ifdef CONFIG_ARCH_EMXX
	if (!mmc_host_is_spi(host)) {
#endif
		mmc_set_bus_mode(host, MMC_BUSMODE_PUSHPULL);
	}

#ifdef CONFIG_ARCH_EMXX
	for (i = 0; i < host->card_num; i++) {
		card = host->card[i];
#endif
	if (!oldcard) {
		/*
		 * Fetch CSD from card.
		 */
		err = mmc_send_csd(card, card->raw_csd);
		if (err)
			goto free_card;

		err = mmc_decode_csd(card);
		if (err)
			goto free_card;
		err = mmc_decode_cid(card);
		if (err)
			goto free_card;
	}

	/*
	 * Select card, as all following commands rely on that.
	 */
	if (!mmc_host_is_spi(host)) {
		err = mmc_select_card(card);
		if (err)
			goto free_card;
	}

	if (!oldcard) {
		/*
		 * Fetch and process extended CSD.
		 */
#ifdef CONFIG_ARCH_EMXX
		if (i == 0) {
			err = mmc_read_ext_csd(card, rocr);
			if (err)
				goto free_card;
		} else {
			host->card[i]->ext_csd = host->card[0]->ext_csd;
			if (host->card[i]->ext_csd.sectors)
				mmc_card_set_blockaddr(card);
		}
#else
		err = mmc_read_ext_csd(card);
		if (err)
			goto free_card;
#endif
	}

	/*
	 * Activate high speed (if supported)
	 */
	if ((card->ext_csd.hs_max_dtr != 0) &&
		(host->caps & MMC_CAP_MMC_HIGHSPEED)) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			EXT_CSD_HS_TIMING, 1);
		if (err)
			goto free_card;

		mmc_card_set_highspeed(card);

		mmc_set_timing(card->host, MMC_TIMING_MMC_HS);
	}

	/*
	 * Compute bus speed.
	 */
	max_dtr = (unsigned int)-1;

	if (mmc_card_highspeed(card)) {
		if (max_dtr > card->ext_csd.hs_max_dtr)
			max_dtr = card->ext_csd.hs_max_dtr;
	} else if (max_dtr > card->csd.max_dtr) {
		max_dtr = card->csd.max_dtr;
	}

	mmc_set_clock(host, max_dtr);

	/*
	 * Activate wide bus (if supported).
	 */
	if ((card->csd.mmca_vsn >= CSD_SPEC_VER_4) &&
	    (host->caps & (MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA))) {
		unsigned ext_csd_bit, bus_width;

		if (host->caps & MMC_CAP_8_BIT_DATA) {
			ext_csd_bit = EXT_CSD_BUS_WIDTH_8;
			bus_width = MMC_BUS_WIDTH_8;
		} else {
			ext_csd_bit = EXT_CSD_BUS_WIDTH_4;
			bus_width = MMC_BUS_WIDTH_4;
		}

		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_BUS_WIDTH, ext_csd_bit);

		if (err)
			goto free_card;

		mmc_set_bus_width(card->host, bus_width);
	}
#ifdef CONFIG_ARCH_EMXX
	} /* for */
#else
	if (!oldcard)
		host->card = card;
#endif

	return 0;

free_card:
#ifdef CONFIG_ARCH_EMXX
	if (!oldcard) {
		for (i = 0; i < host->card_num; i++) {
			mmc_remove_card(host->card[i]);
			host->card[i] = NULL;
		}
	}
#else
	if (!oldcard)
		mmc_remove_card(card);
#endif
err:

	return err;
}

/*
 * Host is being removed. Free up the current card.
 */
static void mmc_remove(struct mmc_host *host)
{
#ifdef CONFIG_ARCH_EMXX
	int i;
#endif
	BUG_ON(!host);
#ifdef CONFIG_ARCH_EMXX
	for (i = 0; i < host->card_num; i++) {
		BUG_ON(!host->card[i]);
		mmc_remove_card(host->card[i]);
		host->card[i] = NULL;
	}
#else
	BUG_ON(!host->card);

	mmc_remove_card(host->card);
	host->card = NULL;
#endif
}

/*
 * Card detection callback from host.
 */
static void mmc_detect(struct mmc_host *host)
{
	int err = 0;
#ifdef CONFIG_ARCH_EMXX
	int i;
#endif

	BUG_ON(!host);
#ifndef CONFIG_ARCH_EMXX
	BUG_ON(!host->card);
#endif

	mmc_claim_host(host);

	/*
	 * Just check if our card has been removed.
	 */
#ifdef CONFIG_ARCH_EMXX
	if (host->ios.power_mode != MMC_POWER_OFF) {
		for (i = 0; i < host->card_num; i++) {
			if (host->card[i])
				err |= mmc_send_status(host->card[i], NULL);
		}
	}
#else
	err = mmc_send_status(host->card, NULL);
#endif
	mmc_release_host(host);

	if (err) {
		mmc_remove(host);

		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_release_host(host);
	}
}

#ifdef CONFIG_MMC_UNSAFE_RESUME

/*
 * Suspend callback from host.
 */
static void mmc_suspend(struct mmc_host *host)
{
#ifdef CONFIG_ARCH_EMXX
	int i;
#endif
	BUG_ON(!host);
#ifdef CONFIG_ARCH_EMXX
	for (i = 0; i < host->card_num; i++)
		BUG_ON(!host->card[i]);
#else
	BUG_ON(!host->card);
#endif

	mmc_claim_host(host);
	if (!mmc_host_is_spi(host))
		mmc_deselect_cards(host);
#ifdef CONFIG_ARCH_EMXX
	for (i = 0; i < host->card_num; i++)
		host->card[i]->state &= ~MMC_STATE_HIGHSPEED;
#else
	host->card->state &= ~MMC_STATE_HIGHSPEED;
#endif
	mmc_release_host(host);
}

/*
 * Resume callback from host.
 *
 * This function tries to determine if the same card is still present
 * and, if so, restore all state to it.
 */
static void mmc_resume(struct mmc_host *host)
{
	int err = 0;
#ifdef CONFIG_ARCH_EMXX
	int i, skip_flag = 0;
#endif

	BUG_ON(!host);
#ifndef CONFIG_ARCH_EMXX
	BUG_ON(!host->card);
#endif

	mmc_claim_host(host);
#ifdef CONFIG_ARCH_EMXX
	for (i = 0; i < host->card_num; i++) {
		if (host->card[i] == NULL)
			skip_flag = 1;
	}
	if (!skip_flag) {
		err = mmc_init_card(host, host->ocr, host->card);
		if (err) {
			mmc_remove(host);
			mmc_detach_bus(host);
		}
	}
#else
	err = mmc_init_card(host, host->ocr, host->card);
#endif
	mmc_release_host(host);

#ifndef CONFIG_ARCH_EMXX
	if (err) {
		mmc_remove(host);

		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_release_host(host);
	}
#endif

}

#else

#define mmc_suspend NULL
#define mmc_resume NULL

#endif

static const struct mmc_bus_ops mmc_ops = {
	.remove = mmc_remove,
	.detect = mmc_detect,
	.suspend = mmc_suspend,
	.resume = mmc_resume,
};

/*
 * Starting point for MMC card init.
 */
int mmc_attach_mmc(struct mmc_host *host, u32 ocr)
{
	int err;
#ifdef CONFIG_ARCH_EMXX
	int i;
#endif

	BUG_ON(!host);
	WARN_ON(!host->claimed);

	mmc_attach_bus(host, &mmc_ops);

	/*
	 * We need to get OCR a different way for SPI.
	 */
	if (mmc_host_is_spi(host)) {
		err = mmc_spi_read_ocr(host, 1, &ocr);
		if (err)
			goto err;
	}

	/*
	 * Sanity check the voltages that the card claims to
	 * support.
	 */
	if (ocr & 0x7F) {
		printk(KERN_WARNING "%s: card claims to support voltages "
		       "below the defined range. These will be ignored.\n",
		       mmc_hostname(host));
		ocr &= ~0x7F;
	}

	host->ocr = mmc_select_voltage(host, ocr);

	/*
	 * Can we support the voltage of the card?
	 */
	if (!host->ocr) {
		err = -EINVAL;
		goto err;
	}

	/*
	 * Detect and init the card.
	 */
	err = mmc_init_card(host, host->ocr, NULL);
	if (err)
		goto err;

	mmc_release_host(host);

#ifdef CONFIG_ARCH_EMXX
	for (i = 0; i < host->card_num; i++) {
		err = mmc_add_card(host->card[i]);
		if (err)
			goto remove_card;
	}
#else
	err = mmc_add_card(host->card);
	if (err)
		goto remove_card;
#endif

	return 0;

remove_card:
#ifdef CONFIG_ARCH_EMXX
	for (i = 0; i < host->card_num; i++) {
		mmc_remove_card(host->card[i]);
		host->card[i] = NULL;
	}
#else
	mmc_remove_card(host->card);
	host->card = NULL;
#endif
	mmc_claim_host(host);
err:
	mmc_detach_bus(host);
	mmc_release_host(host);

	printk(KERN_ERR "%s: error %d whilst initialising MMC card\n",
		mmc_hostname(host), err);

	return err;
}
