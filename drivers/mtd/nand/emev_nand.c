/*
 *  File Name		: drivers/mtd/nand/emxx_nand.c
 *  Function		: NAND Flash memory access on emxx based devices
 *  Release Version 	: Ver 1.02
 *  Release Date	: 2010/07/23
 *
 *  Copyright (C) 2010 Renesas Electronics Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <linux/io.h>
#include <linux/gpio.h>
#include <asm/system.h>

#ifdef CONFIG_MTD_PARTITIONS
const char *part_probes[] = { "cmdlinepart", NULL, };
#endif

static struct mtd_info *emxx_nand_mtd;

static void __iomem *emxx_nand_data;
static void __iomem *emxx_nand_cmd;
static void __iomem *emxx_nand_addr;

static int last_nce;

#define EMXX_NAND_WP		GPIO_EXT1_P24
#define EMXX_NAND_CHKBIT	(0x00000400)	/* 1:nothing, 0: exist */

static void emxx_nand_hwcontrol(struct mtd_info *mtd, int cmd,
 unsigned int ctrl)
{
	struct nand_chip *this = mtd->priv;

	if (ctrl & NAND_CTRL_CHANGE) {
		if (ctrl & NAND_NCE) {
			if (last_nce == 0) {
				gpio_set_value(EMXX_NAND_WP, 1);
				last_nce = 1;
			}
		} else {
			if (last_nce == 1) {
				gpio_set_value(EMXX_NAND_WP, 0);
				last_nce = 0;
			}
		}
		if (ctrl & NAND_CLE)
			this->IO_ADDR_W = emxx_nand_cmd;
		else if (ctrl & NAND_ALE)
			this->IO_ADDR_W = emxx_nand_addr;
		else
			this->IO_ADDR_W = emxx_nand_data;

	}

	if (cmd != NAND_CMD_NONE)
		writew(cmd, this->IO_ADDR_W);

}

static void emxx_nand_command_lp(struct mtd_info *mtd, unsigned int command,
			    int column, int page_addr)
{
	register struct nand_chip *chip = mtd->priv;

	/* Emulate NAND_CMD_READOOB */
	if (command == NAND_CMD_READOOB) {
		column += mtd->writesize;
		command = NAND_CMD_READ0;
	}

	/* Command latch cycle */
	chip->cmd_ctrl(mtd, command & 0xff,
		       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);

	if (column != -1 || page_addr != -1) {
		int ctrl = NAND_CTRL_CHANGE | NAND_NCE | NAND_ALE;

		/* Serially input address */
		if (column != -1) {
			/* Adjust columns for 16 bit buswidth */
			if (chip->options & NAND_BUSWIDTH_16)
				column >>= 1;
			chip->cmd_ctrl(mtd, column, ctrl);
			if (command != NAND_CMD_READID) {
				ctrl &= ~NAND_CTRL_CHANGE;
				chip->cmd_ctrl(mtd, column >> 8, ctrl);
			}
		}
		if (page_addr != -1) {
			chip->cmd_ctrl(mtd, page_addr, ctrl);
			chip->cmd_ctrl(mtd, page_addr >> 8,
				       NAND_NCE | NAND_ALE);
			/* One more address cycle for devices > 128MiB */
			if (chip->chipsize > (128 << 20))
				chip->cmd_ctrl(mtd, page_addr >> 16,
					       NAND_NCE | NAND_ALE);
		}
	}
	chip->cmd_ctrl(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

	/*
	 * program and erase have their own busy handlers
	 * status, sequential in, and deplete1 need no delay
	 */
	switch (command) {

	case NAND_CMD_CACHEDPROG:
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_SEQIN:
	case NAND_CMD_RNDIN:
	case NAND_CMD_STATUS:
	case NAND_CMD_DEPLETE1:
		return;

		/*
		 * read error status commands require only a short delay
		 */
	case NAND_CMD_STATUS_ERROR:
	case NAND_CMD_STATUS_ERROR0:
	case NAND_CMD_STATUS_ERROR1:
	case NAND_CMD_STATUS_ERROR2:
	case NAND_CMD_STATUS_ERROR3:
		udelay(chip->chip_delay);
		return;

	case NAND_CMD_RESET:
		if (chip->dev_ready)
			break;
		udelay(chip->chip_delay);
		chip->cmd_ctrl(mtd, NAND_CMD_STATUS,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		while (!(chip->read_byte(mtd) & NAND_STATUS_READY))
			;

		return;

	case NAND_CMD_RNDOUT:
		/* No ready / busy check necessary */
		chip->cmd_ctrl(mtd, NAND_CMD_RNDOUTSTART,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		return;

	case NAND_CMD_READ0:
		chip->cmd_ctrl(mtd, NAND_CMD_READSTART,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_STATUS, NAND_NCE | NAND_CLE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		while (!(chip->read_byte(mtd) & NAND_STATUS_READY))
			;

		chip->cmd_ctrl(mtd, NAND_CMD_READ0,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		return;

		/* This applies to read commands */
	default:
		/*
		 * If we don't have access to the busy pin, we apply the given
		 * command delay
		 */
		if (!chip->dev_ready) {
			udelay(chip->chip_delay);
			return;
		}
	}

	/* Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine. */
	ndelay(100);
}



static int emxx_nand_probe(struct platform_device *pdev)
{
	int err = 0;
	struct platform_nand_chip *pdata = pdev->dev.platform_data;
	struct resource *res[3];
	unsigned long size[3];
	struct nand_chip *this;
	struct mtd_partition *parts = 0;

	last_nce = 0;

	res[0] = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res[1] = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	res[2] = platform_get_resource(pdev, IORESOURCE_MEM, 2);

	if ((!res[0]) || (!res[1]) || (!res[2])) {
		printk(KERN_WARNING "Unable get resource infomation.\n");
		return -ENOMEM;
	}

	size[0] = res[0]->end - res[0]->start + 1;
	size[1] = res[1]->end - res[1]->start + 1;
	size[2] = res[2]->end - res[2]->start + 1;

	/* Allocate memory for MTD device structure and private data */
	emxx_nand_mtd = kmalloc(sizeof(struct mtd_info)
	 + sizeof(struct nand_chip), GFP_KERNEL);
	if (!emxx_nand_mtd) {
		printk(KERN_WARNING "Unable to allocate NAND MTD device structure.\n");
		return -ENOMEM;
	}

	/* Initialize structures */
	memset((char *)emxx_nand_mtd, 0,
	 sizeof(struct mtd_info) + sizeof(struct nand_chip));

	/* Get pointer to private data */
	this = (struct nand_chip *)(&emxx_nand_mtd[1]);

	/* Link the private data with the MTD structure */
	emxx_nand_mtd->priv = this;
	emxx_nand_mtd->owner = THIS_MODULE;

	/* io address maping */
	emxx_nand_data = ioremap(res[0]->start, size[0]);
	if (emxx_nand_data == 0) {
		err = -ENOMEM;
		goto out_free_info;
	}
	emxx_nand_cmd = ioremap(res[1]->start, size[1]);
	if (emxx_nand_cmd == 0) {
		err = -ENOMEM;
		goto out_iounmap_data;
	}
	emxx_nand_addr = ioremap(res[2]->start, size[2]);
	if (emxx_nand_addr == 0) {
		err = -ENOMEM;
		goto out_iounmap_cmd;
	}

	err = gpio_request(EMXX_NAND_WP, "emxx_nand");
	if (err)
		goto out_iounmap_addr;

	gpio_direction_output(EMXX_NAND_WP, 0);

	this->IO_ADDR_R  = emxx_nand_data;
	this->IO_ADDR_W  = emxx_nand_data;
	this->cmd_ctrl   = emxx_nand_hwcontrol;
	this->chip_delay = pdata->chip_delay;
	this->ecc.mode   = NAND_ECC_SOFT;
	this->options    = pdata->options;
	this->cmdfunc    = emxx_nand_command_lp;

	if (nand_scan(emxx_nand_mtd, pdata->nr_chips) != 0) {
		err = -EIO;
		goto out_gpio_free;
	}

	/* Register the partitions */
#ifdef CONFIG_MTD_PARTITIONS
	err = parse_mtd_partitions(emxx_nand_mtd, part_probes, &parts, 0);
	if (err > 0)
		add_mtd_partitions(emxx_nand_mtd, parts, err);
	else if (pdata->nr_partitions != 0) {
		add_mtd_partitions(emxx_nand_mtd, pdata->partitions,
			pdata->nr_partitions);
	} else
		add_mtd_device(emxx_nand_mtd);
#else
	add_mtd_device(emxx_nand_mtd);
#endif

	return 0;

out_gpio_free:
	gpio_free(EMXX_NAND_WP);
out_iounmap_addr:
	iounmap(emxx_nand_addr);
out_iounmap_cmd:
	iounmap(emxx_nand_cmd);
out_iounmap_data:
	iounmap(emxx_nand_data);
out_free_info:
	kfree(emxx_nand_mtd);

	return err;
}

static int __devexit emxx_nand_remove(struct platform_device *pdev)
{
	if (emxx_nand_mtd != 0) {
		nand_release(emxx_nand_mtd);
		iounmap(emxx_nand_data);
		iounmap(emxx_nand_cmd);
		iounmap(emxx_nand_addr);
		kfree(emxx_nand_mtd);
		emxx_nand_mtd = NULL;
		gpio_free(EMXX_NAND_WP);
	}

	return 0;
}

static struct platform_driver emxx_nand_driver = {
	.probe		= emxx_nand_probe,
	.remove		= __devexit_p(emxx_nand_remove),
	.driver		= {
		.name	= "emxx_nand",
		.owner	= THIS_MODULE,
	},
};


static int __init emxx_nand_init(void)
{
	if (system_rev & EMXX_NAND_CHKBIT)
		return -ENODEV;

	return platform_driver_register(&emxx_nand_driver);
}

static void __exit emxx_nand_exit(void)
{
	platform_driver_unregister(&emxx_nand_driver);
}

module_init(emxx_nand_init);
module_exit(emxx_nand_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("EMXX series MTD NAND driver");
