/* -*- mode: c; c-basic-offset: 8; comment-column: 32; -*- */

/*
 * InterDSP Driver
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

/*
 * 	CPU-DSP communication driver : inter_dsp_ioctl.h
 */

#ifndef INTER_DSP_IOCTL_H
#define INTER_DSP_IOCTL_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/ioctl.h>
#else
#include <inttypes.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#endif

/*
 *	ioctl use structure definition
 */

/*
 * IOCTL struct
 */

struct interdsp_chinfo {
	uint32_t header_offset;	/* offset of header area */
	uint32_t buffer_offset;	/* offset of buffer area */
	uint32_t buffer_blknum;	/* block number of buffer */
	uint32_t buffer_blksize; /* size of buffer block(32bit unit) */
};

/* INTERDSP_READ/INTERDSP_WRITE */

/* Content of flags of struct dsp_cmd_rw */
#define INTERDSP_WRITEBUF_OK	0x0001 /* buffer write-in success */
#define INTERDSP_SENDHDR_OK	0x0002 /* header sending-out success */

struct dsp_cmd_rw {
	uint32_t fdata[2];	/* header data*/
	uint8_t *bdata;		/* copy space of user */
	size_t bsize;		/* copy space size of user(byte) */
	size_t offset;		/* write-in offset */

	int intq_num;		/* not use */

	/* a driver updates it as follows */
	uint32_t flags;		/* situation in nonblocking */
};

/* ch info */
struct dsp_cmd_ch {
	uint32_t dsparm;	/* 0(ARM->DSP), 1(DSP->ARM) */
	uint32_t chnum;		/* channel number(0, 1, 2) */
	struct interdsp_chinfo chinfo; /* channel information (all 0:invalid) */
};

/* TaskID Channel */
struct dsp_cmd_task_ch {
	uint32_t taskid;	/* TaskID */
	int32_t chnum;		/* channel number(0,1,2), -1 is invalid */
};

/* Channel Status */
struct dsp_cmd_ch_status {
	uint32_t armchmask;	/* Ch. No. Mask (1<<0,1<<1,1<<2) */
	uint32_t dspchmask;	/* Ch. No. Mask (1<<0,1<<1,1<<2) */

	/* Block number(0,1,2...) that has locked
	   ARM CH0/1/2 is returned as bitmap.
	   1 is lock, 0 is unlock */
	unsigned long armch_blockbitmap[3];
};


/*
 * Program DownLoad
 */
/* control flags */
#define INTERDSP_DL_PROG	0x01 /* download */
				     /* 0x02 unused (INTERDSP_DL_POWER) */
#define INTERDSP_DL_RESET	0x04 /* reset control */
#define INTERDSP_DL_CLOCK	0x08 /* clock gate */
#define INTERDSP_DL_CLKCTRL	0x10 /* automatic clock control */

/*
 * Download `offset' examples
 *
 *   memtype  target-addr     offset
 *   0        0xb0001234  =>  0x00001234    [target - 0xb0000000]
 *   1        0xb0108000  =>  0x00008000    [target - 0xb0100000]
 *   2        0xa0000100  =>  0x00000100    [target - 0xa0000000]
 *   3        0x33a04000  =>  0x03a04000    [target - 0x30000000]
 */

/* DSP Control */
struct dsp_cmd_download {
	uint32_t control;	/* control flags */

	/* INTERDSP_DL_PROG */
	void *data;		/* Program/Data Address */
	uint32_t memtype;	/* 0:InstSRAM  1:DataSRAM  2:ExtSRAM 3:SDRAM */
	uint32_t offset;	/* Write Offset in SRAM/SDRAM */
	uint32_t size;		/* Copy Size */

	/* INTERDSP_DL_RESET */
	uint32_t reset;		/* 0:clear 1:set 2:(first set, last clear) */

	uint32_t power;		/* unused (backward compatibility) */

	/* INTERDSP_DL_CLOCK */
	uint32_t clock;		/* 1:start, 0:stop */

	/* INTERDSP_DL_CLKCTRL */
	uint32_t clkctrl;	/* 1:on, 0:off (auto clock control) */
};

/*
 * Errot Bit Number (INTERDSP_GET_ERROR)
 */

#define INTERDSP_ERROR_ARMCH_ACK_TIMEOUT(CH)	(CH)

/* wrapper may use bit number upper than this freely */
#define INTERDSP_ERROR_WRAPPER_NR		8

/*
 * DCV control
 */

/* for DCV register */
struct dcv_reg {
	uint16_t bank_no;	/* BANK number 0-15 */
	union {
		struct {
			unsigned bank_offset:12; /* DCV_BANKn_OFFSET */
			unsigned bank_set:4; /* DCV_BANKn_SET */
		} bank_reg;
		uint16_t bank_addr; /* set for (high)16bit */
	} bank_val;
};

/* DCV set/get structure */
struct dsp_cmd_dcv_set {
	unsigned int count;	/* set/get number */
	struct dcv_reg *dcv_regs; /* pointer to `struct dcv_reg' */
};

/* ge/sett DSP communication area */
struct dsp_communication_area {
	uint32_t addr;   /* start address of DSP communication area */
	uint32_t size;   /* size of DSP communication area */
};

/* download possible area acquisition */
struct dsp_download_area {
	unsigned long addr;   /* start address of DSP DOWNLOAD area */
	unsigned long size;   /* size of DSP DOWNLOAD area */
};



/*
 *	ioctl command definition
 */

#define INTERDSP_READ			_IOR('D', 0x01, struct dsp_cmd_rw)
#define INTERDSP_WRITE			_IOW('D', 0x02, struct dsp_cmd_rw)
#define INTERDSP_READ_CANCEL	_IOW('D', 0x03, int)

#define INTERDSP_WRITEBUF		_IOWR('D', 0x08, struct dsp_cmd_rw)
#define INTERDSP_SENDHDR		_IOWR('D', 0x09, struct dsp_cmd_rw)

#define INTERDSP_GET_CHINFO	_IOR('D', 0x10, struct dsp_cmd_ch)
#define INTERDSP_SET_CHINFO	_IOW('D', 0x11, struct dsp_cmd_ch)
#define INTERDSP_GET_TASKCH	_IOR('D', 0x12, struct dsp_cmd_task_ch)
#define INTERDSP_SET_TASKCH	_IOWR('D', 0x13, struct dsp_cmd_task_ch)
#define INTERDSP_SHMEM_INIT	_IO('D', 0x18)

#define INTERDSP_CHECKCH	_IOWR('D', 0x22, struct dsp_cmd_ch_status)
#define INTERDSP_WAITCH		_IOWR('D', 0x23, struct dsp_cmd_ch_status)

#define INTERDSP_DOWNLOAD	_IOWR('D', 0x30, struct dsp_cmd_download)

#define INTERDSP_GET_ERROR	_IOR('D', 0x40, unsigned long)

#define INTERDSP_SET_DCV_REGS	_IOW('D', 0x50, struct dsp_cmd_dcv_set)
#define INTERDSP_GET_DCV_REGS	_IOW('D', 0x51, struct dsp_cmd_dcv_set)

#define DSPDEV_SET_COM_AREA	_IOW('D', 0x60, struct dsp_communication_area)
#define DSPDEV_GET_COM_AREA	_IOW('D', 0x61, struct dsp_communication_area)

#define DSPDEV_GET_DOWNLOAD_AREA \
			_IOW('D', 0x70, struct dsp_download_area)

/*
 * for ODIN
 */

/* Start/Stop Peripherals */
#define DSPDEV_START_PERIPHERALS	_IOW('P', 1, unsigned)
#define DSPDEV_STOP_PERIPHERALS		_IOW('P', 2, unsigned)

#define DSPDEV_PERIP_TI3		(1U<<4)
#define DSPDEV_PERIP_DCV		(1U<<8)

#define DSPDEV_TIN_SEL			_IO('P', 3)

#define DSPDEV_TIN_TI3_BIT		(1U<<8)
#define DSPDEV_TIN_TI3_VAL		(1U<<0)
#define DSPDEV_TIN_TI3_0		DSPDEV_TIN_TI3_BIT
#define DSPDEV_TIN_TI3_1		(DSPDEV_TIN_TI3_BIT|DSPDEV_TIN_TI3_VAL)

/* SRAM automatic clock control */
#define DSPDEV_SRAMCTRL	_IOW('P', 10, unsigned)


/*
 * /dev/dsp/datamgr
 */

struct dspdev_cmd_data {
	uint32_t offset;	/* read/write offset */
	uint32_t size;		/* read/write size */
	void *buf;		/* read/write data */
};

/* Read managed data */
#define DSPDEV_DATA_READ		_IOR('d', 1, struct dspdev_cmd_data)

/* Write managed data */
#define DSPDEV_DATA_WRITE		_IOW('d', 2, struct dspdev_cmd_data)

/* Increment managed data */
#define DSPDEV_DATA_INCREMENT	_IOR('d', 3, struct dspdev_cmd_data)

/* Decrement managed data */
#define DSPDEV_DATA_DECREMENT	_IOW('d', 4, struct dspdev_cmd_data)

/* Lock managed data */
#define DSPDEV_DATA_LOCK	_IOR('d', 5, int)

/* Unlock managed data */
#define DSPDEV_DATA_UNLOCK	_IOW('d', 6, int)

#endif /* INTER_DSP_IOCTL_H */
