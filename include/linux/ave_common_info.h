/*
 * inlcude/linux/ave_common_info.h
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


#ifndef __ave_common_info_h__
#define __ave_common_info_h__

#include <linux/ioctl.h>
#define AVE_IOCTL_MAGIC 'H'
#define AVE_IOCQBUFSIZE      _IO(AVE_IOCTL_MAGIC, 0)
#define AVE_IOCRUNCMD        _IOW(AVE_IOCTL_MAGIC, 1, int)
#define AVE_IOCGBUSYFLAG     _IOR(AVE_IOCTL_MAGIC, 2, int)
#define AVE_IOCSREGISTER     _IOW(AVE_IOCTL_MAGIC, 3, int)
#define AVE_IOCGREGISTER     _IOWR(AVE_IOCTL_MAGIC, 4, int)
#define AVE_IOCSDEBUGLEVEL   _IOW(AVE_IOCTL_MAGIC, 99, int)


#define	AVE_BIGENDIAN		0x0001
#define	AVE_LITTLEENDIAN	0x0000
#define	AVE_32BITSFORMAT	0x0001
#define	AVE_64BITSFORMAT	0x0000
#define	AVE_CHK_DIS_ON		0x0000
#define	AVE_CHK_DIS_OFF		0x0001
#define	AVE_CBCR_SEP		0x0000
#define	AVE_CBCR_INTRLV		0x0001
#define	AVE_CODE_H264		0x0000
#define	AVE_CODE_VC_1		0x0001
#define	AVE_CODE_MP2		0x0002
#define	AVE_CODE_MP4		0x0003
#define	AVE_CODE_DIV3		0x0007
#define	AVE_MPQP_ENA		0x0001
#define	AVE_MPQP_DIS		0x0000
#define	AVE_REODER_ENA		0x0001
#define	AVE_REODER_DIS		0x0000
#define	AVE_FILEPLAY_ENA	0x0001
#define	AVE_FILEPLAY_DIS	0x0000
#define	AVE_DYNALLOC_ENA	0x0001
#define	AVE_DYNALLOC_DIS	0x0000
#define	AVE_MP4C_MP4		0x0000
#define	AVE_MP4C_DIVX5		0x0001
#define	AVE_MP4C_XVID		0x0002
#define	AVE_MP4C_DIVX4		0x0005
#define	AVE_VC1_AUTO		0x0000
#define	AVE_VC1_MANUAL		0x0001
#define	AVE_VC1_RCV_V1		0x0000
#define	AVE_VC1_RCV_V2		0x0001
#define	AVE_VC1_ELEMENTARY	0x0002
#define	AVE_PRESCAN_ENA		0x0001
#define	AVE_PRESCAN_DIS		0x0000
#define	AVE_PRESCAN_DEC		0x0000
#define	AVE_PRESCAN_NODEC	0x0001
#define	AVE_IFRAMES_ENA		0x0001
#define	AVE_IFRAMES_DIS		0x0000
#define	AVE_SKIPF_NON		0x0000
#define	AVE_SKIPF_EXCEPT_I	0x0001
#define	AVE_SKIPF_B		0x0002
#define	AVE_SKIPF_ALL		0x0003
#define	AVE_PARA_TYPE_SEQ	0x0000
#define	AVE_PARA_TYPE_PIC	0x0001
#define	AVE_INITESCAPE_ENA	0x0001
#define	AVE_INITESCAPE_DIS	0x0000
#define	AVE_FLUSHTYPE_SEQ	0x0000
#define	AVE_FLUSHTYPE_RDPTR	0x0001

/*
 * Command for AVC_IOCSREGCMD
 */

enum ave_command_type_t {
	seq_init,
	seq_end,
	pic_run,
	set_frame_buf,
	dec_para_set,
	dec_buf_flush,
	add_stream,
};

struct  ave_interrupt_t{
	union {
		unsigned int l;
		struct {
			unsigned int reserved1:1;
			unsigned int int_seq_init:1;
			unsigned int int_seq_end:1;
			unsigned int int_pic_run:1;
			unsigned int int_set_frame_buf:1;
			unsigned int reserved2:2;
			unsigned int int_dec_para_set:1;
			unsigned int int_dec_buf_flush:1;
			unsigned int reserved3:5;
			unsigned int int_lock_buff:1;
		} b;
	} interrupt;

	unsigned int busy_flag;
	unsigned int ret_status;
	unsigned int lackofslicebuf;
	unsigned int lackofpsbuf;
};

struct ave_base_info_t {

	unsigned int wait_count;
	unsigned int code_ver;

};

struct ave_host_param_t {
	unsigned int work_buff_addr;
	unsigned int para_buf_addr;
	unsigned short bit_sel_bigendian;
	unsigned short bit_sel64bits_endian;
	unsigned short bit_bufsts_checkdis;
	unsigned short frame_sel_bigendian;
	unsigned short frame_sel64bits_endian;
	unsigned short frame_selcinterleave;
	unsigned short stream_end;
	unsigned short seq_init_escape;
	unsigned int stream_rd_ptr;
	unsigned int stream_wr_ptr;
	unsigned int bitframe_disflag;
	unsigned short run_code_std;
};

struct ave_seq_init_set_param_t {
	unsigned int bitbuf_addr;
	unsigned short bitbuf_size;
	unsigned short mpqpreport_en;
	unsigned short reordr_en;
	unsigned short fileplay_en;
	unsigned short dynalloc_en;
	unsigned short cmd_picheight;
	unsigned short cmd_picwidth;
	unsigned short streamstartoffset;
	unsigned int psbufaddr;
	unsigned short psbufsize;
	unsigned short mp4class;
	unsigned short vc1_streamdetect;
	unsigned short vc1_streamfmt;
};

struct ave_seq_init_get_param_t {
	unsigned int aspect_ratio;
	unsigned short ret_picheight;
	unsigned short ret_picwidth;
	unsigned int framerateinfo;
	unsigned short framebufneed;
	unsigned short framebufdelay;
	unsigned short datapart_en;
	unsigned short revvlc_en;
	unsigned short shortvideohead_en;
	unsigned short h263_annexj_en;
	unsigned short croprightoffset;
	unsigned short cropleftoffset;
	unsigned short cropbottomoffset;
	unsigned short croptopoffset;
	unsigned short profile;
	unsigned short level;
	unsigned short interlace;
	unsigned short direct_8x8flag;
	unsigned short vc1_psf;
	unsigned short constraint_setflag[4];
	unsigned short mpeg4_height;
	unsigned short mpeg4_width;
	unsigned int numunits_in_tick;
	unsigned int timescale;
};

struct ave_pic_run_set_param_t {
	unsigned short prescan_en;
	unsigned short prescan_mode;
	unsigned short iframesearch_en;
	unsigned short skipframe_mode;
	unsigned short skipframenum;
	unsigned int framechunksize;
	unsigned int picbitbufstart;
	unsigned short picstartbyteoffset;
};

struct ave_pic_run_get_param_t {
	unsigned short decpicwidth;
	unsigned short decpicheight;
	unsigned short decframenum;
	short decpicldx;
	unsigned short errmbnum;
	unsigned short pictype;
	unsigned short interlacedframe;
	unsigned short picstructure;
	unsigned short topfieldfirst;
	unsigned short repeatfirstfield;
	unsigned short progressiveframe;
	unsigned short fieldsequence;
	unsigned short rangered;
	unsigned short hscaleflag;
	unsigned short vscaleflag;
	unsigned short prescanres;
	unsigned short mp4packedpbframe;
	short decordedpicldx;
	unsigned int consumedbit;
	unsigned short npf_flag;
#ifdef CONFIG_EMXX_AVE_VC1
	unsigned short postsrcbufidx;
#endif

};

struct ave_frame_addr_t {
	unsigned int yaddr[32];
	unsigned int cbaddr[32];
	unsigned int craddr[32];
	unsigned int mvcoladdr[32];
};

struct ave_frame_buf_set_param_t {
	unsigned int slice_bufaddr;
	unsigned short slice_bufsize;
	unsigned short framebufnum;
	unsigned short linestride;
	struct ave_frame_addr_t frame_addr;
};

struct ave_dec_para_set_param_t {
	unsigned short parasettype;
	unsigned short parasetsize;
};

struct ave_flash_set_param_t {
	unsigned int flash_rdptr;
	unsigned short flash_type;
};


struct ave_common_info_t {

	struct ave_base_info_t ave_base_info;

	struct ave_host_param_t ave_host_param;

	struct ave_seq_init_set_param_t ave_seq_init_set_param;
	struct ave_seq_init_get_param_t ave_seq_init_get_param;

	struct ave_pic_run_set_param_t ave_pic_run_set_param;
	struct ave_pic_run_get_param_t ave_pic_run_get_param;

	struct ave_frame_buf_set_param_t ave_frame_buf_set_param;

	struct ave_dec_para_set_param_t ave_dec_para_set_param;

	struct ave_flash_set_param_t ave_flash_set_param;

};

struct ave_register_access_t {
	unsigned int address;
	unsigned int value;
};

#endif /* __ave_common_info_h__ */
