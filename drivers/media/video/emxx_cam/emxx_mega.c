/*
 *  File Name       : emxx_mega.c
 *  Function        : MEGA for CAMERA I/F Driver
 *  Release Version : Ver 1.02
 *  Release Date    : 2011/01/25
 *
 *  Copyright (C) Renesas Electronics Corporation 2010
 *
 *  This program is free software;you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by Free
 *  Softwere Foundation; either version 2 of License, or (at your option) any
 *  later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY;
 *  without even the implied warrnty of MERCHANTABILITY or FITNESS FOR A
 *  PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program;
 *  If not, write to the Free Software Foundation, Inc., 59 Temple Place -
 *  Suite 330, Boston, MA 02111-1307, USA.
 *
 */
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <mach/spi.h>
#include <mach/gpio.h>
#include <mach/pmu.h>
#include <mach/smu.h>
#include <linux/kernel.h>

/* #define EMXX_CAM_MAKING_DEBUG */

#include "emxx_cam.h"


#define DEV_NAME "emxx_mega"

#define CAM_DEBUG 0

#define X1R_NOT_ZERO 0

#define EMXX_CAM_CE14X_MAKING_DEBUG

/*** DEBUG code by the making ->*/
#ifdef EMXX_CAM_CE14X_MAKING_DEBUG

int mega_debug = 4;

#include <linux/moduleparam.h>

#define FNC_ENTRY	\
	if (mega_debug == 1 || mega_debug >= 9) {	\
		printk(KERN_INFO "entry:%s\n", __func__); \
	}

#define FNC_EXIT_N	\
	if (mega_debug == 1 || mega_debug >= 9) {	\
		printk(KERN_INFO "exit: %s:%d\n", __func__, __LINE__); \
	}

#define FNC_EXIT(r)	\
	if (mega_debug == 1 || mega_debug >= 9) {	\
		printk(KERN_INFO "exit:%d :%s:%d\n", r, __func__, __LINE__); \
	}

#define d0b(fmt, args...)	\
	{ \
		printk(KERN_INFO "%s:%d: " fmt, __func__, __LINE__, ## args); \
	}
#define d1b(fmt, args...)	\
	if (mega_debug == 1 || mega_debug >= 9) {	\
		printk(KERN_INFO "%s:%d: " fmt, __func__, __LINE__, ## args); \
	}
#define d2b(fmt, args...)	\
	if (mega_debug == 2 || mega_debug >= 9) {	\
		printk(KERN_INFO "%s:%d: " fmt, __func__, __LINE__, ## args); \
	}
#define d3b(fmt, args...)	\
	if (mega_debug == 3 || mega_debug >= 9) {	\
		printk(KERN_INFO " --M%d-- " fmt, mega_debug, ## args); \
	}
#define d4b(fmt, args...)	\
	if (mega_debug == 4 || mega_debug >= 9) {	\
		printk(KERN_INFO " --M%d-- " fmt, mega_debug, ## args); \
	}
#define d5b(fmt, args...)	\
	if (mega_debug == 5 || mega_debug >= 9) {	\
		printk(KERN_INFO " --M%d-- " fmt, mega_debug, ## args); \
	}
#define d6b(fmt, args...)	\
	if (mega_debug == 6 || mega_debug >= 9) {	\
		printk(KERN_INFO " --M%d-- " fmt, mega_debug, ## args); \
	}

#else
#define FNC_ENTRY do { } while (0);
#define FNC_EXIT_N  do { } while (0);
#define FNC_EXIT(r) do { } while (0);
#define d0b(fmt, args...) do { } while (0);
#define d1b(fmt, args...) do { } while (0);
#define d2b(fmt, args...) do { } while (0);
#define d3b(fmt, args...) do { } while (0);
#define d4b(fmt, args...) do { } while (0);
#define d5b(fmt, args...) do { } while (0);
#define d6b(fmt, args...) do { } while (0);
#endif

/*===============================================================*/
/* I2C Functions                                                 */
/*===============================================================*/
#define RJ6ABA100_I2C (0x50) /* 1010000(bit) */

/* flags */
static int i2c_stop;

/* prottyped */
static int rj6aba100_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id);
static int rj6aba100_i2c_remove(struct i2c_client *client);
static inline int rj6aba100_i2c_write(unsigned char cmd,
				      unsigned char *buf, unsigned char len);
static inline int rj6aba100_i2c_read(unsigned char cmd,
				     unsigned char *buf, unsigned char len);

static struct i2c_device_id cam_i2c_idtable[] = {
	{ I2C_SLAVE_CAM_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cam_i2c_idtable);

static struct i2c_driver rj6aba100_i2c_driver = {
	.driver.name    = "i2c for RJ6ABA100",
	.id             = I2C_DRIVERID_I2CDEV, /* Fake ID */
	.id_table       = cam_i2c_idtable,
	.probe          = rj6aba100_i2c_probe,
	.remove         = rj6aba100_i2c_remove,
};
static struct i2c_client *rj6aba100_i2c_client;


/* i2c registerd function */
static int rj6aba100_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	rj6aba100_i2c_client = client;
	return 0;
}

static int rj6aba100_i2c_remove(struct i2c_client *client)
{
	rj6aba100_i2c_client = NULL;
	return 0;
}

/* usable i2c funcrion */
static inline int rj6aba100_i2c_write(unsigned char cmd,
				      unsigned char *buf, unsigned char len)
{
	int ret = 0;
	char s_buf[15];
	struct i2c_msg msg = { .addr = RJ6ABA100_I2C, .flags = 0,
			       .buf = s_buf, .len = len + 1 };

	s_buf[0] = cmd;
	memcpy(&s_buf[1], buf, len);

	if (i2c_stop)
		return -EIO;

	assert(rj6aba100_i2c_client);

	ret = i2c_transfer(rj6aba100_i2c_client->adapter, &msg, 1);
	if (1 != ret) {
		err("i2c_transfer: cmd : 0x%02x ret = %d\n", cmd, ret);
		i2c_stop = 1;
		ret = -EIO;
	} else {
		ret = 0;
	}
	return ret;
}

static inline int rj6aba100_i2c_read(unsigned char cmd,
				     unsigned char *buf, unsigned char len)
{
	int ret = 0;
	struct i2c_msg msg[] =  {{ .addr = RJ6ABA100_I2C, .flags = 0,
				   .buf = &cmd, .len = 1 },
				 { .addr = RJ6ABA100_I2C, .flags = I2C_M_RD,
				   .buf = buf,  .len = len } };
	if (i2c_stop)
		return -EIO;

	assert(rj6aba100_i2c_client);

	ret = i2c_transfer(rj6aba100_i2c_client->adapter, msg, 2);
	if (2 != ret) {
		err("i2c_transfer: cmd : 0x%02x ret = %d\n", cmd, ret);
		i2c_stop = 1;
		ret = -EIO;
	} else {
		ret = 0;
	}
	return ret;
}



/*===============================================================*/
/* RJ6ABA100 Camera control flags                                */
/*===============================================================*/
enum {
	EMXX_RJ6ABA100_IDLE = 0,
	EMXX_RJ6ABA100_LIVE,
	EMXX_RJ6ABA100_CAPTURE,
	EMXX_RJ6ABA100_BREAK,
};

struct emxx_rj6aba100 {
	int state;
	struct mutex lock;

	__u32 active:1;
	__u32 reset:1;

	__u8 firmware_version;
	__u8 capture_size;
	__u8 outimg_fmt;
	__u8 imgdata_fmt;
	__u8 flicker_manual;
	__u8 wb_mode;
	__u32 wb_manual_gain;
	__u8 brightness;
	__u8 contrast;
	__u32 sharpness;
	__u8 mirror;
	__u8 efct_color;
	__u8 efct_emboss;
	__u8 efct_negative;
	__u8 efct_sketch;
};

struct emxx_rj6aba100 *rj6aba100;


/*===============================================================*/
/* RJ6ABA100 data structure. for VIDIOC_S_CTRL                   */
/*===============================================================*/
struct control_menu_info {
	int value;
	char name[32];
};


struct control_resize_info {
	int value;
	char name[32];
	__s32 x;
	__s32 y;
	__s32 width;
	__s32 height;
};


/* EMXX_RJ6ABA100_CID_OUTIMG_SIZE */
static const struct control_resize_info rj6aba100_outimg_size_menus[] =
{
	{ 0x00, "QQVGA : 160x120",  3, 2, 162, 121 },
	{ 0x01, "QCIF  : 176x144", 11, 2, 144, 176 },
	{ 0x02, "QVGA  : 320x240",  4, 4, 323, 243 },
	{ 0x03, "CIF   : 352x288", 22, 5, 373, 292 },
#if X1R_NOT_ZERO
	{ 0x04, "VGA   : 640x480",  4, 4, 644, 484 },    /* default in EM1*/
#else
	{ 0x04, "VGA   : 640x480",  7, 7, 646, 486 },    /* default */
#endif
};
#define NUM_OUTIMG_SIZE_MENUS ARRAY_SIZE(rj6aba100_outimg_size_menus)
#define BOUNDARY_OUT_IMG_RESIZE 0x04 /* 640x480 */


/* EMXX_RJ6ABA100_CID_OUTIMG_FORMAT */
#define OUT_FORMAT_YCBCR422 0x00
#define OUT_FORMAT_YUV422   0x01
#define OUT_FORMAT_RGB_RAW  0x02
#define OUT_FORMAT_RGB565   0x03
#define OUT_FORMAT_RGB444   0x04
#define OUT_FORMAT_MONO     0x05
static const struct control_menu_info rj6aba100_outimg_fmt_menus[] =
{
	{ 0x00, "YCbCr422",       },
	{ 0x01, "YUV422",         },     /* default */
	{ 0x02, "RGB Bayer(RAW)", },
	{ 0x03, "RGB565",         },
	{ 0x04, "RGB444",         },
	{ 0x05, "Mono",           },
};
#define NUM_OUTIMG_FORMAT_MENUS ARRAY_SIZE(rj6aba100_outimg_fmt_menus)


/* EMXX_RJ6ABA100_CID_OUTDATA_FORMAT */
static const struct control_menu_info
rj6aba100_outdata_fmt_menus[NUM_OUTIMG_FORMAT_MENUS][4] =
{
	{       /* YCbCr422 */
		{ 0x00, "CbYCrY...",   },
		{ 0x01, "CrYCbY...",   },
		{ 0x02, "YCbYCr...",   },
		{ 0x03, "YCrYCb...",   },
	}, {     /* YUV422 */
		{ 0x00, "UYVY...",     },        /* default */
		{ 0x01, "VYUY...",     },
		{ 0x02, "YUYV...",     },
		{ 0x03, "YVYU...",     },
	}, {     /* RGB Bayer */
		{ 0x04, "RGRG...GBGB", },
		{ 0x05, "GBGB...RGRG", },
		{ 0x06, "GRGR...BGBG", },
		{ 0x07, "BGBG...GRGR", },
	}, {     /* RGB565 */
		{ 0x08, "R5G6B5...",   },
		{ 0x09, "B5G6R5...",   },
		{ 0xFF, "Reserved",   },
		{ 0xFF, "Reserved",   },
	}, {     /* RGB444 */
		{ 0x0E, "R4G4B4...",   },
		{ 0xFF, "Reserved",   },
		{ 0xFF, "Reserved",   },
		{ 0xFF, "Reserved",   },
	}, {     /* Mono */
		{ 0x0D, "YYYY...",     },
		{ 0xFF, "Reserved",   },
		{ 0xFF, "Reserved",   },
		{ 0xFF, "Reserved",   },
	}
};
#define NUM_OUTDATA_FORMAT_MENUS ARRAY_SIZE(rj6aba100_outdata_fmt_menus)


/* EMXX_RJ6ABA100_CID_FLICKER */
static const struct control_menu_info rj6aba100_flicker_menus[] =
{
	{ 0x00, "Disable Friker detection"   },
	{ 0x32, "Auto Friker detection mode" },
	{ 0x04, "Manual 60Hz Friker mode"    }, /* default */
	{ 0x08, "Manual 50Hz Friker mode"    },
};
#define NUM_FLICKER_MENUS ARRAY_SIZE(rj6aba100_flicker_menus)


/* EMXX_RJ6ABA100_CID_WB */
static const struct control_menu_info rj6aba100_wb_menus[] =
{
	{ 0x00, "Auto WB"   }, /* default */
	{ 0x01, "Manual WB" },
};
#define NUM_WB_MENUS ARRAY_SIZE(rj6aba100_wb_menus)


/* EMXX_RJ6ABA100_CID_MIRROR */
static const struct control_menu_info rj6aba100_mirror_menus[] =
{
	{ 0x00, "Mirror Off"                       }, /* default */
	{ 0x01, "Mirror Horizontally"              },
	{ 0x02, "Mirror Vertically"                },
	{ 0x03, "Mirror Horizontally & Vertically" },
};
#define NUM_MIRROR_MENUS ARRAY_SIZE(rj6aba100_mirror_menus)


/* EMXX_RJ6ABA100_CID_EFFECT_COLOR */
static const struct control_menu_info rj6aba100_effect_color_menus[] =
{
	{ 0x00, "Color Effect Off"     }, /* default */
	{ 0x01, "Color Effect Sepia"   },
	{ 0x02, "Color Effect Green"   },
	{ 0x03, "Color Effect Aqua"    },
	{ 0x04, "Color Effect Red"     },
	{ 0x05, "Color Effect Cool"    },
	{ 0x06, "Color Effect Warm"    },
	{ 0x07, "Color Effect BW"      },
	{ 0x08, "Color Effect Antique" },
	{ 0x09, "Color Effect UserSet" },
};
#define NUM_EFFECT_COLOR_MENUS   ARRAY_SIZE(rj6aba100_effect_color_menus)
#define NUM_EFFECT_COLOR_MANUAL (NUM_EFFECT_COLOR_MENUS - 1)


/* EMXX_RJ6ABA100_CID_EFFECT_NEGATIVE */
static const struct control_menu_info rj6aba100_effect_negative_menus[] =
{
	{ 0x00, "Negative Effect Off" }, /* default */
	{ 0x01, "Negative Effect On"  },
};
#define NUM_EFFECT_NEGATIVE_MENUS ARRAY_SIZE(rj6aba100_effect_negative_menus)


/* EMXX_RJ6ABA100_CID_EFFECT_EMBOSS */
static const struct control_menu_info rj6aba100_effect_emboss_menus[] =
{
	{ 0x00, "Emboss Effect Off"   }, /* default */
	{ 0x01, "Emboss Effect mode0" },
	{ 0x02, "Emboss Effect mode1" },
};
#define NUM_EFFECT_EMBOSS_MENUS ARRAY_SIZE(rj6aba100_effect_emboss_menus)


/* EMXX_RJ6ABA100_CID_EFFECT_SKETCH */
static const struct control_menu_info rj6aba100_effect_sketch_menus[] =
{
	{ 0x00, "Sketch Effect Off"     }, /* default */
	{ 0x01, "Sketch Effect image1"  },
	{ 0x02, "Sketch Effect image2"  },
	{ 0x03, "Sketch Effect image3"  },
	{ 0x04, "Sketch Effect image4"  },
	{ 0x05, "Sketch Effect UserSet" },
};
#define NUM_EFFECT_SKETCH_MENUS   ARRAY_SIZE(rj6aba100_effect_sketch_menus)
#define NUM_EFFECT_SKETCH_MANUAL (NUM_EFFECT_SKETCH_MENUS - 1)


/*===============================================================*/
/* RJ6ABA100 data structure. for VIDIOC_QUERYCTRL                */
/*===============================================================*/
static const struct v4l2_queryctrl no_ctrl = {
	.name  = "rj6aba100",
	.flags = V4L2_CTRL_FLAG_DISABLED,
};

/* default settings */
static const struct v4l2_queryctrl rj6aba100_ctrls[] = {
	{
		.id            = EMXX_RJ6ABA100_CID_OUTIMG_SIZE,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "Output image size",
#if 0
/* @@		.minimum       = 0, */
/* @@		.maximum       = (NUM_OUTIMG_SIZE_MENUS - 1), */
#else /* VGA only */
		.minimum       = 4,
		.maximum       = 4,
#endif
		.step          = 1,
		.default_value = 4, /* VGA */
		.flags         = 0,
	}, {
		.id            = EMXX_RJ6ABA100_CID_OUTIMG_FORMAT,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "Output image Format",
#if 0
/* @@		.minimum       = 0, */
/* @@		.maximum       = (NUM_OUTIMG_FORMAT_MENUS - 1), */
#else /* YUV422 only */
		.minimum       = 1,
		.maximum       = 1,
#endif
		.step          = 1,
		.default_value = 1, /* YUV422 */
		.flags         = 0,
	}, {
		.id            = EMXX_RJ6ABA100_CID_OUTDATA_FORMAT,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "Output data Format",
		.minimum       = 0,
#if 0
/* @@		.maximum       = (NUM_OUTDATA_FORMAT_MENUS
   / NUM_OUTIMG_FORMAT_MENUS - 1), */
#else /* UYVY... only */
		.maximum       = 0,
#endif
		.step          = 1,
		.default_value = 0, /* UYVY... */
		.flags         = 0,
	}, {
		.id            = EMXX_RJ6ABA100_CID_FW,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "Revision Number of RJ6ABA100",
		.minimum       = 0,
		.maximum       = 0xFF,
		.step          = 1,
		.default_value = 0,
		.flags         = V4L2_CTRL_FLAG_READ_ONLY,
	}, {
		.id            = EMXX_RJ6ABA100_CID_FLICKER,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "Flicker Mode",
		.minimum       = 0,
		.maximum       = (NUM_FLICKER_MENUS - 1),
		.step          = 1,
		.default_value = 2, /* 60Hz freker free */
		.flags         = 0,
	}, {
		.id            = EMXX_RJ6ABA100_CID_WB,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "AWB on/off",
		.minimum       = 0,
		.maximum       = (NUM_WB_MENUS - 1),
		.step          = 1,
		.default_value = 0, /* AWB on */
		.flags         = 0,
	}, {
		.id            = EMXX_RJ6ABA100_CID_WB_MANUAL_GAIN,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "Manual WB(Rgain, Ggain, Bgain)",
		.minimum       = 0,
		.maximum       = 0xFFFFFF,
		.step          = 1,
		.default_value = 0x404040,
		.flags         = 0,
	}, {
		.id            = EMXX_RJ6ABA100_CID_BRIGHTNESS,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "Brightness",
		.minimum       = -128,
		.maximum       = 127,
		.step          = 1,
		.default_value = 0x01,
		.flags         = 0,
	}, {
		.id            = EMXX_RJ6ABA100_CID_CONTRAST,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "Y Contrast",
		.minimum       = 0,
		.maximum       = 255,
		.step          = 1,
		.default_value = 0x40,
		.flags         = 0,
	}, {
		.id            = EMXX_RJ6ABA100_CID_SHARPNESS,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "Edge Gain/Threshold",
		.minimum       = 0,
		.maximum       = 0x3FFF,
		.step          = 1,
		.default_value = 0x2410,
		.flags         = 0,
	}, {
		.id            = EMXX_RJ6ABA100_CID_MIRROR,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "Mirror Horizontally & Vertically",
		.minimum       = 0,
		.maximum       = (NUM_MIRROR_MENUS - 1),
		.step          = 1,
		.default_value = 0, /* Mirror Off */
		.flags         = 0,
	}, {
		.id            = EMXX_RJ6ABA100_CID_EFFECT_COLOR,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "Color Effect",
		.minimum       = 0,
		.maximum       = (NUM_EFFECT_COLOR_MENUS - 1),
		.step          = 1,
		.default_value = 0, /* Color Effect Off */
		.flags         = 0,
	}, {
		.id            = EMXX_RJ6ABA100_CID_EFFECT_COLOR_MANUAL,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "Color Effect (user set)",
		.minimum       = 0,
		.maximum       = 0xFFFF,
		.step          = 1,
		.default_value = 0x30B0,
		.flags         = 0,
	}, {
		.id            = EMXX_RJ6ABA100_CID_EFFECT_NEGATIVE,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "Negative Effect",
		.minimum       = 0,
		.maximum       = (NUM_EFFECT_NEGATIVE_MENUS - 1),
		.step          = 1,
		.default_value = 0, /* Negative Effect Off */
		.flags         = 0,
	}, {
		.id            = EMXX_RJ6ABA100_CID_EFFECT_EMBOSS,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "Emboss Effect",
		.minimum       = 0,
		.maximum       = (NUM_EFFECT_EMBOSS_MENUS - 1),
		.step          = 1,
		.default_value = 0, /* Emboss Effect Off */
		.flags         = 0,
	}, {
		.id            = EMXX_RJ6ABA100_CID_EFFECT_SKETCH,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "Sketch Effect",
		.minimum       = 0,
		.maximum       = (NUM_EFFECT_SKETCH_MENUS - 1),
		.step          = 1,
		.default_value = 0, /* Sketch Effect Off */
		.flags         = 0,
	}, {
		.id            = EMXX_RJ6ABA100_CID_EFFECT_SKETCH_MANUAL,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "Sketch Effect (user set)",
		.minimum       = 0,
		.maximum       = 0xFFFFFFFF,
		.step          = 1,
		.default_value = 0x20C81080,
		.flags         = 0,
	}
};
#define NUM_RJ6ABA100_CTRLS ARRAY_SIZE(rj6aba100_ctrls)


/*===============================================================*/
/* RJ6ABA100 register data structure                             */
/*===============================================================*/
struct register_info {
	unsigned char bank;
	unsigned char address;
	unsigned char data;
	unsigned char mask;
};


/* RJ6ABA100 register groupe : output image size */
static const struct register_info
rj6aba100_register_outimg_size[NUM_OUTIMG_SIZE_MENUS][23] =
{
	{       /* QQVGA */
		{0x01, 0x3D, 0x80, 0xFF},       {0x01, 0x3E, 0x80, 0xFF},
		{0x01, 0x2D, 0x00, 0xFF},       {0x01, 0x2E, 0x15, 0xFF},
		{0x01, 0x2F, 0x01, 0xFF},       {0x01, 0x30, 0xF5, 0xFF},
		{0x02, 0x58, 0x00, 0xFF},       {0x02, 0x59, 0x04, 0xFF},
		{0x02, 0x5A, 0x00, 0xFF},       {0x02, 0x5B, 0x00, 0xFF},
		{0x02, 0x5C, 0x00, 0xFF},       {0x02, 0x5D, 0xA0, 0xFF},
		{0x02, 0x5E, 0x00, 0xFF},       {0x02, 0x5F, 0x78, 0xFF},
		{0x02, 0x60, 0x00, 0xFF},       {0x02, 0x61, 0x38, 0xFF},
		{0x02, 0x62, 0x00, 0xFF},       {0x02, 0x63, 0x2A, 0xFF},
		{0x02, 0x64, 0x00, 0xFF},       {0x02, 0x65, 0x30, 0xFF},
		{0x02, 0x66, 0x00, 0xFF},       {0x02, 0x67, 0x24, 0xFF},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* QCIF */
		{0x01, 0x3D, 0x6A, 0xFF},       {0x01, 0x3E, 0x6A, 0xFF},
		{0x01, 0x2D, 0x00, 0xFF},       {0x01, 0x2E, 0x15, 0xFF},
		{0x01, 0x2F, 0x01, 0xFF},       {0x01, 0x30, 0xF2, 0xFF},
		{0x02, 0x58, 0x00, 0xFF},       {0x02, 0x59, 0x05, 0xFF},
		{0x02, 0x5A, 0x00, 0xFF},       {0x02, 0x5B, 0x00, 0xFF},
		{0x02, 0x5C, 0x00, 0xFF},       {0x02, 0x5D, 0xC1, 0xFF},
		{0x02, 0x5E, 0x00, 0xFF},       {0x02, 0x5F, 0x90, 0xFF},
		{0x02, 0x60, 0x00, 0xFF},       {0x02, 0x61, 0x44, 0xFF},
		{0x02, 0x62, 0x00, 0xFF},       {0x02, 0x63, 0x33, 0xFF},
		{0x02, 0x64, 0x00, 0xFF},       {0x02, 0x65, 0x3A, 0xFF},
		{0x02, 0x66, 0x00, 0xFF},       {0x02, 0x67, 0x2B, 0xFF},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* QVGA */
		{0x01, 0x3D, 0x40, 0xFF},       {0x01, 0x3E, 0x40, 0xFF},
		{0x01, 0x2D, 0x00, 0xFF},       {0x01, 0x2E, 0x15, 0xFF},
		{0x01, 0x2F, 0x01, 0xFF},       {0x01, 0x30, 0xF5, 0xFF},
		{0x02, 0x58, 0x00, 0xFF},       {0x02, 0x59, 0x07, 0xFF},
		{0x02, 0x5A, 0x00, 0xFF},       {0x02, 0x5B, 0x00, 0xFF},
		{0x02, 0x5C, 0x01, 0xFF},       {0x02, 0x5D, 0x40, 0xFF},
		{0x02, 0x5E, 0x00, 0xFF},       {0x02, 0x5F, 0xF0, 0xFF},
		{0x02, 0x60, 0x00, 0xFF},       {0x02, 0x61, 0x70, 0xFF},
		{0x02, 0x62, 0x00, 0xFF},       {0x02, 0x63, 0x54, 0xFF},
		{0x02, 0x64, 0x00, 0xFF},       {0x02, 0x65, 0x60, 0xFF},
		{0x02, 0x66, 0x00, 0xFF},       {0x02, 0x67, 0x48, 0xFF},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* CIF */
		{0x01, 0x3D, 0x35, 0xFF},       {0x01, 0x3E, 0x35, 0xFF},
		{0x01, 0x2D, 0x00, 0xFF},       {0x01, 0x2E, 0x16, 0xFF},
		{0x01, 0x2F, 0x01, 0xFF},       {0x01, 0x30, 0xF4, 0xFF},
		{0x02, 0x58, 0x00, 0xFF},       {0x02, 0x59, 0x09, 0xFF},
		{0x02, 0x5A, 0x00, 0xFF},       {0x02, 0x5B, 0x00, 0xFF},
		{0x02, 0x5C, 0x01, 0xFF},       {0x02, 0x5D, 0x82, 0xFF},
		{0x02, 0x5E, 0x01, 0xFF},       {0x02, 0x5F, 0x21, 0xFF},
		{0x02, 0x60, 0x00, 0xFF},       {0x02, 0x61, 0x88, 0xFF},
		{0x02, 0x62, 0x00, 0xFF},       {0x02, 0x63, 0x66, 0xFF},
		{0x02, 0x64, 0x00, 0xFF},       {0x02, 0x65, 0x74, 0xFF},
		{0x02, 0x66, 0x00, 0xFF},       {0x02, 0x67, 0x56, 0xFF},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* VGA */
		{0x01, 0x3D, 0x20, 0xFF},       {0x01, 0x3E, 0x20, 0xFF},
		{0x01, 0x2D, 0x00, 0xFF},       {0x01, 0x2E, 0x15, 0xFF},
		{0x01, 0x2F, 0x01, 0xFF},       {0x01, 0x30, 0xF5, 0xFF},
		{0x02, 0x58, 0x00, 0xFF},       {0x02, 0x59, 0x0E, 0xFF},
		{0x02, 0x5A, 0x00, 0xFF},       {0x02, 0x5B, 0x00, 0xFF},
		{0x02, 0x5C, 0x02, 0xFF},       {0x02, 0x5D, 0x80, 0xFF},
		{0x02, 0x5E, 0x01, 0xFF},       {0x02, 0x5F, 0xE0, 0xFF},
		{0x02, 0x60, 0x00, 0xFF},       {0x02, 0x61, 0xE0, 0xFF},
		{0x02, 0x62, 0x00, 0xFF},       {0x02, 0x63, 0xA8, 0xFF},
		{0x02, 0x64, 0x00, 0xFF},       {0x02, 0x65, 0xC0, 0xFF},
		{0x02, 0x66, 0x00, 0xFF},       {0x02, 0x67, 0x90, 0xFF},
		{0x00, 0x00, 0x00, 0x00},
	}
};
#define NUM_OUTIMG_SIZE_REGS ARRAY_SIZE(rj6aba100_register_outimg_size)


/* RJ6ABA100 register groupe : output image format */
static const struct register_info
rj6aba100_register_outimg_fmt[NUM_OUTIMG_FORMAT_MENUS][9] =
{
	{       /* YCbCr422 */
		{0x01, 0x49, 0xE0, 0xFF},       {0x01, 0x4A, 0x37, 0xFF},
		{0x01, 0x4B, 0x10, 0xFF},       {0x01, 0x4C, 0xEB, 0xFF},
		{0x01, 0x0C, 0xFF, 0xFF},       {0x01, 0x0D, 0x03, 0x03},
		{0x01, 0x0E, 0x01, 0x01},       {0x01, 0x0F, 0x08, 0x08},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* YUV422 */
		{0x01, 0x49, 0xFF, 0xFF},       {0x01, 0x4A, 0x40, 0xFF},
		{0x01, 0x4B, 0x01, 0xFF},       {0x01, 0x4C, 0xFE, 0xFF},
		{0x01, 0x0C, 0xFF, 0xFF},       {0x01, 0x0D, 0x03, 0x03},
		{0x01, 0x0E, 0x01, 0x01},       {0x01, 0x0F, 0x08, 0x08},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* RGB Bayer(RAW) */
		{0x01, 0x49, 0xFF, 0xFF},       {0x01, 0x4A, 0x40, 0xFF},
		{0x01, 0x4B, 0x01, 0xFF},       {0x01, 0x4C, 0xFE, 0xFF},
		{0x01, 0x0C, 0x43, 0xFF},       {0x01, 0x0D, 0x00, 0x03},
		{0x01, 0x0E, 0x00, 0x01},       {0x01, 0x0F, 0x00, 0x08},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* RGB565 */
		{0x01, 0x49, 0xFF, 0xFF},       {0x01, 0x4A, 0x40, 0xFF},
		{0x01, 0x4B, 0x01, 0xFF},       {0x01, 0x4C, 0xFE, 0xFF},
		{0x01, 0x0C, 0xFF, 0xFF},       {0x01, 0x0D, 0x03, 0x03},
		{0x01, 0x0E, 0x01, 0x01},       {0x01, 0x0F, 0x08, 0x08},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* RGB444 */
		{0x01, 0x49, 0xFF, 0xFF},       {0x01, 0x4A, 0x40, 0xFF},
		{0x01, 0x4B, 0x01, 0xFF},       {0x01, 0x4C, 0xFE, 0xFF},
		{0x01, 0x0C, 0xFF, 0xFF},       {0x01, 0x0D, 0x03, 0x03},
		{0x01, 0x0E, 0x01, 0x01},       {0x01, 0x0F, 0x08, 0x08},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* Mono */
		{0x01, 0x49, 0xFF, 0xFF},       {0x01, 0x4A, 0x40, 0xFF},
		{0x01, 0x4B, 0x01, 0xFF},       {0x01, 0x4C, 0xFE, 0xFF},
		{0x01, 0x0C, 0xFF, 0xFF},       {0x01, 0x0D, 0x03, 0x03},
		{0x01, 0x0E, 0x01, 0x01},       {0x01, 0x0F, 0x08, 0x08},
		{0x00, 0x00, 0x00, 0x00},
	}
};
#define NUM_OUTIMG_FORMAT_REGS ARRAY_SIZE(rj6aba100_register_outimg_fmt)


/* RJ6ABA100 register groupe : output image format & output image size */
#define NUM_PCLK_RATE_RGBRAW_MONO   0  /* RGB Bayer, Mono */
#define NUM_PCLK_RATE_YCBCR_YUV_RGB 1  /* YCbCr422, YUV422, RGB565 */
#define NUM_PCLK_RATE_MAX           2
#define SELECT_PCLK_RATE_DESCRIPTION(format) ((format == OUT_FORMAT_RGB_RAW \
					       || format == OUT_FORMAT_MONO) \
					      ? NUM_PCLK_RATE_RGBRAW_MONO \
					      : NUM_PCLK_RATE_YCBCR_YUV_RGB)

static const struct register_info
rj6aba100_register_pclk_rate[NUM_PCLK_RATE_MAX][NUM_OUTIMG_SIZE_MENUS][4] =
{
	{       /* RGB Bayer, Mono */
		{       /* QQVGA */
			{0x01, 0x19, 0x01, 0x0F},  {0x01, 0x3F, 0x00, 0xFF},
			{0x01, 0x40, 0x17, 0xFF},  {0x00, 0x00, 0x00, 0x00},
		}, {     /* QCIF */
			{0x01, 0x19, 0x03, 0x0F},  {0x01, 0x3F, 0x00, 0xFF},
			{0x01, 0x40, 0x8C, 0xFF},  {0x00, 0x00, 0x00, 0x00},
		}, {     /* QVGA */
			{0x01, 0x19, 0x03, 0x0F},  {0x01, 0x3F, 0x00, 0xFF},
			{0x01, 0x40, 0x01, 0xFF},  {0x00, 0x00, 0x00, 0x00},
		}, {     /* CIF */
			{0x01, 0x19, 0x01, 0x0F},  {0x01, 0x3F, 0x00, 0xFF},
			{0x01, 0x40, 0x17, 0xFF},  {0x00, 0x00, 0x00, 0x00},
		}, {     /* VGA */
			{0x01, 0x19, 0x01, 0x0F},  {0x01, 0x3F, 0x00, 0xFF},
			{0x01, 0x40, 0x01, 0xFF},  {0x00, 0x00, 0x00, 0x00},
		}
	}, {     /* YCbCr422, YUV422, RGB565 */
		{       /* QQVGA */
			{0x01, 0x19, 0x00, 0x0F},  {0x01, 0x3F, 0x01, 0xFF},
			{0x01, 0x40, 0x17, 0xFF},  {0x00, 0x00, 0x00, 0x00},
		}, {     /* QCIF */
			{0x01, 0x19, 0x02, 0x0F},  {0x01, 0x3F, 0x00, 0xFF},
			{0x01, 0x40, 0x8C, 0xFF},  {0x00, 0x00, 0x00, 0x00},
		}, {     /* QVGA */
			{0x01, 0x19, 0x01, 0x0F},  {0x01, 0x3F, 0x00, 0xFF},
			{0x01, 0x40, 0x01, 0xFF},  {0x00, 0x00, 0x00, 0x00},
		}, {     /* CIF */
			{0x01, 0x19, 0x00, 0x0F},  {0x01, 0x3F, 0x01, 0xFF},
			{0x01, 0x40, 0x17, 0xFF},  {0x00, 0x00, 0x00, 0x00},
		}, {     /* VGA */
			{0x01, 0x19, 0x00, 0x0F},  {0x01, 0x3F, 0x00, 0xFF},
			{0x01, 0x40, 0x01, 0xFF},  {0x00, 0x00, 0x00, 0x00},
		}
	}
};
#define NUM_PCLK_RATE_REGS ARRAY_SIZE(rj6aba100_register_pclk_rate)


/* EMXX_RJ6ABA100_CID_MIRROR */
static const struct register_info
rj6aba100_register_mirror[NUM_MIRROR_MENUS + 1][3] =
{
	{       /* Mirror Off */
		{0x01, 0x19, 0x60, 0x60},       {0x01, 0x0F, 0x00, 0x03},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* Mirror Horizontally */
		{0x01, 0x19, 0x60, 0x60},       {0x01, 0x0F, 0x01, 0x03},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* Mirror Virtically */
		{0x01, 0x19, 0x60, 0x60},       {0x01, 0x0F, 0x02, 0x03},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* Mirror Horizontally & Vertically */
		{0x01, 0x19, 0x60, 0x60},       {0x01, 0x0F, 0x03, 0x03},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* Common Postprocessing */
		{0x01, 0x19, 0x00, 0x60},       {0x00, 0x00, 0x00, 0x00},
		{0x00, 0x00, 0x00, 0x00},
	}
};
#define NUM_MIRROR ARRAY_SIZE(rj6aba100_register_mirror)


/* EMXX_RJ6ABA100_CID_EFFECT_COLOR */
static struct register_info
rj6aba100_register_effect_color[NUM_EFFECT_COLOR_MENUS][4] =
{
	{       /* Color Effect Off */
		{0x01, 0x0E, 0x01, 0x81},       {0x01, 0x42, 0x30, 0xFF},
		{0x01, 0x43, 0xB0, 0xFF},       {0x00, 0x00, 0x00, 0x00},
	}, {     /* Color Effect Sepia */
		{0x01, 0x0E, 0x81, 0x81},       {0x01, 0x42, 0xA0, 0xFF},
		{0x01, 0x43, 0x20, 0xFF},       {0x00, 0x00, 0x00, 0x00},
	}, {     /* Color Effect Green */
		{0x01, 0x0E, 0x81, 0x81},       {0x01, 0x42, 0xC0, 0xFF},
		{0x01, 0x43, 0xC0, 0xFF},       {0x00, 0x00, 0x00, 0x00},
	}, {     /* Color Effect Aqua */
		{0x01, 0x0E, 0x81, 0x81},       {0x01, 0x42, 0x20, 0xFF},
		{0x01, 0x43, 0xC0, 0xFF},       {0x00, 0x00, 0x00, 0x00},
	}, {     /* Color Effect Red */
		{0x01, 0x0E, 0x81, 0x81},       {0x01, 0x42, 0x00, 0xFF},
		{0x01, 0x43, 0x50, 0xFF},       {0x00, 0x00, 0x00, 0x00},
	}, {     /* Color Effect Cool */
		{0x01, 0x0E, 0x81, 0x81},       {0x01, 0x42, 0x50, 0xFF},
		{0x01, 0x43, 0xC0, 0xFF},       {0x00, 0x00, 0x00, 0x00},
	}, {     /* Color Effect Warm */
		{0x01, 0x0E, 0x81, 0x81},       {0x01, 0x42, 0x90, 0xFF},
		{0x01, 0x43, 0x30, 0xFF},       {0x00, 0x00, 0x00, 0x00},
	}, {     /* Color Effect BW */
		{0x01, 0x0E, 0x81, 0x81},       {0x01, 0x42, 0x00, 0xFF},
		{0x01, 0x43, 0x00, 0xFF},       {0x00, 0x00, 0x00, 0x00},
	}, {     /* Color Effect Antique */
		{0x01, 0x0E, 0x81, 0x81},       {0x01, 0x42, 0x90, 0xFF},
		{0x01, 0x43, 0x10, 0xFF},       {0x00, 0x00, 0x00, 0x00},
	}, {     /* Color Effect UserSet */
		{0x01, 0x0E, 0x81, 0x81},       {0x01, 0x42, 0x00, 0xFF},
		{0x01, 0x43, 0x00, 0xFF},       {0x00, 0x00, 0x00, 0x00},
	}
};
#define NUM_EFFECT_COLOR_REGS ARRAY_SIZE(rj6aba100_register_effect_color)


/* EMXX_RJ6ABA100_CID_EFFECT_NEGATIVE */
static const struct register_info
rj6aba100_register_effect_negative[NUM_EFFECT_NEGATIVE_MENUS][3] =
{
	{       /* Negative Effect Off */
		{0x02, 0x2C, 0x03, 0xFF},       {0x01, 0x0E, 0x01, 0x09},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* Negative Effect On */
		{0x02, 0x2C, 0x02, 0xFF},       {0x01, 0x0E, 0x09, 0x09},
		{0x00, 0x00, 0x00, 0x00},
	}
};
#define NUM_EFFECT_NEGATIVE_REGS ARRAY_SIZE(rj6aba100_register_effect_negative)


/* EMXX_RJ6ABA100_CID_EFFECT_EMBOSS */
static const struct register_info
rj6aba100_register_effect_emboss[NUM_EFFECT_EMBOSS_MENUS][4] =
{
	{       /* Emboss Effect Off */
		{0x02, 0x2C, 0x03, 0xFF},       {0x01, 0x45, 0xC8, 0x09},
		{0x01, 0x0E, 0x01, 0x31},       {0x00, 0x00, 0x00, 0x00},
	}, {     /* Emboss Effect mode0 */
		{0x02, 0x2C, 0x01, 0xFF},       {0x01, 0x45, 0x80, 0x09},
		{0x01, 0x0E, 0x21, 0x31},       {0x00, 0x00, 0x00, 0x00},
	}, {     /* Emboss Effect mode1 */
		{0x02, 0x2C, 0x01, 0xFF},       {0x01, 0x45, 0x80, 0x09},
		{0x01, 0x0E, 0x31, 0x31},       {0x00, 0x00, 0x00, 0x00},
	}
};
#define NUM_EFFECT_EMBOSS_REGS ARRAY_SIZE(rj6aba100_register_effect_emboss)


/* EMXX_RJ6ABA100_CID_EFFECT_SKETCH */
static struct register_info
rj6aba100_register_effect_sketch[NUM_EFFECT_SKETCH_MENUS][7] =
{
	{       /* Sketch Effect Off */
		{0x02, 0x2C, 0x03, 0xFF},       {0x01, 0x0E, 0x01, 0x41},
		{0x01, 0x44, 0x20, 0xFF},
		{0x01, 0x45, 0xC8, 0xFF},       {0x01, 0x46, 0x10, 0xFF},
		{0x01, 0x47, 0x80, 0xFF},       {0x00, 0x00, 0x00, 0x00},
	}, {     /* Sketch Effect image1 */
		{0x02, 0x2C, 0x01, 0xFF},       {0x01, 0x0E, 0x41, 0x41},
		{0x01, 0x44, 0xFF, 0xFF},
		{0x01, 0x45, 0xFF, 0xFF},       {0x01, 0x46, 0x08, 0xFF},
		{0x01, 0x47, 0xFF, 0xFF},       {0x00, 0x00, 0x00, 0x00},
	}, {     /* Sketch Effect image2 */
		{0x02, 0x2C, 0x01, 0xFF},       {0x01, 0x0E, 0x41, 0x41},
		{0x01, 0x44, 0xFF, 0xFF},
		{0x01, 0x45, 0xFF, 0xFF},       {0x01, 0x46, 0x08, 0xFF},
		{0x01, 0x47, 0x80, 0xFF},       {0x00, 0x00, 0x00, 0x00},
	}, {     /* Sketch Effect image3 */
		{0x02, 0x2C, 0x01, 0xFF},       {0x01, 0x0E, 0x41, 0x41},
		{0x01, 0x44, 0xFF, 0xFF},
		{0x01, 0x45, 0x80, 0xFF},       {0x01, 0x46, 0x08, 0xFF},
		{0x01, 0x47, 0xFF, 0xFF},       {0x00, 0x00, 0x00, 0x00},
	}, {     /* Sketch Effect image4 */
		{0x02, 0x2C, 0x01, 0xFF},       {0x01, 0x0E, 0x41, 0x41},
		{0x01, 0x44, 0x20, 0xFF},
		{0x01, 0x45, 0x80, 0xFF},       {0x01, 0x46, 0x08, 0xFF},
		{0x01, 0x47, 0xFF, 0xFF},       {0x00, 0x00, 0x00, 0x00},
	}, {     /* Sketch Effect UserSet */
		{0x02, 0x2C, 0x01, 0xFF},       {0x01, 0x0E, 0x41, 0x41},
		{0x01, 0x44, 0x00, 0xFF},
		{0x01, 0x45, 0x00, 0xFF},       {0x01, 0x46, 0x00, 0xFF},
		{0x01, 0x47, 0x00, 0xFF},       {0x00, 0x00, 0x00, 0x00},
	}
};
#define NUM_EFFECT_SKETCH_REGS ARRAY_SIZE(rj6aba100_register_effect_sketch)


/*===============================================================*/
/* RJ6ABA100 Camera control function                             */
/*===============================================================*/

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_chkreg_bank
 * RETURN   :
 * NOTE     : check & change RJ6ABA100 register bank
 * UPDATE   :
 ******************************************************************************/
#define NUM_B03_REGBANK_A  0x00
#define NUM_B03_REGBANK_B  0x01
#define NUM_B03_REGBANK_C  0x02
#define NUM_B03_REGBANK_D  0x03

static inline int rj6aba100_chkreg_bank(__u8 value)
{
	int ret;
	unsigned char bank;

	ret = rj6aba100_i2c_read(0x03, &bank, 1); /* get bank */
	if (!ret) {
		if (bank != value)
			ret = rj6aba100_i2c_write(0x03, &value, 1);
			/* set bank */
	}
	return ret;
}


#if CAM_DEBUG
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mage_register_debug
 * RETURN   :
 * NOTE     : check RJ6ABA100 register (debug only)
 * UPDATE   :
 ******************************************************************************/
static inline void emxx_mage_register_debug(void)
{
	unsigned char buf[16];
	int i, j, bank, ret;

	for (bank = NUM_B03_REGBANK_B; bank <= NUM_B03_REGBANK_D; bank++) {
		ret = rj6aba100_chkreg_bank(bank);
		if (ret)
			return;

		d1b("========================================\n");
		d1b(" Group %c\n",
		       ((bank == NUM_B03_REGBANK_B) ? 'B'
			: ((bank == NUM_B03_REGBANK_C) ? 'C' : 'D')));
		d1b("-----------------------------------------\n");

		for (i = 0; i < 0xFF; i += 16) {
			for (j = 0; j < 16; j++) {
				ret = rj6aba100_i2c_read(i + j, &buf[j], 1);
				if (ret)
					return;
			}

			d1b(" %c-%02x:  %02x %02x %02x %02x  "
			       "%02x %02x %02x %02x  %02x %02x "
			       "%02x %02x  %02x %02x %02x %02x\n",
			       ((bank == NUM_B03_REGBANK_B) ? 'B'
				: ((bank == NUM_B03_REGBANK_C) ? 'C' : 'D')),
			       i, buf[0], buf[1], buf[2], buf[3], buf[4],
			       buf[5], buf[6], buf[7], buf[8], buf[9],
			       buf[10], buf[11], buf[12], buf[13], buf[14],
			       buf[15]);
		}
	}
	d1b("============================================\n");

}
#endif /* CAM_DEBUG */


/*****************************************************************************
 * MODULE   : emxx_mega
 * FUNCTION : rj6aba100_setreg_array
 * RETURN   :
 * NOTE     : set RJ6ABA100 register group
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_setreg_array(struct register_info *reg_info)
{
	int i, ret = 0;
	unsigned char data;

	for (i = 0; reg_info[i].bank != 0x00; i++) {
		ret = rj6aba100_chkreg_bank(reg_info[i].bank); /* check bank */
		if (!ret) {
			ret = rj6aba100_i2c_read(reg_info[i].address, &data, 1);
			if (!ret) {
				data = (data & ~reg_info[i].mask)
					| reg_info[i].data;
				ret  = rj6aba100_i2c_write(reg_info[i].address,
							   &data, 1);
			}
		}
	}
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_setreg_pll_control
 * RETURN   :
 * NOTE     : set RJ6ABA100 PLL control register group
 * UPDATE   :
 ******************************************************************************/
#define NUM_B09_PLL_CTRL01      0x09
#define NUM_B0A_PLL_CTRL02      0x0A
#define NUM_B0B_PLL_CTRL03      0x0B
#define NUM_B09_PLL_CTRL01_MASK 0x30
#define NUM_B0A_PLL_CTRL02_MASK 0xFF
#define NUM_B0B_PLL_CTRL03_MASK 0xFF

#define NUM_B09_PLL_CTRL01_X10  0x00
#define NUM_B09_PLL_CTRL01_X12  0x10
#define NUM_B09_PLL_CTRL01_X14  0x20
#define NUM_B09_PLL_CTRL01_X16  0x30

#define NUM_B0A_PLL_CTRL02_X1   0x00
#define NUM_B0A_PLL_CTRL02_X2   0x01
#define NUM_B0A_PLL_CTRL02_X3   0x02
#define NUM_B0A_PLL_CTRL02_X4   0x03
#define NUM_B0A_PLL_CTRL02_X5   0x04
#define NUM_B0A_PLL_CTRL02_X6   0x05
#define NUM_B0A_PLL_CTRL02_X7   0x06
#define NUM_B0A_PLL_CTRL02_X8   0x07

#define NUM_B0B_PLL_CTRL03_X1   0x00
#define NUM_B0B_PLL_CTRL03_X2   0x01
#define NUM_B0B_PLL_CTRL03_X3   0x02
#define NUM_B0B_PLL_CTRL03_X4   0x03
#define NUM_B0B_PLL_CTRL03_X5   0x04
#define NUM_B0B_PLL_CTRL03_X6   0x05
#define NUM_B0B_PLL_CTRL03_X7   0x06
#define NUM_B0B_PLL_CTRL03_X8   0x07

static inline int rj6aba100_setreg_pll_control(int arg)
{
	int i, ret;
	unsigned char add[3], buf[3];

	ret = rj6aba100_chkreg_bank(NUM_B03_REGBANK_B); /* check bank B */
	if (!ret) {
		ret  = rj6aba100_i2c_read(NUM_B0A_PLL_CTRL02, &buf[0], 1);
		ret |= rj6aba100_i2c_read(NUM_B09_PLL_CTRL01, &buf[1], 1);
		ret |= rj6aba100_i2c_read(NUM_B0B_PLL_CTRL03, &buf[2], 1);
		if (!ret) {
			/* camera module clock setting */
			add[0] = NUM_B0A_PLL_CTRL02;
			add[1] = NUM_B09_PLL_CTRL01;
			add[2] = NUM_B0B_PLL_CTRL03;

			if (arg == 344) {
				buf[0] = (buf[0] &
				 (unsigned char)~NUM_B0A_PLL_CTRL02_MASK)
					| NUM_B0A_PLL_CTRL02_X2;
				/* x1/2 : 11.47/2 => 5.74    */
				buf[1] = (buf[1] &
				 (unsigned char)~NUM_B09_PLL_CTRL01_MASK)
					| NUM_B09_PLL_CTRL01_X12;
				/* x12  : 5.74x12 => 68.8    */
				buf[2] = (buf[2] &
				 (unsigned char)~NUM_B0B_PLL_CTRL03_MASK)
					| NUM_B0B_PLL_CTRL03_X2;
				/* x1/2 : 68.8/2  => 34.4Mhz */
			} else if (arg == 574)    {
				buf[0] = (buf[0] &
				 (unsigned char)~NUM_B0A_PLL_CTRL02_MASK)
					| NUM_B0A_PLL_CTRL02_X2;
				/* x1/2 : 11.47/2 => 5.74    */
				buf[1] = (buf[1] &
				 (unsigned char)~NUM_B09_PLL_CTRL01_MASK)
					| NUM_B09_PLL_CTRL01_X10;
				/* x10  : 5.74x10 => 57.4    */
				buf[2] = (buf[2] &
				 (unsigned char)~NUM_B0B_PLL_CTRL03_MASK)
					| NUM_B0B_PLL_CTRL03_X1;
				/* x1/1 : 57.4/1  => 57.4Mhz */
			} else if (arg == 688)    {
				buf[0] = (buf[0] &
				 (unsigned char)~NUM_B0A_PLL_CTRL02_MASK)
					| NUM_B0A_PLL_CTRL02_X2;
				/* x1/2 : 11.47/2 => 5.74    */
				buf[1] = (buf[1] &
				 (unsigned char)~NUM_B09_PLL_CTRL01_MASK)
					| NUM_B09_PLL_CTRL01_X12;
				/* x12  : 5.74x12 => 68.8    */
				buf[2] = (buf[2] &
				 (unsigned char)~NUM_B0B_PLL_CTRL03_MASK)
					| NUM_B0B_PLL_CTRL03_X1;
				/* x1/1 : 68.8/1  => 68.8Mhz */
			}

			for (i = 0; i < ARRAY_SIZE(buf); i++) {
				ret = rj6aba100_i2c_write(add[i], &buf[i], 1);
				if (ret)
					return ret;
			}
		}
	}
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_setreg_outimg_size
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_OUTIMG_SIZE
 *          : set RJ6ABA100 output image size register group
 * UPDATE   :
 ******************************************************************************/
#define NUM_B34_WINDOW_X1H  0x34
#define NUM_B35_WINDOW_X1L  0x35
#define NUM_B36_WINDOW_Y1H  0x36
#define NUM_B37_WINDOW_Y1L  0x37
#define NUM_B38_WINDOW_X2H  0x38
#define NUM_B39_WINDOW_X2L  0x39
#define NUM_B3A_WINDOW_Y2H  0x3A
#define NUM_B3B_WINDOW_Y2L  0x3B

static inline int rj6aba100_setreg_outimg_size(__u8 value)
{
	struct control_resize_info *s;
	unsigned char add[8], buf[8];
	int i, ret;

	ret = rj6aba100_chkreg_bank(NUM_B03_REGBANK_B); /* check bank B */
	if (!ret) {
		s = (struct control_resize_info *)rj6aba100_outimg_size_menus;
		s += value;

		add[0] = NUM_B34_WINDOW_X1H;
		buf[0] = (unsigned char)(s->x >> 8);
		/* windowX1(H) */
		add[1] = NUM_B35_WINDOW_X1L;
		buf[1] = (unsigned char) s->x;
		/* windowX1(L) */
		add[2] = NUM_B36_WINDOW_Y1H;
		buf[2] = (unsigned char)(s->y >> 8);
		/* windowY1(H) */
		add[3] = NUM_B37_WINDOW_Y1L;
		buf[3] = (unsigned char) s->y;
		/* windowY1(L) */
		add[4] = NUM_B38_WINDOW_X2H;
		buf[4] = (unsigned char)(s->width >> 8);
		/* windowX2(H) */
		add[5] = NUM_B39_WINDOW_X2L;
		buf[5] = (unsigned char) s->width;
		/* windowX2(L) */
		add[6] = NUM_B3A_WINDOW_Y2H;
		buf[6] = (unsigned char)(s->height >> 8);
		/* windowY2(H) */
		add[7] = NUM_B3B_WINDOW_Y2L;
		buf[7] = (unsigned char) s->height;
		/* windowY2(L) */

		for (i = 0; i < ARRAY_SIZE(buf); i++) {
			ret = rj6aba100_i2c_write(add[i], &buf[i], 1);
			if (ret)
				return ret;
		}
		ret = rj6aba100_setreg_array(
	    (struct register_info *)(&rj6aba100_register_outimg_size[value]));
	}
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_setreg_outimg_fmt
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_OUTIMG_FORMAT
 *          : set RJ6ABA100 output image format register group
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_setreg_outimg_fmt(__u8 value)
{
	return rj6aba100_setreg_array(
	      (struct register_info *)(&rj6aba100_register_outimg_fmt[value]));
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_setreg_pclk_rate
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_OUTIMG_SIZE &
 *            EMXX_RJ6ABA100_CID_OUTIMG_FORMAT
 *          : set RJ6ABA100 PCLK rate register group
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_setreg_pclk_rate(void)
{
	int pclk_flag = SELECT_PCLK_RATE_DESCRIPTION(rj6aba100->outimg_fmt);

	return rj6aba100_setreg_array(
	 (struct register_info *)
	 (&rj6aba100_register_pclk_rate[pclk_flag][rj6aba100->capture_size]));
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_setreg_outdata_fmt
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_OUTDATA_FORMAT
 *          : set RJ6ABA100 output data array format register group
 * UPDATE   :
 ******************************************************************************/
#define NUM_B18_FORMAT       0x18
#define NUM_B18_FORMAT_MASK  0xFF
#define NUM_B18_FORMAT_SFT   0x00

static inline int rj6aba100_setreg_outdata_fmt(__u8 value)
{
	char buf;
	int ret;

	ret = rj6aba100_chkreg_bank(NUM_B03_REGBANK_B); /* check bank B */
	if (!ret) {
		ret = rj6aba100_i2c_read(NUM_B18_FORMAT, &buf, 1);
		if (!ret) {
			buf = (buf & ~NUM_B18_FORMAT_MASK) | value;
			ret = rj6aba100_i2c_write(NUM_B18_FORMAT, &buf, 1);
			/* OutputFormat */
		}
	}
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_setreg_flicker_manual
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_FLICKER
 *          : set RJ6ABA100 friker free (manual mode) register
 * UPDATE   :
 ******************************************************************************/
#define NUM_BB7_FLICKER_CTRL       0xB7
#define NUM_BB7_FLICKER_CTRL_MASK  0x3E
#define NUM_BB7_FLICKER_CTRL_SFT   0x01

static inline int rj6aba100_setreg_flicker_manual(__u8 value)
{
	char buf;
	int ret;

	ret = rj6aba100_chkreg_bank(NUM_B03_REGBANK_B); /* check bank B */
	if (!ret) {
		ret = rj6aba100_i2c_read(NUM_BB7_FLICKER_CTRL, &buf, 1);
		if (!ret) {
			buf = (buf & ~NUM_BB7_FLICKER_CTRL_MASK)
				| rj6aba100_flicker_menus[value].value;
			ret = rj6aba100_i2c_write(NUM_BB7_FLICKER_CTRL,
						  &buf, 1);
			/* FlikerControl1 */
		}
	}
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_setreg_wb_mode
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_WB
 *          : set RJ6ABA100 brightness register
 * UPDATE   :
 ******************************************************************************/
#define NUM_C04_AUTO_CTRL           0x04
#define NUM_C04_AUTO_CTRL_AWB_MASK  0x04
#define NUM_C04_AUTO_CTRL_AE_MASK   0x03
#define NUM_C04_AUTO_CTRL_AWB_SFT   0x02
#define NUM_C04_AUTO_CTRL_AE_SFT    0x00

static inline int rj6aba100_setreg_wb_mode(__u8 value)
{
	char buf;
	int ret;

	ret = rj6aba100_chkreg_bank(NUM_B03_REGBANK_C); /* check bank C */
	if (!ret) {
		ret = rj6aba100_i2c_read(NUM_C04_AUTO_CTRL, &buf, 1);
		if (!ret) {
			buf = (buf & ~NUM_C04_AUTO_CTRL_AWB_MASK)
				| (value << NUM_C04_AUTO_CTRL_AWB_SFT);
			ret = rj6aba100_i2c_write(NUM_C04_AUTO_CTRL, &buf, 1);
			/* auto_control_1 (AWB Register Update) */
		}
	}
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_setreg_wb_manual_gain
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_WB_MANUAL_GAIN
 *          : set RJ6ABA100 brightness register
 * UPDATE   :
 ******************************************************************************/
#define NUM_BAD_WB_RGAIN             0xAD
#define NUM_BAE_WB_GGAIN             0xAE
#define NUM_BAF_WB_BGAIN             0xAF
#define NUM_BAD_WB_RGAIN_MASK        0x00FF0000
#define NUM_BAE_WB_GGAIN_MASK        0x0000FF00
#define NUM_BAF_WB_BGAIN_MASK        0x000000FF
#define NUM_BAD_WB_RGAIN_SFT         16
#define NUM_BAE_WB_GGAIN_SFT         8
#define NUM_BAF_WB_BGAIN_SFT         0
#define NUM_WB_MANUAL_GAIN_TABLE_MAX 3

static inline int rj6aba100_getreg_wb_manual_gain(__u32 *value)
{
	int i, ret;

	ret = rj6aba100_chkreg_bank(NUM_B03_REGBANK_B); /* check bank B */
	if (!ret) {
		unsigned char addr[NUM_WB_MANUAL_GAIN_TABLE_MAX] =
			{NUM_BAD_WB_RGAIN,
			 NUM_BAE_WB_GGAIN,
			 NUM_BAF_WB_BGAIN};
		unsigned char data[NUM_WB_MANUAL_GAIN_TABLE_MAX];

		for (i = 0; i < NUM_WB_MANUAL_GAIN_TABLE_MAX; i++)
			ret = rj6aba100_i2c_read(addr[i], &data[i], 1);
		*value = (data[0] << NUM_BAD_WB_RGAIN_SFT)
			| (data[1] << NUM_BAE_WB_GGAIN_SFT)
			| (data[2] << NUM_BAF_WB_BGAIN_SFT);
	}
	return ret;
}

static inline int rj6aba100_setreg_wb_manual_gain(__u32 value)
{
	int ret;

	ret = rj6aba100_chkreg_bank(NUM_B03_REGBANK_B); /* check bank B */
	if (!ret) {
		int i;
		unsigned char addr[NUM_WB_MANUAL_GAIN_TABLE_MAX] =
			{NUM_BAD_WB_RGAIN,
			 NUM_BAE_WB_GGAIN,
			 NUM_BAF_WB_BGAIN};
		unsigned char data[NUM_WB_MANUAL_GAIN_TABLE_MAX] =
			{(__u8)((value & NUM_BAD_WB_RGAIN_MASK)
				>> NUM_BAD_WB_RGAIN_SFT),
			 (__u8)((value & NUM_BAE_WB_GGAIN_MASK)
				>> NUM_BAE_WB_GGAIN_SFT),
			 (__u8)((value & NUM_BAF_WB_BGAIN_MASK)
				>> NUM_BAF_WB_BGAIN_SFT)};

		for (i = 0; i < NUM_WB_MANUAL_GAIN_TABLE_MAX; i++)
			ret = rj6aba100_i2c_write(addr[i], &data[i], 1);
			/* Rgain, Ggain, Bgain */
	}
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_setreg_brightness
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_BRIGHTNESS
 *          : set RJ6ABA100 brightness register
 * UPDATE   :
 ******************************************************************************/
#define NUM_B4B_YBLIGHTNESS  0x4B

static inline int rj6aba100_setreg_brightness(__u8 value)
{
	int ret;

	ret = rj6aba100_chkreg_bank(NUM_B03_REGBANK_B); /* check bank B */
	if (!ret)
		ret = rj6aba100_i2c_write(NUM_B4B_YBLIGHTNESS, &value, 1);
		/* Brightness */
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_setreg_contrast
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_CONTRAST
 *          : set RJ6ABA100 contrast register
 * UPDATE   :
 ******************************************************************************/
#define NUM_B4A_YCONTRAST  0x4A

static inline int rj6aba100_setreg_contrast(__u8 value)
{
	int ret;

	ret = rj6aba100_chkreg_bank(NUM_B03_REGBANK_B); /* check bank B */
	if (!ret)
		ret = rj6aba100_i2c_write(NUM_B4A_YCONTRAST, &value, 1);
		/* Y contrast */
	return ret;
}


/* EMXX_RJ6ABA100_CID_SHARPNESS */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_setreg_sharpness
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_SHARPNESS
 *          : set RJ6ABA100 contrast register
 * UPDATE   :
 ******************************************************************************/
#define NUM_B52_EDGE_GAIN           0x52
#define NUM_B54_EDGE_THRESHOLD      0x54
#define NUM_B52_EDGE_GAIN_MASK      0x00003F00
#define NUM_B54_EDGE_THRESHOLD_MASK 0x000000FF
#define NUM_B52_EDGE_GAIN_SFT       8
#define NUM_B54_EDGE_THRESHOLD_SFT  0
#define NUM_SHARPNESS_TABLE_MAX     2

static inline int rj6aba100_setreg_sharpness(__u32 value)
{
	int ret;

	ret = rj6aba100_chkreg_bank(NUM_B03_REGBANK_B); /* check bank B */
	if (!ret) {
		int i;
		unsigned char addr[NUM_SHARPNESS_TABLE_MAX] =
			{NUM_B52_EDGE_GAIN,
			 NUM_B54_EDGE_THRESHOLD};
		unsigned char data[NUM_SHARPNESS_TABLE_MAX] =
			{(__u8)((value & NUM_B52_EDGE_GAIN_MASK)
				>> NUM_B52_EDGE_GAIN_SFT),
			 (__u8)((value & NUM_B54_EDGE_THRESHOLD_MASK)
				>> NUM_B54_EDGE_THRESHOLD_SFT)};

		for (i = 0; i < NUM_SHARPNESS_TABLE_MAX; i++)
			ret = rj6aba100_i2c_write(addr[i], &data[i], 1);
			/* Edge Gain/Threshorld */
	}
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_setreg_mirror
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_MIRROR
 *          : set RJ6ABA100 Mirror register
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_setreg_mirror(__u8 value)
{
	int ret;

	/* Each Priprocessing */
	ret = rj6aba100_setreg_array(
		(struct register_info *)(&rj6aba100_register_mirror[value]));
	if (!ret) {
		/* Common Postprocessing */

		/*
		 * wait 1frame
		 */
		msleep(30);

		ret = rj6aba100_setreg_array(
			(struct register_info *)
			(&rj6aba100_register_mirror[NUM_MIRROR_MENUS]));
	}
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_setreg_effect_color
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_EFFECT_COLOR
 *          : set RJ6ABA100 effect (color) register
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_setreg_effect_color(__u8 value)
{
	return rj6aba100_setreg_array(
	 (struct register_info *)(&rj6aba100_register_effect_color[value]));
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_getreg_effect_color_userset
 *            / rj6aba100_setreg_effect_color_userset
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_EFFECT_COLOR_MANUAL
 *          : set RJ6ABA100 effect (color) register
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_get_effect_color(__u8 *value);
#define NUM_B42_SEPHIA_CB          0x42
#define NUM_B43_SEPHIA_CR          0x43
#define NUM_B42_SEPHIA_CB_MASK     0x0000FF00
#define NUM_B43_SEPHIA_CR_MASK     0x000000FF
#define NUM_B42_SEPHIA_CB_SFT      8
#define NUM_B43_SEPHIA_CR_SFT      0
#define NUM_EFFECT_COLOR_TABLE_MAX 2

static inline int rj6aba100_getreg_effect_color_userset(__u32 *value)
{
	unsigned char now_setting;
	int i, ret;

	ret = rj6aba100_get_effect_color(&now_setting);
	if (!ret) {
		*value = 0;
		for (i = 0; i < NUM_EFFECT_COLOR_TABLE_MAX; i++) {
			switch (rj6aba100_register_effect_color
				[now_setting][i + 1].address) {
			case NUM_B42_SEPHIA_CB:
				*value |=
					((__u32)rj6aba100_register_effect_color
					 [now_setting][i + 1].data
					 << NUM_B42_SEPHIA_CB_SFT);
				break;
			case NUM_B43_SEPHIA_CR:
				*value |=
					((__u32)rj6aba100_register_effect_color
					 [now_setting][i + 1].data
					 << NUM_B43_SEPHIA_CR_SFT);
				break;
			default:
				break;
			}
		}
	}
	return ret;
}

static inline int rj6aba100_setreg_effect_color_userset(__u32 value)
{
	int i;
	unsigned char addr[NUM_EFFECT_COLOR_TABLE_MAX] =
		{NUM_B42_SEPHIA_CB,
		 NUM_B43_SEPHIA_CR};
	unsigned char data[NUM_EFFECT_COLOR_TABLE_MAX] =
		{(__u8)((value & NUM_B42_SEPHIA_CB_MASK)
			>> NUM_B42_SEPHIA_CB_SFT),
		 (__u8)((value & NUM_B43_SEPHIA_CR_MASK)
			>> NUM_B43_SEPHIA_CR_SFT)};

	for (i = 0; i < NUM_EFFECT_COLOR_TABLE_MAX; i++) {
		/* register is actually set in
		   EMXX_RJ6ABA100_CID_EFFECT_COLOR */
		rj6aba100_register_effect_color
			[NUM_EFFECT_COLOR_MANUAL][i + 1].address = addr[i];
		rj6aba100_register_effect_color
			[NUM_EFFECT_COLOR_MANUAL][i + 1].data    = data[i];
	}
	return 0;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_setreg_effect_negative
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_EFFECT_NEGATIVE
 *          : set RJ6ABA100 effect (negative) register
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_setreg_effect_negative(__u8 value)
{
	return rj6aba100_setreg_array(
	 (struct register_info *)(&rj6aba100_register_effect_negative[value]));
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_setreg_effect_emboss
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_EFFECT_EMBOSS
 *          : set RJ6ABA100 effect (emboss) register
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_setreg_effect_emboss(__u8 value)
{
	return rj6aba100_setreg_array(
	   (struct register_info *)(&rj6aba100_register_effect_emboss[value]));
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_setreg_effect_sketch
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_EFFECT_SKETCH
 *          * set RJ6ABA100 effect (sketch) register
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_setreg_effect_sketch(__u8 value)
{
	return rj6aba100_setreg_array(
	 (struct register_info *)(&rj6aba100_register_effect_sketch[value]));
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_getreg_effect_sketch_userset
 *          / rj6aba100_setreg_effect_sketch_userset
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_EFFECT_SKETCH_MANUAL
 *          : set RJ6ABA100 effect (sketch) register
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_get_effect_sketch(__u8 *value);
#define NUM_B44_SKETCH_OFFSET1       0x44
#define NUM_B45_SKETCH_OFFSET2       0x45
#define NUM_B46_SKETCH_OFFSET3       0x46
#define NUM_B47_SKETCH_OFFSET4       0x47
#define NUM_B44_SKETCH_OFFSET1_MASK  0xFF000000
#define NUM_B45_SKETCH_OFFSET2_MASK  0x00FF0000
#define NUM_B46_SKETCH_OFFSET3_MASK  0x0000FF00
#define NUM_B47_SKETCH_OFFSET4_MASK  0x000000FF
#define NUM_B44_SKETCH_OFFSET1_SFT   24
#define NUM_B45_SKETCH_OFFSET2_SFT   16
#define NUM_B46_SKETCH_OFFSET3_SFT   8
#define NUM_B47_SKETCH_OFFSET4_SFT   0
#define NUM_EFFECT_SKETCH_TABLE_MAX  4

static inline int rj6aba100_getreg_effect_sketch_userset(__u32 *value)
{
	unsigned char now_setting;
	int i, ret;

	ret = rj6aba100_get_effect_sketch(&now_setting);
	if (!ret) {
		*value = 0;
		for (i = 0; i < NUM_EFFECT_SKETCH_TABLE_MAX; i++) {
			switch (rj6aba100_register_effect_sketch
				[now_setting][i + 2].address) {
			case NUM_B44_SKETCH_OFFSET1:
				*value |=
				((__u32)rj6aba100_register_effect_sketch
				 [now_setting][i + 2].data
				 << NUM_B44_SKETCH_OFFSET1_SFT);
				break;
			case NUM_B45_SKETCH_OFFSET2:
				*value |=
				((__u32)rj6aba100_register_effect_sketch
				 [now_setting][i + 2].data
				 << NUM_B45_SKETCH_OFFSET2_SFT);
				break;
			case NUM_B46_SKETCH_OFFSET3:
				*value |=
				((__u32)rj6aba100_register_effect_sketch
				 [now_setting][i + 2].data
				 << NUM_B46_SKETCH_OFFSET3_SFT);
				break;
			case NUM_B47_SKETCH_OFFSET4:
				*value |=
				((__u32)rj6aba100_register_effect_sketch
				 [now_setting][i + 2].data
				 << NUM_B47_SKETCH_OFFSET4_SFT);
				break;
			default:
				break;
			}
		}
	}
	return ret;
}

static inline int rj6aba100_setreg_effect_sketch_userset(__u32 value)
{
	int i;
	unsigned char addr[NUM_EFFECT_SKETCH_TABLE_MAX] =
		{NUM_B44_SKETCH_OFFSET1,
		 NUM_B45_SKETCH_OFFSET2,
		 NUM_B46_SKETCH_OFFSET3,
		 NUM_B47_SKETCH_OFFSET4};
	unsigned char data[NUM_EFFECT_SKETCH_TABLE_MAX] =
		{(__u8)((value & NUM_B44_SKETCH_OFFSET1_MASK)
			>> NUM_B44_SKETCH_OFFSET1_SFT),
		 (__u8)((value & NUM_B45_SKETCH_OFFSET2_MASK)
			>> NUM_B45_SKETCH_OFFSET2_SFT),
		 (__u8)((value & NUM_B46_SKETCH_OFFSET3_MASK)
			>> NUM_B46_SKETCH_OFFSET3_SFT),
		 (__u8)((value & NUM_B47_SKETCH_OFFSET4_MASK)
			>> NUM_B47_SKETCH_OFFSET4_SFT)};

	for (i = 0; i < NUM_EFFECT_SKETCH_TABLE_MAX; i++) {
		/* register is actually set in
		   EMXX_RJ6ABA100_CID_EFFECT_SKETCH */
		rj6aba100_register_effect_sketch
			[NUM_EFFECT_SKETCH_MANUAL][i + 2].address = addr[i];
		rj6aba100_register_effect_sketch
			[NUM_EFFECT_SKETCH_MANUAL][i + 2].data    = data[i];
	}
	return 0;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_setreg_control01
 * RETURN   :
 * NOTE     : set RJ6ABA100 register Control01
 * UPDATE   :
 ******************************************************************************/
#define NUM_B04_STDBYMODE_ON    0x00
#define NUM_B04_STDBYMODE_OFF   0x80
#define NUM_B04_CLOCK_KILL      0x40
#define NUM_B04_CLOCK_NOTKILL   0x00
#define NUM_B04_STANDBY_ON      0x20
#define NUM_B04_STANDBY_OFF     0x00
#define NUM_B04_PLL_SELECT      0x10
#define NUM_B04_PLL_NOTSELECT   0x00
#define NUM_B04_BIDIRECT_HIZ    0x08
#define NUM_B04_BIDIRECT_NORMAL 0x00

#define NUM_B04_STDBYMODE_MASK  0x80
#define NUM_B04_CLOCK_MASK      0x40
#define NUM_B04_STANDBY_MASK    0x20
#define NUM_B04_PLL_MASK        0x10
#define NUM_B04_BIDIRECT_MASK   0x08

static inline int rj6aba100_setreg_control01(__u8 mask, __u8 value)
{
	char buf;
	int ret;

	ret = rj6aba100_chkreg_bank(NUM_B03_REGBANK_B); /* check bank B */
	if (!ret) {
		ret = rj6aba100_i2c_read(0x04, &buf, 1);
		if (!ret) {
			buf = (buf & ~mask) | value;
			d1b("send cmd 0x04, 0x%x\n", buf);
			ret = rj6aba100_i2c_write(0x04, &buf, 1);
			/* Control_01 */
		}
	}
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_setreg_softreset
 * RETURN   :
 * NOTE     : set RJ6ABA100 register SoftReset
 * UPDATE   :
 ******************************************************************************/
#define NUM_B06_SOFTRESET     0x06
#define NUM_B06_SOFTRESET_OFF 0x00
#define NUM_B06_SOFTRESET_ON  0x01

static inline int rj6aba100_setreg_softreset(__u8 value)
{
	char buf;
	int ret;

	ret = rj6aba100_chkreg_bank(NUM_B03_REGBANK_B); /* check bank B */
	if (!ret) {
		buf = value;
		ret = rj6aba100_i2c_write(NUM_B06_SOFTRESET, &buf, 1);
		mdelay(100);
		/* Softreset */
	}
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_getreg_firmware_version
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_FW
 *          : get RJ6ABA100 register revesion number
 * UPDATE   :
 ******************************************************************************/
#define NUM_B02_REVNUMBER  0x02

static inline int rj6aba100_getreg_firmware_version(__u8 *val)
{
	char buf;
	int ret;

	ret = rj6aba100_i2c_read(NUM_B02_REVNUMBER, &buf, 1);
	/* revesion number */
	if (!ret)
		*val = buf;
	return ret;
}


/* etc */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_set_liveview
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
#define SET_LIVEVIEW_STOP  0
#define SET_LIVEVIEW_START 1

static inline int rj6aba100_set_liveview(__u8 val)
{
	unsigned char mask, data;
	int ret = 0;

	mask = NUM_B04_STDBYMODE_MASK | NUM_B04_CLOCK_MASK
		| NUM_B04_STANDBY_MASK | NUM_B04_PLL_MASK
		| NUM_B04_BIDIRECT_MASK;
	if (val == SET_LIVEVIEW_START)
		data = NUM_B04_STDBYMODE_OFF | NUM_B04_CLOCK_NOTKILL
			| NUM_B04_STANDBY_OFF | NUM_B04_PLL_SELECT
			| NUM_B04_BIDIRECT_HIZ;    /* 0x98 */
	else
		data = NUM_B04_STDBYMODE_OFF | NUM_B04_CLOCK_NOTKILL
			| NUM_B04_STANDBY_ON  | NUM_B04_PLL_NOTSELECT
			| NUM_B04_BIDIRECT_NORMAL; /* 0xA0 */
/* @@	else	data = NUM_B04_STDBYMODE_OFF | NUM_B04_CLOCK_NOTKILL
| NUM_B04_STANDBY_OFF | NUM_B04_PLL_NOTSELECT | NUM_B04_BIDIRECT_NORMAL;
 / * 0x80 * / */
/* @@	else	data = NUM_B04_STDBYMODE_OFF | NUM_B04_CLOCK_KILL
| NUM_B04_STANDBY_ON  | NUM_B04_PLL_NOTSELECT | NUM_B04_BIDIRECT_NORMAL;
/ * 0xE0 * / */

	ret = rj6aba100_setreg_control01(mask, data);
	if (!ret) {
		if (val == SET_LIVEVIEW_START)
			rj6aba100->state = EMXX_RJ6ABA100_LIVE;
		else
			rj6aba100->state = EMXX_RJ6ABA100_IDLE;
	}

	return ret;
}

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_set_capture_read
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_set_capture_read(__u8 data)
{
	int ret = 0;

	if (EMXX_RJ6ABA100_IDLE != rj6aba100->state)
		return -EBUSY;

	ret = rj6aba100_setreg_outimg_size(data);
	if (!ret)
		rj6aba100->state = EMXX_RJ6ABA100_CAPTURE;

	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_shutdown
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_shutdown(void)
{
	int ret = 0;

	/* rj6aba100 power off
	 */
	gpio_direction_output(GPIO_CAM_RST, 0x0); /* GIO output Low */

	return ret;
}





/* EMXX_RJ6ABA100_CID_FW */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_get_firmware_version
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_FW
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_get_firmware_version(__u8 *value)
{
	*value = rj6aba100->firmware_version;
	return 0;
}


/* EMXX_RJ6ABA100_CID_OUTIMG_SIZE */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_get_outimg_size / rj6aba100_set_outimg_size
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_FW
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_get_outimg_size(__u8 *value)
{
	*value = rj6aba100->capture_size;
	return 0;
}

static inline int rj6aba100_set_outimg_size(__u8 value)
{
	int ret = 0;

	if (value != rj6aba100->capture_size) {
		rj6aba100->capture_size = value;
		rj6aba100->reset = 1;
	}

	return ret;
}

/* EMXX_RJ6ABA100_CID_OUTIMG_FORMAT */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_get_outimg_fmt / rj6aba100_set_outimg_fmt
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_FW
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_get_outimg_fmt(__u8 *value)
{
	*value = rj6aba100->outimg_fmt;
	return 0;
}

static inline int rj6aba100_set_outimg_fmt(__u8 value)
{
	int ret = 0;

	if (value != rj6aba100->outimg_fmt)
		rj6aba100->outimg_fmt = value;

	return ret;
}

/* EMXX_RJ6ABA100_CID_OUTDATA_FORMAT */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_get_imgdata_fmt / rj6aba100_set_imgdata_fmt
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_FW
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_get_imgdata_fmt(__u8 *value)
{
	*value = rj6aba100->imgdata_fmt;
	return 0;
}

static inline int rj6aba100_set_imgdata_fmt(__u8 value)
{
	int ret = 0;

	if (value != rj6aba100->imgdata_fmt)
		rj6aba100->imgdata_fmt = value;

	return ret;
}

/* EMXX_RJ6ABA100_CID_FLICKER */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_get_flicker_manual / rj6aba100_set_flicker_manual
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_FLICKER
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_get_flicker_manual(__u8 *value)
{
	*value = rj6aba100->flicker_manual;
	return 0;
}

static inline int rj6aba100_set_flicker_manual(__u8 value)
{
	int ret = 0;

	ret = rj6aba100_setreg_flicker_manual(value);
	if (!ret)
		rj6aba100->flicker_manual = value;
	return ret;
}


/* EMXX_RJ6ABA100_CID_WB */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_get_wb_mode / rj6aba100_set_wb_mode
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_WB
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_get_wb_mode(__u8 *value)
{
	*value = rj6aba100->wb_mode;
	return 0;
}

static inline int rj6aba100_set_wb_mode(__u8 value)
{
	int ret = 0;

	ret = rj6aba100_setreg_wb_mode(value);
	if (!ret)
		rj6aba100->wb_mode = value;
	return ret;
}


/* EMXX_RJ6ABA100_CID_WB_MANUAL_GAIN */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_get_wb_manual_gain / rj6aba100_set_wb_manual_gain
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_WB_MANUAL_GAIN
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_get_wb_manual_gain(__u32 *value)
{
	int ret = 0;

	ret = rj6aba100_getreg_wb_manual_gain(value);
	if (!ret)
		rj6aba100->wb_manual_gain = *value;
	return ret;
}

static inline int rj6aba100_set_wb_manual_gain(__u32 value)
{
	int ret = 0;

	ret = rj6aba100_setreg_wb_manual_gain(value);
	if (!ret)
		rj6aba100->wb_manual_gain = value;
	return ret;
}


/* EMXX_RJ6ABA100_CID_BRIGHTNESS */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_get_brightness / rj6aba100_set_brightness
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_BRIGHTNESS
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_get_brightness(__u8 *value)
{
	*value = rj6aba100->brightness;
	return 0;
}

static inline int rj6aba100_set_brightness(__u8 value)
{
	int ret = 0;

	ret = rj6aba100_setreg_brightness(value);
	if (!ret)
		rj6aba100->brightness = value;
	return ret;
}


/* EMXX_RJ6ABA100_CID_CONTRAST */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_get_contrast / rj6aba100_set_contrast
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_CONTRAST
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_get_contrast(__u8 *value)
{
	*value = rj6aba100->contrast;
	return 0;
}

static inline int rj6aba100_set_contrast(__u8 value)
{
	int ret = 0;

	ret = rj6aba100_setreg_contrast(value);
	if (!ret)
		rj6aba100->contrast = value;
	return ret;
}


/* EMXX_RJ6ABA100_CID_SHARPNESS */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_get_sharpness / rj6aba100_set_sharpness
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_SHARPNESS
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_get_sharpness(__u32 *value)
{
	*value = rj6aba100->sharpness;
	return 0;
}

static inline int rj6aba100_set_sharpness(__u32 value)
{
	int ret = 0;

	ret = rj6aba100_setreg_sharpness(value);
	if (!ret)
		rj6aba100->sharpness = value;
	return ret;
}


/* EMXX_RJ6ABA100_CID_MIRROR */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_get_mirror / rj6aba100_set_mirror
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_MIRROR
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_get_mirror(__u8 *value)
{
	*value = rj6aba100->mirror;
	return 0;
}

static inline int rj6aba100_set_mirror(__u8 value)
{
	int ret = 0;

	ret = rj6aba100_setreg_mirror(value);
	if (!ret)
		rj6aba100->mirror = value;
	return ret;
}


/* EMXX_RJ6ABA100_CID_EFFECT_COLOR */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_get_effect_color / rj6aba100_set_effect_color
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_EFFECT_COLOR
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_get_effect_color(__u8 *value)
{
	*value = rj6aba100->efct_color;
	return 0;
}

static inline int rj6aba100_set_effect_color(__u8 value)
{
	int ret = 0;

	ret = rj6aba100_setreg_effect_color(value);
	if (!ret)
		rj6aba100->efct_color = value;
	return ret;
}

/* EMXX_RJ6ABA100_CID_EFFECT_COLOR_MANUAL */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_get_effect_color_userset
 *           / rj6aba100_set_effect_color_userset
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_EFFECT_COLOR_USER_CB
 *          : EMXX_RJ6ABA100_CID_EFFECT_COLOR_USER_CR
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_get_effect_color_userset(__u32 *value)
{
	return rj6aba100_getreg_effect_color_userset(value);
}

static inline int rj6aba100_set_effect_color_userset(__u32 value)
{
	return rj6aba100_setreg_effect_color_userset(value);
}


/* EMXX_RJ6ABA100_CID_EFFECT_NEGATIVE */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_get_effect_negative / rj6aba100_set_effect_negative
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_EFFECT_NEGATIVE
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_get_effect_negative(__u8 *value)
{
	*value = rj6aba100->efct_negative;
	return 0;
}

static inline int rj6aba100_set_effect_negative(__u8 value)
{
	int ret = 0;

	ret = rj6aba100_setreg_effect_negative(value);
	if (!ret)
		rj6aba100->efct_negative = value;
	return ret;
}


/* EMXX_RJ6ABA100_CID_EFFECT_EMBOSS */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_get_effect_emboss / rj6aba100_set_effect_emboss
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_EFFECT_EMBOSS
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_get_effect_emboss(__u8 *value)
{
	*value = rj6aba100->efct_emboss;
	return 0;
}

static inline int rj6aba100_set_effect_emboss(__u8 value)
{
	int ret = 0;

	ret = rj6aba100_setreg_effect_emboss(value);
	if (!ret)
		rj6aba100->efct_emboss = value;
	return ret;
}


/* EMXX_RJ6ABA100_CID_EFFECT_SKETCH */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_get_effect_sketch / rj6aba100_set_effect_sketch
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_EFFECT_SKETCH
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_get_effect_sketch(__u8 *value)
{
	*value = rj6aba100->efct_sketch;
	return 0;
}

static inline int rj6aba100_set_effect_sketch(__u8 value)
{
	int ret = 0;

	ret = rj6aba100_setreg_effect_sketch(value);
	if (!ret)
		rj6aba100->efct_sketch = value;
	return ret;
}


/* EMXX_RJ6ABA100_CID_EFFECT_SKETCH_MANUAL */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_get_effect_sketch_userset
 *           / rj6aba100_set_effect_sketch_userset
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_EFFECT_SKETCH_MANUAL
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_get_effect_sketch_userset(__u32 *value)
{
	return rj6aba100_getreg_effect_sketch_userset(value);
}

static inline int rj6aba100_set_effect_sketch_userset(__u32 value)
{
	return rj6aba100_setreg_effect_sketch_userset(value);
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_start_capture_read
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_start_capture_read(void)
{
	int ret = 0;

	ret = rj6aba100_set_capture_read(rj6aba100->capture_size);

	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ctrl_by_id
 * RETURN   :
 * NOTE     : get rj6aba100_ctrls id
 * UPDATE   :
 ******************************************************************************/
static const struct v4l2_queryctrl *ctrl_by_id(unsigned int id)
{
	unsigned int i;

	for (i = 0; i < NUM_RJ6ABA100_CTRLS; i++)
		if (rj6aba100_ctrls[i].id == id)
			return rj6aba100_ctrls + i;
	return NULL;
}


/* i/f function with emxx_cam.c */

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_vidioc_queryctrl
 * RETURN   :
 * NOTE     : called from emxx_cam.c ioctl(VIDIOC_QUERYCTRL)
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_vidioc_queryctrl(struct file *file,
				       void *fh, struct v4l2_queryctrl *a)
{
	int ret = 0;
	const struct v4l2_queryctrl *ctrl;
	FNC_ENTRY;

	if (a->id < EMXX_RJ6ABA100_CID_FW
	    || a->id > EMXX_RJ6ABA100_CID_EFFECT_SKETCH_MANUAL) {
		ret = -EINVAL;
	} else{
		ctrl = ctrl_by_id(a->id);
		*a = (NULL != ctrl) ? *ctrl : no_ctrl;
	}

	FNC_EXIT(ret)
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_vidioc_querymenu
 * RETURN   :
 * NOTE     : called from emxx_cam.c ioctl(VIDIOC_QUERYMENU)
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_vidioc_querymenu(struct file *file,
				       void *fh, struct v4l2_querymenu *m)
{
	const struct v4l2_queryctrl *ctrl;
	int ret = 0;
	FNC_ENTRY;

	ctrl = ctrl_by_id(m->id);
	if (NULL == ctrl) {
		FNC_EXIT(-EINVAL)
		return -EINVAL;
	}
	switch (ctrl->type) {
	case V4L2_CTRL_TYPE_MENU:
		if (m->index < ctrl->minimum || m->index > ctrl->maximum)
			ret = -EINVAL;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	if (ret) {
		FNC_EXIT(ret)
		return ret;
	}

	mutex_lock(&rj6aba100->lock);
	switch (m->id) {
	case EMXX_RJ6ABA100_CID_OUTIMG_SIZE:
		strcpy(m->name, rj6aba100_outimg_size_menus[m->index].name);
		break;
	case EMXX_RJ6ABA100_CID_OUTIMG_FORMAT:
		strcpy(m->name, rj6aba100_outimg_fmt_menus[m->index].name);
		break;
	case EMXX_RJ6ABA100_CID_OUTDATA_FORMAT:
		strcpy(m->name,
		       rj6aba100_outdata_fmt_menus
		       [rj6aba100->outimg_fmt][m->index].name);
		break;
	case EMXX_RJ6ABA100_CID_FLICKER:
		strcpy(m->name, rj6aba100_flicker_menus[m->index].name);
		break;
#if 0 /* NOT SUPPORT */
	case EMXX_RJ6ABA100_CID_FLICKER_AUTO_DETECTION:
	case EMXX_RJ6ABA100_CID_AE:
	case EMXX_RJ6ABA100_CID_AE_MANUAL:
		ret = -EINVAL;
		break;
#endif
	case EMXX_RJ6ABA100_CID_WB:
		strcpy(m->name, rj6aba100_wb_menus[m->index].name);
		break;
	case EMXX_RJ6ABA100_CID_MIRROR:
		strcpy(m->name, rj6aba100_mirror_menus[m->index].name);
		break;
	case EMXX_RJ6ABA100_CID_EFFECT_COLOR:
		strcpy(m->name, rj6aba100_effect_color_menus[m->index].name);
		break;
	case EMXX_RJ6ABA100_CID_EFFECT_NEGATIVE:
		strcpy(m->name,
		       rj6aba100_effect_negative_menus[m->index].name);
		break;
	case EMXX_RJ6ABA100_CID_EFFECT_EMBOSS:
		strcpy(m->name, rj6aba100_effect_emboss_menus[m->index].name);
		break;
	case EMXX_RJ6ABA100_CID_EFFECT_SKETCH:
		strcpy(m->name, rj6aba100_effect_sketch_menus[m->index].name);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	FNC_EXIT(ret)
	mutex_unlock(&rj6aba100->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_vidioc_g_ctrl
 * RETURN   :
 * NOTE     : called from emxx_cam.c ioctl(VIDIOC_G_CTRL)
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_vidioc_g_ctrl(struct file *file, void *fh,
				    struct v4l2_control *c)
{
	__u32 val = 0;
	int ret = 0;
	const struct v4l2_queryctrl *ctrl;
	FNC_ENTRY;

	ctrl = ctrl_by_id(c->id);
	if (NULL == ctrl) {
		FNC_EXIT(-EINVAL)
		return -EINVAL;
	}
	mutex_lock(&rj6aba100->lock);
	switch (c->id) {
	case EMXX_RJ6ABA100_CID_FW:
		ret = rj6aba100_get_firmware_version((__u8 *)&val);
		break;
	case EMXX_RJ6ABA100_CID_OUTIMG_SIZE:
		ret = rj6aba100_get_outimg_size((__u8 *)&val);
		break;
	case EMXX_RJ6ABA100_CID_OUTIMG_FORMAT:
		ret = rj6aba100_get_outimg_fmt((__u8 *)&val);
		break;
	case EMXX_RJ6ABA100_CID_OUTDATA_FORMAT:
		ret = rj6aba100_get_imgdata_fmt((__u8 *)&val);
		break;
	case EMXX_RJ6ABA100_CID_FLICKER:
		ret = rj6aba100_get_flicker_manual((__u8 *)&val);
		break;
#if 0 /* NOT SUPPORT */
	case EMXX_RJ6ABA100_CID_FLICKER_AUTO_DETECTION:
	case EMXX_RJ6ABA100_CID_AE:
	case EMXX_RJ6ABA100_CID_AE_MANUAL:
		ret = -EINVAL;
		break;
#endif
	case EMXX_RJ6ABA100_CID_WB:
		ret = rj6aba100_get_wb_mode((__u8 *)&val);
		break;
	case EMXX_RJ6ABA100_CID_WB_MANUAL_GAIN:
		ret = rj6aba100_get_wb_manual_gain((__u32 *)&val);
		break;
	case EMXX_RJ6ABA100_CID_BRIGHTNESS:
		ret = rj6aba100_get_brightness((__u8 *)&val);
		break;
	case EMXX_RJ6ABA100_CID_CONTRAST:
		ret = rj6aba100_get_contrast((__u8 *)&val);
		break;
	case EMXX_RJ6ABA100_CID_SHARPNESS:
		ret = rj6aba100_get_sharpness((__u32 *)&val);
		break;
	case EMXX_RJ6ABA100_CID_MIRROR:
		ret = rj6aba100_get_mirror((__u8 *)&val);
		break;
	case EMXX_RJ6ABA100_CID_EFFECT_COLOR:
		ret = rj6aba100_get_effect_color((__u8 *)&val);
		break;
	case EMXX_RJ6ABA100_CID_EFFECT_COLOR_MANUAL:
		ret = rj6aba100_get_effect_color_userset((__u32 *)&val);
		break;
	case EMXX_RJ6ABA100_CID_EFFECT_NEGATIVE:
		ret = rj6aba100_get_effect_negative((__u8 *)&val);
		break;
	case EMXX_RJ6ABA100_CID_EFFECT_EMBOSS:
		ret = rj6aba100_get_effect_emboss((__u8 *)&val);
		break;
	case EMXX_RJ6ABA100_CID_EFFECT_SKETCH:
		ret = rj6aba100_get_effect_sketch((__u8 *)&val);
		break;
	case EMXX_RJ6ABA100_CID_EFFECT_SKETCH_MANUAL:
		ret = rj6aba100_get_effect_sketch_userset((__u32 *)&val);
		break;
	default:
		ret = -EINVAL;
	}

	if (!ret)
		c->value = val;

	FNC_EXIT(ret)
	mutex_unlock(&rj6aba100->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_vidioc_s_ctrl
 * RETURN   :
 * NOTE     : called from emxx_cam.c ioctl(VIDIOC_S_CTRL)
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_vidioc_s_ctrl(struct file *file, void *fh,
				    struct v4l2_control *c)
{
	int ret = 0;
	int moving = (int)fh;
	const struct v4l2_queryctrl *ctrl;
	FNC_ENTRY;

	ctrl = ctrl_by_id(c->id);
	if (NULL == ctrl) {
		FNC_EXIT(-EINVAL)
		return -EINVAL;
	}
	switch (ctrl->type) {
	case V4L2_CTRL_TYPE_BOOLEAN:
	case V4L2_CTRL_TYPE_MENU:
	case V4L2_CTRL_TYPE_INTEGER:
#if 0
		if (c->value < ctrl->minimum)
			c->value = ctrl->minimum;
		if (c->value > ctrl->maximum)
			c->value = ctrl->maximum;
#else
		if (c->id == EMXX_RJ6ABA100_CID_EFFECT_SKETCH_MANUAL) {
			if ((__u32)c->value < (__u32)ctrl->minimum)
				c->value = ctrl->minimum;
			if ((__u32)c->value > (__u32)ctrl->maximum)
				c->value = ctrl->maximum;
		} else{
			if (c->value < ctrl->minimum)
				c->value = ctrl->minimum;
			if (c->value > ctrl->maximum)
				c->value = ctrl->maximum;
		}
#endif
		break;
	default:
		/* nothing */;
	};
	mutex_lock(&rj6aba100->lock);
	switch (c->id) {
#if 0 /* READ ONLY */
	case EMXX_RJ6ABA100_CID_FW:
		break;
#endif
	case EMXX_RJ6ABA100_CID_OUTIMG_SIZE:
		if (moving) {
			warn("RJ6ABA100 : s_ctrl OUT_IMG_RESIZE :"
			     "streaming already exists.\n");
			ret = -EBUSY;
			break;
		}
		ret = rj6aba100_set_outimg_size(c->value);
		break;
	case EMXX_RJ6ABA100_CID_OUTIMG_FORMAT:
		ret = rj6aba100_set_outimg_fmt(c->value);
		break;
	case EMXX_RJ6ABA100_CID_OUTDATA_FORMAT:
		ret = rj6aba100_set_imgdata_fmt(c->value);
		break;
	case EMXX_RJ6ABA100_CID_FLICKER:
		ret = rj6aba100_set_flicker_manual(c->value);
		break;
#if 0 /* NOT SUPPORT */
	case EMXX_RJ6ABA100_CID_FLICKER_AUTO_DETECTION:
	case EMXX_RJ6ABA100_CID_AE:
	case EMXX_RJ6ABA100_CID_AE_MANUAL:
		ret = -EINVAL;
		break;
#endif
	case EMXX_RJ6ABA100_CID_WB:
		ret = rj6aba100_set_wb_mode(c->value);
		break;
	case EMXX_RJ6ABA100_CID_WB_MANUAL_GAIN:
		ret = rj6aba100_set_wb_manual_gain(c->value);
		break;
	case EMXX_RJ6ABA100_CID_BRIGHTNESS:
		ret = rj6aba100_set_brightness(c->value);
		break;
	case EMXX_RJ6ABA100_CID_CONTRAST:
		ret = rj6aba100_set_contrast(c->value);
		break;
	case EMXX_RJ6ABA100_CID_SHARPNESS:
		ret = rj6aba100_set_sharpness(c->value);
		break;
	case EMXX_RJ6ABA100_CID_MIRROR:
		ret = rj6aba100_set_mirror(c->value);
		break;
	case EMXX_RJ6ABA100_CID_EFFECT_COLOR:
		ret = rj6aba100_set_effect_color(c->value);
		break;
	case EMXX_RJ6ABA100_CID_EFFECT_COLOR_MANUAL:
		ret = rj6aba100_set_effect_color_userset(c->value);
		break;
	case EMXX_RJ6ABA100_CID_EFFECT_NEGATIVE:
		ret = rj6aba100_set_effect_negative(c->value);
		break;
	case EMXX_RJ6ABA100_CID_EFFECT_EMBOSS:
		ret = rj6aba100_set_effect_emboss(c->value);
		break;
	case EMXX_RJ6ABA100_CID_EFFECT_SKETCH:
		ret = rj6aba100_set_effect_sketch(c->value);
		break;
	case EMXX_RJ6ABA100_CID_EFFECT_SKETCH_MANUAL:
		ret = rj6aba100_set_effect_sketch_userset(c->value);
		break;
	default:
		ret = -EINVAL;
	}

	FNC_EXIT(ret)
	mutex_unlock(&rj6aba100->lock);
	return ret;
}

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : rj6aba100_set_prepare
 * RETURN   :
 * NOTE     : called from emxx_cam.c
 * UPDATE   :
 ******************************************************************************/
static inline int rj6aba100_set_prepare(struct emxx_cam_prepare *prepare)
{
	FNC_ENTRY;

	prepare->syncmode = 0;/* CAM_HS, CAM_VS mode */
	prepare->synctype = 1;/* Enable signal sampling mode */
	prepare->data_id  = 0;/* U->Y->V->Y */
	prepare->vs_det   = 0;/* rising edge */
	prepare->hs_det   = 0;/* rising edge */
	prepare->clk_edge = 0;/* single edge transfer */
	prepare->data_det = 0;/* rising edge */
	prepare->vs_pol   = 0;/* positive logic */
	prepare->hs_pol   = 0;/* positive logic */

	FNC_EXIT(0)
	return 0;
}

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_sync
 * RETURN   :
 * NOTE     : called from emxx_cam.c
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_sync(struct emxx_cam_prepare *prepare)
{
	int ret = 0;
	__u8 size;
	FNC_ENTRY;
	mutex_lock(&rj6aba100->lock);

	ret = rj6aba100_get_outimg_size(&size);
	if (ret) {
		FNC_EXIT(ret)
		mutex_unlock(&rj6aba100->lock);
		return ret;
	}

	if (BOUNDARY_OUT_IMG_RESIZE < size) {
		struct control_resize_info *s;

		if (EMXX_RJ6ABA100_BREAK == rj6aba100->state) {
			FNC_EXIT(-EIO)
			mutex_unlock(&rj6aba100->lock);
			return -EIO;
		}

		ret = rj6aba100_get_outimg_size(&size);
		if (ret) {
			FNC_EXIT(ret)
			mutex_unlock(&rj6aba100->lock);
			return ret;
		}

		s = (struct control_resize_info *)rj6aba100_outimg_size_menus;
		s += size;

#if X1R_NOT_ZERO
		prepare->bounds.left   = s->x;
		prepare->bounds.top    = s->y;
		prepare->bounds.width  = s->width  - (s->x);
		prepare->bounds.height = s->height - (s->y);

		prepare->c = prepare->bounds;
		prepare->width  = prepare->c.width ;
		prepare->height = prepare->c.height;
#else
		prepare->bounds.left   = 0;
		prepare->bounds.top    = 0;
		prepare->bounds.width  = s->width  - (s->x - 1);
		prepare->bounds.height = s->height - (s->y - 1);

		prepare->c = prepare->bounds;
		prepare->width  = prepare->c.width  - (prepare->c.left - 1);
		prepare->height = prepare->c.height - (prepare->c.top  - 1);

#endif

		if (!prepare->actions) {
			FNC_EXIT(ret)
			mutex_unlock(&rj6aba100->lock);
			return ret;
		}

		prepare->actions = 1;

		#if 0
		ret = rj6aba100_set_liveview(SET_LIVEVIEW_START);
		#endif
	} else {
		prepare->actions = 0;
	}

	FNC_EXIT(ret)
	mutex_unlock(&rj6aba100->lock);
	return ret;
}

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_prepare
 * RETURN   :
 * NOTE     : called from emxx_cam.c
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_prepare(struct emxx_cam_prepare *prepare)
{
	__u8 index;
	int ret = 0;
	FNC_ENTRY;

	mutex_lock(&rj6aba100->lock);

	ret = rj6aba100_get_outimg_size(&index);
	if (!ret) {
		struct control_resize_info *s;

		s = (struct control_resize_info *)rj6aba100_outimg_size_menus;
		s += index;

		#if X1R_NOT_ZERO
		prepare->bounds.left   = s->x;
		prepare->bounds.top    = s->y;
		prepare->bounds.width  = s->width  - (s->x);
		prepare->bounds.height = s->height - (s->y);
		#else
		prepare->bounds.left   = 0;
		prepare->bounds.top    = 0;
		prepare->bounds.width  = s->width  - (s->x - 1);
		prepare->bounds.height = s->height - (s->y - 1);
		#endif

		rj6aba100_set_prepare(prepare);

		prepare->reset = rj6aba100->reset;
		d1b("chk reset 0x%02x\n", rj6aba100->reset);

		if (prepare->actions && !rj6aba100->reset
		    && BOUNDARY_OUT_IMG_RESIZE < index)
			rj6aba100->state = EMXX_RJ6ABA100_IDLE;

		rj6aba100->reset = 0;
	}

	FNC_EXIT(ret)
	mutex_unlock(&rj6aba100->lock);
	return ret;
}

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_trigger
 * RETURN   :
 * NOTE     : called from emxx_cam.c
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_trigger(int flag)
{
	__u8 size;
	int ret = 0;
	FNC_ENTRY;

	mutex_lock(&rj6aba100->lock);

	ret = rj6aba100_get_outimg_size(&size);
	if (ret) {
		FNC_EXIT(ret)
		mutex_unlock(&rj6aba100->lock);
		return ret;
	}

	if (BOUNDARY_OUT_IMG_RESIZE < size) {
		ret = rj6aba100_start_capture_read();
		if (EMXX_RJ6ABA100_BREAK == rj6aba100->state) {
			FNC_EXIT(-EIO)
			mutex_unlock(&rj6aba100->lock);
			return -EIO;
		}
	}

	FNC_EXIT(ret)
	mutex_unlock(&rj6aba100->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_stream_on
 * RETURN   :
 * NOTE     : called from emxx_cam.c ioctl(VIDIOC_STREAMON)
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_stream_on(int flag)
{
	int ret = 0;
	FNC_ENTRY;
	mutex_lock(&rj6aba100->lock);

	ret = rj6aba100_set_liveview(SET_LIVEVIEW_START);
	msleep(500);
#if CAM_DEBUG
	emxx_mage_register_debug();
#endif

	FNC_EXIT(ret)
	mutex_unlock(&rj6aba100->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_stream_off
 * RETURN   :
 * NOTE     : called from emxx_cam.c ioctl(VIDIOC_STREAMOFF)
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_stream_off(int flag)
{
	int ret = 0;
	FNC_ENTRY;
	mutex_lock(&rj6aba100->lock);

	ret = rj6aba100_set_liveview(SET_LIVEVIEW_STOP);
#if CAM_DEBUG
	emxx_mage_register_debug();
#endif

	FNC_EXIT(ret)
	mutex_unlock(&rj6aba100->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_startup
 * RETURN   :
 * NOTE     : initialize RJ6ABA100. called from emxx_cam.c open()
 * UPDATE   :
 ******************************************************************************/
#define END_MARK 0xFF
static int emxx_mega_startup(int flag)
{
	int ret = 0;
	FNC_ENTRY;
	mutex_lock(&rj6aba100->lock);

	i2c_stop = 0;
	rj6aba100->state = EMXX_RJ6ABA100_IDLE;

	/* Reset Camera Module
	 */
	mdelay(100);
	gpio_direction_output(GPIO_CAM_RST, 0x1); /* output High */
	mdelay(10);

	ret = i2c_add_driver(&rj6aba100_i2c_driver);
	d1b("i2c_add_driver(%d, %p)\n", ret, rj6aba100_i2c_client);

	if (ret < 0) {
		err("i2c: Driver registration failed,"
		    "module not inserted.\n");
		goto done_mega_startup;
	} else if (NULL == rj6aba100_i2c_client)    {
		i2c_del_driver(&rj6aba100_i2c_driver);
		err("i2c: Device was not detected.\n");
		ret = -EINVAL;

		goto done_mega_startup;
	} else {

		ret = rj6aba100_set_liveview(SET_LIVEVIEW_STOP);
		if (ret) {
			err("i2c: Failed in writing to ");
			err("the softstandby[B-04:Control_01].\n");
			goto done_mega_startup;
		}

		ret = rj6aba100_setreg_softreset(NUM_B06_SOFTRESET_OFF);
		if (ret) {
			err("i2c: Failed in writing to ");
			err("the softreset[B-06:Softreset].\n");
			goto done_mega_startup;
		}

		ret = rj6aba100_getreg_firmware_version(
			&rj6aba100->firmware_version);
		if (ret) {
			err("i2c: Failed in reading of the version");
			err("information[x-02: RevNumber].\n");
			goto done_mega_startup;
		}
	}

	/* rj6aba100 initialization */
	{
		ret = rj6aba100_setreg_pll_control(688);
		/* ret = rj6aba100_setreg_pll_control(480); */
		if (ret)
			goto done_mega_startup;

		ret = rj6aba100_setreg_outimg_size(rj6aba100->capture_size);
		if (ret)
			goto done_mega_startup;

		ret = rj6aba100_setreg_outimg_fmt(rj6aba100->outimg_fmt);
		if (ret)
			goto done_mega_startup;

		ret = rj6aba100_setreg_pclk_rate();
		if (ret)
			goto done_mega_startup;

		ret = rj6aba100_setreg_outdata_fmt(rj6aba100->imgdata_fmt);
		if (ret)
			goto done_mega_startup;

		ret = rj6aba100_setreg_flicker_manual(
			rj6aba100->flicker_manual);
		if (ret)
			goto done_mega_startup;

		ret = rj6aba100_setreg_wb_manual_gain(
			rj6aba100->wb_manual_gain);
		if (ret)
			goto done_mega_startup;

		ret = rj6aba100_setreg_wb_mode(rj6aba100->wb_mode);
		if (ret)
			goto done_mega_startup;

		ret = rj6aba100_setreg_brightness(rj6aba100->brightness);
		if (ret)
			goto done_mega_startup;

		ret = rj6aba100_setreg_contrast(rj6aba100->contrast);
		if (ret)
			goto done_mega_startup;

		ret = rj6aba100_setreg_sharpness(rj6aba100->sharpness);
		if (ret)
			goto done_mega_startup;

		ret = rj6aba100_setreg_mirror(rj6aba100->mirror);
		if (ret)
			goto done_mega_startup;

		ret = rj6aba100_setreg_effect_color(rj6aba100->efct_color);
		if (ret)
			goto done_mega_startup;

		ret = rj6aba100_setreg_effect_negative(
			rj6aba100->efct_negative);
		if (ret)
			goto done_mega_startup;

		ret = rj6aba100_setreg_effect_emboss(rj6aba100->efct_emboss);
		if (ret)
			goto done_mega_startup;

		ret = rj6aba100_setreg_effect_sketch(rj6aba100->efct_sketch);
		if (ret)
			goto done_mega_startup;

		rj6aba100->state = EMXX_RJ6ABA100_IDLE;

#if CAM_DEBUG
		/* debug only */
		emxx_mage_register_debug();
#endif /* CAM_DEBUG */
	}

	rj6aba100->active = 1;

	FNC_EXIT(ret)
	mutex_unlock(&rj6aba100->lock);
	return ret;

done_mega_startup:

	rj6aba100_shutdown();
	if (rj6aba100_i2c_client)
		i2c_del_driver(&rj6aba100_i2c_driver);
	rj6aba100_i2c_client = NULL;
	rj6aba100->active = 0;

	FNC_EXIT(ret)
	mutex_unlock(&rj6aba100->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_shutdown
 * RETURN   :
 * NOTE     : uninitialize RJ6ABA100. called from emxx_cam.c close()
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_shutdown(int flag)
{
	int ret = 0;
	FNC_ENTRY;
	mutex_lock(&rj6aba100->lock);

	ret = rj6aba100_shutdown();

	if (rj6aba100_i2c_client)
		i2c_del_driver(&rj6aba100_i2c_driver);
	/* rj6aba100_i2c_client = NULL; */

	FNC_EXIT(ret)
	mutex_unlock(&rj6aba100->lock);
	return 0;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_unregister
 * RETURN   :
 * NOTE     : unregist RJ6ABA100. called from emxx_cam.c shutdown
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_unregister(int flag)
{
	int ret = 0;
	FNC_ENTRY;

	kfree(rj6aba100);
	rj6aba100 = NULL;

	FNC_EXIT(ret)
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_cam_hw_register
 * RETURN   :
 * NOTE     : regist RJ6ABA100. called from emxx_cam.c startup
 * UPDATE   :
 ******************************************************************************/
int emxx_cam_hw_register(struct emxx_cam_hw_operations *hw)
{
	int ret = 0;
	const struct v4l2_queryctrl *ctrl;
	FNC_ENTRY;

	rj6aba100 = kzalloc(sizeof(*rj6aba100), GFP_KERNEL);

	if (rj6aba100 == NULL) {
		err("RJ6ABA100: rj6aba100 allocation failed!\n");
		FNC_EXIT(-ENOMEM)
		return -ENOMEM;
	}

	strlcpy(hw->name, "EMXX: RJ6ABA100 Camera", sizeof(hw->name));

	hw->vidioc_queryctrl = emxx_mega_vidioc_queryctrl;
	hw->vidioc_querymenu = emxx_mega_vidioc_querymenu;
	hw->vidioc_g_ctrl    = emxx_mega_vidioc_g_ctrl;
	hw->vidioc_s_ctrl    = emxx_mega_vidioc_s_ctrl;
	hw->prepare          = emxx_mega_prepare;
	hw->trigger          = emxx_mega_trigger;
	hw->sync             = emxx_mega_sync;
	hw->stream_on        = emxx_mega_stream_on;
	hw->stream_off       = emxx_mega_stream_off;
	hw->startup          = emxx_mega_startup;
	hw->shutdown         = emxx_mega_shutdown;
	hw->unregister       = emxx_mega_unregister;

	/* initial rj6aba100 */
	mutex_init(&rj6aba100->lock);

	rj6aba100->firmware_version = 0xff;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_OUTIMG_SIZE);
	rj6aba100->capture_size  = ctrl->default_value;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_OUTIMG_FORMAT);
	rj6aba100->outimg_fmt = ctrl->default_value;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_OUTDATA_FORMAT);
	rj6aba100->imgdata_fmt = ctrl->default_value;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_FLICKER);
	rj6aba100->flicker_manual = ctrl->default_value;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_WB_MANUAL_GAIN);
	rj6aba100->wb_manual_gain = ctrl->default_value;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_WB);
	rj6aba100->wb_mode = ctrl->default_value;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_BRIGHTNESS);
	rj6aba100->brightness = ctrl->default_value;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_CONTRAST);
	rj6aba100->contrast = ctrl->default_value;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_SHARPNESS);
	rj6aba100->sharpness = ctrl->default_value;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_MIRROR);
	rj6aba100->efct_color = ctrl->default_value;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_EFFECT_COLOR);
	rj6aba100->efct_color = ctrl->default_value;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_EFFECT_NEGATIVE);
	rj6aba100->efct_negative = ctrl->default_value;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_EFFECT_EMBOSS);
	rj6aba100->efct_emboss = ctrl->default_value;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_EFFECT_SKETCH);
	rj6aba100->efct_sketch = ctrl->default_value;

	hw->private = rj6aba100;

	FNC_EXIT(ret)
	return ret;
}
EXPORT_SYMBOL(emxx_cam_hw_register);


