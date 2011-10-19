/*
 * drivers/video/emxx/emxx_lcd_perf.h
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

#define LCD_PERF  0

#if LCD_PERF
#define LOG_MAX		0x500
#define STAR_PERF_INIT	0
#define CNT_TIME_INIT	0
/*******************************************************/
enum Sequence{
	SEQ_FB,
	SEQ_V4L2,
	SEQ_IMC,
	SEQ_VSYNC,
};

struct time_log{
	enum Sequence	seq;
	struct timeval	now_time;
};

/*******************************************************/
static int		start_perf = START_PERF_INIT;
static unsigned long	cnt_time = CNT_TIME_INIT;
static struct time_log	*logtime;

/*******************************************************/
void map_nowtime(void);
void set_nowtime(enum Sequence seq);

/*******************************************************/
void map_nowtime()
{
	logtime = (struct time_log *)ioremap_nocache(
		  NTSC_FRAME_BUFFER_ADDR + NTSC_FRAME_BUFFER_SIZE / 2,
		  (sizeof(struct time_log) * LOG_MAX));
}

void set_nowtime(enum Sequence seq)
{
	int	cnt_perf, cnt_sps, cnt_seq, i = 0;

	if (start_perf == 1) {
		if (cnt_time != LOG_MAX) {
			logtime[cnt_time].seq = seq;
			do_gettimeofday(&logtime[cnt_time].now_time);
			cnt_time++;
		} else {
			start_perf = 2;
			printk(KERN_INFO ",FB,,V4L2,,IMC,,VSYNC\n");
			for (cnt_seq = 0; cnt_seq <= SEQ_VSYNC; cnt_seq++) {
				for (cnt_perf = 0; cnt_perf < LOG_MAX;
				     cnt_perf++) {
					if (cnt_seq == logtime[cnt_perf].seq) {
						printk(KERN_INFO "%d,",
						 logtime[cnt_perf].seq);
						for (cnt_sps = 0;
						     cnt_sps <
						     logtime[cnt_perf].seq;
						     cnt_sps++)
							printk(KERN_INFO ",,");
						printk(KERN_INFO "%ld,%ld,",
						 logtime[cnt_perf].
						 now_time.tv_sec,
						 logtime[cnt_perf].
						 now_time.tv_usec);
						for (cnt_sps =
						     logtime[cnt_perf].seq;
						     cnt_sps < SEQ_VSYNC;
						     cnt_sps++)
							printk(KERN_INFO ",,");
						printk(KERN_INFO "\n");

						if (cnt_seq == SEQ_VSYNC)
							printk(KERN_INFO "\n");
						else if (i % 2)
							printk(KERN_INFO "\n");
						i++;
					}
				}
				printk(KERN_INFO "\n");
			}
		}
	}
}
#else
#endif
