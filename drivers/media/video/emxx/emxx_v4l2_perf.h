/*
 * File Name       : drivers/media/video/emxx/emxx_v4l2_perf.h
 * Function        : Video for Linux driver for EM/EV driver core
 * Release Version : Ver 1.00
 * Release Date    : 2010.03.05
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


#ifndef EMXX_V4L2_PERF__H
#define EMXX_V4L2_PERF__H


#if _PERF1_DBG

#define PERF_LOG_MAX		0x500
#define PERF_LOG_CNT_INIT	0
#define PERF_LOG_INIT		NULL
/*******************************************************/
enum StreamPhase{
	PHS_ROT,
	PHS_TMR,
	PHS_LCD,
};

struct PerfLog{
	unsigned int     sequence;
	enum StreamPhase phase;
	struct timeval   now_time;
};


/*******************************************************/
static int    perf_log_max = PERF_LOG_MAX;
static int    perf_log_cnt = PERF_LOG_CNT_INIT;

static struct PerfLog	*perf_log = PERF_LOG_INIT;

/*******************************************************/
void init_timelog(void);
void set_timelog(unsigned int sequence, enum StreamPhase phase);
void output_timelog(void);

/*******************************************************/
/* --- init perf log buffer ------------------------------- */
void init_timelog()
{
	if (perf_log == NULL) {
		perf_log = kmalloc(sizeof(struct PerfLog) * perf_log_max,
		 GFP_KERNEL);
	}
	memset(perf_log, 0, sizeof(struct PerfLog) * perf_log_max);
	perf_log_cnt = 0;
}

void set_timelog(unsigned int sequence, enum StreamPhase phase)
{
	if (perf_log) {
		perf_log[perf_log_cnt].sequence  = sequence;
		perf_log[perf_log_cnt].phase     = phase;
		do_gettimeofday(&perf_log[perf_log_cnt].now_time);
		perf_log_cnt++;
		if (perf_log_cnt == perf_log_max)
			perf_log_cnt = 0;
	}
}

void output_timelog(void)
{
	int cnt, log_cnt, cnt_sps, cnt_phs;
	unsigned int old_sequence = 0;

	printk(KERN_INFO "SEQUENCE,,ROT,,TMR,,LCD\n");
	for (cnt_phs = 0; cnt_phs <= PHS_LCD; cnt_phs++) {
		old_sequence = 0;
		for (cnt = 1; cnt <= perf_log_max; cnt++) {
			if (perf_log_cnt - cnt >= 0)
				log_cnt = perf_log_cnt - cnt;
			else
				log_cnt = perf_log_max + (perf_log_cnt - cnt);

			if (cnt_phs == perf_log[log_cnt].phase) {
				if (perf_log[log_cnt].sequence !=
				    old_sequence) {
					old_sequence =
					 perf_log[log_cnt].sequence;
					printk(KERN_INFO "\n");
				}

				printk(KERN_INFO "%d,",
				 perf_log[log_cnt].sequence);
				for (cnt_sps = 0;
				     cnt_sps < perf_log[log_cnt].phase;
				     cnt_sps++)
					printk(",,");
				printk(KERN_INFO "%ld,%ld,",
				 perf_log[log_cnt].now_time.tv_sec,
				 perf_log[log_cnt].now_time.tv_usec);
				for (cnt_sps = perf_log[log_cnt].phase;
				     cnt_sps < PHS_LCD; cnt_sps++)
					printk(",,");
				printk(KERN_INFO "\n");
			}
		}
		printk(KERN_INFO "\n");
	}
}

#define perf_clr() \
	do { \
		init_timelog(); \
	} while (0)

#define perf_add(seq, phase) \
	do { \
		set_timelog(seq, phase); \
	} while (0)

#define perf_add_th(seq) \
	do { \
		if (th_obj->th_idle_fixed == STATE_QUEUED_ROT_PREPARED) \
			set_timelog(seq, PHS_ROT); \
	} while (0)

#define perf_out() \
	do { \
		output_timelog(); \
		init_timelog();   \
	} while (0)
#else  /* _PERF1_DBG */
#define perf_clr()
#define perf_add(seq, phase)
#define perf_add_th(seq)
#define perf_out()
#endif /* _PERF1_DBG */




#if _PERF2_DBG
#define MICRO_SEC 1000000

int perf_interval = 5;
long delta_time, delta_max, delta_min, delta_sum;
unsigned long disp_frame;
struct timeval  chk_time;


struct timeval2{
	unsigned long h;
	unsigned long m;
	unsigned long s;
	unsigned long ms;
	unsigned long us;
};


inline void conv_timeval2(struct timeval *val, struct timeval2 *val_d)
{
	val_d->h  = val->tv_sec  / 3600;
	val_d->m  = val->tv_sec  /   60 - val_d->h *   60;
	val_d->s  = val->tv_sec         - val_d->h * 3600 - val_d->m * 60;
	val_d->ms = val->tv_usec / 1000;
	val_d->us = val->tv_usec        - val_d->ms * 1000;
}


void init_disptime(struct emxx_v4l2_device *dev)
{
	if (dev->vb_old == 0) {
		struct timeval2 now_time;
		delta_time = 0;
		delta_max  = 0;
		delta_min  = 0;
		delta_sum  = 0;
		disp_frame = 0;
		do_gettimeofday(&chk_time);
		conv_timeval2(&chk_time, &now_time);
		printk(KERN_INFO
		 "#### NOW  ,%2ld,h,%2ld,m,%2ld,s,%3ld,ms,%3ld,us\n",
		 now_time.h, now_time.m, now_time.s, now_time.ms, now_time.us);
	}
}


void deinit_disptime(struct emxx_v4l2_device *dev)
{
	if (dev->vb_old != 0) {
		struct timeval  end_time;
		struct timeval2 now_time;
		do_gettimeofday(&end_time);
		conv_timeval2(&end_time, &now_time);
		printk(KERN_INFO
		 "#### NOW  ,%2ld,h,%2ld,m,%2ld,s,%3ld,ms,%3ld,us,,",
		 now_time.h, now_time.m, now_time.s, now_time.ms, now_time.us);
		printk(KERN_INFO "#### FRAME,%ld,,", disp_frame);
		printk(KERN_INFO
		 "#### DELTA,MAX/MIN(,%ld,%3d,%3d,/,%ld,%3d,%3d,)\n",
		 delta_max/1000000, abs(delta_max%1000000)/1000,
		 abs(delta_max%1000), delta_min/1000000,
		 abs(delta_min%1000000)/1000, abs(delta_min%1000));
		delta_time = 0;
		delta_max  = 0;
		delta_min  = 0;
		delta_sum  = 0;
		disp_frame = 0;
	}
}


void calc_disptime(struct emxx_v4l2_device *dev, struct videobuf_buffer *vb)
{
	long sec, usec, delta;
	struct timeval *start_time, *end_time;

	if (dev->vb_old == 0) {
		init_disptime(dev);
	} else {
		start_time = &vb->ts;
		end_time   = &dev->timer.comp_time;
		if (start_time->tv_sec != -1 && start_time->tv_sec != -1) {
			delta_time =
			 (end_time->tv_sec - start_time->tv_sec) * MICRO_SEC +
			 (end_time->tv_usec - start_time->tv_usec);
			delta_max  = delta_time > delta_max ?
			 delta_time : delta_max;
			delta_min  = delta_time < delta_min ?
			 delta_time : delta_min;
			delta_sum += delta_time;
		}
		disp_frame++;

		/* output */
		sec   = end_time->tv_sec  - chk_time.tv_sec;
		usec  = end_time->tv_usec - chk_time.tv_usec;
		delta = (sec * 1000) + (usec / 1000 - usec % 1000);
		if (delta >= perf_interval * 60 * 1000) {
			struct timeval2 now_time;
			conv_timeval2(end_time, &now_time);
			printk(KERN_INFO
			 "#### NOW  ,%2ld,h,%2ld,m,%2ld,s,%3ld,ms,%3ld,us,,",
			 now_time.h, now_time.m, now_time.s, now_time.ms,
			 now_time.us);
			printk(KERN_INFO "#### FRAME,%ld,,", disp_frame);
			printk(KERN_INFO
			 "#### DELTA,MAX/MIN(,%ld,%3d,%3d,/,%ld,%3d,%3d,)\n",
			 delta_max/1000000, abs(delta_max%1000000)/1000,
			 abs(delta_max%1000), delta_min/1000000,
			 abs(delta_min%1000000)/1000, abs(delta_min%1000));
			disp_frame = 0;
			memcpy(&chk_time, end_time, sizeof(struct timeval));
		}
	}
}

#define Perf2Init(dev) \
	do { \
		inti_disptime(dev); \
	} while (0)

#define Perf2Deinit(dev) \
	do { \
		deinit_disptime(dev); \
	} while (0)

#define Perf2Calc(dev, vb) \
	do { \
		calc_disptime(dev, vb); \
	} while (0)
#else  /* _PERF2_DBG */
#define Perf2Init(dev)
#define Perf2Deinit(dev)
#define calc_perf(dev, vb)
#endif /* _PERF2_DBG */


#endif /* EMXX_V4L2_PERF__H */
