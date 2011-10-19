/*
 * arch/arm/mm/cache-l2x0.c - L210/L220 cache controller support
 *
 * Copyright (C) 2010 Renesas Electronics Corporation
 *
 * Copyright (C) 2007 ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/io.h>

#include <asm/cacheflush.h>
#include <asm/hardware/cache-l2x0.h>

#define CACHE_LINE_SIZE		32

static void __iomem *l2x0_base;
bool l2x0_disabled;

static inline void cache_wait_always(void __iomem *reg, unsigned long mask)
{
	/* wait for the operation to complete */
	while (readl(reg) & mask)
		;
}

#ifdef CONFIG_CACHE_PL310

static inline void cache_wait(void __iomem *reg, unsigned long mask)
{
	/* cache operations are atomic */
}

#define _l2x0_lock(lock, flags)		((void)(flags))
#define _l2x0_unlock(lock, flags)	((void)(flags))

#define block_end(start, end)		(end)

#define L2CC_TYPE			"PL310/L2C-310"

#else	/* !CONFIG_CACHE_PL310 */

#define cache_wait			cache_wait_always

static DEFINE_SPINLOCK(l2x0_lock);
#define _l2x0_lock(lock, flags)		spin_lock_irqsave(lock, flags)
#define _l2x0_unlock(lock, flags)	spin_unlock_irqrestore(lock, flags)

#define block_end(start, end)		((start) + min((end) - (start), 4096UL))

#define L2CC_TYPE			"L2x0"

#endif	/* CONFIG_CACHE_PL310 */

static inline void cache_sync(void)
{
	void __iomem *base = l2x0_base;
	writel(0, base + L2X0_CACHE_SYNC);
	cache_wait(base + L2X0_CACHE_SYNC, 1);
}

static inline void l2x0_clean_line(unsigned long addr)
{
	void __iomem *base = l2x0_base;
	cache_wait(base + L2X0_CLEAN_LINE_PA, 1);
	writel(addr, base + L2X0_CLEAN_LINE_PA);
}

static inline void l2x0_inv_line(unsigned long addr)
{
	void __iomem *base = l2x0_base;
	cache_wait(base + L2X0_INV_LINE_PA, 1);
	writel(addr, base + L2X0_INV_LINE_PA);
}

static inline void l2x0_flush_line(unsigned long addr)
{
	void __iomem *base = l2x0_base;
	cache_wait(base + L2X0_CLEAN_INV_LINE_PA, 1);
	writel(addr, base + L2X0_CLEAN_INV_LINE_PA);
}

static void l2x0_cache_sync(void)
{
	unsigned long flags;

	_l2x0_lock(&l2x0_lock, flags);
	cache_sync();
	_l2x0_unlock(&l2x0_lock, flags);
}

static inline void l2x0_inv_all(void)
{
	unsigned long flags;

	/* invalidate all ways */
	_l2x0_lock(&l2x0_lock, flags);
#ifndef CONFIG_EMXX_L310_NORAM
#ifdef CONFIG_EMXX_L310_16WAY
	writel(0xffff, l2x0_base + L2X0_INV_WAY);
	cache_wait_always(l2x0_base + L2X0_INV_WAY, 0xffff);
#else
	writel(0xff, l2x0_base + L2X0_INV_WAY);
	cache_wait_always(l2x0_base + L2X0_INV_WAY, 0xff);
#endif
#endif
	cache_sync();
	_l2x0_unlock(&l2x0_lock, flags);
}

static void l2x0_flush_all(void)
{
	unsigned long flags;
#ifdef CONFIG_PL310_ERRATA_727915
	__u32 debug_ctrl;
#endif

	/* invalidate all ways */
	_l2x0_lock(&l2x0_lock, flags);
#ifndef CONFIG_EMXX_L310_NORAM
#ifdef CONFIG_PL310_ERRATA_727915
	debug_ctrl = readl(l2x0_base + L2X0_DEBUG_CTRL);
	writel(debug_ctrl | 0x3, l2x0_base + L2X0_DEBUG_CTRL);
#endif
#ifdef CONFIG_EMXX_L310_16WAY
	writel(0xffff, l2x0_base + L2X0_CLEAN_INV_WAY);
	cache_wait_always(l2x0_base + L2X0_CLEAN_INV_WAY, 0xffff);
#else
	writel(0xff, l2x0_base + L2X0_CLEAN_INV_WAY);
	cache_wait_always(l2x0_base + L2X0_CLEAN_INV_WAY, 0xff);
#endif
#ifdef CONFIG_PL310_ERRATA_727915
	writel(debug_ctrl, l2x0_base + L2X0_DEBUG_CTRL);
#endif
#endif
	cache_sync();
	_l2x0_unlock(&l2x0_lock, flags);
}

static void l2x0_inv_range(unsigned long start, unsigned long end)
{
#ifndef CONFIG_EMXX_L310_NORAM
	void __iomem *base = l2x0_base;
#endif
	unsigned long flags;

	_l2x0_lock(&l2x0_lock, flags);
#ifndef CONFIG_EMXX_L310_NORAM
	if (start & (CACHE_LINE_SIZE - 1)) {
		start &= ~(CACHE_LINE_SIZE - 1);
		l2x0_flush_line(start);
		start += CACHE_LINE_SIZE;
	}

	if (end & (CACHE_LINE_SIZE - 1)) {
		end &= ~(CACHE_LINE_SIZE - 1);
		l2x0_flush_line(end);
	}

	while (start < end) {
		unsigned long blk_end = block_end(start, end);

		while (start < blk_end) {
			l2x0_inv_line(start);
			start += CACHE_LINE_SIZE;
		}

		if (blk_end < end) {
			_l2x0_unlock(&l2x0_lock, flags);
			_l2x0_lock(&l2x0_lock, flags);
		}
	}
	cache_wait(base + L2X0_INV_LINE_PA, 1);
#endif
	cache_sync();
	_l2x0_unlock(&l2x0_lock, flags);
}

static void l2x0_clean_range(unsigned long start, unsigned long end)
{
#ifndef CONFIG_EMXX_L310_NORAM
	void __iomem *base = l2x0_base;
#endif
	unsigned long flags;

	_l2x0_lock(&l2x0_lock, flags);
#ifndef CONFIG_EMXX_L310_NORAM
	start &= ~(CACHE_LINE_SIZE - 1);
	while (start < end) {
		unsigned long blk_end = block_end(start, end);

		while (start < blk_end) {
			l2x0_clean_line(start);
			start += CACHE_LINE_SIZE;
		}

		if (blk_end < end) {
			_l2x0_unlock(&l2x0_lock, flags);
			_l2x0_lock(&l2x0_lock, flags);
		}
	}
	cache_wait(base + L2X0_CLEAN_LINE_PA, 1);
#endif
	cache_sync();
	_l2x0_unlock(&l2x0_lock, flags);
}

static void l2x0_flush_range(unsigned long start, unsigned long end)
{
#ifndef CONFIG_EMXX_L310_NORAM
	void __iomem *base = l2x0_base;
#endif
	unsigned long flags;

	_l2x0_lock(&l2x0_lock, flags);
#ifndef CONFIG_EMXX_L310_NORAM
	start &= ~(CACHE_LINE_SIZE - 1);
	while (start < end) {
		unsigned long blk_end = block_end(start, end);

		while (start < blk_end) {
			l2x0_flush_line(start);
			start += CACHE_LINE_SIZE;
		}

		if (blk_end < end) {
			_l2x0_unlock(&l2x0_lock, flags);
			_l2x0_lock(&l2x0_lock, flags);
		}
	}
	cache_wait(base + L2X0_CLEAN_INV_LINE_PA, 1);
#endif
	cache_sync();
	_l2x0_unlock(&l2x0_lock, flags);
}

void __init l2x0_init(void __iomem *base, __u32 aux_val, __u32 aux_mask)
{
	__u32 aux;

	if (l2x0_disabled) {
		printk(KERN_INFO "L2X0 cache controller disabled\n");
		return;
	}

	l2x0_base = base;

	/*
	 * Check if l2x0 controller is already enabled.
	 * If you are booting from non-secure mode
	 * accessing the below registers will fault.
	 */
	if (!(readl(l2x0_base + L2X0_CTRL) & 1)) {
		/* l2x0 controller is disabled */
		aux = readl(l2x0_base + L2X0_AUX_CTRL);
		aux &= aux_mask;
		aux |= aux_val;
		writel(aux, l2x0_base + L2X0_AUX_CTRL);

		l2x0_inv_all();

		/* enable L2X0 */
		writel(1, l2x0_base + L2X0_CTRL);
	}

	outer_cache.inv_range = l2x0_inv_range;
	outer_cache.clean_range = l2x0_clean_range;
	outer_cache.flush_range = l2x0_flush_range;
	outer_cache.flush_all = l2x0_flush_all;
	outer_cache.sync = l2x0_cache_sync;

	pr_info(L2CC_TYPE " cache controller enabled\n");
}

static int __init l2x0_disable(char *unused)
{
	l2x0_disabled = 1;
	return 0;
}
early_param("nol2x0", l2x0_disable);

#ifdef CONFIG_MACH_EMEV
static inline void l2x0_clean_all(void)
{
	/* invalidate all ways */
#ifdef CONFIG_EMXX_L310_16WAY
	writel(0xffff, l2x0_base + L2X0_CLEAN_WAY);
	cache_wait_always(l2x0_base + L2X0_CLEAN_WAY, 0xffff);
#else
	writel(0xff, l2x0_base + L2X0_CLEAN_WAY);
	cache_wait_always(l2x0_base + L2X0_CLEAN_WAY, 0xff);
#endif
	cache_sync();
}

void l2x0_suspend(void)
{
	if (l2x0_base != NULL) {
		if (readl(l2x0_base + L2X0_CTRL) & 1) {
			/* disable L2X0 */
			flush_cache_all();
			asm("dsb");
			writel(0, l2x0_base + L2X0_CTRL);
			l2x0_clean_all();
		}
	}
}

void l2x0_resume(void)
{
	if (l2x0_base != NULL) {
		if (!(readl(l2x0_base + L2X0_CTRL) & 1)) {
			/* enable L2X0 */
			l2x0_inv_all();
			writel(1, l2x0_base + L2X0_CTRL);
		}
	}
}
#endif

