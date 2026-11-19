// SPDX-License-Identifier: GPL-2.0-only
/*
 * wm_sdr_remap.c - WM memory access through the connsys L2 remap window.
 *
 * Window protocol (see mt_wifi pci_io_remap_l2_read32/write32):
 *   control 0x184001b8: [31:16] page, [15:0] preserved
 *   data    0x18440000 + (addr & 0xffff)
 *
 * The window is shared with mt_wifi when claim_pci=0, so every access keeps
 * the vendor's low 16 bits and restores the previous page.  A bulk transfer
 * switches page once per 64 KiB and restores after each page; concurrent
 * mt_wifi remap users can still race, therefore bulk reads are meant for
 * quiescent periods.
 */
#include <linux/module.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>

#include "wm_sdr.h"

static inline u32 wmsdr_page_of(u32 addr)
{
	return addr & WMSDR_REMAP_PAGE_MASK;
}

u32 wmsdr_direct_read32(struct wmsdr_dev *w, u32 off)
{
	return readl(w->bar + off);
}

void wmsdr_direct_write32(struct wmsdr_dev *w, u32 off, u32 val)
{
	writel(val, w->bar + off);
}

u32 wmsdr_fw_state(struct wmsdr_dev *w)
{
	u32 v = readl(w->bar + WMSDR_FW_STAT);

	return v & WMSDR_FW_STAT_MASK;
}

bool wmsdr_addr_ok(u32 addr, u32 len)
{
	u64 end = (u64)addr + len;

	if (len == 0 || end > 0x100000000ULL)
		return false;

	if (addr >= 0x00400000UL && end <= 0x00420000UL)
		return true;
	if (addr >= WMSDR_WM_ROM_LO && end <= WMSDR_WM_ROM_HI)
		return true;
	if (addr >= WMSDR_WM_RAM_LO && end <= WMSDR_WM_RAM_HI)
		return true;
	if (addr >= WMSDR_WM_ICAP_LO && end <= WMSDR_WM_ICAP_HI)
		return true;
	if (addr >= 0x88009000UL && end <= 0x8800A000UL)
		return true;
	if (addr >= 0xE0000000UL && end <= 0xE0300000UL)
		return true;
	if (addr >= WMSDR_WM_EMI_LO && end <= WMSDR_WM_EMI_HI)
		return true;

	return false;
}

u32 wmsdr_remap_read32(struct wmsdr_dev *w, u32 addr)
{
	void __iomem *ctl = w->bar + WMSDR_REMAP_CTL;
	void __iomem *win = w->bar + WMSDR_REMAP_WIN;
	u32 bak, tmp, val;

	mutex_lock(&w->lock);
	bak = readl(ctl);
	tmp = (bak & WMSDR_REMAP_LOW_MASK) | wmsdr_page_of(addr);
	writel(tmp, ctl);
	(void)readl(ctl);		/* push the remap write */
	val = readl(win + (addr & WMSDR_REMAP_LOW_MASK));
	writel(bak, ctl);
	mutex_unlock(&w->lock);

	return val;
}

void wmsdr_remap_write32(struct wmsdr_dev *w, u32 addr, u32 val)
{
	void __iomem *ctl = w->bar + WMSDR_REMAP_CTL;
	void __iomem *win = w->bar + WMSDR_REMAP_WIN;
	u32 bak, tmp;

	mutex_lock(&w->lock);
	bak = readl(ctl);
	tmp = (bak & WMSDR_REMAP_LOW_MASK) | wmsdr_page_of(addr);
	writel(tmp, ctl);
	(void)readl(ctl);
	writel(val, win + (addr & WMSDR_REMAP_LOW_MASK));
	writel(bak, ctl);
	mutex_unlock(&w->lock);
}

/*
 * Bulk read into a kernel buffer.  One remap switch per 64 KiB page.
 * Unaligned addresses/lengths are handled byte-wise: the connsys window
 * returns an alignment fault for unaligned 32-bit loads.
 * Caller must not hold w->lock.  Returns 0 or a negative errno.
 */
int wmsdr_remap_read(struct wmsdr_dev *w, u32 addr, void *buf, u32 len)
{
	void __iomem *ctl = w->bar + WMSDR_REMAP_CTL;
	void __iomem *win = w->bar + WMSDR_REMAP_WIN;
	u8 *out = buf;
	bool bytes = (addr | len) & 3;

	if (!wmsdr_addr_ok(addr, len))
		return -EINVAL;

	while (len) {
		u32 page = wmsdr_page_of(addr);
		u32 off = addr & WMSDR_REMAP_LOW_MASK;
		u32 chunk = min_t(u32, len, 0x10000 - off);
		u32 bak, tmp, i;

		mutex_lock(&w->lock);
		bak = readl(ctl);
		tmp = (bak & WMSDR_REMAP_LOW_MASK) | page;
		writel(tmp, ctl);
		(void)readl(ctl);
		if (bytes) {
			for (i = 0; i < chunk; i++)
				out[i] = readb(win + off + i);
		} else {
			for (i = 0; i < chunk; i += 4)
				*(u32 *)(out + i) = readl(win + off + i);
		}
		writel(bak, ctl);
		mutex_unlock(&w->lock);

		addr += chunk;
		out += chunk;
		len -= chunk;
	}

	return 0;
}

int wmsdr_remap_write(struct wmsdr_dev *w, u32 addr, const void *buf, u32 len)
{
	void __iomem *ctl = w->bar + WMSDR_REMAP_CTL;
	void __iomem *win = w->bar + WMSDR_REMAP_WIN;
	const u8 *in = buf;
	bool bytes = (addr | len) & 3;

	if (!wmsdr_addr_ok(addr, len))
		return -EINVAL;

	while (len) {
		u32 page = wmsdr_page_of(addr);
		u32 off = addr & WMSDR_REMAP_LOW_MASK;
		u32 chunk = min_t(u32, len, 0x10000 - off);
		u32 bak, tmp, i;

		mutex_lock(&w->lock);
		bak = readl(ctl);
		tmp = (bak & WMSDR_REMAP_LOW_MASK) | page;
		writel(tmp, ctl);
		(void)readl(ctl);
		if (bytes) {
			for (i = 0; i < chunk; i++)
				writeb(in[i], win + off + i);
		} else {
			for (i = 0; i < chunk; i += 4)
				writel(*(const u32 *)(in + i), win + off + i);
		}
		writel(bak, ctl);
		mutex_unlock(&w->lock);

		addr += chunk;
		in += chunk;
		len -= chunk;
	}

	return 0;
}
