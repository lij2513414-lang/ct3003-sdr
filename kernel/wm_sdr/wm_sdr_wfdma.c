// SPDX-License-Identifier: GPL-2.0-only
/*
 * wm_sdr_wfdma.c - Hardware-accelerated reads via MCU_DMA0 TX3 -> HOST_DMA0 RX3.
 *
 * Uses the idle event-pipe pair MCU_DMA0 TX3 (0x54000330, BAR0 + 0x402330)
 * feeding HOST_DMA0 RX3 (0x18024530, BAR0 + 0x024530) to DMA directly from
 * WM/UMAC memory. Descriptor rings live in uncached conninfra EMI (0xF00B0000,
 * PA emi_start + 0xB0000), while RX payload buffers target a contiguous 512 KiB
 * cacheable page block (with 4 KiB user header headroom at offset 0x0..0xfff)
 * synced via DMA API and optionally mmap()'d directly to userspace.
 */
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/sched.h>

#include "wm_sdr.h"

static bool wfdma_enable = true;
module_param(wfdma_enable, bool, 0444);
MODULE_PARM_DESC(wfdma_enable, "Enable MCU_DMA0 TX3 -> HOST_DMA0 RX3 read path");

#define WMSDR_EMI_SLICE_OFF	0x000B0000U
#define WMSDR_EMI_SLICE_SIZE	0x00050000U
#define WMSDR_WM_EMI_BASE	0xF0000000U

#define WMSDR_MCU_DMA0_BASE	0x402000UL	/* WM 0x54000000 */
#define WMSDR_MCU_INT_STA	(WMSDR_MCU_DMA0_BASE + 0x200UL)
#define WMSDR_MCU_GLO_CFG	(WMSDR_MCU_DMA0_BASE + 0x208UL)
#define WMSDR_MCU_RST_DTX	(WMSDR_MCU_DMA0_BASE + 0x20CUL)
#define WMSDR_MCU_TX3		(WMSDR_MCU_DMA0_BASE + 0x330UL)
#define WMSDR_MCU_TX3_EXT	(WMSDR_MCU_DMA0_BASE + 0x60CUL)

#define WMSDR_HOST_DMA0_BASE	0x024000UL	/* AP 0x18024000 */
#define WMSDR_HOST_INT_STA	(WMSDR_HOST_DMA0_BASE + 0x200UL)
#define WMSDR_HOST_INT_ENA	(WMSDR_HOST_DMA0_BASE + 0x204UL)
#define WMSDR_HOST_GLO_CFG	(WMSDR_HOST_DMA0_BASE + 0x208UL)
#define WMSDR_HOST_INT_ENA_CLR	(WMSDR_HOST_DMA0_BASE + 0x22CUL)
#define WMSDR_HOST_RST_DRX	(WMSDR_HOST_DMA0_BASE + 0x280UL)
#define WMSDR_HOST_RX_PRI_SEL	(WMSDR_HOST_DMA0_BASE + 0x298UL)
#define WMSDR_HOST_RX3		(WMSDR_HOST_DMA0_BASE + 0x530UL)
#define WMSDR_HOST_RX3_EXT	(WMSDR_HOST_DMA0_BASE + 0x68CUL)

/* Stock idle register state restored on exit. */
#define WMSDR_ORIG_TX3_BASE	0x10231840U
#define WMSDR_ORIG_TX3_CNT	8U
#define WMSDR_ORIG_TX3_EXT	0x02400002U
#define WMSDR_ORIG_RX3_CNT	0x200U
#define WMSDR_ORIG_RX3_EXT	0x00c00004U

#define WMSDR_DMA_TX3_CNT	38U
#define WMSDR_DMA_RX3_CNT	38U
#define WMSDR_DMA_DESC_SIZE	16U
#define WMSDR_DMA_CHUNK_SIZE	8192U	/* 256 KiB = 32 descriptors (1 batch) */
#define WMSDR_DMA_RX_BUF_STRIDE	8192U

#define WMSDR_DMA_TX3_OFF	0x0000U
#define WMSDR_DMA_RX3_OFF	0x0400U
#define WMSDR_DMA_BUF_OFF	0x1000U

#define WMSDR_DMA_MAP_SIZE	(512U * 1024U)
#define WMSDR_DMA_DATA_OFF	0x1000U	/* 4 KiB headroom for user WMIQ/WS headers */
#define WMSDR_DMA_STAGE_OFF	0x41000U /* 64 KiB staging for dual-bank spectrum DMA */

#define WMSDR_WF_ICAP_MUX_REG	0x88009004U
#define WMSDR_WF_ICAP_MUX_ON	0x0F000000U
#define WMSDR_SPEC_BANK0_ADDR	0x00400000U
#define WMSDR_SPEC_BANK1_ADDR	0x00408000U

struct wmsdr_dma_seg {
	u32 addr;
	u32 len;
};

#define WMSDR_MCU_GLO_REQ_MASK	(BIT(0) | BIT(2) | BIT(28))
#define WMSDR_HOST_GLO_REQ_MASK	(BIT(0) | BIT(2) | BIT(15) | BIT(28))
#define WMSDR_DESC_DDONE	BIT(31)
#define WMSDR_DESC_LS0		BIT(30)
#define WMSDR_DMA_TIMEOUT_NS	(20LL * NSEC_PER_MSEC)

bool wmsdr_direct_addr_ok(u32 off)
{
	if (off & 3U)
		return false;
	if (off >= WMSDR_ICAP_BAR_LO && off < WMSDR_ICAP_BAR_HI)
		return true;
	if (off >= WMSDR_MEM_DMA_BASE && off < WMSDR_MEM_DMA_BASE + 0x1000UL)
		return true;
	if (off >= WMSDR_MCU_DMA0_BASE && off < WMSDR_MCU_DMA0_BASE + 0x1000UL)
		return true;
	if (off >= WMSDR_HOST_DMA0_BASE && off < WMSDR_HOST_DMA0_BASE + 0x1000UL)
		return true;
	if (off >= 0x027000UL && off < 0x028000UL)	/* EXT_WRAP_CSR */
		return true;
	if (off >= 0x00E000UL && off < 0x00F000UL)	/* CONN_BUS_CR */
		return true;
	if (off >= 0x060000UL && off < 0x061000UL)	/* CONN_HOST_CSR_TOP */
		return true;
	return false;
}

bool wmsdr_wfdma_addr_ok(u32 addr, u32 len)
{
	if ((addr | len) & 3U)
		return false;
	return wmsdr_addr_ok(addr, len);
}

static inline void wmsdr_write_desc(void __iomem *desc_base, u32 idx,
				    u32 buf_addr, u32 ctrl)
{
	void __iomem *d = desc_base + (size_t)idx * WMSDR_DMA_DESC_SIZE;

	writel_relaxed(buf_addr, d + 0x00);
	writel_relaxed(0, d + 0x08);
	writel_relaxed(0, d + 0x0C);
	writel_relaxed(ctrl, d + 0x04);
}

static int wmsdr_poll_idx(void __iomem *reg, u32 target, u32 *last_val)
{
	ktime_t deadline;
	int i;
	u32 v;

	for (i = 0; i < 4000; i++) {
		v = readl(reg) & 0xFFFU;
		if (v == target) {
			*last_val = v;
			return 0;
		}
		cpu_relax();
	}

	deadline = ktime_add_ns(ktime_get(), WMSDR_DMA_TIMEOUT_NS);
	do {
		v = readl(reg) & 0xFFFU;
		if (v == target) {
			*last_val = v;
			return 0;
		}
		udelay(5);
	} while (ktime_before(ktime_get(), deadline));

	*last_val = v;
	return -ETIMEDOUT;
}

static void wmsdr_wfdma_isolate_rx3(struct wmsdr_dev *w)
{
	u32 pri = readl(w->bar + WMSDR_HOST_RX_PRI_SEL);

	if (!(pri & BIT(3)))
		writel(pri | BIT(3), w->bar + WMSDR_HOST_RX_PRI_SEL);
	writel(BIT(3), w->bar + WMSDR_HOST_INT_ENA_CLR);
	(void)readl(w->bar + WMSDR_HOST_RX_PRI_SEL);
}

static void wmsdr_wfdma_free_rx_pages(struct wmsdr_dev *w)
{
	if (w->rx_pages) {
		if (w->dma_dev && w->rx_pa)
			dma_unmap_single(w->dma_dev, w->rx_pa, w->rx_map_size,
					 DMA_FROM_DEVICE);
		__free_pages(w->rx_pages, get_order(w->rx_map_size));
		w->rx_pages = NULL;
		w->rx_va = NULL;
		w->rx_pa = 0;
	}
	if (w->dma_pdev) {
		pci_dev_put(w->dma_pdev);
		w->dma_pdev = NULL;
		w->dma_dev = NULL;
	}
	w->rx_map_size = 0;
	w->rx_data_off = 0;
}

static void wmsdr_wfdma_alloc_rx_pages(struct wmsdr_dev *w)
{
	unsigned int order = get_order(WMSDR_DMA_MAP_SIZE);
	dma_addr_t pa;

	w->dma_pdev = pci_get_device(0x14c3, 0x7981, NULL);
	if (!w->dma_pdev) {
		pr_warn("wmsdr: PCI 14c3:7981 not found, mmap cacheable RX disabled\n");
		return;
	}
	w->dma_dev = &w->dma_pdev->dev;

	w->rx_pages = alloc_pages(GFP_KERNEL | __GFP_ZERO | __GFP_NOWARN, order);
	if (!w->rx_pages) {
		pr_warn("wmsdr: alloc_pages order=%u failed, mmap disabled\n", order);
		goto err_put;
	}

	w->rx_map_size = WMSDR_DMA_MAP_SIZE;
	w->rx_data_off = WMSDR_DMA_DATA_OFF;
	w->rx_va = page_address(w->rx_pages);
	pa = dma_map_single(w->dma_dev, w->rx_va, w->rx_map_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(w->dma_dev, pa) ||
	    (u64)pa + w->rx_map_size > 0x100000000ULL) {
		pr_warn("wmsdr: dma_map_single failed (pa=%pad)\n", &pa);
		if (!dma_mapping_error(w->dma_dev, pa))
			dma_unmap_single(w->dma_dev, pa, w->rx_map_size,
					 DMA_FROM_DEVICE);
		__free_pages(w->rx_pages, order);
		w->rx_pages = NULL;
		w->rx_va = NULL;
		w->rx_map_size = 0;
		w->rx_data_off = 0;
		goto err_put;
	}
	w->rx_pa = pa;
	return;

err_put:
	pci_dev_put(w->dma_pdev);
	w->dma_pdev = NULL;
	w->dma_dev = NULL;
}

static int wmsdr_wfdma_wait_glo(struct wmsdr_dev *w, bool wait_ser)
{
	int tries = wait_ser ? 40 : 1;

	while (tries-- > 0) {
		u32 mcu_glo = readl(w->bar + WMSDR_MCU_GLO_CFG);
		u32 host_glo = readl(w->bar + WMSDR_HOST_GLO_CFG);

		if ((mcu_glo & WMSDR_MCU_GLO_REQ_MASK) == WMSDR_MCU_GLO_REQ_MASK &&
		    (host_glo & WMSDR_HOST_GLO_REQ_MASK) == WMSDR_HOST_GLO_REQ_MASK)
			return 0;
		if (tries > 0)
			usleep_range(1000, 2000);
	}
	return -EAGAIN;
}

static int wmsdr_wfdma_program_rings(struct wmsdr_dev *w)
{
	u32 tx3_wm = w->emi_wm_base + WMSDR_DMA_TX3_OFF;
	u32 rx3_pa = (u32)(w->emi_phys + WMSDR_DMA_RX3_OFF);
	u32 tx_didx = readl(w->bar + WMSDR_MCU_TX3 + 0x0C) & 0xFFFU;
	u32 rx_didx = readl(w->bar + WMSDR_HOST_RX3 + 0x0C) & 0xFFFU;
	u32 i;

	if (tx_didx || rx_didx)
		return -EBUSY;

	if ((readl(w->bar + WMSDR_MCU_TX3 + 0x08) & 0xFFFU) ||
	    (readl(w->bar + WMSDR_HOST_RX3 + 0x08) & 0xFFFU)) {
		writel(0, w->bar + WMSDR_MCU_TX3 + 0x08);
		writel(0, w->bar + WMSDR_HOST_RX3 + 0x08);
		wmb();
	}

	for (i = 0; i < WMSDR_DMA_TX3_CNT; i++)
		wmsdr_write_desc(w->emi + WMSDR_DMA_TX3_OFF, i, 0,
				 WMSDR_DESC_DDONE | WMSDR_DESC_LS0);
	for (i = 0; i < WMSDR_DMA_RX3_CNT; i++)
		wmsdr_write_desc(w->emi + WMSDR_DMA_RX3_OFF, i, 0,
				 WMSDR_DESC_DDONE | WMSDR_DESC_LS0);

	wmb();
	(void)readl(w->emi + WMSDR_DMA_RX3_OFF +
		    (WMSDR_DMA_RX3_CNT - 1U) * WMSDR_DMA_DESC_SIZE + 0x04);

	/* Route RX3 completion IRQ off legacy GIC IRQ 237 before touching RX3. */
	wmsdr_wfdma_isolate_rx3(w);
	writel(BIT(3), w->bar + WMSDR_HOST_INT_STA);

	writel(BIT(3), w->bar + WMSDR_MCU_RST_DTX);
	writel(BIT(3), w->bar + WMSDR_HOST_RST_DRX);
	(void)readl(w->bar + WMSDR_HOST_RST_DRX);

	writel(WMSDR_DMA_TX3_CNT, w->bar + WMSDR_MCU_TX3 + 0x04);
	writel(0, w->bar + WMSDR_MCU_TX3 + 0x08);
	writel(tx3_wm, w->bar + WMSDR_MCU_TX3 + 0x00);

	writel(WMSDR_DMA_RX3_CNT, w->bar + WMSDR_HOST_RX3 + 0x04);
	writel(0, w->bar + WMSDR_HOST_RX3 + 0x08);
	writel(rx3_pa, w->bar + WMSDR_HOST_RX3 + 0x00);

	w->tx3_idx = 0;
	w->rx3_idx = 0;
	return 0;
}

int wmsdr_wfdma_init(struct wmsdr_dev *w)
{
	u32 emi_start, emi_end, pri;
	u32 tx3_base, rx3_base, tx3_ext, rx3_ext;
	int rc;

	mutex_init(&w->dma_lock);

	if (!wfdma_enable) {
		pr_info("wmsdr: WFDMA disabled by module parameter\n");
		return -ENODEV;
	}

	emi_start = readl(w->bar + WMSDR_CONN2AP_EMI_START);
	emi_end = readl(w->bar + WMSDR_CONN2AP_EMI_END);
	if (!emi_start || emi_end <= emi_start ||
	    (emi_end - emi_start) < (WMSDR_EMI_SLICE_OFF + WMSDR_EMI_SLICE_SIZE)) {
		pr_warn("wmsdr: unexpected EMI window [%#x..%#x], WFDMA disabled\n",
			emi_start, emi_end);
		return -ENODEV;
	}

	if (wmsdr_wfdma_wait_glo(w, false)) {
		pr_warn("wmsdr: WFDMA not ready (mcu_glo=%#x host_glo=%#x)\n",
			readl(w->bar + WMSDR_MCU_GLO_CFG),
			readl(w->bar + WMSDR_HOST_GLO_CFG));
		return -ENODEV;
	}

	tx3_base = readl(w->bar + WMSDR_MCU_TX3);
	rx3_base = readl(w->bar + WMSDR_HOST_RX3);
	tx3_ext = readl(w->bar + WMSDR_MCU_TX3_EXT);
	rx3_ext = readl(w->bar + WMSDR_HOST_RX3_EXT);
	if (tx3_ext != WMSDR_ORIG_TX3_EXT || rx3_ext != WMSDR_ORIG_RX3_EXT ||
	    (tx3_base && tx3_base != WMSDR_ORIG_TX3_BASE) ||
	    rx3_base ||
	    (readl(w->bar + WMSDR_MCU_TX3 + 0x0C) & 0xFFFU) ||
	    (readl(w->bar + WMSDR_HOST_RX3 + 0x0C) & 0xFFFU)) {
		pr_warn("wmsdr: TX3/RX3 not in idle stock state (TX3=%#x/%#x RX3=%#x/%#x)\n",
			tx3_base, tx3_ext, rx3_base, rx3_ext);
		return -EBUSY;
	}

	w->emi_phys = (phys_addr_t)emi_start + WMSDR_EMI_SLICE_OFF;
	w->emi_size = WMSDR_EMI_SLICE_SIZE;
	w->emi_wm_base = WMSDR_WM_EMI_BASE + WMSDR_EMI_SLICE_OFF;

	w->emi = ioremap_wc(w->emi_phys, w->emi_size);
	if (!w->emi) {
		pr_warn("wmsdr: ioremap_wc(0x%llx, %#x) failed\n",
			(unsigned long long)w->emi_phys, w->emi_size);
		return -ENOMEM;
	}

	wmsdr_wfdma_alloc_rx_pages(w);

	rc = wmsdr_wfdma_program_rings(w);
	if (rc) {
		wmsdr_wfdma_free_rx_pages(w);
		iounmap(w->emi);
		w->emi = NULL;
		return rc;
	}

	pri = readl(w->bar + WMSDR_HOST_RX_PRI_SEL);
	w->dma_ready = true;

	pr_info("wmsdr: MCU_TX3->HOST_RX3 ready emi_pa=%#llx rx_pa=%pad map=%u pri=%#x ena=%#x TX3(base=%#x cnt=%u) RX3(base=%#x cnt=%u)\n",
		(unsigned long long)w->emi_phys, &w->rx_pa, w->rx_map_size,
		pri, readl(w->bar + WMSDR_HOST_INT_ENA),
		readl(w->bar + WMSDR_MCU_TX3),
		readl(w->bar + WMSDR_MCU_TX3 + 0x04),
		readl(w->bar + WMSDR_HOST_RX3),
		readl(w->bar + WMSDR_HOST_RX3 + 0x04));
	return 0;
}

static void wmsdr_wfdma_park_zero(struct wmsdr_dev *w)
{
	u32 rem, k, rx_ctrl, tx_ctrl, buf_pa, tx_didx = 0, rx_didx = 0;

	if (!w->tx3_idx && !w->rx3_idx)
		return;
	if (w->tx3_idx != w->rx3_idx)
		return;

	rem = WMSDR_DMA_RX3_CNT - w->rx3_idx;
	buf_pa = w->rx_pages ? (u32)(w->rx_pa + w->rx_data_off) :
			       (u32)(w->emi_phys + WMSDR_DMA_BUF_OFF);
	wmsdr_wfdma_isolate_rx3(w);
	rx_ctrl = WMSDR_DESC_LS0 | (64U << 16);
	tx_ctrl = WMSDR_DESC_LS0 | (4U << 16);

	for (k = 0; k < rem; k++) {
		u32 slot = w->rx3_idx + k;

		wmsdr_write_desc(w->emi + WMSDR_DMA_RX3_OFF, slot, buf_pa, rx_ctrl);
		wmsdr_write_desc(w->emi + WMSDR_DMA_TX3_OFF, slot, 0x00903800U, tx_ctrl);
	}
	wmb();
	(void)readl(w->emi + WMSDR_DMA_TX3_OFF +
		    (size_t)(WMSDR_DMA_RX3_CNT - 1U) * WMSDR_DMA_DESC_SIZE + 0x04);
	writel(0, w->bar + WMSDR_HOST_RX3 + 0x08);
	wmb();
	writel(0, w->bar + WMSDR_MCU_TX3 + 0x08);
	if (!wmsdr_poll_idx(w->bar + WMSDR_HOST_RX3 + 0x0C, 0, &rx_didx))
		(void)wmsdr_poll_idx(w->bar + WMSDR_MCU_TX3 + 0x0C, 0, &tx_didx);
	writel(BIT(3), w->bar + WMSDR_HOST_INT_STA);
	w->tx3_idx = 0;
	w->rx3_idx = 0;
}

static int wmsdr_wfdma_ensure_rings(struct wmsdr_dev *w)
{
	u32 exp_tx = w->emi_wm_base + WMSDR_DMA_TX3_OFF;
	u32 exp_rx = (u32)(w->emi_phys + WMSDR_DMA_RX3_OFF);
	u32 tx_base, rx_base, tx_cidx, tx_didx, rx_cidx, rx_didx;
	int rc;

	rc = wmsdr_wfdma_wait_glo(w, true);
	if (rc)
		return rc;

	tx_base = readl(w->bar + WMSDR_MCU_TX3);
	rx_base = readl(w->bar + WMSDR_HOST_RX3);
	tx_cidx = readl(w->bar + WMSDR_MCU_TX3 + 0x08) & 0xFFFU;
	tx_didx = readl(w->bar + WMSDR_MCU_TX3 + 0x0C) & 0xFFFU;
	rx_cidx = readl(w->bar + WMSDR_HOST_RX3 + 0x08) & 0xFFFU;
	rx_didx = readl(w->bar + WMSDR_HOST_RX3 + 0x0C) & 0xFFFU;

	if (tx_base == exp_tx && rx_base == exp_rx &&
	    tx_cidx == w->tx3_idx && tx_didx == w->tx3_idx &&
	    rx_cidx == w->rx3_idx && rx_didx == w->rx3_idx)
		return 0;

	if (tx_cidx == tx_didx && rx_cidx == rx_didx &&
	    tx_didx == 0 && rx_didx == 0 &&
	    (tx_base == 0 || tx_base == WMSDR_ORIG_TX3_BASE || tx_base == exp_tx) &&
	    (rx_base == 0 || rx_base == exp_rx)) {
		pr_info("wmsdr: WFDMA SER/reset detected (TX3=%#x idx=%u->%u/%u RX3=%#x idx=%u->%u/%u), re-arming rings\n",
			tx_base, w->tx3_idx, tx_cidx, tx_didx,
			rx_base, w->rx3_idx, rx_cidx, rx_didx);
		return wmsdr_wfdma_program_rings(w);
	}

	pr_err("wmsdr: TX3/RX3 not idle tx=%u/%u/%u rx=%u/%u/%u (base=%#x/%#x)\n",
	       w->tx3_idx, tx_cidx, tx_didx, w->rx3_idx, rx_cidx, rx_didx,
	       tx_base, rx_base);
	return -EBUSY;
}

void wmsdr_wfdma_exit(struct wmsdr_dev *w)
{
	if (w->dma_ready) {
		u32 exp_tx = w->emi_wm_base + WMSDR_DMA_TX3_OFF;
		u32 exp_rx = (u32)(w->emi_phys + WMSDR_DMA_RX3_OFF);
		u32 pri;

		mutex_lock(&w->dma_lock);
		if (readl(w->bar + WMSDR_MCU_TX3) == exp_tx &&
		    readl(w->bar + WMSDR_HOST_RX3) == exp_rx)
			wmsdr_wfdma_park_zero(w);
		else
			w->tx3_idx = w->rx3_idx = 0;

		writel(WMSDR_ORIG_TX3_BASE, w->bar + WMSDR_MCU_TX3 + 0x00);
		writel(WMSDR_ORIG_TX3_CNT, w->bar + WMSDR_MCU_TX3 + 0x04);
		writel(0, w->bar + WMSDR_HOST_RX3 + 0x00);
		writel(WMSDR_ORIG_RX3_CNT, w->bar + WMSDR_HOST_RX3 + 0x04);

		writel(BIT(3), w->bar + WMSDR_HOST_INT_STA);
		pri = readl(w->bar + WMSDR_HOST_RX_PRI_SEL);
		writel(pri & ~BIT(3), w->bar + WMSDR_HOST_RX_PRI_SEL);
		w->dma_ready = false;
		mutex_unlock(&w->dma_lock);
	}
	wmsdr_wfdma_free_rx_pages(w);
	if (w->emi) {
		iounmap(w->emi);
		w->emi = NULL;
	}
}

static int wmsdr_wfdma_xfer_segs(struct wmsdr_dev *w,
				 const struct wmsdr_dma_seg *segs, u32 n_segs,
				 u32 dst_off, u8 *out)
{
	u32 total_len = 0, n_desc = 0, s, idx = 0, tx_next, rx_next;
	u32 tx_didx = 0, rx_didx = 0, last_tslot = 0, last_rslot = 0;
	u32 rx_ctrl, rxd_w1, buf_off = 0;
	bool use_cached = (w->rx_pages != NULL);
	u32 buf_base_pa = use_cached ? (u32)(w->rx_pa + dst_off) :
				       (u32)(w->emi_phys + WMSDR_DMA_BUF_OFF);
	size_t sync_len;
	ktime_t t0, t1, t2;
	int rc;

	t0 = ktime_get();
	for (s = 0; s < n_segs; s++) {
		if (!segs[s].len || !wmsdr_wfdma_addr_ok(segs[s].addr, segs[s].len))
			return -EINVAL;
		total_len += segs[s].len;
		n_desc += DIV_ROUND_UP(segs[s].len, WMSDR_DMA_CHUNK_SIZE);
	}
	if (!n_desc || n_desc >= WMSDR_DMA_RX3_CNT || n_desc >= WMSDR_DMA_TX3_CNT)
		return -EINVAL;
	if (use_cached && (dst_off + total_len > w->rx_map_size))
		return -EINVAL;
	sync_len = ALIGN(total_len, 64U);

	rc = wmsdr_wfdma_ensure_rings(w);
	if (rc)
		return rc;

	wmsdr_wfdma_isolate_rx3(w);

	if (use_cached)
		dma_sync_single_for_device(w->dma_dev,
					   w->rx_pa + dst_off,
					   sync_len, DMA_BIDIRECTIONAL);

	rx_ctrl = WMSDR_DESC_LS0 | ((WMSDR_DMA_RX_BUF_STRIDE & 0x3FFFU) << 16);
	for (s = 0; s < n_segs; s++) {
		u32 off = 0;

		while (off < segs[s].len) {
			u32 chunk = min_t(u32, segs[s].len - off, WMSDR_DMA_CHUNK_SIZE);
			u32 tslot = (w->tx3_idx + idx) % WMSDR_DMA_TX3_CNT;
			u32 rslot = (w->rx3_idx + idx) % WMSDR_DMA_RX3_CNT;
			u32 tx_ctrl = WMSDR_DESC_LS0 | ((chunk & 0x3FFFU) << 16);

			wmsdr_write_desc(w->emi + WMSDR_DMA_RX3_OFF, rslot,
					 buf_base_pa + buf_off + off, rx_ctrl);
			wmsdr_write_desc(w->emi + WMSDR_DMA_TX3_OFF, tslot,
					 segs[s].addr + off, tx_ctrl);
			last_rslot = rslot;
			last_tslot = tslot;
			off += chunk;
			idx++;
		}
		buf_off += segs[s].len;
	}

	wmb();
	(void)readl(w->emi + WMSDR_DMA_RX3_OFF +
		    (size_t)last_rslot * WMSDR_DMA_DESC_SIZE + 0x04);
	(void)readl(w->emi + WMSDR_DMA_TX3_OFF +
		    (size_t)last_tslot * WMSDR_DMA_DESC_SIZE + 0x04);

	rx_next = (w->rx3_idx + n_desc) % WMSDR_DMA_RX3_CNT;
	tx_next = (w->tx3_idx + n_desc) % WMSDR_DMA_TX3_CNT;

	writel(rx_next, w->bar + WMSDR_HOST_RX3 + 0x08);
	wmb();
	writel(tx_next, w->bar + WMSDR_MCU_TX3 + 0x08);

	rc = wmsdr_poll_idx(w->bar + WMSDR_HOST_RX3 + 0x0C, rx_next, &rx_didx);
	if (!rc)
		rc = wmsdr_poll_idx(w->bar + WMSDR_MCU_TX3 + 0x0C, tx_next, &tx_didx);

	rxd_w1 = readl(w->emi + WMSDR_DMA_RX3_OFF +
		       (size_t)last_rslot * WMSDR_DMA_DESC_SIZE + 0x04);
	writel(BIT(3), w->bar + WMSDR_HOST_INT_STA);
	t1 = ktime_get();

	if (rc) {
		pr_err("wmsdr: WFDMA timeout addr=%#x len=%u n=%u TX3(%u->%u didx=%u mcu_sta=%#x mcu_err=%#x) RX3(%u->%u didx=%u rxd_w1=%#x host_sta=%#x)\n",
		       segs[0].addr, total_len, n_desc, w->tx3_idx, tx_next, tx_didx,
		       readl(w->bar + WMSDR_MCU_INT_STA),
		       readl(w->bar + WMSDR_MCU_DMA0_BASE + 0x110UL),
		       w->rx3_idx, rx_next, rx_didx, rxd_w1,
		       readl(w->bar + WMSDR_HOST_INT_STA));
		return -ETIMEDOUT;
	}

	if (use_cached) {
		dma_sync_single_for_cpu(w->dma_dev,
					w->rx_pa + dst_off,
					sync_len, DMA_FROM_DEVICE);
		if (out)
			memcpy(out, (const u8 *)w->rx_va + dst_off, total_len);
	} else if (out) {
		rmb();
		memcpy_fromio(out, w->emi + WMSDR_DMA_BUF_OFF, total_len);
	}
	t2 = ktime_get();

	if (total_len >= 131072U)
		pr_info_once("wmsdr: cached=%u len=%u hw=%lldus cpu=%lldus\n",
			     use_cached, total_len,
			     ktime_to_us(ktime_sub(t1, t0)),
			     ktime_to_us(ktime_sub(t2, t1)));

	w->tx3_idx = tx_next;
	w->rx3_idx = rx_next;
	return 0;
}

static int wmsdr_wfdma_xfer_batch(struct wmsdr_dev *w, u32 addr, u8 *out, u32 len)
{
	struct wmsdr_dma_seg seg = { .addr = addr, .len = len };

	return wmsdr_wfdma_xfer_segs(w, &seg, 1U, w->rx_data_off, out);
}

int wmsdr_wfdma_read(struct wmsdr_dev *w, u32 addr, void *buf, u32 len)
{
	const u32 max_batch = (WMSDR_DMA_RX3_CNT - 2U) * WMSDR_DMA_CHUNK_SIZE;
	u32 off = 0;
	u8 *out = buf;
	int rc = 0;

	if (!w->dma_ready)
		return -EOPNOTSUPP;
	if (!len || len > WMSDR_MAX_XFER || !wmsdr_wfdma_addr_ok(addr, len))
		return -EINVAL;

	mutex_lock(&w->dma_lock);
	while (off < len) {
		u32 batch = min_t(u32, len - off, max_batch);

		rc = wmsdr_wfdma_xfer_batch(w, addr + off, out + off, batch);
		if (rc)
			break;
		off += batch;
	}
	mutex_unlock(&w->dma_lock);
	return rc;
}

int wmsdr_wfdma_mmap_read(struct wmsdr_dev *w, u32 addr, u32 len)
{
	int rc;

	if (!w->dma_ready || !w->rx_pages)
		return -EOPNOTSUPP;
	if (!len || len > WMSDR_MAX_XFER || !wmsdr_wfdma_addr_ok(addr, len))
		return -EINVAL;

	mutex_lock(&w->dma_lock);
	rc = wmsdr_wfdma_xfer_batch(w, addr, NULL, len);
	mutex_unlock(&w->dma_lock);
	return rc;
}

int wmsdr_wfdma_icap_to_va(struct wmsdr_dev *w, u32 len_words, u32 mode,
			   u32 tmo_ms, void *dst_va, u32 *cr0_out,
			   u32 *cr2_out, u32 *cr9_out, u32 *sample_cnt_out)
{
	u32 aligned_words = ALIGN(len_words, 4U);
	u32 byte_len = aligned_words * 4U;
	u32 end_word, cr0 = 0, cr2 = 0, cr9 = 0, polls = 0;
	u32 tmo = tmo_ms ? min_t(u32, tmo_ms, 1000U) : 50U;
	void *mmap_dst;
	ktime_t deadline;
	bool timed_out = false;
	int rc;

	if (!w->dma_ready || !w->rx_pages)
		return -EOPNOTSUPP;
	if (mode > 1U || len_words < 16U)
		return -EINVAL;
	if (mode == 1U) {
		if (aligned_words > WMIQ_SPECTRUM_MAX_WORDS ||
		    WMSDR_DMA_STAGE_OFF + byte_len > w->rx_map_size ||
		    w->rx_data_off + byte_len > WMSDR_DMA_STAGE_OFF)
			return -EINVAL;
		end_word = min_t(u32, aligned_words - 4U, 0x3FFCU);
	} else {
		if (aligned_words > 65536U ||
		    byte_len > w->rx_map_size - w->rx_data_off)
			return -EINVAL;
		end_word = min_t(u32, aligned_words - 4U, 0x1FFFCU);
	}

	mmap_dst = (u8 *)w->rx_va + w->rx_data_off;
	if (!dst_va)
		dst_va = mmap_dst;

	mutex_lock(&w->dma_lock);

	rc = wmsdr_wfdma_ensure_rings(w);
	if (rc) {
		mutex_unlock(&w->dma_lock);
		return rc;
	}

	if (mode == 1U)
		wmsdr_remap_write32(w, WMSDR_WF_ICAP_MUX_REG, WMSDR_WF_ICAP_MUX_ON);

	writel(0U, w->bar + WMSDR_ICAP_CR0);
	writel(0x400U, w->bar + WMSDR_ICAP_CR0);
	writel(0U, w->bar + WMSDR_ICAP_CR2);
	writel(end_word, w->bar + WMSDR_ICAP_CR3);
	writel(0x401U, w->bar + WMSDR_ICAP_CR0);
	writel(0x403U, w->bar + WMSDR_ICAP_CR0);
	(void)readl(w->bar + WMSDR_ICAP_CR0);

	deadline = ktime_add_ms(ktime_get(), tmo);
	if (!mode && aligned_words >= 32768U)
		usleep_range(350, 450);

	for (;;) {
		cr0 = readl(w->bar + WMSDR_ICAP_CR0);
		cr9 = readl(w->bar + WMSDR_ICAP_CR9);
		if (cr9 >= end_word || !(cr0 & 0x2U))
			break;
		if (ktime_after(ktime_get(), deadline)) {
			timed_out = true;
			break;
		}
		if ((++polls & 0x7FFU) == 0U)
			cond_resched();
		else
			cpu_relax();
	}

	writel(0x400U, w->bar + WMSDR_ICAP_CR0);
	cr0 = readl(w->bar + WMSDR_ICAP_CR0);
	cr2 = readl(w->bar + WMSDR_ICAP_CR2);
	cr9 = readl(w->bar + WMSDR_ICAP_CR9);
	if (mode == 1U)
		wmsdr_remap_write32(w, WMSDR_WF_ICAP_MUX_REG, 0U);

	if (cr0_out)
		*cr0_out = cr0;
	if (cr2_out)
		*cr2_out = cr2;
	if (cr9_out)
		*cr9_out = cr9;
	if (sample_cnt_out)
		*sample_cnt_out = (cr9 >= end_word) ? aligned_words : cr9;

	if (timed_out) {
		mutex_unlock(&w->dma_lock);
		return -ETIMEDOUT;
	}
	if (cr9 < 4U) {
		mutex_unlock(&w->dma_lock);
		return -EIO;
	}

	if (mode == 1U) {
		u32 half_len = byte_len >> 1U;
		struct wmsdr_dma_seg segs[2] = {
			{ .addr = WMSDR_SPEC_BANK0_ADDR, .len = half_len },
			{ .addr = WMSDR_SPEC_BANK1_ADDR, .len = half_len },
		};

		rc = wmsdr_wfdma_xfer_segs(w, segs, 2U, WMSDR_DMA_STAGE_OFF, NULL);
		if (!rc) {
			const u64 *b0 = (const u64 *)((const u8 *)w->rx_va +
						      WMSDR_DMA_STAGE_OFF);
			const u64 *b1 = (const u64 *)((const u8 *)w->rx_va +
						      WMSDR_DMA_STAGE_OFF + half_len);
			u64 *dst = (u64 *)dst_va;
			u32 g, n_grp = byte_len >> 4U;

			for (g = 0; g < n_grp; g++) {
				dst[2U * g] = b0[g];
				dst[2U * g + 1U] = b1[g];
			}
		}
	} else {
		u8 *out = (dst_va != mmap_dst) ? (u8 *)dst_va : NULL;

		rc = wmsdr_wfdma_xfer_batch(w, 0xE0200000U, out, byte_len);
	}
	mutex_unlock(&w->dma_lock);
	return rc;
}

int wmsdr_wfdma_icap_mmap(struct wmsdr_dev *w, struct wmsdr_icap_mmap_req *req)
{
	if (!w->dma_ready || !w->rx_pages)
		return -EOPNOTSUPP;
	req->data_off = w->rx_data_off;
	req->map_size = w->rx_map_size;
	return wmsdr_wfdma_icap_to_va(w, req->len_words, req->mode,
				      req->timeout_ms, NULL,
				      &req->cr0, &req->cr2, &req->cr9,
				      &req->sample_count);
}

int wmsdr_wfdma_mmap(struct wmsdr_dev *w, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;

	if (!w->dma_ready || !w->rx_pages)
		return -ENODEV;
	if (vma->vm_pgoff != 0 || !size || size > w->rx_map_size)
		return -EINVAL;

	vma->vm_flags |= VM_IO | VM_DONTCOPY | VM_DONTEXPAND | VM_DONTDUMP;
	return remap_pfn_range(vma, vma->vm_start,
			       page_to_pfn(w->rx_pages),
			       size, vma->vm_page_prot);
}
