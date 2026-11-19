// SPDX-License-Identifier: GPL-2.0-only
/*
 * wmrom - CT3003 (MT7981) WM ROM firmware loader driver.
 *
 * Claims 14c3:7981 (mt_wifi must be unloaded), powers on the WF subsystem
 * via conninfra_pwr_on(CONNDRV_TYPE_WIFI), acquires Driver Own on LPCTL
 * (0x18060010), programs WFDMA TX16 (scatter), TX17 (command), and RX0
 * (event reply) rings with full EXT_CTRL prefetch configuration, writes
 * EMI regions (0xF0000000+) directly to physical DRAM (0x4FC00000), sets
 * ICapMode=1 (0x0041143C), and drives the ROM's firmware downloader:
 *
 *   patch: PatchSem(0x10, 1) -> PatchStartReq(0x05) -> Scatters ->
 *          PatchFinish(0x07, 0) -> PatchSem(0x10, 3)
 *   fw:    AddressLenReq(0x01) -> Scatters ... -> FwStartReq(0x02)
 */
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/kallsyms.h>

#define WMROM_NAME		"wmrom"
#define WMROM_BAR_SIZE		0x01000000UL

/* WFDMA0 registers (offset from BAR0 0x18000000) */
#define WMROM_CONN_HIF_RST	0x024100UL
#define WMROM_INT_STA		0x024200UL
#define WMROM_INT_MSK		0x024204UL
#define WMROM_GLO_CFG		0x024208UL
#define WMROM_RST_DTX		0x02420CUL
#define WMROM_RST_DRX		0x024280UL
#define WMROM_GLO_CFG_EXT1	0x0242B4UL
#define WMROM_T16		0x024400UL
#define WMROM_T17		0x024410UL
#define WMROM_R0		0x024500UL
#define WMROM_TX16_EXT_CTRL	0x024640UL
#define WMROM_RX0_EXT_CTRL	0x024680UL

/* MCU_DMA0 & PDA registers via RBUS BAR0 (+0x402000 maps 0x54000000) */
#define WMROM_MCU_INT_STA	0x402200UL
#define WMROM_MCU_GLO_CFG	0x402208UL
#define WMROM_PDA_BASE		0x402800UL
#define WMROM_PDA_STATUS	0x402808UL

/* CONN_HOST_CSR_TOP registers (offset from BAR0 0x18000000) */
#define WMROM_LPCTL		0x060010UL
#define WMROM_IRQ_STAT		0x060014UL
#define WMROM_FW_STAT		0x0600F0UL

/* L2 remap window for WM SRAM access (64 KiB page in [31:16]) */
#define WMROM_L2_REMAP_CTL	0x4001B8UL
#define WMROM_L2_REMAP_WIN	0x440000UL
#define WMROM_L2_REMAP_MASK	0x0000FFFFUL
#define WMROM_ICAP_MODE_CR	0x0041143CU

/* EMI physical DRAM reserve for WM 0xF0000000+ regions */
#define WMROM_EMI_PHYS_DEFAULT	0x4FC00000ULL
#define WMROM_EMI_SIZE_DEFAULT	0x00100000U
#define WMROM_MCU_EMI_BASE	0xF0000000U

/* Ring sizing */
#define WMROM_T16_COUNT		128
#define WMROM_T17_COUNT		256
#define WMROM_R0_COUNT		16
#define WMROM_DESC_SIZE		16
#define WMROM_PKT_SIZE		8192
#define WMROM_R0_BUF_SIZE	2048
#define WMROM_SCATTER_MAX	4032

/* Flat image format ("WMSD") */
#define WMROM_MAGIC		0x44534D57U
#define WMROM_FLAG_PATCH	0x01U
#define WMROM_FLAG_WA		0x02U
#define WMROM_FLAG_OVERRIDE	0x20U
#define WMROM_FLAG_EMI		0x80U

/* WFDMA GLO_CFG bits (MT7981 wf_wfdma_host_dma0.h) */
#define WMROM_DMA_EN_TX		BIT(0)
#define WMROM_DMA_BUSY_TX	BIT(1)
#define WMROM_DMA_EN_RX		BIT(2)
#define WMROM_DMA_BUSY_RX	BIT(3)
#define WMROM_DMA_TX_WB_DDONE	BIT(6)
#define WMROM_DMA_BIG_ENDIAN	BIT(7)
#define WMROM_DMA_FIFO_LE	BIT(12)
#define WMROM_DMA_RX_WB_DDONE	BIT(13)
#define WMROM_DMA_CHAIN_EN	BIT(15)
#define WMROM_DMA_OMIT_RX_PFET2	BIT(21)
#define WMROM_DMA_OMIT_RX_INFO	BIT(27)
#define WMROM_DMA_OMIT_TX_INFO	BIT(28)
#define WMROM_DMA_BYTE_SWAP	BIT(29)

/* LPCTL Driver/FW Own bits */
#define WMROM_LPCTL_SET_FW_OWN	BIT(0)
#define WMROM_LPCTL_CLR_FW_OWN	BIT(1)
#define WMROM_LPCTL_OWN_SYNC	BIT(2)

#define WMROM_FW_STAT_MASK	0x7U
#define WMROM_TIMEOUT_US	2000000

/* MCU FW_TXD & ROM command IDs */
#define WMROM_PQ_ID_CMD		0x8000U
#define WMROM_PKT_ID_CMD	0xA0U
#define WMROM_PKT_ID_EVENT	0xE000U
#define WMROM_CMD_NA		3U

#define WMROM_CMD_ADDRLEN	0x01U
#define WMROM_CMD_START		0x02U
#define WMROM_CMD_NIC_PWR_CTRL	0x04U
#define WMROM_CMD_PATCH_START	0x05U
#define WMROM_CMD_PATCH_FINISH	0x07U
#define WMROM_CMD_PATCH_SEM	0x10U
#define WMROM_CMD_RESTART	0xEFU

#define WMROM_EVENT_CMD_RESULT	0x01U
#define WMROM_EVENT_PATCH_SEM	0x04U
#define WMROM_PATCH_OFFSET	0x00900000U
#define WMROM_ADDRLEN_WA	BIT(4)
#define WMROM_ADDRLEN_NEED_RSP	0x80000000U
#define WMROM_START_OVERRIDE	BIT(0)
#define WMROM_START_WA		BIT(2)

#define CONNDRV_TYPE_WIFI	3
#define CONNSYS_EMI_FW		0

typedef int (*conninfra_pwr_on_fn)(int drv_type);
typedef int (*conninfra_pwr_off_fn)(int drv_type);
typedef void (*conninfra_get_emi_fn)(int type, phys_addr_t *base, unsigned int *size);

struct wmrom_region {
	__le32 addr;
	__le32 len;
	__le32 flags;
	__le32 offset;
};

struct wmrom_ring {
	void *desc;
	dma_addr_t desc_dma;
	u32 count;
	u32 idx;
	void *pkt;
	dma_addr_t pkt_dma;
	size_t pkt_alloc_size;
};

struct wmrom_dev {
	struct pci_dev *pdev;
	void __iomem *bar;
	void __iomem *emi;
	phys_addr_t emi_phys;
	u32 emi_size;
	conninfra_pwr_on_fn pwr_on;
	conninfra_pwr_off_fn pwr_off;
	bool pwr_held;
	struct wmrom_ring t16, t17, r0;
	struct mutex lock;
	u8 seq;
	struct cdev cdev;
	dev_t devt;
	struct class *class;
	struct device *device;
};

static struct wmrom_dev *wmrom;

static u32 wmrom_fw_state(struct wmrom_dev *w)
{
	return readl(w->bar + WMROM_FW_STAT) & WMROM_FW_STAT_MASK;
}

static void wmrom_l2_write32(struct wmrom_dev *w, u32 wm_addr, u32 val)
{
	u32 base = wm_addr & ~WMROM_L2_REMAP_MASK;
	u32 off = wm_addr & WMROM_L2_REMAP_MASK;
	u32 bak = readl(w->bar + WMROM_L2_REMAP_CTL);

	writel((bak & WMROM_L2_REMAP_MASK) | base, w->bar + WMROM_L2_REMAP_CTL);
	(void)readl(w->bar + WMROM_L2_REMAP_CTL);
	writel(val, w->bar + WMROM_L2_REMAP_WIN + off);
	(void)readl(w->bar + WMROM_L2_REMAP_WIN + off);
	writel(bak, w->bar + WMROM_L2_REMAP_CTL);
}

static u32 wmrom_l2_read32(struct wmrom_dev *w, u32 wm_addr)
{
	u32 base = wm_addr & ~WMROM_L2_REMAP_MASK;
	u32 off = wm_addr & WMROM_L2_REMAP_MASK;
	u32 bak = readl(w->bar + WMROM_L2_REMAP_CTL);
	u32 val;

	writel((bak & WMROM_L2_REMAP_MASK) | base, w->bar + WMROM_L2_REMAP_CTL);
	(void)readl(w->bar + WMROM_L2_REMAP_CTL);
	val = readl(w->bar + WMROM_L2_REMAP_WIN + off);
	writel(bak, w->bar + WMROM_L2_REMAP_CTL);
	return val;
}

static int wmrom_acquire_driver_own(struct wmrom_dev *w)
{
	u32 val, st, st1620, cb0 = 0, mode = 0, i;

	val = readl(w->bar + WMROM_LPCTL);
	st = wmrom_fw_state(w);
	st1620 = readl(w->bar + 0x001620);
	cb0 = wmrom_l2_read32(w, 0x02205070U);
	mode = wmrom_l2_read32(w, 0x0224d968U);
	dev_info(&w->pdev->dev,
		 "pre-wait: lpctl=%#x fw_state=%u csr1620=%#x cb0=%#x mode=%#x\n",
		 val, st, st1620, cb0, mode);

	/* Wait until WM WIFI task registers its ownership callbacks (0x02205070)
	 * before touching LPCTL, otherwise CLR_FW_OWN causes an IRQ 0x36 storm. */
	for (i = 0; i < 200; i++) {
		cb0 = wmrom_l2_read32(w, 0x02205070U);
		st = wmrom_fw_state(w);
		val = readl(w->bar + WMROM_LPCTL);
		if (cb0 != 0 && cb0 != 0xdeadfeedU)
			break;
		msleep(5);
	}
	st1620 = readl(w->bar + 0x001620);
	mode = wmrom_l2_read32(w, 0x0224d968U);
	dev_info(&w->pdev->dev,
		 "post-wait(%u): lpctl=%#x fw_state=%u csr1620=%#x cb0=%#x mode=%#x\n",
		 i, val, st, st1620, cb0, mode);

	if ((val & WMROM_LPCTL_OWN_SYNC) == 0 && st != 0) {
		writel(0x1, w->bar + WMROM_IRQ_STAT);
		dev_info(&w->pdev->dev,
			 "acquired driver own (already awake, lpctl=%#x fw_state=%u mcu_glo=%#x)\n",
			 val, st, readl(w->bar + 0x402208));
		return 0;
	}

	writel(WMROM_LPCTL_CLR_FW_OWN, w->bar + WMROM_LPCTL);
	for (i = 0; i < 500; i++) {
		val = readl(w->bar + WMROM_LPCTL);
		st = wmrom_fw_state(w);
		if ((val & WMROM_LPCTL_OWN_SYNC) == 0 && st != 0) {
			writel(0x1, w->bar + WMROM_IRQ_STAT);
			dev_info(&w->pdev->dev,
				 "acquired driver own (lpctl=%#x fw_state=%u i=%u mcu_glo=%#x)\n",
				 val, st, i, readl(w->bar + 0x402208));
			return 0;
		}
		if ((i % 25) == 24)
			writel(WMROM_LPCTL_CLR_FW_OWN, w->bar + WMROM_LPCTL);
		msleep(2);
	}
	dev_err(&w->pdev->dev,
		"driver own timeout (lpctl=%#x fw_state=%u csr1620=%#x mcu_glo=%#x)\n",
		readl(w->bar + WMROM_LPCTL), wmrom_fw_state(w),
		readl(w->bar + 0x001620), readl(w->bar + 0x402208));
	return -ETIMEDOUT;
}

static int wmrom_hw_power_on(struct wmrom_dev *w)
{
	int ret;

	if (w->pwr_on) {
		ret = w->pwr_on(CONNDRV_TYPE_WIFI);
		if (ret)
			dev_warn(&w->pdev->dev, "conninfra_pwr_on(WIFI) returned %d\n", ret);
		else {
			dev_info(&w->pdev->dev, "conninfra_pwr_on(WIFI) ok\n");
			w->pwr_held = true;
		}
		msleep(50);
	}
	return wmrom_acquire_driver_own(w);
}

static int wmrom_ring_alloc(struct wmrom_dev *w, struct wmrom_ring *r,
			    u32 count, size_t pkt_size)
{
	r->desc = dma_alloc_coherent(&w->pdev->dev, count * WMROM_DESC_SIZE,
				     &r->desc_dma, GFP_KERNEL);
	if (!r->desc)
		return -ENOMEM;
	r->pkt = dma_alloc_coherent(&w->pdev->dev, pkt_size,
				    &r->pkt_dma, GFP_KERNEL);
	if (!r->pkt)
		return -ENOMEM;
	r->count = count;
	r->pkt_alloc_size = pkt_size;
	r->idx = 0;
	return 0;
}

static void wmrom_ring_free(struct wmrom_dev *w, struct wmrom_ring *r)
{
	if (r->pkt)
		dma_free_coherent(&w->pdev->dev, r->pkt_alloc_size, r->pkt, r->pkt_dma);
	if (r->desc)
		dma_free_coherent(&w->pdev->dev, r->count * WMROM_DESC_SIZE,
				  r->desc, r->desc_dma);
	memset(r, 0, sizeof(*r));
}

static void wmrom_tx_ring_bind(struct wmrom_dev *w, struct wmrom_ring *r, u32 off)
{
	memset(r->desc, 0, r->count * WMROM_DESC_SIZE);
	r->idx = 0;
	dma_wmb();
	writel(lower_32_bits(r->desc_dma), w->bar + off);
	writel(r->count, w->bar + off + 4);
	writel(0, w->bar + off + 8);
	(void)readl(w->bar + off + 0xC);
}

static void wmrom_rx_ring_bind(struct wmrom_dev *w, struct wmrom_ring *r, u32 off)
{
	u32 i;

	memset(r->desc, 0, r->count * WMROM_DESC_SIZE);
	memset(r->pkt, 0, r->pkt_alloc_size);
	for (i = 0; i < r->count; i++) {
		__le32 *d = (__le32 *)((u8 *)r->desc + i * WMROM_DESC_SIZE);
		dma_addr_t buf_dma = r->pkt_dma + (size_t)i * WMROM_R0_BUF_SIZE;

		d[0] = cpu_to_le32(lower_32_bits(buf_dma));
		d[1] = cpu_to_le32((WMROM_R0_BUF_SIZE & 0x3FFFU) << 16);
		d[2] = cpu_to_le32(upper_32_bits(buf_dma));
		d[3] = 0;
	}
	r->idx = r->count - 1;
	dma_wmb();
	writel(lower_32_bits(r->desc_dma), w->bar + off);
	writel(r->count, w->bar + off + 4);
	writel(r->idx, w->bar + off + 8);
	(void)readl(w->bar + off + 0xC);
}

static void wmrom_dma_hw_init(struct wmrom_dev *w)
{
	u32 glo, ext1, rst, i;

	writel(0, w->bar + WMROM_INT_MSK);
	writel(0xFFFFFFFFU, w->bar + WMROM_INT_STA);

	/* Disable TX/RX DMA and reset WFDMA logic + DMASHDL (matches mt7981 hif_reset_WFDMA) */
	glo = readl(w->bar + WMROM_GLO_CFG);
	writel(glo & ~(WMROM_DMA_EN_TX | WMROM_DMA_EN_RX), w->bar + WMROM_GLO_CFG);
	for (i = 0; i < 20; i++) {
		if (!(readl(w->bar + WMROM_GLO_CFG) & (WMROM_DMA_BUSY_TX | WMROM_DMA_BUSY_RX)))
			break;
		msleep(1);
	}

	rst = readl(w->bar + WMROM_CONN_HIF_RST);
	writel(rst & ~0x30U, w->bar + WMROM_CONN_HIF_RST);
	(void)readl(w->bar + WMROM_CONN_HIF_RST);
	udelay(50);
	writel(rst | 0x30U, w->bar + WMROM_CONN_HIF_RST);
	(void)readl(w->bar + WMROM_CONN_HIF_RST);
	msleep(2);

	/* Program WFDMA ring prefetch EXT_CTRL (matches mt7981 hif_init_WFDMA) */
	for (i = 0; i < 6; i++)
		writel((i * 0x00400000U) | 0x4U, w->bar + WMROM_RX0_EXT_CTRL + i * 4);
	for (i = 0; i < 5; i++)
		writel(((i + 6) * 0x00400000U) | 0x4U, w->bar + WMROM_TX16_EXT_CTRL + i * 4);

	/* Configure GLO_CFG (keep FIFO_LITTLE_ENDIAN bit 12 set!) and GLO_CFG_EXT1 */
	glo = readl(w->bar + WMROM_GLO_CFG);
	glo |= WMROM_DMA_OMIT_TX_INFO | WMROM_DMA_OMIT_RX_PFET2 |
	       WMROM_DMA_FIFO_LE | WMROM_DMA_RX_WB_DDONE | WMROM_DMA_TX_WB_DDONE;
	glo &= ~(WMROM_DMA_OMIT_RX_INFO | WMROM_DMA_CHAIN_EN |
		 WMROM_DMA_BIG_ENDIAN | WMROM_DMA_BYTE_SWAP);
	writel(glo, w->bar + WMROM_GLO_CFG);

	ext1 = readl(w->bar + WMROM_GLO_CFG_EXT1);
	writel(ext1 | BIT(28), w->bar + WMROM_GLO_CFG_EXT1);

	/* Reset DMA pointers, then bind T16 (scatter), T17 (cmd), and R0 (event) */
	writel(0xFFFFFFFFU, w->bar + WMROM_RST_DTX);
	writel(0xFFFFFFFFU, w->bar + WMROM_RST_DRX);
	wmrom_tx_ring_bind(w, &w->t16, WMROM_T16);
	wmrom_tx_ring_bind(w, &w->t17, WMROM_T17);
	wmrom_rx_ring_bind(w, &w->r0, WMROM_R0);

	/* Enable both TX and RX DMA so ROM command replies on R0 are drained */
	glo = readl(w->bar + WMROM_GLO_CFG);
	writel(glo | WMROM_DMA_EN_TX | WMROM_DMA_EN_RX, w->bar + WMROM_GLO_CFG);
	(void)readl(w->bar + WMROM_GLO_CFG);
}

static void wmrom_r0_drain(struct wmrom_dev *w)
{
	u32 guard;

	for (guard = 0; guard < w->r0.count; guard++) {
		u32 dma_idx = readl(w->bar + WMROM_R0 + 0xC) & 0xFFFFU;
		u32 next = (w->r0.idx + 1) % w->r0.count;
		__le32 *d;

		if (next == dma_idx)
			break;
		d = (__le32 *)((u8 *)w->r0.desc + next * WMROM_DESC_SIZE);
		d[1] = cpu_to_le32((WMROM_R0_BUF_SIZE & 0x3FFFU) << 16);
		dma_wmb();
		w->r0.idx = next;
		writel(w->r0.idx, w->bar + WMROM_R0 + 8);
	}
}

static int wmrom_r0_wait_rsp(struct wmrom_dev *w, u8 cmd_type, u8 seq, u8 *out_status)
{
	int us;

	for (us = 0; us < WMROM_TIMEOUT_US; us += 10) {
		u32 dma_idx = readl(w->bar + WMROM_R0 + 0xC) & 0xFFFFU;
		u32 next = (w->r0.idx + 1) % w->r0.count;

		if (next != dma_idx) {
			static const u32 hdr_offs[] = { 24, 0, 16, 32 };
			__le32 *d = (__le32 *)((u8 *)w->r0.desc + next * WMROM_DESC_SIZE);
			const u8 *pkt = (const u8 *)w->r0.pkt + (size_t)next * WMROM_R0_BUF_SIZE;
			const u8 *hdr = NULL;
			u32 rxd0 = 0, k;
			u16 pkt_id = 0;
			u8 eid = 0, rseq = 0, status = 0;

			dma_rmb();
			for (k = 0; k < ARRAY_SIZE(hdr_offs); k++) {
				const u8 *cand = pkt + hdr_offs[k];
				u32 w0 = le32_to_cpu(*((const __le32 *)cand));
				u16 pid = (w0 >> 16) & 0xFFFFU;

				if (pid == WMROM_PKT_ID_EVENT || pid == 0x0007U) {
					hdr = cand;
					rxd0 = w0;
					pkt_id = pid;
					break;
				}
			}
			if (hdr) {
				eid = hdr[4];
				rseq = hdr[5];
				status = hdr[8];
			} else {
				dev_warn(&w->pdev->dev, "R0 slot %u unknown hdr: %*ph\n",
					 next, 40, pkt);
			}

			/* Return descriptor to R0 immediately */
			d[1] = cpu_to_le32((WMROM_R0_BUF_SIZE & 0x3FFFU) << 16);
			dma_wmb();
			w->r0.idx = next;
			writel(w->r0.idx, w->bar + WMROM_R0 + 8);

			if (!hdr || rseq != seq)
				continue;

			if (out_status)
				*out_status = status;
			dev_info(&w->pdev->dev,
				 "R0 rsp: cmd=%#x eid=%#x seq=%u status=%u (hdr_off=%td)\n",
				 cmd_type, eid, rseq, status, hdr - pkt);
			return 0;
		}
		udelay(10);
	}
	dev_err(&w->pdev->dev,
		"R0 timeout cmd=%#x seq=%u r0=(%u/%u) t16=(%u/%u) t17=(%u/%u) glo=%#x mcu_glo=%#x mcu_sta=%#x pda=(%#x,%#x,%#x)\n",
		cmd_type, seq, w->r0.idx, readl(w->bar + WMROM_R0 + 0xC),
		readl(w->bar + WMROM_T16 + 8), readl(w->bar + WMROM_T16 + 0xC),
		readl(w->bar + WMROM_T17 + 8), readl(w->bar + WMROM_T17 + 0xC),
		readl(w->bar + WMROM_GLO_CFG),
		readl(w->bar + WMROM_MCU_GLO_CFG), readl(w->bar + WMROM_MCU_INT_STA),
		readl(w->bar + WMROM_PDA_BASE), readl(w->bar + WMROM_PDA_BASE + 4),
		readl(w->bar + WMROM_PDA_STATUS));
	return -ETIMEDOUT;
}

/* Send one packet on a TX ring and wait until the device consumed it. */
static int wmrom_ring_send(struct wmrom_dev *w, struct wmrom_ring *r, u32 off,
			   const void *data, u32 len)
{
	__le32 *d;
	u32 next;
	int us;

	if (len > WMROM_PKT_SIZE)
		return -EINVAL;
	memcpy(r->pkt, data, len);
	dma_wmb();

	d = (__le32 *)((u8 *)r->desc + r->idx * WMROM_DESC_SIZE);
	d[0] = cpu_to_le32(lower_32_bits(r->pkt_dma));
	d[1] = cpu_to_le32(((len & 0x3FFFU) << 16) | BIT(30));
	d[2] = cpu_to_le32(upper_32_bits(r->pkt_dma));
	d[3] = 0;
	dma_wmb();

	next = (r->idx + 1) % r->count;
	writel(next, w->bar + off + 8);
	for (us = 0; us < WMROM_TIMEOUT_US; us += 10) {
		if ((readl(w->bar + off + 0xC) & 0xFFFFU) == next) {
			r->idx = next;
			return 0;
		}
		udelay(10);
	}
	dev_err(&w->pdev->dev, "ring %#x timeout idx=%u dma_idx=%u glo=%#x fw_stat=%#x\n",
		off, next, readl(w->bar + off + 0xC),
		readl(w->bar + WMROM_GLO_CFG), readl(w->bar + WMROM_FW_STAT));
	return -ETIMEDOUT;
}

static int wmrom_cmd_ex(struct wmrom_dev *w, u8 type, u8 ext_cid, u8 set_query,
			const void *payload, u32 len, bool wait_rsp, u8 *out_status)
{
	u8 buf[512];
	u32 total = 32 + 32 + len;
	__le32 *txd = (__le32 *)buf;
	__le32 *fw = (__le32 *)(buf + 32);
	u8 seq;
	int ret;

	if (total > sizeof(buf))
		return -EINVAL;

	wmrom_r0_drain(w);

	w->seq = (w->seq % 15) + 1;
	seq = w->seq;

	memset(buf, 0, total);
	txd[0] = cpu_to_le32(total | (0x20U << 25) | (2U << 23));
	txd[1] = cpu_to_le32(BIT(31) | BIT(16));
	fw[0] = cpu_to_le32((32 + len) | (WMROM_PQ_ID_CMD << 16));
	fw[1] = cpu_to_le32(type | (WMROM_PKT_ID_CMD << 8) |
			    ((u32)set_query << 16) | ((u32)seq << 24));
	fw[2] = cpu_to_le32(ext_cid);
	if (len)
		memcpy(buf + 64, payload, len);

	ret = wmrom_ring_send(w, &w->t17, WMROM_T17, buf, total);
	if (ret || !wait_rsp)
		return ret;
	return wmrom_r0_wait_rsp(w, type, seq, out_status);
}

static int wmrom_cmd(struct wmrom_dev *w, u8 type, const void *payload, u32 len)
{
	u8 status = 0;
	int ret;

	ret = wmrom_cmd_ex(w, type, 0, WMROM_CMD_NA, payload, len, true, &status);
	if (ret)
		return ret;
	if (type == WMROM_CMD_PATCH_SEM) {
		/* status: 0=PATCH_IS_DL, 2=PATCH_NOT_DL_YET, 3=SEM_RELEASE */
		if (status != 0 && status != 2 && status != 3) {
			dev_err(&w->pdev->dev, "patch_sem unexpected status=%u\n", status);
			return -EPROTO;
		}
		return status;
	}
	if (status != 0) {
		dev_err(&w->pdev->dev, "cmd %#x failed with ROM status=%u\n", type, status);
		return -EPROTO;
	}
	return 0;
}

static int wmrom_addrlen(struct wmrom_dev *w, u8 type, u32 addr, u32 len, u32 mode)
{
	__le32 p[3];

	p[0] = cpu_to_le32(addr);
	p[1] = cpu_to_le32(len);
	p[2] = cpu_to_le32(mode | WMROM_ADDRLEN_NEED_RSP);
	return wmrom_cmd(w, type, p, sizeof(p));
}

static int wmrom_scatter(struct wmrom_dev *w, const u8 *data, u32 len)
{
	while (len) {
		u32 chunk = min_t(u32, len, WMROM_SCATTER_MAX);
		int ret = wmrom_ring_send(w, &w->t16, WMROM_T16, data, chunk);

		if (ret)
			return ret;
		data += chunk;
		len -= chunk;
	}
	return 0;
}

static int wmrom_hw_power_cycle(struct wmrom_dev *w)
{
	u32 glo, i;
	int ret;

	if (w->bar) {
		glo = readl(w->bar + WMROM_GLO_CFG);
		writel(glo & ~(WMROM_DMA_EN_TX | WMROM_DMA_EN_RX),
		       w->bar + WMROM_GLO_CFG);
		msleep(2);
	}
	if (w->pwr_off) {
		w->pwr_off(CONNDRV_TYPE_WIFI);
		msleep(20);
	}
	ret = wmrom_hw_power_on(w);
	if (ret)
		return ret;
	wmrom_dma_hw_init(w);

	for (i = 0; i < 500; i++) {
		if (wmrom_fw_state(w) == 1) {
			dev_info(&w->pdev->dev, "power cycle complete, fw_state=1\n");
			return 0;
		}
		msleep(1);
	}
	dev_warn(&w->pdev->dev, "power cycle finished with fw_state=%u\n",
		 wmrom_fw_state(w));
	return 0;
}

static int wmrom_restart(struct wmrom_dev *w)
{
	u32 i;
	u8 pwr_payload[4] = { 1, 0, 0, 0 };

	if (wmrom_fw_state(w) == 1)
		return 0;

	/* Try MT7981 polling-mode NIC_POWER_CTRL(0x04) first if FW is running */
	if (wmrom_fw_state(w) == 3) {
		(void)wmrom_cmd_ex(w, WMROM_CMD_NIC_PWR_CTRL, 0, 1,
				   pwr_payload, sizeof(pwr_payload), false, NULL);
		for (i = 0; i < 200; i++) {
			if (wmrom_fw_state(w) == 1) {
				wmrom_dma_hw_init(w);
				return 0;
			}
			msleep(1);
		}
	}

	/* Fall back to deterministic conninfra power cycle */
	return wmrom_hw_power_cycle(w);
}

static int wmrom_load_patch(struct wmrom_dev *w, const u8 *data, u32 len)
{
	__le32 sem;
	int ret;

	sem = cpu_to_le32(1);
	ret = wmrom_cmd(w, WMROM_CMD_PATCH_SEM, &sem, 4);
	if (ret < 0)
		return ret;

	ret = wmrom_addrlen(w, WMROM_CMD_PATCH_START, WMROM_PATCH_OFFSET, len, 0);
	if (ret)
		return ret;
	ret = wmrom_scatter(w, data, len);
	if (ret)
		return ret;
	sem = 0;
	ret = wmrom_cmd(w, WMROM_CMD_PATCH_FINISH, &sem, 4);
	if (ret)
		return ret;
	sem = cpu_to_le32(3);
	ret = wmrom_cmd(w, WMROM_CMD_PATCH_SEM, &sem, 4);
	return ret < 0 ? ret : 0;
}

static int wmrom_load_emi_region(struct wmrom_dev *w, u32 addr, const u8 *data, u32 len)
{
	u32 off;

	if (!w->emi || addr < WMROM_MCU_EMI_BASE)
		return -EINVAL;
	off = addr - WMROM_MCU_EMI_BASE;
	if ((u64)off + len > w->emi_size) {
		dev_err(&w->pdev->dev, "EMI region %#x+%#x exceeds %#x\n",
			addr, len, w->emi_size);
		return -EINVAL;
	}
	memcpy_toio(w->emi + off, data, len);
	wmb();
	(void)readl(w->emi + off);
	return 0;
}

static int wmrom_load_region(struct wmrom_dev *w, u32 addr, const u8 *data,
			     u32 len, u32 flags)
{
	u8 feat = (flags >> 8) & 0xFFU;
	u32 mode = 0;
	int ret;

	if (feat & 0x01U)
		mode |= BIT(0) | BIT(3);
	mode |= ((u32)(feat & 0x0EU));
	if (feat & 0x10U)
		mode |= BIT(6);
	if (flags & WMROM_FLAG_WA)
		mode |= WMROM_ADDRLEN_WA;

	ret = wmrom_addrlen(w, WMROM_CMD_ADDRLEN, addr, len, mode);
	if (ret)
		return ret;
	return wmrom_scatter(w, data, len);
}

static int wmrom_load_group(struct wmrom_dev *w, const u8 *img, u32 size,
			  const struct wmrom_region *regions, u32 count,
			  bool wa, bool *has_group, u32 *override_flag, u32 *entry)
{
	u32 i;
	int ret;

	for (i = 0; i < count; i++) {
		u32 addr = le32_to_cpu(regions[i].addr);
		u32 len = le32_to_cpu(regions[i].len);
		u32 off = le32_to_cpu(regions[i].offset);
		u32 flags = le32_to_cpu(regions[i].flags);
		bool is_wa = !!(flags & WMROM_FLAG_WA);

		if (is_wa != wa)
			continue;
		if ((u64)off + len > size)
			return -EINVAL;
		*has_group = true;
		if (flags & WMROM_FLAG_PATCH) {
			if (addr != WMROM_PATCH_OFFSET)
				return -EINVAL;
			ret = wmrom_load_patch(w, img + off, len);
		} else if ((flags & WMROM_FLAG_EMI) || addr >= WMROM_MCU_EMI_BASE) {
			ret = wmrom_load_emi_region(w, addr, img + off, len);
		} else {
			if (flags & WMROM_FLAG_OVERRIDE) {
				*override_flag = 1;
				*entry = addr;
			}
			ret = wmrom_load_region(w, addr, img + off, len, flags);
		}
		if (ret) {
			dev_err(&w->pdev->dev,
				"region %u (%s %#x len=%u flags=%#x) failed: %d\n",
				i, wa ? "WA" : "WM", addr, len, flags, ret);
			return ret;
		}
	}
	return 0;
}

static int wmrom_load_image(struct wmrom_dev *w, const u8 *img, u32 size)
{
	const struct wmrom_region *regions;
	__le32 hdr[4];
	u32 count, wm_entry, wm_override = 0, wa_entry = 0, wa_override = 0, i;
	bool has_wm = false, has_wa = false, seen_normal = false;
	u32 normal_wait = 0, sw_cr;
	int ret;

	if (size < sizeof(hdr) || le32_to_cpu(((const __le32 *)img)[0]) != WMROM_MAGIC)
		return -EINVAL;
	memcpy(hdr, img, sizeof(hdr));
	count = le32_to_cpu(hdr[3]);
	wm_entry = le32_to_cpu(hdr[2]);
	if (count == 0 || count > 64 || sizeof(hdr) + count * sizeof(*regions) > size)
		return -EINVAL;
	regions = (const struct wmrom_region *)(img + sizeof(hdr));

	ret = wmrom_restart(w);
	if (ret)
		return ret;

	/* Pre-set WF_SW_DEF_CR_ICAP_SPECTRUM_MODE_ADDR (0x0041143C) = 1 (ICapMode) */
	sw_cr = wmrom_l2_read32(w, WMROM_ICAP_MODE_CR);
	wmrom_l2_write32(w, WMROM_ICAP_MODE_CR, (sw_cr & ~0x3U) | 0x1U);

	ret = wmrom_load_group(w, img, size, regions, count, false,
			       &has_wm, &wm_override, &wm_entry);
	if (ret)
		return ret;

	/* Ensure ICapMode=1 is still latched right before WM FwStartReq */
	sw_cr = wmrom_l2_read32(w, WMROM_ICAP_MODE_CR);
	wmrom_l2_write32(w, WMROM_ICAP_MODE_CR, (sw_cr & ~0x3U) | 0x1U);

	if (has_wm) {
		__le32 p[2] = {
			cpu_to_le32((wm_override || wm_entry) ? WMROM_START_OVERRIDE : 0U),
			cpu_to_le32(wm_entry)
		};

		ret = wmrom_cmd(w, WMROM_CMD_START, p, sizeof(p));
		if (ret)
			return ret;
	}

	ret = wmrom_load_group(w, img, size, regions, count, true,
			       &has_wa, &wa_override, &wa_entry);
	if (ret)
		return ret;

	if (has_wa) {
		__le32 p[2] = {
			cpu_to_le32((wa_override ? WMROM_START_OVERRIDE : 0U) | WMROM_START_WA),
			cpu_to_le32(wa_override ? wa_entry : 0U)
		};

		ret = wmrom_cmd(w, WMROM_CMD_START, p, sizeof(p));
		if (ret)
			return ret;
	}

	for (i = 0; i < 3000; i++) {
		u32 st = wmrom_fw_state(w);

		if (st == 7 || (st == 3 && !has_wa)) {
			dev_info(&w->pdev->dev, "firmware running, fw_state=%u\n", st);
			return 0;
		}
		if (st == 3 && has_wa) {
			seen_normal = true;
			if (++normal_wait >= 500) {
				dev_info(&w->pdev->dev, "firmware running, fw_state=%u\n", st);
				return 0;
			}
		}
		msleep(1);
	}
	if (seen_normal) {
		u32 st = wmrom_fw_state(w);

		dev_info(&w->pdev->dev, "firmware running, fw_state=%u\n", st);
		return 0;
	}
	dev_err(&w->pdev->dev, "firmware start timeout, fw_state=%u\n",
		wmrom_fw_state(w));
	return -ETIMEDOUT;
}

struct wmrom_load_req {
	__u64 user_ptr;
	__u32 len;
	__u32 entry;
};

struct wmrom_reg {
	__u32 off;
	__u32 val;
};

#define WMROM_IOC_MAGIC		'R'
#define WMROM_IOC_LOAD		_IOW(WMROM_IOC_MAGIC, 1, struct wmrom_load_req)
#define WMROM_IOC_FWSTATE	_IOR(WMROM_IOC_MAGIC, 2, __u32)
#define WMROM_IOC_RESTART	_IO(WMROM_IOC_MAGIC, 3)
#define WMROM_IOC_BAR_READ	_IOWR(WMROM_IOC_MAGIC, 4, struct wmrom_reg)
#define WMROM_IOC_PWRCYCLE	_IO(WMROM_IOC_MAGIC, 5)

static long wmrom_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct wmrom_dev *w = wmrom;
	struct wmrom_load_req req;
	struct wmrom_reg reg;
	u32 state;
	void *buf;
	int ret;

	if (!w || !w->bar)
		return -ENODEV;
	switch (cmd) {
	case WMROM_IOC_FWSTATE:
		state = wmrom_fw_state(w);
		return copy_to_user((void __user *)arg, &state, sizeof(state)) ? -EFAULT : 0;
	case WMROM_IOC_RESTART:
		mutex_lock(&w->lock);
		ret = wmrom_restart(w);
		mutex_unlock(&w->lock);
		return ret;
	case WMROM_IOC_PWRCYCLE:
		mutex_lock(&w->lock);
		ret = wmrom_hw_power_cycle(w);
		mutex_unlock(&w->lock);
		return ret;
	case WMROM_IOC_BAR_READ:
		if (copy_from_user(&reg, (void __user *)arg, sizeof(reg)))
			return -EFAULT;
		if (reg.off > WMROM_BAR_SIZE - 4 || (reg.off & 3))
			return -EINVAL;
		reg.val = readl(w->bar + reg.off);
		return copy_to_user((void __user *)arg, &reg, sizeof(reg)) ? -EFAULT : 0;
	case WMROM_IOC_LOAD:
		if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
			return -EFAULT;
		if (req.len < 16 || req.len > 8 * 1024 * 1024)
			return -EINVAL;
		buf = vmalloc(req.len);
		if (!buf)
			return -ENOMEM;
		if (copy_from_user(buf, u64_to_user_ptr(req.user_ptr), req.len)) {
			vfree(buf);
			return -EFAULT;
		}
		mutex_lock(&w->lock);
		ret = wmrom_load_image(w, buf, req.len);
		mutex_unlock(&w->lock);
		vfree(buf);
		return ret;
	default:
		return -ENOTTY;
	}
}

static const struct file_operations wmrom_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = wmrom_ioctl,
	.llseek = no_llseek,
};

static int wmrom_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	conninfra_get_emi_fn get_emi;
	unsigned int emi_sz = 0;
	phys_addr_t emi_pa = 0;
	int ret;

	if (wmrom)
		return -EBUSY;
	wmrom = kzalloc(sizeof(*wmrom), GFP_KERNEL);
	if (!wmrom)
		return -ENOMEM;
	wmrom->pdev = pdev;
	mutex_init(&wmrom->lock);

	wmrom->pwr_on = (conninfra_pwr_on_fn)kallsyms_lookup_name("conninfra_pwr_on");
	wmrom->pwr_off = (conninfra_pwr_off_fn)kallsyms_lookup_name("conninfra_pwr_off");
	get_emi = (conninfra_get_emi_fn)kallsyms_lookup_name("conninfra_get_emi_phy_addr");
	if (get_emi)
		get_emi(CONNSYS_EMI_FW, &emi_pa, &emi_sz);
	wmrom->emi_phys = emi_pa ? emi_pa : WMROM_EMI_PHYS_DEFAULT;
	wmrom->emi_size = emi_sz ? emi_sz : WMROM_EMI_SIZE_DEFAULT;

	ret = pci_request_region(pdev, 0, WMROM_NAME);
	if (ret)
		goto err_free;
	if (pci_enable_device(pdev))
		dev_warn(&pdev->dev, "pci_enable_device failed; forcing MEM/MASTER\n");
	pci_set_master(pdev);
	pci_write_config_word(pdev, PCI_COMMAND,
			      PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER | PCI_COMMAND_SERR);
	wmrom->bar = pci_iomap(pdev, 0, 0);
	if (!wmrom->bar) {
		ret = -ENOMEM;
		goto err_disable;
	}
	wmrom->emi = ioremap(wmrom->emi_phys, wmrom->emi_size);
	if (!wmrom->emi) {
		ret = -ENOMEM;
		goto err_unmap_bar;
	}

	ret = wmrom_ring_alloc(wmrom, &wmrom->t16, WMROM_T16_COUNT, WMROM_PKT_SIZE);
	if (ret)
		goto err_unmap_emi;
	ret = wmrom_ring_alloc(wmrom, &wmrom->t17, WMROM_T17_COUNT, WMROM_PKT_SIZE);
	if (ret)
		goto err_unmap_emi;
	ret = wmrom_ring_alloc(wmrom, &wmrom->r0, WMROM_R0_COUNT,
			       WMROM_R0_COUNT * WMROM_R0_BUF_SIZE);
	if (ret)
		goto err_unmap_emi;

	ret = wmrom_hw_power_on(wmrom);
	if (ret)
		goto err_unmap_emi;
	wmrom_dma_hw_init(wmrom);

	ret = alloc_chrdev_region(&wmrom->devt, 0, 1, WMROM_NAME);
	if (ret)
		goto err_unmap_emi;
	cdev_init(&wmrom->cdev, &wmrom_fops);
	wmrom->cdev.owner = THIS_MODULE;
	ret = cdev_add(&wmrom->cdev, wmrom->devt, 1);
	if (ret)
		goto err_region;
	wmrom->class = class_create(THIS_MODULE, WMROM_NAME);
	if (IS_ERR(wmrom->class)) {
		ret = PTR_ERR(wmrom->class);
		goto err_cdev;
	}
	wmrom->device = device_create(wmrom->class, &pdev->dev, wmrom->devt, NULL, WMROM_NAME);
	if (IS_ERR(wmrom->device)) {
		ret = PTR_ERR(wmrom->device);
		goto err_class;
	}
	dev_info(&pdev->dev, "wmrom ready, fw_state=%u emi=%pa+%#x\n",
		 wmrom_fw_state(wmrom), &wmrom->emi_phys, wmrom->emi_size);
	return 0;

err_class:
	class_destroy(wmrom->class);
err_cdev:
	cdev_del(&wmrom->cdev);
err_region:
	unregister_chrdev_region(wmrom->devt, 1);
err_unmap_emi:
	if (wmrom->pwr_held && wmrom->pwr_off) {
		wmrom->pwr_off(CONNDRV_TYPE_WIFI);
		wmrom->pwr_held = false;
	}
	wmrom_ring_free(wmrom, &wmrom->r0);
	wmrom_ring_free(wmrom, &wmrom->t17);
	wmrom_ring_free(wmrom, &wmrom->t16);
	iounmap(wmrom->emi);
err_unmap_bar:
	pci_iounmap(pdev, wmrom->bar);
err_disable:
	pci_disable_device(pdev);
	pci_release_region(pdev, 0);
err_free:
	kfree(wmrom);
	wmrom = NULL;
	return ret;
}

static void wmrom_pci_remove(struct pci_dev *pdev)
{
	u32 glo;

	if (!wmrom)
		return;
	if (wmrom->bar) {
		glo = readl(wmrom->bar + WMROM_GLO_CFG);
		writel(glo & ~(WMROM_DMA_EN_TX | WMROM_DMA_EN_RX),
		       wmrom->bar + WMROM_GLO_CFG);
		msleep(2);
	}
	if (wmrom->pwr_held && wmrom->pwr_off) {
		wmrom->pwr_off(CONNDRV_TYPE_WIFI);
		wmrom->pwr_held = false;
	}
	device_destroy(wmrom->class, wmrom->devt);
	class_destroy(wmrom->class);
	cdev_del(&wmrom->cdev);
	unregister_chrdev_region(wmrom->devt, 1);
	wmrom_ring_free(wmrom, &wmrom->r0);
	wmrom_ring_free(wmrom, &wmrom->t17);
	wmrom_ring_free(wmrom, &wmrom->t16);
	if (wmrom->emi)
		iounmap(wmrom->emi);
	if (wmrom->bar)
		pci_iounmap(pdev, wmrom->bar);
	if (pci_is_enabled(pdev))
		pci_disable_device(pdev);
	pci_release_region(pdev, 0);
	kfree(wmrom);
	wmrom = NULL;
}

static const struct pci_device_id wmrom_ids[] = {
	{ PCI_DEVICE(0x14c3, 0x7981) },
	{ }
};
MODULE_DEVICE_TABLE(pci, wmrom_ids);

static struct pci_driver wmrom_driver = {
	.name = WMROM_NAME,
	.id_table = wmrom_ids,
	.probe = wmrom_pci_probe,
	.remove = wmrom_pci_remove,
};

module_pci_driver(wmrom_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CT3003 WM ROM firmware loader");
