/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * wm_sdr.h - CT3003 WM access / ROM-loader / IQ snapshot driver definitions.
 *
 * Two backends:
 *   remap : coexists with mt_wifi, reads/writes WM memory through the
 *           connsys L2 remap window (control 0x184001b8, data 0x18440000).
 *   pci   : claims 14c3:7981 and drives the WM ROM downloader (FWDL).
 */
#ifndef WM_SDR_H
#define WM_SDR_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define WMSDR_NAME		"wmsdr"
#define WMSDR_MAX_XFER		0x40000	/* 256 KiB per READ_MEM/DMA_READ */

/* Host physical window that contains the connsys block (BAR0). */
#define WMSDR_BAR_PHYS		0x18000000UL
#define WMSDR_BAR_LEN		0x00800000UL

/* L2 remap: page number in [31:16], 64 KiB data window. */
#define WMSDR_REMAP_CTL		0x4001B8UL
#define WMSDR_REMAP_WIN		0x440000UL
#define WMSDR_REMAP_PAGE_MASK	0xffff0000UL
#define WMSDR_REMAP_LOW_MASK	0x0000ffffUL

/* Direct RBUS offsets inside BAR0. */
#define WMSDR_CONN2AP_EMI_START	0x00E360UL
#define WMSDR_CONN2AP_EMI_END	0x00E364UL

#define WMSDR_MEM_DMA_BASE	0x406000UL	/* 0x58000000 */
#define WMSDR_MEM_DMA_LPBK	(WMSDR_MEM_DMA_BASE + 0x140UL)
#define WMSDR_MEM_DMA_GLO_CFG	(WMSDR_MEM_DMA_BASE + 0x208UL)
#define WMSDR_MEM_DMA_RST_DTX	(WMSDR_MEM_DMA_BASE + 0x20CUL)
#define WMSDR_MEM_DMA_RST_DRX	(WMSDR_MEM_DMA_BASE + 0x280UL)
#define WMSDR_MEM_DMA_T2	(WMSDR_MEM_DMA_BASE + 0x320UL)
#define WMSDR_MEM_DMA_R2	(WMSDR_MEM_DMA_BASE + 0x520UL)
#define WMSDR_MEM_DMA_T2_EXT	(WMSDR_MEM_DMA_BASE + 0x608UL)
#define WMSDR_MEM_DMA_R2_EXT	(WMSDR_MEM_DMA_BASE + 0x688UL)

#define WMSDR_ICAP_BAR_LO	0x4B3000UL	/* 0x80023000 */
#define WMSDR_ICAP_BAR_HI	0x4B3100UL
#define WMSDR_ICAP_CR0		(WMSDR_ICAP_BAR_LO + 0x90UL)
#define WMSDR_ICAP_CR2		(WMSDR_ICAP_BAR_LO + 0x98UL)
#define WMSDR_ICAP_CR3		(WMSDR_ICAP_BAR_LO + 0x9CUL)
#define WMSDR_ICAP_CR9		(WMSDR_ICAP_BAR_LO + 0xB4UL)
#define WMSDR_ICAP_CR10		(WMSDR_ICAP_BAR_LO + 0xB8UL)

/* FW status register (CONN_HOST_CSR_TOP + 0xf0). */
#define WMSDR_FW_STAT		0x0600F0UL
#define WMSDR_FW_STAT_MASK	0x7

/* WFDMA rings (host view, offsets from BAR). */
#define WMSDR_WFDMA_GLO_CFG	0x024208UL
#define WMSDR_WFDMA_RST_DTX	0x02420CUL
#define WMSDR_WFDMA_RST_DRX	0x024280UL
#define WMSDR_WFDMA_INT_STA	0x024200UL
#define WMSDR_WFDMA_INT_ENA	0x024204UL
#define WMSDR_WFDMA_T16		0x024400UL	/* FWDL */
#define WMSDR_WFDMA_T17		0x024410UL	/* cmd -> WM */
#define WMSDR_WFDMA_R0		0x024500UL	/* event <- WM */

/* WM address map (for the remap check). */
#define WMSDR_WM_RAM_LO		0x02200000UL
#define WMSDR_WM_RAM_HI		0x02350000UL
#define WMSDR_WM_ROM_LO		0x00800000UL
#define WMSDR_WM_ROM_HI		0x00910000UL
#define WMSDR_WM_ICAP_LO	0x80000000UL
#define WMSDR_WM_ICAP_HI	0x84000000UL
#define WMSDR_WM_EMI_LO		0xF0000000UL
#define WMSDR_WM_EMI_HI		0xF0100000UL

/* Firmware states (DRV_FW_STAT_SYNC[2:0]). */
#define WMSDR_FW_INITIAL	0
#define WMSDR_FW_DOWNLOAD	1
#define WMSDR_FW_NORMAL_OP	2
#define WMSDR_FW_NORMAL_TRX	3
#define WMSDR_FW_WACPU_RDY	7

/* Backends. */
#define WMSDR_BACKEND_REMAP	0
#define WMSDR_BACKEND_PCI	1

/* ------------------------------------------------------------------ */
/* user <-> kernel ABI                                                */
/* ------------------------------------------------------------------ */

#define WMSDR_INFO_F_DMA_READY	(1U << 0)

struct wmsdr_info {
	__u32 version;
	__u32 backend;
	__u32 fw_state;
	__u32 fw_stat_addr;
	__u32 bar_phys;
	__u32 bar_len;
	__u32 remap_ctl;
	__u32 remap_win;
	__u32 chip_vendor;
	__u32 chip_device;
	__u32 loaded;
	__u32 flags;
};

struct wmsdr_word {
	__u32 addr;
	__u32 value;
};

struct wmsdr_mem {
	__u32 addr;
	__u32 len;
	__u64 user_ptr;
};

struct wmsdr_mmap_req {
	__u32 addr;	/* in: WM/UMAC address to DMA read */
	__u32 len;	/* in: byte length (0 = query mmap geometry only) */
	__u32 data_off;	/* out: byte offset of payload in mmap region */
	__u32 map_size;	/* out: total mappable bytes */
};

#define WMIQ_SPECTRUM_MAX_WORDS	16384U

struct wmsdr_icap_mmap_req {
	__u32 len_words;	/* in: IQ words to capture (16..65536; mode 1 max 16384) */
	__u32 timeout_ms;	/* in: capture timeout in ms (0 = default 50 ms) */
	__u32 mode;		/* in: 0 = 0xE0200000 PLE, 1 = 0x00400000/0x00408000 spectrum */
	__u32 cr0;		/* out: final ICAP CR0 */
	__u32 cr2;		/* out: final ICAP CR2 */
	__u32 cr9;		/* out: final ICAP CR9 */
	__u32 data_off;		/* out: byte offset of payload in mmap region */
	__u32 map_size;		/* out: total mappable bytes */
	__u32 sample_count;	/* out: captured word count */
};

#define WMSDR_NET_F_RAW_WSIQ	(1U << 0)	/* 0 = WS binary frame + WSIQ; 1 = raw WSIQ */
#define WMSDR_NET_MAX_SOCKS	8U

struct wmsdr_net_cfg {
	__u32 node;		/* in: WSIQ header node */
	__u32 len_words;	/* in: IQ words per frame (16..65536; mode 1 max 16384) */
	__u32 bw;		/* in: WSIQ header bw */
	__u32 band;		/* in: WSIQ header band (0=2.4G, 1=5G) */
	__u32 direct;		/* in: WSIQ header flags bit0 */
	__u32 mode;		/* in: 0 = PLE 0xE0200000, 1 = spectrum dual-bank */
	__u32 flags;		/* in: WMSDR_NET_F_* */
	__u32 sndbuf_kb;	/* in: SO_SNDBUF hint per socket in KiB (0 = 192 KiB) */
	__u32 seq_start;	/* in: initial WSIQ sequence number */
	__u32 slot_count;	/* in/out: requested/actual ring slots (0 = auto) */
};

struct wmsdr_net_sock {
	__s32 fd;		/* in: connected TCP socket fd */
	__u32 flags;		/* in: reserved (0) */
};

struct wmsdr_net_stat {
	__u32 active;		/* 1 = capture kthread running */
	__u32 n_socks;		/* currently attached TX sockets */
	__u32 next_seq;		/* next WSIQ sequence number */
	__u32 ring_depth;	/* queued frames waiting in ready ring */
	__u64 frames_captured;
	__u64 frames_sent;
	__u64 frames_dropped;
	__u64 bytes_sent;
	__u64 send_eagain;
	__u64 send_errors;
};

#define WMSDR_IOC_MAGIC		'W'

#define WMSDR_IOC_GET_INFO	_IOWR(WMSDR_IOC_MAGIC, 1, struct wmsdr_info)
#define WMSDR_IOC_RD32		_IOWR(WMSDR_IOC_MAGIC, 2, struct wmsdr_word)
#define WMSDR_IOC_WR32		_IOW(WMSDR_IOC_MAGIC, 3, struct wmsdr_word)
#define WMSDR_IOC_READ_MEM	_IOWR(WMSDR_IOC_MAGIC, 4, struct wmsdr_mem)
#define WMSDR_IOC_WRITE_MEM	_IOW(WMSDR_IOC_MAGIC, 5, struct wmsdr_mem)
#define WMSDR_IOC_GET_FW_STATE	_IOR(WMSDR_IOC_MAGIC, 6, __u32)
/* P3/P5 */
#define WMSDR_IOC_LOAD_IMAGE	_IOWR(WMSDR_IOC_MAGIC, 7, struct wmsdr_mem)
#define WMSDR_IOC_START		_IOW(WMSDR_IOC_MAGIC, 8, __u32)
#define WMSDR_IOC_RESTART_DL	_IO(WMSDR_IOC_MAGIC, 9)
#define WMSDR_IOC_DIRECT_RD32	_IOWR(WMSDR_IOC_MAGIC, 10, struct wmsdr_word)
#define WMSDR_IOC_DMA_READ	_IOWR(WMSDR_IOC_MAGIC, 11, struct wmsdr_mem)
#define WMSDR_IOC_DMA_MMAP	_IOWR(WMSDR_IOC_MAGIC, 12, struct wmsdr_mmap_req)
#define WMSDR_IOC_ICAP_DMA_MMAP	_IOWR(WMSDR_IOC_MAGIC, 13, struct wmsdr_icap_mmap_req)
#define WMSDR_IOC_NET_START	_IOWR(WMSDR_IOC_MAGIC, 14, struct wmsdr_net_cfg)
#define WMSDR_IOC_NET_ADD_SOCK	_IOW(WMSDR_IOC_MAGIC,  15, struct wmsdr_net_sock)
#define WMSDR_IOC_NET_DEL_SOCK	_IOW(WMSDR_IOC_MAGIC,  16, __s32)
#define WMSDR_IOC_NET_STOP	_IOR(WMSDR_IOC_MAGIC,  17, struct wmsdr_net_stat)
#define WMSDR_IOC_NET_STAT	_IOR(WMSDR_IOC_MAGIC,  18, struct wmsdr_net_stat)

#ifdef __KERNEL__

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mm_types.h>
#include <linux/mutex.h>

struct pci_dev;
struct file;
struct wmsdr_net_ctx;

struct wmsdr_dev {
	void __iomem *bar;
	void __iomem *emi;
	phys_addr_t emi_phys;
	u32 emi_size;
	u32 emi_wm_base;
	struct pci_dev *dma_pdev;
	struct device *dma_dev;
	struct page *rx_pages;
	void *rx_va;
	dma_addr_t rx_pa;
	u32 rx_map_size;
	u32 rx_data_off;
	u32 tx3_idx;
	u32 rx3_idx;
	bool dma_ready;
	struct mutex lock;	/* serializes remap window users */
	struct mutex dma_lock;	/* serializes MCU_TX3->HOST_RX3 users */
	struct mutex net_lock;	/* serializes async network streaming state */
	struct wmsdr_net_ctx *net;
	const char *backend;
	u32 fw_state;
	bool pci_claimed;
	struct cdev cdev;
	dev_t devt;
	struct class *class;
	struct device *device;
};

u32 wmsdr_remap_read32(struct wmsdr_dev *w, u32 addr);
void wmsdr_remap_write32(struct wmsdr_dev *w, u32 addr, u32 val);
int wmsdr_remap_read(struct wmsdr_dev *w, u32 addr, void *buf, u32 len);
int wmsdr_remap_write(struct wmsdr_dev *w, u32 addr, const void *buf, u32 len);
u32 wmsdr_direct_read32(struct wmsdr_dev *w, u32 off);
void wmsdr_direct_write32(struct wmsdr_dev *w, u32 off, u32 val);
u32 wmsdr_fw_state(struct wmsdr_dev *w);
bool wmsdr_addr_ok(u32 addr, u32 len);
bool wmsdr_direct_addr_ok(u32 off);

int wmsdr_wfdma_init(struct wmsdr_dev *w);
void wmsdr_wfdma_exit(struct wmsdr_dev *w);
bool wmsdr_wfdma_addr_ok(u32 addr, u32 len);
int wmsdr_wfdma_read(struct wmsdr_dev *w, u32 addr, void *buf, u32 len);
int wmsdr_wfdma_mmap_read(struct wmsdr_dev *w, u32 addr, u32 len);
int wmsdr_wfdma_icap_to_va(struct wmsdr_dev *w, u32 len_words, u32 mode,
			   u32 tmo_ms, void *dst_va, u32 *cr0_out,
			   u32 *cr2_out, u32 *cr9_out, u32 *sample_cnt_out);
int wmsdr_wfdma_icap_mmap(struct wmsdr_dev *w, struct wmsdr_icap_mmap_req *req);
int wmsdr_wfdma_mmap(struct wmsdr_dev *w, struct vm_area_struct *vma);

int wmsdr_net_init(struct wmsdr_dev *w);
void wmsdr_net_exit(struct wmsdr_dev *w);
long wmsdr_net_ioctl(struct wmsdr_dev *w, struct file *file,
		     unsigned int cmd, unsigned long arg);
void wmsdr_net_release_file(struct wmsdr_dev *w, struct file *file);

#endif /* __KERNEL__ */
#endif /* WM_SDR_H */
