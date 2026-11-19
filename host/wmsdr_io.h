#ifndef WMSDR_IO_H
#define WMSDR_IO_H

#include <stdint.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define WMSDR_DEV_PATH "/dev/wmsdr"

#define WMIQ_RAW_TICK_ADDR     0x00903800U
#define WMIQ_PAGE_ADDR         0x00903840U
#define WMIQ_MAGIC             0x51494D57U /* "WMIQ" LE */
#define WSIQ_MAGIC             0x51495357U /* "WSIQ" LE */
#define WMIQ_POOL_ADDR         0xE0200000U
#define WMIQ_MAX_IO_CHUNK      262144U
#define WMIQ_MAX_WORDS         65536U
#define WMIQ_SPECTRUM_MAX_WORDS 16384U

#define CHINFO_BAND0_LO        0xE00F8524U
#define CHINFO_BAND0_BW        0xE00F8530U
#define CHINFO_BAND1_LO        0xE00F8544U
#define CHINFO_BAND1_BW        0xE00F8550U

#define WMSDR_INFO_F_DMA_READY (1U << 0)

struct wmsdr_info {
	uint32_t version, backend, fw_state, fw_stat_addr;
	uint32_t bar_phys, bar_len, remap_ctl, remap_win;
	uint32_t chip_vendor, chip_device, loaded, flags;
};

struct wmsdr_word { uint32_t addr, value; };
struct wmsdr_mem { uint32_t addr, len; uint64_t user_ptr; };
struct wmsdr_mmap_req { uint32_t addr, len, data_off, map_size; };
struct wmsdr_icap_mmap_req {
	uint32_t len_words, timeout_ms, mode;
	uint32_t cr0, cr2, cr9, data_off, map_size, sample_count;
};

#define WMSDR_NET_F_RAW_WSIQ   (1U << 0)
#define WMSDR_NET_MAX_SOCKS    8U

struct wmsdr_net_cfg {
	uint32_t node, len_words, bw, band, direct, mode, flags, sndbuf_kb, seq_start, slot_count;
};

struct wmsdr_net_sock {
	int32_t fd;
	uint32_t flags;
};

struct wmsdr_net_stat {
	uint32_t active, n_socks, next_seq, ring_depth;
	uint64_t frames_captured, frames_sent, frames_dropped;
	uint64_t bytes_sent, send_eagain, send_errors;
};

struct wmsdr_mmap {
	uint8_t *base;
	uint32_t map_size;
	uint32_t data_off;
};

struct wmsdr_iq_stream {
	int fd;
	struct wmsdr_mmap *mmap;
	uint32_t node, len, bw, band;
	int direct;
	int active;
	int fast_bar0_ok;
	int cmd8_ok;
	int v1_keep;
	int spec_mode;
	uint64_t armed_ms;
};

struct wm_iq_page {
	uint32_t magic;
	uint32_t version;
	uint32_t cmd;
	uint32_t status;
	uint32_t node;
	uint32_t length;
	uint32_t data_addr;
	uint32_t sample_count;
	uint32_t tick;
	uint32_t error;
	uint32_t cr9;
	uint32_t cr2;
	uint32_t cr0;
	uint32_t bw;
	uint32_t arg0;
	uint32_t arg1;
};

#define WMSDR_IOC_MAGIC 'W'
#define WMSDR_IOC_GET_INFO     _IOWR(WMSDR_IOC_MAGIC, 1, struct wmsdr_info)
#define WMSDR_IOC_RD32         _IOWR(WMSDR_IOC_MAGIC, 2, struct wmsdr_word)
#define WMSDR_IOC_WR32         _IOW(WMSDR_IOC_MAGIC, 3, struct wmsdr_word)
#define WMSDR_IOC_READ_MEM     _IOWR(WMSDR_IOC_MAGIC, 4, struct wmsdr_mem)
#define WMSDR_IOC_WRITE_MEM    _IOW(WMSDR_IOC_MAGIC, 5, struct wmsdr_mem)
#define WMSDR_IOC_GET_FW_STATE _IOR(WMSDR_IOC_MAGIC, 6, uint32_t)
#define WMSDR_IOC_DIRECT_RD32  _IOWR(WMSDR_IOC_MAGIC, 10, struct wmsdr_word)
#define WMSDR_IOC_DMA_READ     _IOWR(WMSDR_IOC_MAGIC, 11, struct wmsdr_mem)
#define WMSDR_IOC_DMA_MMAP     _IOWR(WMSDR_IOC_MAGIC, 12, struct wmsdr_mmap_req)
#define WMSDR_IOC_ICAP_DMA_MMAP _IOWR(WMSDR_IOC_MAGIC, 13, struct wmsdr_icap_mmap_req)
#define WMSDR_IOC_NET_START    _IOWR(WMSDR_IOC_MAGIC, 14, struct wmsdr_net_cfg)
#define WMSDR_IOC_NET_ADD_SOCK _IOW(WMSDR_IOC_MAGIC,  15, struct wmsdr_net_sock)
#define WMSDR_IOC_NET_DEL_SOCK _IOW(WMSDR_IOC_MAGIC,  16, int32_t)
#define WMSDR_IOC_NET_STOP     _IOR(WMSDR_IOC_MAGIC,  17, struct wmsdr_net_stat)
#define WMSDR_IOC_NET_STAT     _IOR(WMSDR_IOC_MAGIC,  18, struct wmsdr_net_stat)

int wmsdr_open_dev(void);
int wmsdr_rd32(int fd, uint32_t addr, uint32_t *val);
int wmsdr_direct_rd32(int fd, uint32_t off, uint32_t *val);
int wmsdr_wr32(int fd, uint32_t addr, uint32_t val);
int wmsdr_remap_read_mem(int fd, uint32_t addr, uint8_t *buf, uint32_t len);
int wmsdr_dma_read(int fd, uint32_t addr, uint8_t *buf, uint32_t len);
int wmsdr_read_mem(int fd, uint32_t addr, uint8_t *buf, uint32_t len);
int wmsdr_write_mem(int fd, uint32_t addr, const uint8_t *buf, uint32_t len);
int wmsdr_mmap_open(int fd, struct wmsdr_mmap *m);
void wmsdr_mmap_close(struct wmsdr_mmap *m);
int wmsdr_dma_mmap_read(int fd, struct wmsdr_mmap *m, uint32_t addr, uint32_t len, uint8_t **data_out);
int wmsdr_get_fw_state(int fd, uint32_t *fw_state);
int wmsdr_get_info(int fd, struct wmsdr_info *info);
const char *wmsdr_status_str(uint32_t st);
int wmsdr_read_iq_page(int fd, struct wm_iq_page *p);
int wmsdr_wait_iq_done_us(int fd, struct wm_iq_page *p, int timeout_ms, useconds_t poll_us);
int wmsdr_wmpeek(int fd, uint32_t addr, uint32_t *val);
int wmsdr_wmpoke(int fd, uint32_t addr, uint32_t val);
int wmsdr_wmcall(int fd, uint32_t fn, uint32_t a0, uint32_t a1, uint32_t a2, uint32_t a3, uint32_t *ret_out);
int wmsdr_iq_arm(int fd, uint32_t node, uint32_t len, uint32_t bw, uint32_t band, int direct);
int wmsdr_iq_arm_ex(int fd, uint32_t node, uint32_t len, uint32_t bw, uint32_t band, int direct, int keep);
int wmsdr_iq_stop(int fd);
int wmsdr_iq_capture_buf(int fd, uint32_t node, uint32_t len, uint32_t bw, uint32_t band,
			 int direct, struct wm_iq_page *page_out, uint8_t *buf_out);
int wmsdr_iq_capture_mmap(int fd, struct wmsdr_mmap *m, uint32_t node, uint32_t len,
			  uint32_t bw, uint32_t band, int direct,
			  struct wm_iq_page *page_out, uint8_t **data_out);
void wmsdr_iq_stream_init(struct wmsdr_iq_stream *s, int fd, struct wmsdr_mmap *m);
void wmsdr_iq_stream_stop(struct wmsdr_iq_stream *s);
int wmsdr_iq_stream_next_mmap(struct wmsdr_iq_stream *s, uint32_t node, uint32_t len,
			      uint32_t bw, uint32_t band, int direct,
			      struct wm_iq_page *page_out, uint8_t **data_out);
int wmsdr_net_arm_hw(struct wmsdr_iq_stream *s, uint32_t node, uint32_t len,
		     uint32_t bw, uint32_t band, int direct, int *mode_out);
int wmsdr_net_start(int fd, struct wmsdr_net_cfg *cfg);
int wmsdr_net_add_sock(int fd, int sock_fd);
int wmsdr_net_del_sock(int fd, int sock_fd);
int wmsdr_net_stop(int fd, struct wmsdr_net_stat *stat_out);
int wmsdr_net_stat(int fd, struct wmsdr_net_stat *stat_out);
void wmsdr_put_le32(uint8_t *dst, uint32_t v);
int wmsdr_httpd_main(int argc, char **argv);

#endif
