#include "wmsdr_io.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>

static uint64_t stream_now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

int wmsdr_open_dev(void)
{
	return open(WMSDR_DEV_PATH, O_RDWR);
}

int wmsdr_rd32(int fd, uint32_t addr, uint32_t *val)
{
	struct wmsdr_word w = { .addr = addr };

	if (ioctl(fd, WMSDR_IOC_RD32, &w)) {
		perror("RD32");
		return -1;
	}
	*val = w.value;
	return 0;
}

int wmsdr_direct_rd32(int fd, uint32_t off, uint32_t *val)
{
	struct wmsdr_word w = { .addr = off };

	if (ioctl(fd, WMSDR_IOC_DIRECT_RD32, &w)) {
		perror("DIRECT_RD32");
		return -1;
	}
	*val = w.value;
	return 0;
}

int wmsdr_wr32(int fd, uint32_t addr, uint32_t val)
{
	struct wmsdr_word w = { .addr = addr, .value = val };

	if (ioctl(fd, WMSDR_IOC_WR32, &w)) {
		perror("WR32");
		return -1;
	}
	return 0;
}

static int wmsdr_mem_ioctl(int fd, unsigned long req, const char *tag,
			   uint32_t addr, uint8_t *buf, uint32_t len, int quiet)
{
	uint32_t off = 0;

	while (off < len) {
		uint32_t chunk = len - off;
		struct wmsdr_mem m;

		if (chunk > WMIQ_MAX_IO_CHUNK)
			chunk = WMIQ_MAX_IO_CHUNK;
		m.addr = addr + off;
		m.len = chunk;
		m.user_ptr = (uint64_t)(uintptr_t)(buf + off);
		if (ioctl(fd, req, &m)) {
			if (!quiet)
				perror(tag);
			return -1;
		}
		off += chunk;
	}
	return 0;
}

int wmsdr_remap_read_mem(int fd, uint32_t addr, uint8_t *buf, uint32_t len)
{
	return wmsdr_mem_ioctl(fd, WMSDR_IOC_READ_MEM, "READ_MEM", addr, buf, len, 0);
}

int wmsdr_dma_read(int fd, uint32_t addr, uint8_t *buf, uint32_t len)
{
	return wmsdr_mem_ioctl(fd, WMSDR_IOC_DMA_READ, "DMA_READ", addr, buf, len, 0);
}

int wmsdr_read_mem(int fd, uint32_t addr, uint8_t *buf, uint32_t len)
{
	if (wmsdr_mem_ioctl(fd, WMSDR_IOC_DMA_READ, "DMA_READ", addr, buf, len, 1) == 0)
		return 0;
	if (errno == EOPNOTSUPP || errno == ENODEV || errno == ENOTTY ||
	    errno == EBUSY || errno == EAGAIN || errno == ETIMEDOUT || errno == EIO)
		return wmsdr_remap_read_mem(fd, addr, buf, len);
	perror("DMA_READ");
	return -1;
}

int wmsdr_mmap_open(int fd, struct wmsdr_mmap *m)
{
	struct wmsdr_mmap_req req = { 0 };
	void *p;

	memset(m, 0, sizeof(*m));
	if (ioctl(fd, WMSDR_IOC_DMA_MMAP, &req) != 0)
		return -1;
	if (!req.map_size || req.data_off >= req.map_size)
		return -1;
	p = mmap(NULL, req.map_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (p == MAP_FAILED)
		return -1;
	m->base = (uint8_t *)p;
	m->map_size = req.map_size;
	m->data_off = req.data_off;
	return 0;
}

void wmsdr_mmap_close(struct wmsdr_mmap *m)
{
	if (m && m->base && m->map_size)
		munmap(m->base, m->map_size);
	if (m)
		memset(m, 0, sizeof(*m));
}

int wmsdr_dma_mmap_read(int fd, struct wmsdr_mmap *m, uint32_t addr, uint32_t len, uint8_t **data_out)
{
	struct wmsdr_mmap_req req = { .addr = addr, .len = len };

	if (!m || !m->base || !len || len > WMIQ_MAX_IO_CHUNK)
		return -1;
	if (ioctl(fd, WMSDR_IOC_DMA_MMAP, &req) != 0) {
		perror("DMA_MMAP");
		return -1;
	}
	*data_out = m->base + req.data_off;
	return 0;
}

int wmsdr_write_mem(int fd, uint32_t addr, const uint8_t *buf, uint32_t len)
{
	struct wmsdr_mem m;

	m.addr = addr;
	m.len = len;
	m.user_ptr = (uint64_t)(uintptr_t)buf;
	if (ioctl(fd, WMSDR_IOC_WRITE_MEM, &m)) {
		perror("WRITE_MEM");
		return -1;
	}
	return 0;
}

int wmsdr_get_fw_state(int fd, uint32_t *fw_state)
{
	if (ioctl(fd, WMSDR_IOC_GET_FW_STATE, fw_state)) {
		perror("GET_FW_STATE");
		return -1;
	}
	return 0;
}

int wmsdr_get_info(int fd, struct wmsdr_info *info)
{
	if (ioctl(fd, WMSDR_IOC_GET_INFO, info)) {
		perror("WMSDR_IOC_GET_INFO");
		return -1;
	}
	return 0;
}

const char *wmsdr_status_str(uint32_t st)
{
	switch (st) {
	case 0: return "IDLE";
	case 1: return "ARMED";
	case 2: return "DONE";
	case 3: return "ERROR";
	default: return "UNKNOWN";
	}
}

int wmsdr_read_iq_page(int fd, struct wm_iq_page *p)
{
	return wmsdr_read_mem(fd, WMIQ_PAGE_ADDR, (uint8_t *)p, (uint32_t)sizeof(*p));
}

int wmsdr_wait_iq_done_us(int fd, struct wm_iq_page *p, int timeout_ms, useconds_t poll_us)
{
	long elapsed_us = 0;
	long limit_us = (long)timeout_ms * 1000L;
	useconds_t step_us;

	if (poll_us == 0U)
		poll_us = 200U;
	while (elapsed_us < limit_us) {
		if (wmsdr_read_iq_page(fd, p) != 0)
			return -1;
		if (p->cmd == 0U && (p->status == 2U || p->status == 3U))
			return (p->status == 2U) ? 0 : -2;
		step_us = (elapsed_us < 3000L && poll_us > 50U) ? 50U : poll_us;
		usleep(step_us);
		elapsed_us += (long)step_us;
	}
	(void)wmsdr_read_iq_page(fd, p);
	return -ETIMEDOUT;
}

int wmsdr_wmpeek(int fd, uint32_t addr, uint32_t *val)
{
	struct wm_iq_page p = { 0 };

	if (wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x08U, 0U) ||
	    wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x0CU, 0U) ||
	    wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x38U, addr) ||
	    wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x08U, 5U))
		return -1;
	if (wmsdr_wait_iq_done_us(fd, &p, 1000, 200U) != 0)
		return -1;
	*val = p.arg1;
	return 0;
}

int wmsdr_wmpoke(int fd, uint32_t addr, uint32_t val)
{
	struct wm_iq_page p = { 0 };

	if (wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x08U, 0U) ||
	    wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x0CU, 0U) ||
	    wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x38U, addr) ||
	    wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x3CU, val) ||
	    wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x08U, 6U))
		return -1;
	if (wmsdr_wait_iq_done_us(fd, &p, 1000, 200U) != 0)
		return -1;
	return 0;
}

int wmsdr_wmcall(int fd, uint32_t fn, uint32_t a0, uint32_t a1, uint32_t a2, uint32_t a3, uint32_t *ret_out)
{
	struct wm_iq_page p = { 0 };

	if (wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x08U, 0U) ||
	    wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x0CU, 0U) ||
	    wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x38U, fn) ||
	    wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x3CU, a0) ||
	    wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x34U, a1) ||
	    wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x10U, a2) ||
	    wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x14U, a3) ||
	    wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x08U, 7U))
		return -1;
	if (wmsdr_wait_iq_done_us(fd, &p, 2000, 200U) != 0)
		return -1;
	if (ret_out)
		*ret_out = p.arg1;
	return 0;
}

void wmsdr_put_le32(uint8_t *dst, uint32_t v)
{
	dst[0] = (uint8_t)(v & 0xffU);
	dst[1] = (uint8_t)((v >> 8) & 0xffU);
	dst[2] = (uint8_t)((v >> 16) & 0xffU);
	dst[3] = (uint8_t)((v >> 24) & 0xffU);
}

int wmsdr_iq_arm_ex(int fd, uint32_t node, uint32_t len, uint32_t bw, uint32_t band, int direct, int keep)
{
	uint8_t hdr16[16];
	uint32_t arm_cmd;
	uint32_t bw_word;

	/* Band 1 (5G) and node 15 (ADC) require full API init (cmd=1 or cmd=8). */
	if (band != 0U || node == 15U)
		direct = 0;
	arm_cmd = keep ? 8U : (direct ? 4U : 1U);
	bw_word = (bw & 0xFFU) | ((band & 1U) << 8);

	wmsdr_put_le32(hdr16 + 0,  0U);		/* cmd = 0 */
	wmsdr_put_le32(hdr16 + 4,  0U);		/* status = 0 */
	wmsdr_put_le32(hdr16 + 8,  node);	/* node */
	wmsdr_put_le32(hdr16 + 12, len);	/* length */

	if (wmsdr_write_mem(fd, WMIQ_PAGE_ADDR + 0x08U, hdr16, sizeof(hdr16)) ||
	    wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x34U, bw_word) ||
	    wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x08U, arm_cmd))
		return 1;
	return 0;
}

int wmsdr_iq_arm(int fd, uint32_t node, uint32_t len, uint32_t bw, uint32_t band, int direct)
{
	return wmsdr_iq_arm_ex(fd, node, len, bw, band, direct, 0);
}

int wmsdr_iq_stop(int fd)
{
	struct wm_iq_page p = { 0 };

	if (wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x08U, 2U))
		return -1;
	return wmsdr_wait_iq_done_us(fd, &p, 500, 100U);
}

static int wmsdr_iq_arm_and_wait_ex(int fd, uint32_t node, uint32_t len, uint32_t bw,
				      uint32_t band, int direct, int keep,
				      struct wm_iq_page *page_out, uint32_t *src_addr_out)
{
	struct wm_iq_page p = { 0 };
	int rc;

	if (len == 0U || len > WMIQ_MAX_WORDS)
		return -EINVAL;
	if (wmsdr_iq_arm_ex(fd, node, len, bw, band, direct, keep))
		return -1;

	rc = wmsdr_wait_iq_done_us(fd, &p, 3000, 200U);
	if (page_out)
		*page_out = p;
	if (rc != 0)
		return rc;

	*src_addr_out = p.data_addr ? p.data_addr : WMIQ_POOL_ADDR;
	return 0;
}

static int wmsdr_iq_arm_and_wait(int fd, uint32_t node, uint32_t len, uint32_t bw,
				   uint32_t band, int direct, struct wm_iq_page *page_out,
				   uint32_t *src_addr_out)
{
	return wmsdr_iq_arm_and_wait_ex(fd, node, len, bw, band, direct, 0,
					page_out, src_addr_out);
}

static uint32_t spec_align_words(uint32_t len)
{
	return (len + 3U) & ~3U;
}

static int wmsdr_spectrum_arm(int fd, uint32_t node, uint32_t len, uint32_t bw,
			      uint32_t band, struct wm_iq_page *page_out)
{
	uint8_t req[64];
	uint32_t node_key = node;
	uint32_t dummy = 0, cr0 = 0x403U, cr9 = 0;
	uint32_t aligned = spec_align_words(len);
	uint32_t eff_band = band & 1U;
	int i;

	if (len == 0U || aligned > WMIQ_SPECTRUM_MAX_WORDS)
		return -EINVAL;
	if (node < 39U && wmsdr_rd32(fd, 0x02238C4CU + node * 12U, &node_key))
		return -1;

	memset(req, 0, sizeof(req));
	wmsdr_put_le32(req + 0x00, 1U);
	wmsdr_put_le32(req + 0x08, 0xFFFFFFFFU);
	wmsdr_put_le32(req + 0x0C, node_key);
	wmsdr_put_le32(req + 0x10, aligned);
	wmsdr_put_le32(req + 0x14, aligned);
	wmsdr_put_le32(req + 0x24, eff_band);
	wmsdr_put_le32(req + 0x28, bw & 0xFFU);
	wmsdr_put_le32(req + 0x34, eff_band);

	if (wmsdr_wr32(fd, 0x0231FEB4U, 0U) ||
	    wmsdr_write_mem(fd, WMIQ_PAGE_ADDR + 0x40U, req, sizeof(req)) ||
	    wmsdr_wmcall(fd, 0xE006CC50U, WMIQ_PAGE_ADDR + 0x40U, 1U, 0U, 0U, &dummy))
		return -1;

	for (i = 0; i < 600; i++) {
		if (wmsdr_rd32(fd, 0x80023090U, &cr0))
			return -1;
		if (!(cr0 & 0x2U))
			break;
		usleep(50);
	}
	(void)wmsdr_rd32(fd, 0x800230B4U, &cr9);
	(void)wmsdr_wr32(fd, 0x80023090U, 0x400U);
	(void)wmsdr_wr32(fd, 0x88009004U, 0U);

	if (page_out) {
		memset(page_out, 0, sizeof(*page_out));
		page_out->magic = WMIQ_MAGIC;
		page_out->version = 1U;
		page_out->status = (cr0 & 0x2U) ? 1U : 2U;
		page_out->node = node;
		page_out->length = len;
		page_out->data_addr = 0x00400000U;
		page_out->sample_count = aligned;
		page_out->cr0 = 0x400U;
		page_out->cr9 = cr9;
		page_out->bw = (bw & 0xFFU) | (eff_band << 8);
	}
	return (cr0 & 0x2U) ? -ETIMEDOUT : 0;
}

static void wmsdr_spectrum_stop(int fd)
{
	uint32_t dummy = 0;

	(void)wmsdr_wr32(fd, 0x80023090U, 0x400U);
	(void)wmsdr_wr32(fd, WMIQ_PAGE_ADDR + 0x40U, 0U);
	(void)wmsdr_wmcall(fd, 0xE006CBC2U, WMIQ_PAGE_ADDR + 0x40U, 1U, 0U, 0U, &dummy);
	(void)wmsdr_wr32(fd, 0x88009004U, 0U);
}

static int wmsdr_spectrum_read_banks_buf(int fd, uint32_t len, uint8_t *buf_out)
{
	uint32_t aligned = spec_align_words(len);
	uint32_t half_bytes = aligned * 2U;
	uint8_t *tmp = malloc((size_t)half_bytes * 2U);
	uint8_t *b0, *b1;
	uint32_t g, n_grp;

	if (!tmp)
		return -1;
	b0 = tmp;
	b1 = tmp + half_bytes;
	if (wmsdr_read_mem(fd, 0x00400000U, b0, half_bytes) ||
	    wmsdr_read_mem(fd, 0x00408000U, b1, half_bytes)) {
		free(tmp);
		return -1;
	}
	n_grp = len / 4U;
	for (g = 0; g < n_grp; g++) {
		memcpy(buf_out + (size_t)g * 16U, b0 + (size_t)g * 8U, 8U);
		memcpy(buf_out + (size_t)g * 16U + 8U, b1 + (size_t)g * 8U, 8U);
	}
	if (len & 3U) {
		uint8_t tail[16];

		memcpy(tail, b0 + (size_t)n_grp * 8U, 8U);
		memcpy(tail + 8U, b1 + (size_t)n_grp * 8U, 8U);
		memcpy(buf_out + (size_t)n_grp * 16U, tail, (len & 3U) * 4U);
	}
	free(tmp);
	return 0;
}

static int wmsdr_icap_mmap_run(int fd, struct wmsdr_mmap *m, uint32_t node,
			       uint32_t len, uint32_t bw, uint32_t eff_band,
			       uint32_t mode, struct wm_iq_page *page_out,
			       uint8_t **data_out)
{
	struct wmsdr_icap_mmap_req req = {
		.len_words = len,
		.timeout_ms = 50U,
		.mode = mode,
	};

	if (ioctl(fd, WMSDR_IOC_ICAP_DMA_MMAP, &req) != 0)
		return -1;
	if (page_out) {
		memset(page_out, 0, sizeof(*page_out));
		page_out->magic = WMIQ_MAGIC;
		page_out->version = 1U;
		page_out->status = 2U;
		page_out->node = node;
		page_out->length = len;
		page_out->data_addr = mode ? 0x00400000U : WMIQ_POOL_ADDR;
		page_out->sample_count = req.sample_count;
		page_out->cr0 = req.cr0;
		page_out->cr2 = req.cr2;
		page_out->cr9 = req.cr9;
		page_out->bw = (bw & 0xFFU) | (eff_band << 8);
	}
	*data_out = m->base + req.data_off;
	return 0;
}

int wmsdr_iq_capture_buf(int fd, uint32_t node, uint32_t len, uint32_t bw, uint32_t band,
			 int direct, struct wm_iq_page *page_out, uint8_t *buf_out)
{
	uint32_t src_addr = 0;
	int rc;

	if (len <= WMIQ_SPECTRUM_MAX_WORDS && len >= 16U) {
		rc = wmsdr_spectrum_arm(fd, node, len, bw, band, page_out);
		if (rc == 0)
			rc = wmsdr_spectrum_read_banks_buf(fd, len, buf_out);
		wmsdr_spectrum_stop(fd);
		return rc;
	}

	rc = wmsdr_iq_arm_and_wait(fd, node, len, bw, band, direct, page_out, &src_addr);
	if (rc != 0)
		return rc;
	if (wmsdr_read_mem(fd, src_addr, buf_out, len * 4U))
		return -1;
	return 0;
}

int wmsdr_iq_capture_mmap(int fd, struct wmsdr_mmap *m, uint32_t node, uint32_t len,
			  uint32_t bw, uint32_t band, int direct,
			  struct wm_iq_page *page_out, uint8_t **data_out)
{
	uint32_t src_addr = 0;
	int rc;

	if (len <= WMIQ_SPECTRUM_MAX_WORDS && len >= 16U) {
		if (!m || !m->base || (size_t)len * 4U > m->map_size - m->data_off)
			return -1;
		rc = wmsdr_spectrum_arm(fd, node, len, bw, band, page_out);
		if (rc == 0) {
			uint8_t *dst = m->base + m->data_off;

			rc = wmsdr_spectrum_read_banks_buf(fd, len, dst);
			if (rc == 0)
				*data_out = dst;
		}
		wmsdr_spectrum_stop(fd);
		return rc;
	}

	rc = wmsdr_iq_arm_and_wait(fd, node, len, bw, band, direct, page_out, &src_addr);
	if (rc != 0)
		return rc;
	if (wmsdr_dma_mmap_read(fd, m, src_addr, len * 4U, data_out))
		return -1;
	return 0;
}

void wmsdr_iq_stream_init(struct wmsdr_iq_stream *s, int fd, struct wmsdr_mmap *m)
{
	memset(s, 0, sizeof(*s));
	s->fd = fd;
	s->mmap = m;
	s->fast_bar0_ok = (m && m->base) ? 1 : 0;
	s->cmd8_ok = 1;
}

void wmsdr_iq_stream_stop(struct wmsdr_iq_stream *s)
{
	if (!s || !s->active)
		return;
	s->active = 0;
	if (s->spec_mode) {
		s->spec_mode = 0;
		wmsdr_spectrum_stop(s->fd);
		return;
	}
	if (s->v1_keep) {
		uint32_t dummy = 0;

		s->v1_keep = 0;
		(void)wmsdr_wr32(s->fd, WMIQ_PAGE_ADDR + 0x40U, 0U);
		(void)wmsdr_wmcall(s->fd, 0xE006CBC2U, WMIQ_PAGE_ADDR + 0x40U, 0U, 0U, 0U, &dummy);
		(void)wmsdr_wr32(s->fd, 0x80023090U, 0x400U);
		return;
	}
	(void)wmsdr_iq_stop(s->fd);
}

static int wmsdr_v1_keep_arm(struct wmsdr_iq_stream *s, uint32_t node, uint32_t len,
			     uint32_t bw, uint32_t eff_band, int eff_dir,
			     struct wm_iq_page *page_out, uint8_t **data_out)
{
	uint32_t dummy = 0;

	if (wmsdr_iq_capture_mmap(s->fd, s->mmap, node, len, bw, eff_band,
				  eff_dir, page_out, data_out) != 0)
		return -1;
	if (wmsdr_wr32(s->fd, WMIQ_PAGE_ADDR + 0x40U, 1U) == 0 &&
	    wmsdr_wmcall(s->fd, 0xE006CC50U, WMIQ_PAGE_ADDR + 0x40U, 0U, 0U, 0U, &dummy) == 0) {
		s->active = 1;
		s->v1_keep = 1;
		s->spec_mode = 0;
		s->node = node;
		s->len = len;
		s->bw = bw;
		s->band = eff_band;
		s->direct = eff_dir;
		s->armed_ms = stream_now_ms();
	}
	return 0;
}

int wmsdr_iq_stream_next_mmap(struct wmsdr_iq_stream *s, uint32_t node, uint32_t len,
			      uint32_t bw, uint32_t band, int direct,
			      struct wm_iq_page *page_out, uint8_t **data_out)
{
	uint32_t eff_band = band & 1U;
	int eff_dir = (eff_band == 0U && node != 15U && direct) ? 1 : 0;
	int want_spec = (len <= WMIQ_SPECTRUM_MAX_WORDS && len >= 16U) ? 1 : 0;

	if (!s || !s->mmap || !s->mmap->base)
		return -1;
	if (len == 0U || len > WMIQ_MAX_WORDS)
		return -EINVAL;

	if (s->active && (s->spec_mode != want_spec ||
			  s->node != node || s->bw != bw || s->band != eff_band))
		wmsdr_iq_stream_stop(s);
	else if (s->active && !s->spec_mode && s->armed_ms &&
		 (stream_now_ms() - s->armed_ms) >= 800ULL) {
		/* Periodically release ICAP mux in PLE mode 0 so Wi-Fi avoids SER. */
		wmsdr_iq_stream_stop(s);
		usleep(2000);
	}

	if (s->active && s->fast_bar0_ok) {
		if (wmsdr_icap_mmap_run(s->fd, s->mmap, node, len, bw, eff_band,
					(uint32_t)s->spec_mode, page_out, data_out) == 0) {
			s->len = len;
			return 0;
		}
		if (errno == ENOTTY || errno == EOPNOTSUPP) {
			s->fast_bar0_ok = 0;
			wmsdr_iq_stream_stop(s);
		} else {
			/* Recover cleanly if Wi-Fi SER or reset cleared ICAP/WFDMA state. */
			wmsdr_iq_stream_stop(s);
			usleep(5000);
		}
	}

	if (!s->active && s->fast_bar0_ok && want_spec) {
		if (wmsdr_spectrum_arm(s->fd, node, len, bw, eff_band, page_out) != 0)
			return -1;
		s->active = 1;
		s->spec_mode = 1;
		s->v1_keep = 0;
		s->node = node;
		s->len = len;
		s->bw = bw;
		s->band = eff_band;
		s->direct = 0;
		s->armed_ms = stream_now_ms();
		if (wmsdr_icap_mmap_run(s->fd, s->mmap, node, len, bw, eff_band,
					1U, page_out, data_out) == 0)
			return 0;
		wmsdr_iq_stream_stop(s);
		if (errno != ENOTTY && errno != EOPNOTSUPP)
			return -1;
		s->fast_bar0_ok = 0;
	}

	if (!s->active && s->fast_bar0_ok && s->cmd8_ok) {
		struct wm_iq_page p = { 0 };
		int rc;

		if (wmsdr_iq_arm_ex(s->fd, node, len, bw, eff_band, eff_dir, 1))
			return -1;
		rc = wmsdr_wait_iq_done_us(s->fd, &p, 30, 50U);
		if (page_out)
			*page_out = p;
		if (rc == 0) {
			uint32_t src = p.data_addr ? p.data_addr : WMIQ_POOL_ADDR;

			if (wmsdr_dma_mmap_read(s->fd, s->mmap, src, len * 4U, data_out)) {
				(void)wmsdr_iq_stop(s->fd);
				return -1;
			}
			s->active = 1;
			s->spec_mode = 0;
			s->node = node;
			s->len = len;
			s->bw = bw;
			s->band = eff_band;
			s->direct = eff_dir;
			s->armed_ms = stream_now_ms();
			return 0;
		}
		if (rc == -ETIMEDOUT && p.cmd == 8U) {
			(void)wmsdr_wr32(s->fd, WMIQ_PAGE_ADDR + 0x08U, 0U);
			s->cmd8_ok = 0;
			return wmsdr_v1_keep_arm(s, node, len, bw, eff_band, eff_dir,
						 page_out, data_out);
		} else {
			return rc;
		}
	}

	if (!s->active && s->fast_bar0_ok && !s->cmd8_ok)
		return wmsdr_v1_keep_arm(s, node, len, bw, eff_band, eff_dir,
					 page_out, data_out);

	return wmsdr_iq_capture_mmap(s->fd, s->mmap, node, len, bw, eff_band,
				     eff_dir, page_out, data_out);
}

int wmsdr_net_arm_hw(struct wmsdr_iq_stream *s, uint32_t node, uint32_t len,
		     uint32_t bw, uint32_t band, int direct, int *mode_out)
{
	uint32_t eff_band = band & 1U;
	int eff_dir = (eff_band == 0U && node != 15U && direct) ? 1 : 0;
	int want_spec = (len <= WMIQ_SPECTRUM_MAX_WORDS && len >= 16U) ? 1 : 0;

	if (!s || len == 0U || len > WMIQ_MAX_WORDS)
		return -EINVAL;
	if (s->active)
		wmsdr_iq_stream_stop(s);

	if (want_spec) {
		if (wmsdr_spectrum_arm(s->fd, node, len, bw, eff_band, NULL) != 0)
			return -1;
		s->active = 1;
		s->spec_mode = 1;
		s->v1_keep = 0;
		s->node = node;
		s->len = len;
		s->bw = bw;
		s->band = eff_band;
		s->direct = 0;
		s->armed_ms = stream_now_ms();
		if (mode_out)
			*mode_out = 1;
		return 0;
	}

	if (wmsdr_iq_arm_ex(s->fd, node, len, bw, eff_band, eff_dir, 1))
		return -1;
	s->active = 1;
	s->spec_mode = 0;
	s->v1_keep = 0;
	s->node = node;
	s->len = len;
	s->bw = bw;
	s->band = eff_band;
	s->direct = eff_dir;
	s->armed_ms = stream_now_ms();
	if (mode_out)
		*mode_out = 0;
	return 0;
}

int wmsdr_net_start(int fd, struct wmsdr_net_cfg *cfg)
{
	return ioctl(fd, WMSDR_IOC_NET_START, cfg) == 0 ? 0 : -1;
}

int wmsdr_net_add_sock(int fd, int sock_fd)
{
	struct wmsdr_net_sock req = { .fd = sock_fd, .flags = 0U };

	return ioctl(fd, WMSDR_IOC_NET_ADD_SOCK, &req) == 0 ? 0 : -1;
}

int wmsdr_net_del_sock(int fd, int sock_fd)
{
	int32_t sfd = (int32_t)sock_fd;

	return ioctl(fd, WMSDR_IOC_NET_DEL_SOCK, &sfd) == 0 ? 0 : -1;
}

int wmsdr_net_stop(int fd, struct wmsdr_net_stat *stat_out)
{
	struct wmsdr_net_stat tmp;

	if (!stat_out)
		stat_out = &tmp;
	memset(stat_out, 0, sizeof(*stat_out));
	return ioctl(fd, WMSDR_IOC_NET_STOP, stat_out) == 0 ? 0 : -1;
}

int wmsdr_net_stat(int fd, struct wmsdr_net_stat *stat_out)
{
	memset(stat_out, 0, sizeof(*stat_out));
	return ioctl(fd, WMSDR_IOC_NET_STAT, stat_out) == 0 ? 0 : -1;
}
