// SPDX-License-Identifier: GPL-2.0-only
/*
 * wmsdr - userspace CLI and HTTP/WS server for /dev/wmsdr and WM IQ capture.
 *
 *   wmsdr info|fwstate|peek ADDR|poke ADDR VAL|dump ADDR LEN [FILE]|write ADDR FILE
 *   wmsdr iqstatus|iqarm [NODE] [LEN] [BW] [direct] [BAND]
 *   wmsdr iqcapture OUTFILE [NODE] [LEN] [BW] [direct] [BAND]
 *   wmsdr iqstream [NODE] [LEN] [BW] [direct|0|1] [INTERVAL_MS] [COUNT] [BAND]
 *   wmsdr serve [PORT] [WEB_ROOT]
 *   wmsdr wmpeek ADDR|wmpoke ADDR VAL|wmcall FN [A0] [A1] [A2] [A3]
 */
#include "wmsdr_io.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include <signal.h>
#include <sys/stat.h>
#include <time.h>

static volatile sig_atomic_t g_stream_stop;

static void on_stream_sig(int sig)
{
	(void)sig;
	g_stream_stop = 1;
}

static uint32_t parse_u32(const char *s)
{
	char *end = NULL;
	unsigned long v;

	errno = 0;
	v = strtoul(s, &end, 0);
	if (errno || !end || *end || v > 0xffffffffUL) {
		fprintf(stderr, "bad number: %s\n", s);
		exit(2);
	}
	return (uint32_t)v;
}

static int open_dev(void)
{
	int fd = wmsdr_open_dev();

	if (fd < 0) {
		fprintf(stderr, "open %s: %s\n", WMSDR_DEV_PATH, strerror(errno));
		fprintf(stderr, "load wm_sdr.ko first\n");
		exit(1);
	}
	return fd;
}
static int cmd_info(int fd)
{
	struct wmsdr_info i;

	if (wmsdr_get_info(fd, &i))
		return 1;
	printf("version      %u\n", i.version);
	printf("backend      %u (%s)\n", i.backend,
	       i.backend == 0 ? "remap" : "pci");
	printf("chip         %04x:%04x\n", i.chip_vendor, i.chip_device);
	printf("fw_state     %u\n", i.fw_state);
	printf("fw_stat      0x%08x\n", i.fw_stat_addr);
	printf("bar          phys=0x%08x len=0x%x\n", i.bar_phys, i.bar_len);
	printf("remap        ctl=0x%08x win=0x%08x\n", i.remap_ctl, i.remap_win);
	printf("loaded       %u\n", i.loaded);
	printf("flags        0x%08x%s\n", i.flags,
	       (i.flags & WMSDR_INFO_F_DMA_READY) ? " (dma_ready)" : "");
	return 0;
}

static uint64_t bench_now_ns(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static int cmd_barpeek(int fd, const char *arg)
{
	uint32_t off = parse_u32(arg), val = 0;

	if (wmsdr_direct_rd32(fd, off, &val))
		return 1;
	printf("BAR[0x%06x] = 0x%08x\n", off, val);
	return 0;
}

static int bench_verify_bytes(const uint8_t *ref, const uint8_t *got, uint32_t len,
			      uint32_t it, const char *tag)
{
	uint32_t k;

	if (memcmp(ref, got, len) == 0)
		return 0;
	for (k = 0; k < len; k++) {
		if (ref[k] != got[k]) {
			fprintf(stderr, "MISMATCH (%s) at iter=%u off=0x%x: remap=%02x got=%02x\n",
				tag, it, k, ref[k], got[k]);
			break;
		}
	}
	return -1;
}

static int cmd_dmabench(int fd, uint32_t addr, uint32_t len, uint32_t iters)
{
	struct wmsdr_mmap dm;
	uint8_t *ref = malloc(len ? len : 1U);
	uint8_t *got = malloc(len ? len : 1U);
	uint64_t t0, t1, remap_ns, dma_ns = 0, mmap_ns = 0;
	uint32_t it;
	int have_mmap;

	if (!ref || !got) {
		fprintf(stderr, "oom\n");
		free(ref);
		free(got);
		return 1;
	}
	have_mmap = (wmsdr_mmap_open(fd, &dm) == 0);

	t0 = bench_now_ns();
	if (wmsdr_remap_read_mem(fd, addr, ref, len)) {
		wmsdr_mmap_close(&dm);
		free(ref);
		free(got);
		return 1;
	}
	t1 = bench_now_ns();
	remap_ns = t1 - t0;

	for (it = 0; it < iters; it++) {
		memset(got, 0xA5, len);
		t0 = bench_now_ns();
		if (wmsdr_dma_read(fd, addr, got, len)) {
			wmsdr_mmap_close(&dm);
			free(ref);
			free(got);
			return 1;
		}
		t1 = bench_now_ns();
		dma_ns += t1 - t0;

		if (bench_verify_bytes(ref, got, len, it, "wfdma") != 0) {
			wmsdr_mmap_close(&dm);
			free(ref);
			free(got);
			return 1;
		}
	}

	if (have_mmap) {
		for (it = 0; it < iters; it++) {
			uint8_t *mptr = NULL;

			t0 = bench_now_ns();
			if (wmsdr_dma_mmap_read(fd, &dm, addr, len, &mptr)) {
				wmsdr_mmap_close(&dm);
				free(ref);
				free(got);
				return 1;
			}
			t1 = bench_now_ns();
			mmap_ns += t1 - t0;

			if (bench_verify_bytes(ref, mptr, len, it, "mmap") != 0) {
				wmsdr_mmap_close(&dm);
				free(ref);
				free(got);
				return 1;
			}
		}
	}

	{
		double remap_us = (double)remap_ns / 1000.0;
		double dma_us = (double)dma_ns / (double)iters / 1000.0;
		double mib = (double)len / (1024.0 * 1024.0);

		printf("addr=0x%08x len=%u iters=%u MATCH=OK\n", addr, len, iters);
		printf("  remap: %.1f us (%.2f MiB/s)\n", remap_us, mib * 1e6 / remap_us);
		printf("  wfdma: %.1f us (%.2f MiB/s)  speedup=%.1fx\n",
		       dma_us, mib * 1e6 / dma_us, remap_us / dma_us);
		if (have_mmap) {
			double mmap_us = (double)mmap_ns / (double)iters / 1000.0;
			printf("  mmap : %.1f us (%.2f MiB/s)  speedup=%.1fx\n",
			       mmap_us, mib * 1e6 / mmap_us, remap_us / mmap_us);
		}
	}
	wmsdr_mmap_close(&dm);
	free(ref);
	free(got);
	return 0;
}

static int cmd_peek(int fd, const char *arg)
{
	uint32_t addr = parse_u32(arg), val = 0;

	if (wmsdr_rd32(fd, addr, &val))
		return 1;
	printf("0x%08x = 0x%08x\n", addr, val);
	return 0;
}

static int cmd_poke(int fd, const char *a, const char *v)
{
	uint32_t addr = parse_u32(a), val = parse_u32(v);

	if (wmsdr_wr32(fd, addr, val))
		return 1;
	printf("wrote 0x%08x <- 0x%08x\n", addr, val);
	return 0;
}

static int cmd_fwstate(int fd)
{
	uint32_t v;

	if (wmsdr_get_fw_state(fd, &v))
		return 1;
	printf("fw_state %u\n", v);
	return 0;
}

static int cmd_dump(int fd, const char *a, const char *l, const char *path)
{
	uint32_t addr = parse_u32(a), len = parse_u32(l);
	uint8_t *buf = calloc(1, len);
	FILE *f = stdout;

	if (!buf) {
		fprintf(stderr, "oom\n");
		return 1;
	}
	if (wmsdr_read_mem(fd, addr, buf, len)) {
		free(buf);
		return 1;
	}
	if (path) {
		f = fopen(path, "wb");
		if (!f) {
			perror(path);
			free(buf);
			return 1;
		}
		fwrite(buf, 1, len, f);
		fclose(f);
		printf("wrote %u bytes to %s\n", len, path);
	} else {
		uint32_t i;
		for (i = 0; i < len; i += 16) {
			uint32_t j;
			printf("%08x  ", addr + i);
			for (j = 0; j < 16 && i + j < len; j++)
				printf("%02x ", buf[i + j]);
			printf("\n");
		}
	}
	free(buf);
	return 0;
}

static int cmd_write(int fd, const char *a, const char *path)
{
	uint32_t addr = parse_u32(a);
	FILE *f = fopen(path, "rb");
	long len;
	uint8_t *buf;

	if (!f) {
		perror(path);
		return 1;
	}
	fseek(f, 0, SEEK_END);
	len = ftell(f);
	fseek(f, 0, SEEK_SET);
	if (len <= 0 || len > 0x10000) {
		fprintf(stderr, "file size %ld out of range\n", len);
		fclose(f);
		return 1;
	}
	buf = calloc(1, len);
	if (!buf || fread(buf, 1, len, f) != (size_t)len) {
		fprintf(stderr, "read %s failed\n", path);
		free(buf);
		fclose(f);
		return 1;
	}
	fclose(f);

	if (wmsdr_write_mem(fd, addr, buf, (uint32_t)len)) {
		free(buf);
		return 1;
	}
	printf("wrote %ld bytes to 0x%08x\n", len, addr);
	free(buf);
	return 0;
}

static int cmd_iqstatus(int fd)
{
	uint32_t raw_tick = 0;
	struct wm_iq_page p;

	if (wmsdr_rd32(fd, WMIQ_RAW_TICK_ADDR, &raw_tick))
		return 1;
	if (wmsdr_read_iq_page(fd, &p))
		return 1;

	printf("raw_tick     0x%08x (%u)\n", raw_tick, raw_tick);
	printf("magic        0x%08x (%s)\n", p.magic,
	       p.magic == WMIQ_MAGIC ? "WMIQ" : "UNINIT");
	printf("version      %u\n", p.version);
	printf("cmd          %u\n", p.cmd);
	printf("status       %u (%s)\n", p.status, wmsdr_status_str(p.status));
	printf("node         0x%08x (%u)\n", p.node, p.node);
	printf("length       0x%08x (%u words)\n", p.length, p.length);
	printf("data_addr    0x%08x\n", p.data_addr);
	printf("sample_count 0x%08x (%u words)\n", p.sample_count, p.sample_count);
	printf("tick         0x%08x (%u)\n", p.tick, p.tick);
	printf("error        0x%08x\n", p.error);
	printf("cr0          0x%08x\n", p.cr0);
	printf("cr2          0x%08x\n", p.cr2);
	printf("cr9          0x%08x (%u)\n", p.cr9, p.cr9);
	printf("bw           %u (band=%u)\n", p.bw & 0xFFU, (p.bw >> 8) & 1U);
	printf("arg0         0x%08x\n", p.arg0);
	printf("arg1/ret     0x%08x\n", p.arg1);
	return 0;
}

static int16_t sext16(uint16_t v, int bits)
{
	uint16_t m = (uint16_t)(1U << (bits - 1));
	uint16_t mask = (uint16_t)((1U << bits) - 1U);

	v &= mask;
	return (int16_t)((v ^ m) - m);
}

static int cmd_iqcapture(int fd, const char *outfile, uint32_t node,
			 uint32_t len, uint32_t bw, uint32_t band, int direct)
{
	struct wm_iq_page p = { 0 };
	uint32_t *words;
	uint32_t byte_len, src_addr, i, nonzero = 0, changes = 0;
	int16_t min_i = 32767, max_i = -32768, min_q = 32767, max_q = -32768;
	FILE *f;
	int rc;

	if (len == 0 || len > WMIQ_MAX_WORDS) {
		fprintf(stderr, "length %u out of range (1..%u words)\n", len, WMIQ_MAX_WORDS);
		return 2;
	}
	byte_len = len * 4U;
	words = calloc(len, sizeof(uint32_t));
	if (!words) {
		fprintf(stderr, "oom\n");
		return 1;
	}
	rc = wmsdr_iq_capture_buf(fd, node, len, bw, band, direct, &p, (uint8_t *)words);
	if (rc == -2) {
		fprintf(stderr, "ICAP arm failed: status=ERROR error=0x%08x (cr0=0x%08x cr9=0x%08x)\n",
			p.error, p.cr0, p.cr9);
		free(words);
		return 1;
	}
	if (rc != 0) {
		fprintf(stderr, "ICAP capture failed/timeout: status=%u (%s) cmd=%u cr0=0x%08x cr9=0x%08x\n",
			p.status, wmsdr_status_str(p.status), p.cmd, p.cr0, p.cr9);
		free(words);
		return 1;
	}
	src_addr = p.data_addr ? p.data_addr : WMIQ_POOL_ADDR;

	f = fopen(outfile, "wb");
	if (!f) {
		perror(outfile);
		free(words);
		return 1;
	}
	fwrite(words, 1, byte_len, f);
	fclose(f);

	for (i = 0; i < len; i++) {
		uint32_t w = words[i];
		int16_t i_val = sext16((uint16_t)(w & 0xFFFFU), 12);
		int16_t q_val = sext16((uint16_t)((w >> 16) & 0xFFFFU), 12);

		if (w != 0)
			nonzero++;
		if (i > 0 && w != words[i - 1])
			changes++;
		if (i_val < min_i) min_i = i_val;
		if (i_val > max_i) max_i = i_val;
		if (q_val < min_q) min_q = q_val;
		if (q_val > max_q) max_q = q_val;
	}

	printf("capture complete -> %s (%u bytes from 0x%08x)\n",
	       outfile, byte_len, src_addr);
	printf("  band=%u mode=%s node=0x%x len=%u sample_count=%u cr0=0x%08x cr9=0x%08x err=0x%x\n",
	       band & 1U, (band || node == 15U || !direct) ? "api" : "direct",
	       p.node, len, p.sample_count, p.cr0, p.cr9, p.error);
	printf("  stats: nonzero=%u/%u transitions=%u/%u I_12b=[%d..%d] Q_12b=[%d..%d]\n",
	       nonzero, len, changes, len > 0 ? len - 1 : 0,
	       min_i, max_i, min_q, max_q);
	printf("  first 8 words:\n");
	for (i = 0; i < 8 && i < len; i++) {
		uint32_t w = words[i];
		int16_t i12 = sext16((uint16_t)(w & 0xFFFFU), 12);
		int16_t q12 = sext16((uint16_t)((w >> 16) & 0xFFFFU), 12);

		printf("    [%2u] 0x%08x  (I12=%6d, Q12=%6d)\n", i, w, i12, q12);
	}
	free(words);
	return 0;
}

static int cmd_wmpeek(int fd, uint32_t addr)
{
	uint32_t val = 0;

	if (wmsdr_wmpeek(fd, addr, &val))
		return 1;
	printf("WM[0x%08x] = 0x%08x\n", addr, val);
	return 0;
}

static int cmd_wmpoke(int fd, uint32_t addr, uint32_t val)
{
	if (wmsdr_wmpoke(fd, addr, val))
		return 1;
	printf("WM[0x%08x] <- 0x%08x\n", addr, val);
	return 0;
}

static int cmd_wmcall(int fd, uint32_t fn, uint32_t a0, uint32_t a1,
		      uint32_t a2, uint32_t a3)
{
	uint32_t ret_val = 0;

	if (wmsdr_wmcall(fd, fn, a0, a1, a2, a3, &ret_val))
		return 1;
	printf("WM call 0x%08x(0x%x, 0x%x, 0x%x, 0x%x) -> 0x%08x\n",
	       fn, a0, a1, a2, a3, ret_val);
	return 0;
}

static int write_all(int out_fd, const void *buf, size_t len)
{
	const uint8_t *p = (const uint8_t *)buf;
	size_t off = 0;

	while (off < len) {
		ssize_t n = write(out_fd, p + off, len - off);

		if (n < 0) {
			if (errno == EINTR)
				continue;
			return -1;
		}
		if (n == 0)
			return -1;
		off += (size_t)n;
	}
	return 0;
}

static int stdin_stop_requested(int *monitor_stdin, int timeout_ms)
{
	struct pollfd pfd;
	int r;

	if (!*monitor_stdin) {
		if (timeout_ms > 0)
			usleep((useconds_t)timeout_ms * 1000U);
		return 0;
	}
	pfd.fd = STDIN_FILENO;
	pfd.events = POLLIN;
	pfd.revents = 0;
	r = poll(&pfd, 1, timeout_ms);
	if (r > 0) {
		if (pfd.revents & POLLIN) {
			char buf[64];
			ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
			ssize_t i;

			if (n <= 0) {
				*monitor_stdin = 0;
				return 0;
			}
			for (i = 0; i < n; i++) {
				if (buf[i] == 'q' || buf[i] == 'Q')
					return 1;
			}
		} else if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
			*monitor_stdin = 0;
		}
	}
	return 0;
}

static int cmd_iqstream(int fd, uint32_t node, uint32_t len, uint32_t bw,
			uint32_t band, int direct, uint32_t interval_ms,
			uint32_t max_frames)
{
	struct wmsdr_mmap dm = { 0 };
	struct wmsdr_iq_stream strm;
	uint32_t byte_len;
	size_t frame_len;
	uint8_t *frame = NULL;
	struct stat st;
	int monitor_stdin, use_mmap, use_fast_stream;
	uint32_t seq = 0;

	if (len == 0 || len > WMIQ_MAX_WORDS) {
		fprintf(stderr, "length %u out of range (1..%u words)\n", len, WMIQ_MAX_WORDS);
		return 2;
	}

	g_stream_stop = 0;
	signal(SIGPIPE, SIG_IGN);
	signal(SIGINT, on_stream_sig);
	signal(SIGTERM, on_stream_sig);
	monitor_stdin = (fstat(STDIN_FILENO, &st) == 0 &&
			 (S_ISFIFO(st.st_mode) || S_ISSOCK(st.st_mode)));

	byte_len = len * 4U;
	frame_len = 32U + (size_t)byte_len;
	use_mmap = (wmsdr_mmap_open(fd, &dm) == 0 && dm.data_off >= 32U &&
		    dm.data_off + byte_len <= dm.map_size);
	use_fast_stream = (use_mmap && interval_ms <= 20U);
	wmsdr_iq_stream_init(&strm, fd, use_mmap ? &dm : NULL);
	if (!use_mmap) {
		wmsdr_mmap_close(&dm);
		frame = calloc(1, frame_len);
		if (!frame) {
			fprintf(stderr, "oom\n");
			return 1;
		}
	}

	while (!g_stream_stop && (!max_frames || seq < max_frames)) {
		struct wm_iq_page p = { 0 };
		uint32_t flags;
		int actual_direct = (band == 0U && node != 15U && direct) ? 1 : 0;
		uint8_t *out_hdr;
		int rc;

		if (stdin_stop_requested(&monitor_stdin, 0))
			break;
		if (use_mmap) {
			uint8_t *iq_data = NULL;

			if (use_fast_stream)
				rc = wmsdr_iq_stream_next_mmap(&strm, node, len, bw, band,
							       actual_direct, &p, &iq_data);
			else
				rc = wmsdr_iq_capture_mmap(fd, &dm, node, len, bw, band,
							   actual_direct, &p, &iq_data);
			out_hdr = iq_data ? (iq_data - 32U) : NULL;
		} else {
			out_hdr = frame;
			rc = wmsdr_iq_capture_buf(fd, node, len, bw, band,
						  actual_direct, &p, frame + 32);
		}
		if (rc == -2) {
			fprintf(stderr, "ICAP arm failed: error=0x%08x (cr0=0x%08x cr9=0x%08x)\n",
				p.error, p.cr0, p.cr9);
			wmsdr_iq_stream_stop(&strm);
			wmsdr_mmap_close(&dm);
			free(frame);
			return 1;
		}
		if (rc != 0) {
			fprintf(stderr, "ICAP capture failed/timeout: status=%u cmd=%u cr0=0x%08x cr9=0x%08x\n",
				p.status, p.cmd, p.cr0, p.cr9);
			wmsdr_iq_stream_stop(&strm);
			wmsdr_mmap_close(&dm);
			free(frame);
			return 1;
		}

		seq++;
		flags = (actual_direct ? 1U : 0U) | ((bw & 0xffU) << 8) | ((band & 1U) << 16);
		wmsdr_put_le32(out_hdr + 0,  WSIQ_MAGIC);
		wmsdr_put_le32(out_hdr + 4,  1U);
		wmsdr_put_le32(out_hdr + 8,  seq);
		wmsdr_put_le32(out_hdr + 12, node);
		wmsdr_put_le32(out_hdr + 16, len);
		wmsdr_put_le32(out_hdr + 20, p.cr0);
		wmsdr_put_le32(out_hdr + 24, p.cr9);
		wmsdr_put_le32(out_hdr + 28, flags);

		if (write_all(STDOUT_FILENO, out_hdr, frame_len) != 0)
			break;

		if (interval_ms > 0 && stdin_stop_requested(&monitor_stdin, (int)interval_ms))
			break;
	}

	wmsdr_iq_stream_stop(&strm);
	wmsdr_mmap_close(&dm);
	free(frame);
	return 0;
}

static int strip_band_flag(int *argc, char **argv, uint32_t *band)
{
	int out = 2, i;

	for (i = 2; i < *argc; i++) {
		if (!strcmp(argv[i], "--band")) {
			if (i + 1 >= *argc) {
				fprintf(stderr, "--band requires 0 or 1\n");
				return -1;
			}
			*band = parse_u32(argv[++i]) & 1U;
		} else if (!strncmp(argv[i], "--band=", 7)) {
			*band = parse_u32(argv[i] + 7) & 1U;
		} else {
			argv[out++] = argv[i];
		}
	}
	*argc = out;
	return 0;
}

int main(int argc, char **argv)
{
	int fd, ret;
	uint32_t band = 0U;

	if (argc < 2) {
		fprintf(stderr,
			"usage: %s info|fwstate|peek ADDR|barpeek OFF|poke ADDR VAL|dump ADDR LEN [FILE]|write ADDR FILE\n"
			"       %s dmabench [ADDR] [LEN] [ITERS]\n"
			"       %s iqstatus|iqarm [NODE] [LEN] [BW] [direct] [BAND]\n"
			"       %s iqcapture FILE [NODE] [LEN] [BW] [direct] [BAND] [--band 0|1]\n"
			"       %s iqstream [NODE] [LEN] [BW] [direct|0|1] [INTERVAL_MS] [COUNT] [BAND] [--band 0|1]\n"
			"       %s serve [PORT] [WEB_ROOT]\n"
			"       %s wmpeek ADDR|wmpoke ADDR VAL|wmcall FN [A0] [A1] [A2] [A3]\n",
			argv[0], argv[0], argv[0], argv[0], argv[0], argv[0], argv[0]);
		return 2;
	}

	if (!strcmp(argv[1], "serve"))
		return wmsdr_httpd_main(argc - 1, argv + 1);

	if (strip_band_flag(&argc, argv, &band) != 0)
		return 2;

	fd = open_dev();
	if (!strcmp(argv[1], "info")) {
		ret = cmd_info(fd);
	} else if (!strcmp(argv[1], "fwstate")) {
		ret = cmd_fwstate(fd);
	} else if (!strcmp(argv[1], "peek") && argc == 3) {
		ret = cmd_peek(fd, argv[2]);
	} else if (!strcmp(argv[1], "barpeek") && argc == 3) {
		ret = cmd_barpeek(fd, argv[2]);
	} else if (!strcmp(argv[1], "dmabench")) {
		uint32_t addr  = argc > 2 ? parse_u32(argv[2]) : WMIQ_POOL_ADDR;
		uint32_t len   = argc > 3 ? parse_u32(argv[3]) : 32768U;
		uint32_t iters = argc > 4 ? parse_u32(argv[4]) : 20U;

		ret = cmd_dmabench(fd, addr, len, iters ? iters : 1U);
	} else if (!strcmp(argv[1], "poke") && argc == 4) {
		ret = cmd_poke(fd, argv[2], argv[3]);
	} else if (!strcmp(argv[1], "dump") && (argc == 4 || argc == 5)) {
		ret = cmd_dump(fd, argv[2], argv[3], argc == 5 ? argv[4] : NULL);
	} else if (!strcmp(argv[1], "write") && argc == 4) {
		ret = cmd_write(fd, argv[2], argv[3]);
	} else if (!strcmp(argv[1], "iqstatus")) {
		ret = cmd_iqstatus(fd);
	} else if (!strcmp(argv[1], "iqarm")) {
		uint32_t node = argc > 2 ? parse_u32(argv[2]) : 15U;
		uint32_t len  = argc > 3 ? parse_u32(argv[3]) : 8192U;
		uint32_t bw   = argc > 4 ? parse_u32(argv[4]) : 0U;
		int direct    = (argc > 5 && (!strcmp(argv[5], "direct") || !strcmp(argv[5], "1"))) ? 1 : 0;
		if (argc > 6)
			band = parse_u32(argv[6]) & 1U;

		ret = wmsdr_iq_arm(fd, node, len, bw, band, direct);
		if (!ret)
			printf("armed (band=%u %s): node=0x%x len=%u bw=%u\n",
			       band, (band || node == 15U || !direct) ? "api" : "direct",
			       node, len, bw);
	} else if (!strcmp(argv[1], "iqcapture") && argc >= 3) {
		const char *out = argv[2];
		uint32_t node = argc > 3 ? parse_u32(argv[3]) : 15U;
		uint32_t len  = argc > 4 ? parse_u32(argv[4]) : 8192U;
		uint32_t bw   = argc > 5 ? parse_u32(argv[5]) : 0U;
		int direct    = (argc > 6 && (!strcmp(argv[6], "direct") || !strcmp(argv[6], "1"))) ? 1 : 0;
		if (argc > 7)
			band = parse_u32(argv[7]) & 1U;

		ret = cmd_iqcapture(fd, out, node, len, bw, band, direct);
	} else if (!strcmp(argv[1], "iqstream")) {
		uint32_t node        = argc > 2 ? parse_u32(argv[2]) : 15U;
		uint32_t len         = argc > 3 ? parse_u32(argv[3]) : 8192U;
		uint32_t bw          = argc > 4 ? parse_u32(argv[4]) : 0U;
		int direct           = (argc > 5 && (!strcmp(argv[5], "direct") || !strcmp(argv[5], "1"))) ? 1 : 0;
		uint32_t interval_ms = argc > 6 ? parse_u32(argv[6]) : 0U;
		uint32_t max_frames  = argc > 7 ? parse_u32(argv[7]) : 0U;
		if (argc > 8)
			band = parse_u32(argv[8]) & 1U;

		ret = cmd_iqstream(fd, node, len, bw, band, direct, interval_ms, max_frames);
	} else if (!strcmp(argv[1], "wmpeek") && argc == 3) {
		ret = cmd_wmpeek(fd, parse_u32(argv[2]));
	} else if (!strcmp(argv[1], "wmpoke") && argc == 4) {
		ret = cmd_wmpoke(fd, parse_u32(argv[2]), parse_u32(argv[3]));
	} else if (!strcmp(argv[1], "wmcall") && argc >= 3) {
		uint32_t fn = parse_u32(argv[2]);
		uint32_t a0 = argc > 3 ? parse_u32(argv[3]) : 0U;
		uint32_t a1 = argc > 4 ? parse_u32(argv[4]) : 0U;
		uint32_t a2 = argc > 5 ? parse_u32(argv[5]) : 0U;
		uint32_t a3 = argc > 6 ? parse_u32(argv[6]) : 0U;

		ret = cmd_wmcall(fd, fn, a0, a1, a2, a3);
	} else {
		fprintf(stderr, "bad arguments\n");
		ret = 2;
	}

	close(fd);
	return ret;
}
