#define _GNU_SOURCE
#include "wmsdr_io.h"

#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <time.h>
#include <unistd.h>

#define HTTPD_DEFAULT_PORT      8088
#define HTTPD_DEFAULT_WEB_ROOT  "/root/wmtest/web"
#define HTTPD_DEFAULT_NODE      15U
#define HTTPD_DEFAULT_LEN       8192U
#define HTTPD_DEFAULT_BW        0U
#define HTTPD_DEFAULT_BAND      0U
#define HTTPD_MAX_HDR_BYTES     16384
#define HTTPD_MAX_WS_IN_BYTES   65536
#define BAND0_ADC_FS_MHZ        80.0
#define BAND1_ADC_FS_MHZ        160.0
#define RXFD_BW20_FS_MHZ        40.0

static const char WS_GUID[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

struct rf_band_state {
	int ok;
	uint32_t lo_khz;
	double lo_mhz;
	uint32_t bw_word;
	uint32_t pri;
	uint32_t bw_idx;
	uint32_t bw_mhz;
	uint32_t rx_n;
	uint32_t tx_n;
};

struct httpd_ctx {
	int dev_fd;
	struct wmsdr_mmap dma_map;
	struct wmsdr_iq_stream stream;
	int port;
	char web_root[512];
	pthread_mutex_t iq_lock;
	uint32_t seq;
	int kernel_net_active;
	uint32_t kernel_net_sid;
	int kernel_net_lanes;
	volatile sig_atomic_t stop;
};

static struct httpd_ctx g_ctx;

static void on_sigint(int sig)
{
	(void)sig;
	g_ctx.stop = 1;
}

static uint64_t now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

/* ---- Minimal SHA-1 & Base64 for RFC 6455 handshake ---- */

static uint32_t rol32(uint32_t v, int bits)
{
	return (v << bits) | (v >> (32 - bits));
}

static void sha1_transform(uint32_t st[5], const uint8_t blk[64])
{
	uint32_t w[80];
	uint32_t a = st[0], b = st[1], c = st[2], d = st[3], e = st[4];
	int t;

	for (t = 0; t < 16; t++) {
		w[t] = ((uint32_t)blk[t * 4] << 24) |
		       ((uint32_t)blk[t * 4 + 1] << 16) |
		       ((uint32_t)blk[t * 4 + 2] << 8) |
		       (uint32_t)blk[t * 4 + 3];
	}
	for (t = 16; t < 80; t++)
		w[t] = rol32(w[t - 3] ^ w[t - 8] ^ w[t - 14] ^ w[t - 16], 1);

	for (t = 0; t < 80; t++) {
		uint32_t f, k, tmp;

		if (t < 20) {
			f = (b & c) | ((~b) & d);
			k = 0x5A827999U;
		} else if (t < 40) {
			f = b ^ c ^ d;
			k = 0x6ED9EBA1U;
		} else if (t < 60) {
			f = (b & c) | (b & d) | (c & d);
			k = 0x8F1BBCDCU;
		} else {
			f = b ^ c ^ d;
			k = 0xCA62C1D6U;
		}
		tmp = rol32(a, 5) + f + e + k + w[t];
		e = d;
		d = c;
		c = rol32(b, 30);
		b = a;
		a = tmp;
	}
	st[0] += a;
	st[1] += b;
	st[2] += c;
	st[3] += d;
	st[4] += e;
}

static void sha1_digest(const uint8_t *msg, size_t len, uint8_t out[20])
{
	uint32_t st[5] = {
		0x67452301U, 0xEFCDAB89U, 0x98BADCFEU, 0x10325476U, 0xC3D2E1F0U
	};
	uint8_t blk[64];
	size_t off = 0, rem;
	uint64_t bit_len = (uint64_t)len * 8ULL;
	int i;

	while (off + 64 <= len) {
		sha1_transform(st, msg + off);
		off += 64;
	}
	rem = len - off;
	memset(blk, 0, sizeof(blk));
	if (rem)
		memcpy(blk, msg + off, rem);
	blk[rem] = 0x80U;
	if (rem >= 56) {
		sha1_transform(st, blk);
		memset(blk, 0, sizeof(blk));
	}
	for (i = 0; i < 8; i++)
		blk[56 + i] = (uint8_t)((bit_len >> ((7 - i) * 8)) & 0xFFU);
	sha1_transform(st, blk);

	for (i = 0; i < 5; i++) {
		out[i * 4 + 0] = (uint8_t)((st[i] >> 24) & 0xFFU);
		out[i * 4 + 1] = (uint8_t)((st[i] >> 16) & 0xFFU);
		out[i * 4 + 2] = (uint8_t)((st[i] >> 8) & 0xFFU);
		out[i * 4 + 3] = (uint8_t)(st[i] & 0xFFU);
	}
}

static void base64_encode(const uint8_t *in, size_t len, char *out)
{
	static const char tbl[] =
		"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
	size_t i = 0, j = 0;

	while (i < len) {
		uint32_t a = in[i++];
		uint32_t b = (i < len) ? in[i++] : 0;
		uint32_t c = (i < len) ? in[i++] : 0;
		uint32_t triple = (a << 16) | (b << 8) | c;

		out[j++] = tbl[(triple >> 18) & 0x3FU];
		out[j++] = tbl[(triple >> 12) & 0x3FU];
		out[j++] = tbl[(triple >> 6) & 0x3FU];
		out[j++] = tbl[triple & 0x3FU];
	}
	if ((len % 3) == 1 && j >= 2) {
		out[j - 2] = '=';
		out[j - 1] = '=';
	} else if ((len % 3) == 2 && j >= 1) {
		out[j - 1] = '=';
	}
	out[j] = '\0';
}

/* ---- Socket I/O helpers ---- */

static int send_all(int fd, const void *buf, size_t len)
{
	const uint8_t *p = (const uint8_t *)buf;
	size_t off = 0;

	while (off < len) {
		ssize_t n = send(fd, p + off, len - off, 0);

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

static int send_two_iov(int fd, const void *p0, size_t n0, const void *p1, size_t n1)
{
	struct iovec iov[2];
	int iovcnt = 0;

	if (n0 > 0) {
		iov[iovcnt].iov_base = (void *)p0;
		iov[iovcnt].iov_len = n0;
		iovcnt++;
	}
	if (n1 > 0) {
		iov[iovcnt].iov_base = (void *)p1;
		iov[iovcnt].iov_len = n1;
		iovcnt++;
	}
	while (iovcnt > 0) {
		ssize_t n = writev(fd, iov, iovcnt);

		if (n < 0) {
			if (errno == EINTR)
				continue;
			return -1;
		}
		if (n == 0)
			return -1;
		while (iovcnt > 0 && (size_t)n >= iov[0].iov_len) {
			n -= (ssize_t)iov[0].iov_len;
			if (--iovcnt > 0)
				iov[0] = iov[1];
		}
		if (iovcnt > 0 && n > 0) {
			iov[0].iov_base = (uint8_t *)iov[0].iov_base + n;
			iov[0].iov_len -= (size_t)n;
		}
	}
	return 0;
}

static int recv_exact(int fd, uint8_t *buf, size_t len)
{
	size_t off = 0;

	while (off < len) {
		ssize_t n = recv(fd, buf + off, len - off, 0);

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

/* ---- RF ChInfo & IQ capture helpers ---- */

static uint32_t bw_idx_to_mhz(uint32_t bw_idx)
{
	switch (bw_idx) {
	case 0: return 20U;
	case 1: return 40U;
	case 2: return 80U;
	case 3: return 160U;
	case 4: return 80U;
	case 5: return 160U;
	default: return 20U;
	}
}

static void read_rf_band_locked(int dev_fd, uint32_t lo_addr, uint32_t bw_addr,
				struct rf_band_state *st)
{
	uint32_t lo_khz = 0, bw_word = 0;

	memset(st, 0, sizeof(*st));
	if (wmsdr_wmpeek(dev_fd, lo_addr, &lo_khz) != 0 ||
	    wmsdr_wmpeek(dev_fd, bw_addr, &bw_word) != 0)
		return;
	if (lo_khz < 100000U || lo_khz > 8000000U)
		return;
	st->ok = 1;
	st->lo_khz = lo_khz;
	st->lo_mhz = (double)lo_khz / 1000.0;
	st->bw_word = bw_word;
	st->pri = bw_word & 0xFFU;
	st->bw_idx = (bw_word >> 8) & 0xFFU;
	st->bw_mhz = bw_idx_to_mhz(st->bw_idx);
	st->rx_n = (bw_word >> 16) & 0xFFU;
	st->tx_n = (bw_word >> 24) & 0xFFU;
}

static int build_status_json(struct httpd_ctx *ctx, char *out, size_t cap,
			     int streaming, const char *host_hdr)
{
	uint32_t fw_state = 0, raw_tick = 0;
	struct wm_iq_page p;
	struct wmsdr_net_stat nst;
	struct rf_band_state b0, b1;
	char rf_buf[512];
	int pos = 0, knet_active;

	memset(&p, 0, sizeof(p));
	memset(&nst, 0, sizeof(nst));
	pthread_mutex_lock(&ctx->iq_lock);
	knet_active = ctx->kernel_net_active;
	if (wmsdr_get_fw_state(ctx->dev_fd, &fw_state) != 0 ||
	    wmsdr_rd32(ctx->dev_fd, WMIQ_RAW_TICK_ADDR, &raw_tick) != 0 ||
	    wmsdr_read_iq_page(ctx->dev_fd, &p) != 0) {
		pthread_mutex_unlock(&ctx->iq_lock);
		return snprintf(out, cap,
				"{\"type\":\"status\",\"ok\":false,\"error\":\"wmsdr read failed\"}");
	}
	read_rf_band_locked(ctx->dev_fd, CHINFO_BAND0_LO, CHINFO_BAND0_BW, &b0);
	read_rf_band_locked(ctx->dev_fd, CHINFO_BAND1_LO, CHINFO_BAND1_BW, &b1);
	if (knet_active)
		(void)wmsdr_net_stat(ctx->dev_fd, &nst);
	pthread_mutex_unlock(&ctx->iq_lock);

	pos += snprintf(rf_buf + pos, sizeof(rf_buf) - (size_t)pos, "\"rf\":{");
	if (b0.ok) {
		pos += snprintf(rf_buf + pos, sizeof(rf_buf) - (size_t)pos,
				"\"band0\":{\"lo_khz\":%u,\"lo_mhz\":%.3f,\"bw_idx\":%u,"
				"\"bw_mhz\":%u,\"pri\":%u,\"rx_n\":%u,\"tx_n\":%u}",
				b0.lo_khz, b0.lo_mhz, b0.bw_idx, b0.bw_mhz,
				b0.pri, b0.rx_n, b0.tx_n);
	}
	if (b1.ok) {
		pos += snprintf(rf_buf + pos, sizeof(rf_buf) - (size_t)pos,
				"%s\"band1\":{\"lo_khz\":%u,\"lo_mhz\":%.3f,\"bw_idx\":%u,"
				"\"bw_mhz\":%u,\"pri\":%u,\"rx_n\":%u,\"tx_n\":%u}",
				b0.ok ? "," : "", b1.lo_khz, b1.lo_mhz, b1.bw_idx,
				b1.bw_mhz, b1.pri, b1.rx_n, b1.tx_n);
	}
	snprintf(rf_buf + pos, sizeof(rf_buf) - (size_t)pos, "}");

	return snprintf(out, cap,
			"{\"type\":\"status\",\"ok\":true,\"demo\":false,\"direct_router\":true,"
			"\"host\":\"%s\",\"streaming\":%s,\"kernel_net\":%s,\"net_socks\":%u,"
			"\"net_captured\":%llu,\"net_sent\":%llu,\"net_dropped\":%llu,"
			"\"net_bytes\":%llu,\"net_eagain\":%llu,\"net_errors\":%llu,"
			"\"fw_state\":%u,\"raw_tick\":%u,"
			"\"magic\":%u,\"magic_ok\":%s,\"version\":%u,\"cmd\":%u,"
			"\"status\":%u,\"status_code\":%u,\"node\":%u,\"length\":%u,"
			"\"data_addr\":%u,\"sample_count\":%u,\"tick\":%u,\"error\":%u,"
			"\"cr0\":%u,\"cr2\":%u,\"cr9\":%u,\"bw\":%u,\"band\":%u,"
			"\"arg0\":%u,\"arg1\":%u,%s}",
			host_hdr ? host_hdr : "router",
			(streaming || knet_active) ? "true" : "false",
			knet_active ? "true" : "false",
			nst.n_socks,
			(unsigned long long)nst.frames_captured,
			(unsigned long long)nst.frames_sent,
			(unsigned long long)nst.frames_dropped,
			(unsigned long long)nst.bytes_sent,
			(unsigned long long)nst.send_eagain,
			(unsigned long long)nst.send_errors,
			fw_state, raw_tick, p.magic,
			p.magic == WMIQ_MAGIC ? "true" : "false",
			p.version, p.cmd, p.status, p.status, p.node, p.length,
			p.data_addr, p.sample_count, p.tick, p.error,
			p.cr0, p.cr2, p.cr9, p.bw & 0xFFU, (p.bw >> 8) & 1U,
			p.arg0, p.arg1, rf_buf);
}

static void fill_wsiq_hdr(uint8_t *dst, uint32_t seq, uint32_t node, uint32_t len,
			  const struct wm_iq_page *p, int actual_direct,
			  uint32_t bw, uint32_t band)
{
	uint32_t flags = (actual_direct ? 1U : 0U) | ((bw & 0xFFU) << 8) | ((band & 1U) << 16);

	wmsdr_put_le32(dst + 0,  WSIQ_MAGIC);
	wmsdr_put_le32(dst + 4,  1U);
	wmsdr_put_le32(dst + 8,  seq);
	wmsdr_put_le32(dst + 12, node);
	wmsdr_put_le32(dst + 16, len);
	wmsdr_put_le32(dst + 20, p->cr0);
	wmsdr_put_le32(dst + 24, p->cr9);
	wmsdr_put_le32(dst + 28, flags);
}

static int capture_wsiq_locked(struct httpd_ctx *ctx, uint32_t node, uint32_t len,
			       uint32_t bw, uint32_t band, int direct,
			       uint8_t *frame_buf, struct wm_iq_page *page_out,
			       uint32_t *seq_out, int *actual_direct_out)
{
	struct wm_iq_page p;
	int actual_direct = (band == 0U && node != 15U && direct) ? 1 : 0;
	uint32_t seq;
	int rc;

	memset(&p, 0, sizeof(p));
	pthread_mutex_lock(&ctx->iq_lock);
	if (ctx->stream.active)
		wmsdr_iq_stream_stop(&ctx->stream);
	rc = wmsdr_iq_capture_buf(ctx->dev_fd, node, len, bw, band, actual_direct,
				  &p, frame_buf + 32);
	if (rc == 0) {
		ctx->seq++;
		seq = ctx->seq;
	} else {
		seq = ctx->seq;
	}
	pthread_mutex_unlock(&ctx->iq_lock);

	if (page_out)
		*page_out = p;
	if (seq_out)
		*seq_out = seq;
	if (actual_direct_out)
		*actual_direct_out = actual_direct;
	if (rc != 0)
		return rc;

	fill_wsiq_hdr(frame_buf, seq, node, len, &p, actual_direct, bw, band);
	return 0;
}

/* ---- HTTP response & query helpers ---- */

static int format_http_hdr(char *hdr, size_t cap, int status, const char *status_text,
			   const char *ctype, size_t body_len, const char *extra_hdrs)
{
	return snprintf(hdr, cap,
			"HTTP/1.1 %d %s\r\n"
			"Content-Type: %s\r\n"
			"Content-Length: %zu\r\n"
			"Cache-Control: no-store\r\n"
			"Access-Control-Allow-Origin: *\r\n"
			"Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
			"Access-Control-Allow-Headers: Content-Type\r\n"
			"Access-Control-Expose-Headers: X-IQ-Node, X-IQ-Words, X-IQ-Sample-Count, X-IQ-CR0, X-IQ-CR9, X-IQ-Direct, X-IQ-Seq, X-IQ-BW, X-IQ-Band, X-IQ-Demo\r\n"
			"Connection: close\r\n"
			"%s\r\n",
			status, status_text, ctype, body_len,
			extra_hdrs ? extra_hdrs : "");
}

static void send_http_response(int fd, int status, const char *status_text,
			       const char *ctype, const void *body, size_t body_len,
			       const char *extra_hdrs)
{
	char hdr[2048];
	int n = format_http_hdr(hdr, sizeof(hdr), status, status_text, ctype, body_len, extra_hdrs);

	if (n > 0)
		(void)send_two_iov(fd, hdr, (size_t)n, body, body_len);
}

static void send_json_response(int fd, int status, const char *status_text, const char *json)
{
	send_http_response(fd, status, status_text, "application/json; charset=utf-8",
			   json, strlen(json), NULL);
}

static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

static int query_get(const char *qs, const char *key, char *out, size_t cap)
{
	size_t klen = strlen(key);
	const char *p = qs;

	while (p && *p) {
		const char *eq = strchr(p, '=');
		const char *amp = strchr(p, '&');
		size_t tok_len = amp ? (size_t)(amp - p) : strlen(p);

		if (eq && (size_t)(eq - p) == klen && strncmp(p, key, klen) == 0) {
			size_t vlen = tok_len - klen - 1U;
			if (vlen >= cap)
				vlen = cap - 1U;
			memcpy(out, eq + 1, vlen);
			out[vlen] = '\0';
			return 1;
		}
		p = amp ? (amp + 1) : NULL;
	}
	return 0;
}

static uint32_t parse_u32_def(const char *s, uint32_t def)
{
	char *end = NULL;
	unsigned long v;

	if (!s || !*s)
		return def;
	if (!strcasecmp(s, "true")) return 1U;
	if (!strcasecmp(s, "false")) return 0U;
	errno = 0;
	v = strtoul(s, &end, 0);
	if (errno || !end || *end)
		return def;
	return (uint32_t)v;
}

/* ---- Minimal flat JSON field extractor for /ws client commands ---- */

static int json_find_val(const char *json, const char *key, const char **val_out)
{
	char pat[64];
	const char *p;

	snprintf(pat, sizeof(pat), "\"%s\"", key);
	p = strstr(json, pat);
	while (p) {
		const char *q = p + strlen(pat);

		while (*q == ' ' || *q == '\t' || *q == '\r' || *q == '\n')
			q++;
		if (*q == ':') {
			q++;
			while (*q == ' ' || *q == '\t' || *q == '\r' || *q == '\n')
				q++;
			*val_out = q;
			return 1;
		}
		p = strstr(q, pat);
	}
	return 0;
}

static int json_get_str(const char *json, const char *key, char *out, size_t cap)
{
	const char *v;
	size_t i = 0;

	if (!json_find_val(json, key, &v) || *v != '"')
		return 0;
	v++;
	while (*v && *v != '"' && i + 1 < cap) {
		if (*v == '\\' && v[1])
			v++;
		out[i++] = *v++;
	}
	out[i] = '\0';
	return 1;
}

static uint32_t json_get_u32(const char *json, const char *key, uint32_t def)
{
	const char *v;
	char *end = NULL;
	unsigned long n;

	if (!json_find_val(json, key, &v))
		return def;
	if (!strncmp(v, "true", 4)) return 1U;
	if (!strncmp(v, "false", 5)) return 0U;
	if (*v == '"') {
		char tmp[32];
		size_t i = 0;
		v++;
		while (*v && *v != '"' && i + 1 < sizeof(tmp))
			tmp[i++] = *v++;
		tmp[i] = '\0';
		return parse_u32_def(tmp, def);
	}
	errno = 0;
	n = strtoul(v, &end, 0);
	if (errno || end == v)
		return def;
	return (uint32_t)n;
}

/* ---- WebSocket RFC 6455 session ---- */

static size_t ws_encode_hdr(uint8_t opcode, size_t len, uint8_t hdr[10])
{
	hdr[0] = (uint8_t)(0x80U | (opcode & 0x0FU));
	if (len < 126) {
		hdr[1] = (uint8_t)len;
		return 2;
	}
	if (len < 65536) {
		hdr[1] = 126U;
		hdr[2] = (uint8_t)((len >> 8) & 0xFFU);
		hdr[3] = (uint8_t)(len & 0xFFU);
		return 4;
	}
	{
		uint64_t l64 = (uint64_t)len;
		int i;

		hdr[1] = 127U;
		for (i = 0; i < 8; i++)
			hdr[2 + i] = (uint8_t)((l64 >> ((7 - i) * 8)) & 0xFFU);
		return 10;
	}
}

static int ws_send_frame(int fd, uint8_t opcode, const void *data, size_t len)
{
	uint8_t hdr[10];
	size_t hlen = ws_encode_hdr(opcode, len, hdr);

	return send_two_iov(fd, hdr, hlen, data, len);
}

/* Returns 0 on success, -1 on ICAP capture error, -2 on socket/oom error. */
static int send_wsiq_ws_locked(struct httpd_ctx *ctx, int client_fd,
			       uint32_t node, uint32_t len, uint32_t bw,
			       uint32_t band, int direct, int use_stream,
			       uint8_t **frame_buf_p, size_t *frame_cap_p,
			       struct wm_iq_page *page_out)
{
	size_t need = 32U + (size_t)len * 4U;

	if (ctx->dma_map.base && ctx->dma_map.data_off >= 64U &&
	    (size_t)len * 4U <= ctx->dma_map.map_size - ctx->dma_map.data_off) {
		struct wm_iq_page p;
		int actual_direct = (band == 0U && node != 15U && direct) ? 1 : 0;
		uint8_t *iq_data = NULL;
		uint8_t ws_hdr[10];
		size_t hlen;
		uint32_t seq;
		int rc, s_rc;

		memset(&p, 0, sizeof(p));
		pthread_mutex_lock(&ctx->iq_lock);
		if (use_stream) {
			rc = wmsdr_iq_stream_next_mmap(&ctx->stream, node, len, bw,
						       band, actual_direct, &p, &iq_data);
		} else {
			if (ctx->stream.active)
				wmsdr_iq_stream_stop(&ctx->stream);
			rc = wmsdr_iq_capture_mmap(ctx->dev_fd, &ctx->dma_map, node, len, bw,
						   band, actual_direct, &p, &iq_data);
		}
		if (rc == 0) {
			ctx->seq++;
			seq = ctx->seq;
			fill_wsiq_hdr(iq_data - 32U, seq, node, len, &p, actual_direct, bw, band);
			hlen = ws_encode_hdr(0x2U, need, ws_hdr);
			memcpy(iq_data - 32U - hlen, ws_hdr, hlen);
			s_rc = send_all(client_fd, iq_data - 32U - hlen, hlen + need);
		} else {
			s_rc = 0;
		}
		pthread_mutex_unlock(&ctx->iq_lock);

		if (page_out)
			*page_out = p;
		if (rc != 0)
			return -1;
		return s_rc == 0 ? 0 : -2;
	}

	if (need > *frame_cap_p) {
		uint8_t *nb = realloc(*frame_buf_p, need);

		if (!nb)
			return -2;
		*frame_buf_p = nb;
		*frame_cap_p = need;
	}
	if (capture_wsiq_locked(ctx, node, len, bw, band, direct,
				*frame_buf_p, page_out, NULL, NULL) != 0)
		return -1;
	return ws_send_frame(client_fd, 0x2U, *frame_buf_p, need) == 0 ? 0 : -2;
}

static int ws_send_json(int fd, const char *json)
{
	return ws_send_frame(fd, 0x1, json, strlen(json));
}

static int ws_read_frame(int fd, uint8_t *opcode_out, uint8_t *payload,
			 size_t max_payload, size_t *len_out)
{
	uint8_t h[2], mask[4];
	uint64_t plen;
	int masked;
	size_t i;

	if (recv_exact(fd, h, 2) != 0)
		return -1;
	*opcode_out = h[0] & 0x0FU;
	masked = (h[1] & 0x80U) ? 1 : 0;
	plen = (uint64_t)(h[1] & 0x7FU);
	if (plen == 126U) {
		uint8_t ext[2];
		if (recv_exact(fd, ext, 2) != 0)
			return -1;
		plen = ((uint64_t)ext[0] << 8) | (uint64_t)ext[1];
	} else if (plen == 127U) {
		uint8_t ext[8];
		if (recv_exact(fd, ext, 8) != 0)
			return -1;
		plen = 0;
		for (i = 0; i < 8; i++)
			plen = (plen << 8) | (uint64_t)ext[i];
	}
	if (plen > max_payload)
		return -1;
	if (masked && recv_exact(fd, mask, 4) != 0)
		return -1;
	if (plen > 0 && recv_exact(fd, payload, (size_t)plen) != 0)
		return -1;
	if (masked) {
		for (i = 0; i < (size_t)plen; i++)
			payload[i] ^= mask[i & 3U];
	}
	*len_out = (size_t)plen;
	return 0;
}

static void httpd_stop_knet_locked(struct httpd_ctx *ctx)
{
	struct wmsdr_net_stat st;

	if (!ctx->kernel_net_active)
		return;
	if (wmsdr_net_stop(ctx->dev_fd, &st) == 0 && st.next_seq > ctx->seq)
		ctx->seq = st.next_seq;
	wmsdr_iq_stream_stop(&ctx->stream);
	__atomic_store_n(&ctx->kernel_net_active, 0, __ATOMIC_RELEASE);
	ctx->kernel_net_lanes = 0;
}

static void httpd_detach_sock_locked(struct httpd_ctx *ctx, int client_fd,
				     int *sock_attached, uint32_t expected_sid)
{
	if (!*sock_attached)
		return;
	if (ctx->kernel_net_active &&
	    (expected_sid == 0U || ctx->kernel_net_sid == expected_sid)) {
		(void)wmsdr_net_del_sock(ctx->dev_fd, client_fd);
		if (ctx->kernel_net_lanes > 0)
			ctx->kernel_net_lanes--;
	}
	*sock_attached = 0;
}

static int httpd_start_knet_locked(struct httpd_ctx *ctx, uint32_t node, uint32_t len,
				   uint32_t bw, uint32_t band, int direct,
				   uint32_t *sid_out, uint32_t *slots_out)
{
	struct wmsdr_net_cfg cfg;
	int hw_mode = 1;
	uint32_t sid;

	httpd_stop_knet_locked(ctx);
	wmsdr_iq_stream_stop(&ctx->stream);

	if (wmsdr_net_arm_hw(&ctx->stream, node, len, bw, band, direct, &hw_mode) != 0)
		return -1;

	memset(&cfg, 0, sizeof(cfg));
	cfg.node = node;
	cfg.len_words = len;
	cfg.bw = bw;
	cfg.band = band;
	cfg.direct = (uint32_t)(ctx->stream.direct ? 1U : 0U);
	cfg.mode = (uint32_t)hw_mode;
	cfg.flags = 0U;
	cfg.sndbuf_kb = 192U;
	cfg.seq_start = ctx->seq;

	if (wmsdr_net_start(ctx->dev_fd, &cfg) != 0) {
		int saved = errno;
		wmsdr_iq_stream_stop(&ctx->stream);
		errno = saved;
		return -1;
	}

	sid = ctx->kernel_net_sid + 1U;
	if (sid == 0U)
		sid = 1U;
	__atomic_store_n(&ctx->kernel_net_sid, sid, __ATOMIC_RELEASE);
	__atomic_store_n(&ctx->kernel_net_active, 1, __ATOMIC_RELEASE);
	*sid_out = sid;
	*slots_out = cfg.slot_count;
	return 0;
}

static void ws_run_session(int client_fd, const char *ws_key, const char *host_hdr)
{
	char accept_in[256], accept_b64[64], hs[512];
	uint8_t sha[20];
	uint8_t *in_buf, *frame_buf = NULL;
	size_t frame_cap = 0;
	int streaming = 0, used_stream = 0, sock_attached = 0;
	uint32_t ctrl_owns_sid = 0, attached_sid = 0;
	uint32_t node = HTTPD_DEFAULT_NODE, len = HTTPD_DEFAULT_LEN;
	uint32_t bw = HTTPD_DEFAULT_BW, band = HTTPD_DEFAULT_BAND;
	uint32_t interval_ms = 0;
	int direct = 0;
	uint64_t next_due = 0;
	int one = 1;

	snprintf(accept_in, sizeof(accept_in), "%s%s", ws_key, WS_GUID);
	sha1_digest((const uint8_t *)accept_in, strlen(accept_in), sha);
	base64_encode(sha, 20, accept_b64);

	snprintf(hs, sizeof(hs),
		 "HTTP/1.1 101 Switching Protocols\r\n"
		 "Upgrade: websocket\r\n"
		 "Connection: Upgrade\r\n"
		 "Sec-WebSocket-Accept: %s\r\n"
		 "Access-Control-Allow-Origin: *\r\n\r\n",
		 accept_b64);
	if (send_all(client_fd, hs, strlen(hs)) != 0)
		return;

	(void)setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
	in_buf = malloc(HTTPD_MAX_WS_IN_BYTES + 1U);
	if (!in_buf)
		return;

	while (!g_ctx.stop) {
		struct pollfd pfd = { .fd = client_fd, .events = POLLIN, .revents = 0 };
		int wait_ms = 200;
		int pr;

		if (attached_sid != 0U) {
			if (!__atomic_load_n(&g_ctx.kernel_net_active, __ATOMIC_ACQUIRE) ||
			    __atomic_load_n(&g_ctx.kernel_net_sid, __ATOMIC_ACQUIRE) != attached_sid)
				break;
			wait_ms = 100;
		} else if (streaming) {
			uint64_t t = now_ms();
			if (t >= next_due)
				wait_ms = 0;
			else {
				uint64_t diff = next_due - t;
				wait_ms = diff > 50ULL ? 50 : (int)diff;
			}
		}

		pr = poll(&pfd, 1, wait_ms);
		if (pr < 0) {
			if (errno == EINTR)
				continue;
			break;
		}

		if (pr > 0 && (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)))
			break;

		if (pr > 0 && (pfd.revents & POLLIN)) {
			uint8_t op = 0;
			size_t plen = 0;

			if (ws_read_frame(client_fd, &op, in_buf, HTTPD_MAX_WS_IN_BYTES, &plen) != 0)
				break;
			if (op == 0x8U) {
				if (sock_attached) {
					pthread_mutex_lock(&g_ctx.iq_lock);
					httpd_detach_sock_locked(&g_ctx, client_fd,
								 &sock_attached, attached_sid);
					pthread_mutex_unlock(&g_ctx.iq_lock);
				}
				(void)ws_send_frame(client_fd, 0x8U, in_buf, plen >= 2 ? 2 : 0);
				break;
			}
			if (op == 0x9U) {
				if (!sock_attached && ws_send_frame(client_fd, 0xAU, in_buf, plen) != 0)
					break;
				continue;
			}
			if (op == 0xAU)
				continue;
			if (op == 0x1U && attached_sid == 0U) {
				char mtype[32] = "";
				in_buf[plen] = '\0';
				if (!json_get_str((const char *)in_buf, "type", mtype, sizeof(mtype))) {
					if (!sock_attached)
						ws_send_json(client_fd, "{\"type\":\"error\",\"message\":\"Missing type\"}");
					continue;
				}
				if (!strcmp(mtype, "status")) {
					char sbuf[2048];
					build_status_json(&g_ctx, sbuf, sizeof(sbuf), streaming, host_hdr);
					if (!sock_attached && ws_send_json(client_fd, sbuf) != 0)
						break;
				} else if (!strcmp(mtype, "stop")) {
					char sbuf[256];
					pthread_mutex_lock(&g_ctx.iq_lock);
					httpd_detach_sock_locked(&g_ctx, client_fd,
								 &sock_attached, 0U);
					if (ctrl_owns_sid && g_ctx.kernel_net_active &&
					    ctrl_owns_sid == g_ctx.kernel_net_sid)
						httpd_stop_knet_locked(&g_ctx);
					ctrl_owns_sid = 0U;
					if (used_stream) {
						wmsdr_iq_stream_stop(&g_ctx.stream);
						used_stream = 0;
					}
					streaming = 0;
					pthread_mutex_unlock(&g_ctx.iq_lock);
					snprintf(sbuf, sizeof(sbuf),
						 "{\"type\":\"status\",\"ok\":true,\"demo\":false,\"direct_router\":true,"
						 "\"host\":\"%s\",\"streaming\":false}", host_hdr);
					if (ws_send_json(client_fd, sbuf) != 0)
						break;
				} else if (!strcmp(mtype, "start")) {
					char sbuf[512];
					int control_only = json_get_u32((const char *)in_buf, "controlOnly", 0U) ? 1 : 0;
					uint32_t sid = 0U, slots = 0U;
					int try_knet, krc = -1;

					node = json_get_u32((const char *)in_buf, "node", node);
					len = clamp_u32(json_get_u32((const char *)in_buf, "len", len), 64U, WMIQ_SPECTRUM_MAX_WORDS);
					bw = clamp_u32(json_get_u32((const char *)in_buf, "bw", bw), 0U, 3U);
					band = json_get_u32((const char *)in_buf, "band", band) & 1U;
					direct = json_get_u32((const char *)in_buf, "direct", 0U) ? 1 : 0;
					if (band != 0U || node == 15U)
						direct = 0;
					interval_ms = clamp_u32(json_get_u32((const char *)in_buf, "intervalMs", 0U), 0U, 10000U);
					try_knet = (control_only || interval_ms <= 20U) ? 1 : 0;

					pthread_mutex_lock(&g_ctx.iq_lock);
					httpd_detach_sock_locked(&g_ctx, client_fd,
								 &sock_attached, 0U);
					if (ctrl_owns_sid && g_ctx.kernel_net_active &&
					    ctrl_owns_sid == g_ctx.kernel_net_sid)
						httpd_stop_knet_locked(&g_ctx);
					ctrl_owns_sid = 0U;
					if (used_stream) {
						wmsdr_iq_stream_stop(&g_ctx.stream);
						used_stream = 0;
					}
					streaming = 0;

					if (try_knet) {
						krc = httpd_start_knet_locked(&g_ctx, node, len, bw,
									      band, direct, &sid, &slots);
						if (krc == 0)
							ctrl_owns_sid = sid;
					} else {
						httpd_stop_knet_locked(&g_ctx);
						streaming = 1;
						next_due = now_ms();
					}
					pthread_mutex_unlock(&g_ctx.iq_lock);

					if (try_knet && krc == 0) {
						snprintf(sbuf, sizeof(sbuf),
							 "{\"type\":\"status\",\"ok\":true,\"demo\":false,\"direct_router\":true,"
							 "\"host\":\"%s\",\"streaming\":true,\"kernelNet\":true,"
							 "\"controlOnly\":%s,\"sid\":%u,\"slots\":%u,"
							 "\"node\":%u,\"len\":%u,\"bw\":%u,\"band\":%u,"
							 "\"direct\":%s,\"intervalMs\":%u}",
							 host_hdr, control_only ? "true" : "false",
							 sid, slots, node, len, bw, band,
							 direct ? "true" : "false", interval_ms);
						if (ws_send_json(client_fd, sbuf) != 0)
							break;
						if (!control_only) {
							int arc = -1;
							pthread_mutex_lock(&g_ctx.iq_lock);
							if (g_ctx.kernel_net_active &&
							    g_ctx.kernel_net_sid == sid) {
								arc = wmsdr_net_add_sock(g_ctx.dev_fd, client_fd);
								if (arc == 0) {
									sock_attached = 1;
									g_ctx.kernel_net_lanes++;
								}
							}
							pthread_mutex_unlock(&g_ctx.iq_lock);
							if (arc != 0)
								break;
						}
					} else if (try_knet && (errno == ENOTTY || errno == EOPNOTSUPP) &&
						   !control_only) {
						streaming = 1;
						used_stream = 1;
						next_due = now_ms();
						snprintf(sbuf, sizeof(sbuf),
							 "{\"type\":\"status\",\"ok\":true,\"demo\":false,\"direct_router\":true,"
							 "\"host\":\"%s\",\"streaming\":true,\"kernelNet\":false,"
							 "\"node\":%u,\"len\":%u,\"bw\":%u,\"band\":%u,"
							 "\"direct\":%s,\"intervalMs\":%u}",
							 host_hdr, node, len, bw, band,
							 direct ? "true" : "false", interval_ms);
						if (ws_send_json(client_fd, sbuf) != 0)
							break;
					} else if (try_knet) {
						if (ws_send_json(client_fd,
								 "{\"type\":\"error\",\"message\":\"Kernel net stream start failed\"}") != 0)
							break;
					} else {
						snprintf(sbuf, sizeof(sbuf),
							 "{\"type\":\"status\",\"ok\":true,\"demo\":false,\"direct_router\":true,"
							 "\"host\":\"%s\",\"streaming\":true,\"kernelNet\":false,"
							 "\"node\":%u,\"len\":%u,\"bw\":%u,\"band\":%u,"
							 "\"direct\":%s,\"intervalMs\":%u}",
							 host_hdr, node, len, bw, band,
							 direct ? "true" : "false", interval_ms);
						if (ws_send_json(client_fd, sbuf) != 0)
							break;
					}
				} else if (!strcmp(mtype, "attach")) {
					uint32_t sid = json_get_u32((const char *)in_buf, "sid", 0U);
					int ok = 0, arc = -1;
					char abuf[160];

					if (sock_attached)
						continue;
					pthread_mutex_lock(&g_ctx.iq_lock);
					ok = (sid != 0U && g_ctx.kernel_net_active &&
					      g_ctx.kernel_net_sid == sid);
					pthread_mutex_unlock(&g_ctx.iq_lock);
					if (!ok) {
						if (ws_send_json(client_fd,
								 "{\"type\":\"error\",\"message\":\"Stream session inactive\"}") != 0)
							break;
						continue;
					}
					snprintf(abuf, sizeof(abuf),
						 "{\"type\":\"status\",\"ok\":true,\"attached\":true,\"sid\":%u}", sid);
					if (ws_send_json(client_fd, abuf) != 0)
						break;
					pthread_mutex_lock(&g_ctx.iq_lock);
					if (g_ctx.kernel_net_active && g_ctx.kernel_net_sid == sid) {
						arc = wmsdr_net_add_sock(g_ctx.dev_fd, client_fd);
						if (arc == 0) {
							attached_sid = sid;
							sock_attached = 1;
							g_ctx.kernel_net_lanes++;
						}
					}
					pthread_mutex_unlock(&g_ctx.iq_lock);
					if (arc != 0)
						break;
				} else if (!strcmp(mtype, "oneshot") && !sock_attached) {
					uint32_t onode = json_get_u32((const char *)in_buf, "node", node);
					uint32_t olen = clamp_u32(json_get_u32((const char *)in_buf, "len", len), 64U, WMIQ_SPECTRUM_MAX_WORDS);
					uint32_t obw = clamp_u32(json_get_u32((const char *)in_buf, "bw", bw), 0U, 3U);
					uint32_t oband = json_get_u32((const char *)in_buf, "band", band) & 1U;
					int odir = json_get_u32((const char *)in_buf, "direct", 0U) ? 1 : 0;
					int src = send_wsiq_ws_locked(&g_ctx, client_fd, onode, olen,
								      obw, oband, odir, 0,
								      &frame_buf, &frame_cap, NULL);

					if (src == -2)
						break;
					if (src != 0)
						ws_send_json(client_fd, "{\"type\":\"error\",\"message\":\"ICAP oneshot failed\"}");
				} else if (!sock_attached) {
					ws_send_json(client_fd, "{\"type\":\"error\",\"message\":\"Unknown message type\"}");
				}
			}
		}

		if (streaming && now_ms() >= next_due) {
			struct wm_iq_page p;
			int fast = (interval_ms <= 20U) ? 1 : 0;
			int rc;

			memset(&p, 0, sizeof(p));
			if (fast)
				used_stream = 1;
			rc = send_wsiq_ws_locked(&g_ctx, client_fd, node, len, bw, band, direct,
						 fast, &frame_buf, &frame_cap, &p);
			if (rc == -2)
				break;
			if (rc != 0) {
				char ebuf[160];
				snprintf(ebuf, sizeof(ebuf),
					 "{\"type\":\"error\",\"message\":\"ICAP capture err=%d status=%u error=0x%x\"}",
					 rc, p.status, p.error);
				streaming = 0;
				if (ws_send_json(client_fd, ebuf) != 0)
					break;
			}
			next_due = now_ms() + (uint64_t)interval_ms;
		}
	}

	pthread_mutex_lock(&g_ctx.iq_lock);
	httpd_detach_sock_locked(&g_ctx, client_fd, &sock_attached, attached_sid);
	if (ctrl_owns_sid && g_ctx.kernel_net_active &&
	    ctrl_owns_sid == g_ctx.kernel_net_sid)
		httpd_stop_knet_locked(&g_ctx);
	if (used_stream)
		wmsdr_iq_stream_stop(&g_ctx.stream);
	pthread_mutex_unlock(&g_ctx.iq_lock);
	free(frame_buf);
	free(in_buf);
}

/* ---- Static file & REST HTTP handling ---- */

static const char *mime_for_path(const char *path)
{
	const char *dot = strrchr(path, '.');

	if (!dot) return "application/octet-stream";
	if (!strcmp(dot, ".html")) return "text/html; charset=utf-8";
	if (!strcmp(dot, ".js"))   return "application/javascript; charset=utf-8";
	if (!strcmp(dot, ".css"))  return "text/css; charset=utf-8";
	if (!strcmp(dot, ".json")) return "application/json; charset=utf-8";
	if (!strcmp(dot, ".md"))   return "text/markdown; charset=utf-8";
	if (!strcmp(dot, ".png"))  return "image/png";
	if (!strcmp(dot, ".svg"))  return "image/svg+xml";
	return "application/octet-stream";
}

static void handle_static(int fd, const char *req_path)
{
	char full[1024];
	const char *rel = (!strcmp(req_path, "/") || !req_path[0]) ? "index.html" : (req_path + 1);
	FILE *f;
	struct stat st;
	uint8_t *buf;

	if (strstr(rel, "..") || strchr(rel, '\\')) {
		send_json_response(fd, 403, "Forbidden", "{\"ok\":false,\"error\":\"Forbidden\"}");
		return;
	}
	snprintf(full, sizeof(full), "%s/%s", g_ctx.web_root, rel);
	if (stat(full, &st) != 0 || !S_ISREG(st.st_mode) || st.st_size < 0 || st.st_size > 2 * 1024 * 1024) {
		send_json_response(fd, 404, "Not Found", "{\"ok\":false,\"error\":\"File not found\"}");
		return;
	}
	f = fopen(full, "rb");
	if (!f) {
		send_json_response(fd, 500, "Internal Server Error", "{\"ok\":false,\"error\":\"Open failed\"}");
		return;
	}
	buf = malloc((size_t)st.st_size);
	if (!buf) {
		fclose(f);
		send_json_response(fd, 500, "Internal Server Error", "{\"ok\":false,\"error\":\"OOM\"}");
		return;
	}
	if (st.st_size > 0 && fread(buf, 1, (size_t)st.st_size, f) != (size_t)st.st_size) {
		free(buf);
		fclose(f);
		send_json_response(fd, 500, "Internal Server Error", "{\"ok\":false,\"error\":\"Read failed\"}");
		return;
	}
	fclose(f);
	send_http_response(fd, 200, "OK", mime_for_path(full), buf, (size_t)st.st_size, NULL);
	free(buf);
}

static void handle_config(int fd, const char *host_hdr)
{
	char j[2048];

	snprintf(j, sizeof(j),
		 "{\"ok\":true,\"host\":\"%s\",\"port\":%d,\"demo\":false,\"direct_router\":true,"
		 "\"default_node\":%u,\"default_len\":%u,\"default_bw\":%u,\"default_band\":%u,"
		 "\"default_format\":\"u32_12\",\"default_stream_mode\":\"even\","
		 "\"default_fs_mhz\":%.1f,\"default_lo_mhz\":2462.0,"
		 "\"bands\":{"
		 "\"0\":{\"band\":0,\"name\":\"2.4G\",\"label\":\"2.4 GHz (Band 0 / PHY0)\","
		 "\"adc_fs_mhz\":%.1f,\"rxfd_fs_mhz\":%.1f,\"default_lo_mhz\":2462.0},"
		 "\"1\":{\"band\":1,\"name\":\"5G\",\"label\":\"5 GHz (Band 1 / PHY1)\","
		 "\"adc_fs_mhz\":%.1f,\"rxfd_fs_mhz\":%.1f,\"default_lo_mhz\":5250.0}"
		 "},"
		 "\"node_profiles\":{"
		 "\"15\":{\"label\":\"ADC WF1/0 (Node 15, 0x4000)\",\"format\":\"u32_12\","
		 "\"streamMode\":\"even\",\"fsMhz\":%.1f,\"recommendedMode\":\"api\"},"
		 "\"6\":{\"label\":\"RXFD WF1/0 (Node 6, 0x10604000)\",\"format\":\"u32_14\","
		 "\"streamMode\":\"even\",\"fsMhz\":%.1f,\"recommendedMode\":\"api\"},"
		 "\"8\":{\"label\":\"RXFE/AGC (Node 8, 0x10804000)\",\"format\":\"u32_14\","
		 "\"streamMode\":\"even\",\"fsMhz\":%.1f,\"recommendedMode\":\"api\"}"
		 "}}",
		 host_hdr, g_ctx.port,
		 HTTPD_DEFAULT_NODE, HTTPD_DEFAULT_LEN, HTTPD_DEFAULT_BW, HTTPD_DEFAULT_BAND,
		 BAND0_ADC_FS_MHZ,
		 BAND0_ADC_FS_MHZ, RXFD_BW20_FS_MHZ,
		 BAND1_ADC_FS_MHZ, RXFD_BW20_FS_MHZ,
		 BAND0_ADC_FS_MHZ, RXFD_BW20_FS_MHZ, RXFD_BW20_FS_MHZ);
	send_json_response(fd, 200, "OK", j);
}

static void format_iq_extra_hdrs(char *extra, size_t cap, uint32_t node, uint32_t len,
				 const struct wm_iq_page *p, int actual_direct,
				 uint32_t seq, uint32_t bw, uint32_t band)
{
	snprintf(extra, cap,
		 "X-IQ-Node: %u\r\n"
		 "X-IQ-Words: %u\r\n"
		 "X-IQ-Sample-Count: %u\r\n"
		 "X-IQ-CR0: 0x%08x\r\n"
		 "X-IQ-CR9: 0x%08x\r\n"
		 "X-IQ-Direct: %d\r\n"
		 "X-IQ-Seq: %u\r\n"
		 "X-IQ-BW: %u\r\n"
		 "X-IQ-Band: %u\r\n"
		 "X-IQ-Demo: 0\r\n",
		 node, len, p->sample_count, p->cr0, p->cr9, actual_direct, seq, bw, band);
}

static void handle_capture_http(int fd, const char *qs)
{
	char val[32];
	uint32_t node = HTTPD_DEFAULT_NODE, len = HTTPD_DEFAULT_LEN;
	uint32_t bw = HTTPD_DEFAULT_BW, band = HTTPD_DEFAULT_BAND;
	int direct = 0, actual_direct = 0;
	struct wm_iq_page p;
	uint32_t seq = 0;
	int rc;
	char extra[384];
	char hdr[2048];
	size_t bytes;

	if (query_get(qs, "node", val, sizeof(val))) node = parse_u32_def(val, node);
	if (query_get(qs, "len",  val, sizeof(val))) len  = clamp_u32(parse_u32_def(val, len), 64U, WMIQ_SPECTRUM_MAX_WORDS);
	if (query_get(qs, "bw",   val, sizeof(val))) bw   = clamp_u32(parse_u32_def(val, bw), 0U, 3U);
	if (query_get(qs, "band", val, sizeof(val))) band = parse_u32_def(val, band) & 1U;
	if (query_get(qs, "direct", val, sizeof(val))) direct = parse_u32_def(val, 0U) ? 1 : 0;

	bytes = (size_t)len * 4U;
	if (g_ctx.dma_map.base && bytes <= g_ctx.dma_map.map_size - g_ctx.dma_map.data_off) {
		uint8_t *iq_data = NULL;
		int hlen;

		actual_direct = (band == 0U && node != 15U && direct) ? 1 : 0;
		memset(&p, 0, sizeof(p));
		pthread_mutex_lock(&g_ctx.iq_lock);
		if (g_ctx.stream.active)
			wmsdr_iq_stream_stop(&g_ctx.stream);
		rc = wmsdr_iq_capture_mmap(g_ctx.dev_fd, &g_ctx.dma_map, node, len, bw,
					   band, actual_direct, &p, &iq_data);
		if (rc == 0) {
			g_ctx.seq++;
			seq = g_ctx.seq;
			format_iq_extra_hdrs(extra, sizeof(extra), node, len, &p, actual_direct,
					     seq, bw, band);
			hlen = format_http_hdr(hdr, sizeof(hdr), 200, "OK",
					       "application/octet-stream", bytes, extra);
			if (hlen > 0)
				(void)send_two_iov(fd, hdr, (size_t)hlen, iq_data, bytes);
		}
		pthread_mutex_unlock(&g_ctx.iq_lock);
		if (rc == 0)
			return;
	} else {
		uint8_t *frame_buf = malloc(32U + bytes);

		if (!frame_buf) {
			send_json_response(fd, 500, "Internal Server Error",
					   "{\"ok\":false,\"error\":\"OOM\"}");
			return;
		}
		rc = capture_wsiq_locked(&g_ctx, node, len, bw, band, direct,
					 frame_buf, &p, &seq, &actual_direct);
		if (rc == 0) {
			format_iq_extra_hdrs(extra, sizeof(extra), node, len, &p, actual_direct,
					     seq, bw, band);
			send_http_response(fd, 200, "OK", "application/octet-stream",
					   frame_buf + 32, bytes, extra);
			free(frame_buf);
			return;
		}
		free(frame_buf);
	}

	{
		char err[192];

		snprintf(err, sizeof(err),
			 "{\"ok\":false,\"error\":\"ICAP capture failed rc=%d status=%u err=0x%x\"}",
			 rc, p.status, p.error);
		send_json_response(fd, 502, "Bad Gateway", err);
	}
}

static int find_header(const char *hdrs, const char *name, char *out, size_t cap)
{
	size_t nlen = strlen(name);
	const char *p = hdrs;

	while (p && *p) {
		const char *eol = strstr(p, "\r\n");
		const char *colon = strchr(p, ':');

		if (!eol)
			break;
		if (colon && colon < eol && (size_t)(colon - p) == nlen &&
		    strncasecmp(p, name, nlen) == 0) {
			const char *v = colon + 1;
			size_t vlen;
			while (v < eol && (*v == ' ' || *v == '\t'))
				v++;
			vlen = (size_t)(eol - v);
			if (vlen >= cap)
				vlen = cap - 1U;
			memcpy(out, v, vlen);
			out[vlen] = '\0';
			return 1;
		}
		p = eol + 2;
	}
	return 0;
}

static void *client_thread(void *arg)
{
	int client_fd = *(int *)arg;
	char *req;
	size_t used = 0;
	char *hdr_end = NULL;
	char method[16] = "", target[512] = "", host_hdr[128] = "192.168.6.2";
	char upgrade[64] = "", ws_key[128] = "";
	char *qmark;
	const char *qs = "";
	struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
	int one = 1, sndbuf = 512 * 1024;

	free(arg);
	(void)setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	(void)setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
	(void)setsockopt(client_fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
	(void)setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

	req = malloc(HTTPD_MAX_HDR_BYTES + 1U);
	if (!req) {
		close(client_fd);
		return NULL;
	}
	while (used < HTTPD_MAX_HDR_BYTES) {
		ssize_t n = recv(client_fd, req + used, HTTPD_MAX_HDR_BYTES - used, 0);
		if (n <= 0)
			break;
		used += (size_t)n;
		req[used] = '\0';
		hdr_end = strstr(req, "\r\n\r\n");
		if (hdr_end)
			break;
	}
	if (!hdr_end) {
		free(req);
		close(client_fd);
		return NULL;
	}

	if (sscanf(req, "%15s %511s", method, target) != 2) {
		free(req);
		close(client_fd);
		return NULL;
	}
	if (find_header(req, "Host", host_hdr, sizeof(host_hdr))) {
		char *colon = strchr(host_hdr, ':');
		if (colon)
			*colon = '\0';
	}
	(void)find_header(req, "Upgrade", upgrade, sizeof(upgrade));
	(void)find_header(req, "Sec-WebSocket-Key", ws_key, sizeof(ws_key));
	free(req);

	/* Clear read timeout for long-lived WS session / normal handling */
	tv.tv_sec = 0;
	(void)setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

	qmark = strchr(target, '?');
	if (qmark) {
		*qmark = '\0';
		qs = qmark + 1;
	}

	if (!strcasecmp(method, "OPTIONS")) {
		send_http_response(client_fd, 204, "No Content", "text/plain", "", 0, NULL);
	} else if (!strcasecmp(method, "GET") && !strcmp(target, "/ws") &&
		   !strcasecmp(upgrade, "websocket") && ws_key[0]) {
		ws_run_session(client_fd, ws_key, host_hdr);
	} else if (!strcasecmp(method, "GET") && !strcmp(target, "/api/config")) {
		handle_config(client_fd, host_hdr);
	} else if (!strcasecmp(method, "GET") && !strcmp(target, "/api/status")) {
		char sbuf[2048];
		build_status_json(&g_ctx, sbuf, sizeof(sbuf), 0, host_hdr);
		send_json_response(client_fd, 200, "OK", sbuf);
	} else if (!strcasecmp(method, "GET") && !strcmp(target, "/api/capture")) {
		handle_capture_http(client_fd, qs);
	} else if (!strcasecmp(method, "GET")) {
		handle_static(client_fd, target);
	} else {
		send_json_response(client_fd, 405, "Method Not Allowed",
				   "{\"ok\":false,\"error\":\"Method Not Allowed\"}");
	}

	close(client_fd);
	return NULL;
}

int wmsdr_httpd_main(int argc, char **argv)
{
	struct sockaddr_in addr;
	int srv_fd, one = 1;
	pthread_attr_t attr;

	memset(&g_ctx, 0, sizeof(g_ctx));
	g_ctx.port = (argc > 1) ? atoi(argv[1]) : HTTPD_DEFAULT_PORT;
	if (g_ctx.port <= 0 || g_ctx.port > 65535)
		g_ctx.port = HTTPD_DEFAULT_PORT;
	snprintf(g_ctx.web_root, sizeof(g_ctx.web_root), "%s",
		 (argc > 2) ? argv[2] : HTTPD_DEFAULT_WEB_ROOT);
	pthread_mutex_init(&g_ctx.iq_lock, NULL);

	g_ctx.dev_fd = wmsdr_open_dev();
	if (g_ctx.dev_fd < 0) {
		fprintf(stderr, "open %s: %s\n", WMSDR_DEV_PATH, strerror(errno));
		return 1;
	}
	if (wmsdr_mmap_open(g_ctx.dev_fd, &g_ctx.dma_map) != 0)
		memset(&g_ctx.dma_map, 0, sizeof(g_ctx.dma_map));
	wmsdr_iq_stream_init(&g_ctx.stream, g_ctx.dev_fd, &g_ctx.dma_map);

	signal(SIGPIPE, SIG_IGN);
	signal(SIGINT, on_sigint);
	signal(SIGTERM, on_sigint);

	srv_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (srv_fd < 0) {
		perror("socket");
		close(g_ctx.dev_fd);
		return 1;
	}
	(void)setsockopt(srv_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons((uint16_t)g_ctx.port);
	if (bind(srv_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
		perror("bind");
		close(srv_fd);
		close(g_ctx.dev_fd);
		return 1;
	}
	if (listen(srv_fd, 16) != 0) {
		perror("listen");
		close(srv_fd);
		close(g_ctx.dev_fd);
		return 1;
	}

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	pthread_attr_setstacksize(&attr, 256U * 1024U);

	printf("wmsdr serve listening on http://0.0.0.0:%d (web=%s, mmap=%s)\n",
	       g_ctx.port, g_ctx.web_root, g_ctx.dma_map.base ? "on" : "off");
	fflush(stdout);

	while (!g_ctx.stop) {
		struct pollfd pfd = { .fd = srv_fd, .events = POLLIN, .revents = 0 };
		int pr = poll(&pfd, 1, 500);
		int cfd, *pfd_arg;
		pthread_t tid;

		if (pr < 0) {
			if (errno == EINTR)
				continue;
			break;
		}
		if (pr == 0)
			continue;

		cfd = accept(srv_fd, NULL, NULL);
		if (cfd < 0) {
			if (errno == EINTR)
				continue;
			break;
		}
		pfd_arg = malloc(sizeof(int));
		if (!pfd_arg) {
			close(cfd);
			continue;
		}
		*pfd_arg = cfd;
		if (pthread_create(&tid, &attr, client_thread, pfd_arg) != 0) {
			free(pfd_arg);
			close(cfd);
		}
	}

	pthread_attr_destroy(&attr);
	close(srv_fd);
	pthread_mutex_lock(&g_ctx.iq_lock);
	httpd_stop_knet_locked(&g_ctx);
	wmsdr_iq_stream_stop(&g_ctx.stream);
	pthread_mutex_unlock(&g_ctx.iq_lock);
	wmsdr_mmap_close(&g_ctx.dma_map);
	close(g_ctx.dev_fd);
	pthread_mutex_destroy(&g_ctx.iq_lock);
	return 0;
}
