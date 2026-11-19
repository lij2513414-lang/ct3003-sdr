// SPDX-License-Identifier: GPL-2.0-only
/*
 * wm_sdr_net.c - Zero-copy asynchronous multi-stream IQ network TX offload.
 *
 * Architecture:
 *   1. Producer kthread calls wmsdr_wfdma_icap_to_va() directly into pre-allocated
 *      compound order-3/4 payload pages (32/64 KiB) paired with 4 KiB header pages
 *      holding [RFC6455 binary frame hdr + 32B WSIQ hdr].
 *   2. Captured slots enter a lock-protected shared ready ring and kick N attached
 *      TCP sockets running non-blocking work-stealing kernel_sendpage() workers on
 *      an unbound high-priority workqueue.
 *   3. Fast sockets pull more frames while slower/lossy sockets yield cleanly on
 *      -EAGAIN and resume via sk_write_space callbacks (with 20 ms watchdog).
 */
#include <linux/atomic.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/in.h>
#include <linux/in6.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/net.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/socket.h>
#include <linux/spinlock.h>
#include <linux/tcp.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <net/sock.h>
#include <net/tcp.h>
#include <asm/unaligned.h>

#include "wm_sdr.h"

#define WMSDR_WSIQ_MAGIC		0x51495357U
#define WMSDR_NET_MAX_SLOTS		64U
#define WMSDR_NET_MIN_SLOTS		8U
#define WMSDR_NET_DEF_SNDBUF_KB		192U
#define WMSDR_NET_TX_BATCH		24U
#define WMSDR_NET_RETRY_MS		20U
#define WMSDR_NET_DRAIN_MS		60U

struct wmsdr_net_slot {
	struct page *hdr_page;
	struct page *pay_page;
	void *hdr_va;
	void *pay_va;
	u32 hdr_len;
	atomic_t in_use;
};

struct wmsdr_net_ctx;

struct wmsdr_sock_lane {
	bool active;
	struct socket *sock;
	struct wmsdr_net_ctx *net;
	struct work_struct tx_work;
	struct delayed_work retry_work;
	void (*orig_write_space)(struct sock *sk);
	void (*orig_state_change)(struct sock *sk);
	atomic_t draining;
	atomic_t dead;
	atomic_t cur_busy;
	int cur_slot;
	u32 cur_stage;	/* 0 = hdr_page, 1 = pay_page */
	u32 cur_off;
	atomic64_t frames_sent;
	atomic64_t bytes_sent;
	atomic64_t send_eagain;
	atomic64_t send_errors;
};

struct wmsdr_net_ctx {
	struct wmsdr_dev *w;
	struct file *owner_file;
	struct wmsdr_net_cfg cfg;
	u32 aligned_words;
	u32 pay_bytes;
	u32 dma_bytes;
	u32 pay_order;
	u32 n_slots;
	struct wmsdr_net_slot slots[WMSDR_NET_MAX_SLOTS];
	spinlock_t ring_lock;
	u32 ready_ring[WMSDR_NET_MAX_SLOTS];
	u32 ring_head;
	u32 ring_tail;
	u32 ring_count;
	u32 scan_idx;
	atomic_t stop_cap;
	atomic_t next_seq;
	atomic64_t frames_captured;
	atomic64_t frames_dropped;
	atomic64_t closed_frames_sent;
	atomic64_t closed_bytes_sent;
	atomic64_t closed_send_eagain;
	atomic64_t closed_send_errors;
	struct workqueue_struct *tx_wq;
	struct task_struct *cap_thread;
	wait_queue_head_t slot_wq;
	wait_queue_head_t lane_wq;
	struct wmsdr_sock_lane lanes[WMSDR_NET_MAX_SOCKS];
};

static u32 wmsdr_net_encode_hdr(const struct wmsdr_net_cfg *cfg, u32 seq,
				u32 cr0, u32 cr9, u8 *dst)
{
	u32 iq_bytes = cfg->len_words * 4U;
	u32 wsiq_len = 32U + iq_bytes;
	u32 ws_hlen = 0;
	u32 wflags = (cfg->direct ? 1U : 0U) |
		     ((cfg->bw & 0xFFU) << 8) |
		     ((cfg->band & 1U) << 16);

	if (!(cfg->flags & WMSDR_NET_F_RAW_WSIQ)) {
		dst[0] = 0x82U;	/* FIN + binary opcode */
		if (wsiq_len < 126U) {
			dst[1] = (u8)wsiq_len;
			ws_hlen = 2U;
		} else if (wsiq_len < 65536U) {
			dst[1] = 126U;
			put_unaligned_be16((u16)wsiq_len, dst + 2);
			ws_hlen = 4U;
		} else {
			dst[1] = 127U;
			put_unaligned_be64((u64)wsiq_len, dst + 2);
			ws_hlen = 10U;
		}
	}

	dst += ws_hlen;
	put_unaligned_le32(WMSDR_WSIQ_MAGIC, dst + 0);
	put_unaligned_le32(1U, dst + 4);
	put_unaligned_le32(seq, dst + 8);
	put_unaligned_le32(cfg->node, dst + 12);
	put_unaligned_le32(cfg->len_words, dst + 16);
	put_unaligned_le32(cr0, dst + 20);
	put_unaligned_le32(cr9, dst + 24);
	put_unaligned_le32(wflags, dst + 28);
	return ws_hlen + 32U;
}

static int wmsdr_net_find_free_slot(struct wmsdr_net_ctx *net)
{
	u32 i;

	for (i = 0; i < net->n_slots; i++) {
		u32 idx = (net->scan_idx + i) % net->n_slots;
		struct wmsdr_net_slot *s = &net->slots[idx];

		if (atomic_read(&s->in_use) == 0 &&
		    page_count(s->hdr_page) == 1 &&
		    page_count(s->pay_page) == 1) {
			net->scan_idx = (idx + 1U) % net->n_slots;
			return (int)idx;
		}
	}
	return -1;
}

static void wmsdr_net_put_slot(struct wmsdr_net_ctx *net, int slot_idx)
{
	if (slot_idx < 0 || (u32)slot_idx >= net->n_slots)
		return;
	atomic_set(&net->slots[slot_idx].in_use, 0);
	smp_mb__after_atomic();
	wake_up(&net->slot_wq);
}

static int wmsdr_net_pop_ready(struct wmsdr_net_ctx *net)
{
	unsigned long flags;
	int slot_idx = -1;

	spin_lock_irqsave(&net->ring_lock, flags);
	if (net->ring_count > 0U) {
		slot_idx = (int)net->ready_ring[net->ring_head];
		net->ring_head = (net->ring_head + 1U) % net->n_slots;
		net->ring_count--;
	}
	spin_unlock_irqrestore(&net->ring_lock, flags);
	return slot_idx;
}

static void wmsdr_net_kick_lanes(struct wmsdr_net_ctx *net)
{
	u32 i;

	for (i = 0; i < WMSDR_NET_MAX_SOCKS; i++) {
		struct wmsdr_sock_lane *lane = &net->lanes[i];

		if (READ_ONCE(lane->active) &&
		    !atomic_read(&lane->draining) &&
		    !atomic_read(&lane->dead))
			queue_work(net->tx_wq, &lane->tx_work);
	}
}

static int wmsdr_send_page_range(struct socket *sock, struct page *base_page,
				 u32 *off_io, u32 total_len,
				 atomic64_t *bytes_cnt, bool more)
{
	while (*off_io < total_len) {
		u32 off = *off_io;
		struct page *p = base_page + (off >> PAGE_SHIFT);
		u32 p_off = off & (PAGE_SIZE - 1U);
		u32 rem = total_len - off;
		u32 chunk = min_t(u32, rem, (u32)PAGE_SIZE - p_off);
		int flags = MSG_DONTWAIT | MSG_NOSIGNAL;
		int rc;

		if (more || off + chunk < total_len)
			flags |= MSG_MORE | MSG_SENDPAGE_NOTLAST;

		rc = kernel_sendpage(sock, p, p_off, chunk, flags);
		if (rc > 0) {
			*off_io = off + (u32)rc;
			atomic64_add((s64)rc, bytes_cnt);
			continue;
		}
		if (rc == -EAGAIN)
			return -EAGAIN;
		return rc ? rc : -EPIPE;
	}
	return 0;
}

static void wmsdr_lane_tx_work(struct work_struct *work)
{
	struct wmsdr_sock_lane *lane =
		container_of(work, struct wmsdr_sock_lane, tx_work);
	struct wmsdr_net_ctx *net = lane->net;
	struct socket *sock = lane->sock;
	struct sock *sk = sock ? sock->sk : NULL;
	u32 batch = 0;

	if (!sock || !sk || atomic_read(&lane->dead)) {
		if (lane->cur_slot >= 0) {
			wmsdr_net_put_slot(net, lane->cur_slot);
			lane->cur_slot = -1;
			atomic_set(&lane->cur_busy, 0);
		}
		goto out_wake_drain;
	}

	while (batch < WMSDR_NET_TX_BATCH && !atomic_read(&lane->dead)) {
		struct wmsdr_net_slot *slot;
		int rc = 0;

		if (lane->cur_slot < 0) {
			int next;

			if (atomic_read(&lane->draining))
				break;
			next = wmsdr_net_pop_ready(net);
			if (next < 0)
				break;
			lane->cur_slot = next;
			lane->cur_stage = 0U;
			lane->cur_off = 0U;
			atomic_set(&lane->cur_busy, 1);
		}

		slot = &net->slots[lane->cur_slot];
		if (lane->cur_stage == 0U) {
			rc = wmsdr_send_page_range(sock, slot->hdr_page,
						   &lane->cur_off, slot->hdr_len,
						   &lane->bytes_sent, true);
			if (rc) {
				if (rc == -EAGAIN)
					goto handle_eagain;
				goto handle_err;
			}
			lane->cur_stage = 1U;
			lane->cur_off = 0U;
		}

		rc = wmsdr_send_page_range(sock, slot->pay_page,
					   &lane->cur_off, net->pay_bytes,
					   &lane->bytes_sent, false);
		if (rc) {
			if (rc == -EAGAIN)
				goto handle_eagain;
			goto handle_err;
		}

		atomic64_inc(&lane->frames_sent);
		wmsdr_net_put_slot(net, lane->cur_slot);
		lane->cur_slot = -1;
		lane->cur_stage = 0U;
		lane->cur_off = 0U;
		atomic_set(&lane->cur_busy, 0);
		batch++;

		if (atomic_read(&lane->draining))
			break;
		continue;

handle_eagain:
		atomic64_inc(&lane->send_eagain);
		set_bit(SOCK_NOSPACE, &sock->flags);
		smp_mb__after_atomic();
		if (sk_stream_memory_free(sk) && !atomic_read(&lane->dead))
			continue;
		if (!atomic_read(&lane->dead))
			mod_delayed_work(net->tx_wq, &lane->retry_work,
					 msecs_to_jiffies(WMSDR_NET_RETRY_MS));
		return;

handle_err:
		atomic64_inc(&lane->send_errors);
		atomic_set(&lane->dead, 1);
		wmsdr_net_put_slot(net, lane->cur_slot);
		lane->cur_slot = -1;
		atomic_set(&lane->cur_busy, 0);
		break;
	}

	if (batch >= WMSDR_NET_TX_BATCH && !atomic_read(&lane->dead) &&
	    !atomic_read(&lane->draining)) {
		unsigned long flags;
		bool more = false;

		spin_lock_irqsave(&net->ring_lock, flags);
		more = (net->ring_count > 0U);
		spin_unlock_irqrestore(&net->ring_lock, flags);
		if (more)
			queue_work(net->tx_wq, &lane->tx_work);
	}

out_wake_drain:
	if (atomic_read(&lane->draining) && !atomic_read(&lane->cur_busy))
		wake_up(&net->lane_wq);
	else if (atomic_read(&lane->dead))
		wake_up(&net->lane_wq);
}

static void wmsdr_lane_retry_work(struct work_struct *work)
{
	struct wmsdr_sock_lane *lane =
		container_of(to_delayed_work(work), struct wmsdr_sock_lane, retry_work);

	if (READ_ONCE(lane->active) && !atomic_read(&lane->dead))
		queue_work(lane->net->tx_wq, &lane->tx_work);
}

static void wmsdr_sk_write_space(struct sock *sk)
{
	struct wmsdr_sock_lane *lane;
	void (*orig)(struct sock *sk) = NULL;

	read_lock_bh(&sk->sk_callback_lock);
	lane = sk->sk_user_data;
	if (lane) {
		orig = lane->orig_write_space;
		if (READ_ONCE(lane->active) && !atomic_read(&lane->dead) &&
		    sk_stream_memory_free(sk))
			queue_work(lane->net->tx_wq, &lane->tx_work);
	}
	read_unlock_bh(&sk->sk_callback_lock);

	if (orig)
		orig(sk);
}

static void wmsdr_sk_state_change(struct sock *sk)
{
	struct wmsdr_sock_lane *lane;
	void (*orig)(struct sock *sk) = NULL;

	read_lock_bh(&sk->sk_callback_lock);
	lane = sk->sk_user_data;
	if (lane) {
		orig = lane->orig_state_change;
		if (sk->sk_state != TCP_ESTABLISHED &&
		    sk->sk_state != TCP_CLOSE_WAIT)
			atomic_set(&lane->dead, 1);
		if (READ_ONCE(lane->active))
			queue_work(lane->net->tx_wq, &lane->tx_work);
	}
	read_unlock_bh(&sk->sk_callback_lock);

	if (orig)
		orig(sk);
}

static int wmsdr_net_cap_fn(void *arg)
{
	struct wmsdr_net_ctx *net = arg;
	struct wmsdr_dev *w = net->w;

	while (!kthread_should_stop() && !atomic_read(&net->stop_cap)) {
		int slot_idx;
		struct wmsdr_net_slot *slot;
		u32 cr0 = 0, cr9 = 0, seq;
		unsigned long flags;
		int rc;

		slot_idx = wmsdr_net_find_free_slot(net);
		if (slot_idx < 0) {
			wait_event_interruptible_timeout(
				net->slot_wq,
				kthread_should_stop() ||
				atomic_read(&net->stop_cap) ||
				wmsdr_net_find_free_slot(net) >= 0,
				usecs_to_jiffies(1000));
			continue;
		}

		slot = &net->slots[slot_idx];
		atomic_set(&slot->in_use, 1);

		rc = wmsdr_wfdma_icap_to_va(w, net->cfg.len_words,
					    net->cfg.mode, 50U, slot->pay_va,
					    &cr0, NULL, &cr9, NULL);
		if (rc) {
			atomic_set(&slot->in_use, 0);
			if (rc != -EINTR && !atomic_read(&net->stop_cap))
				pr_warn_ratelimited("wmsdr_net: capture err=%d cr0=%#x cr9=%#x\n",
						    rc, cr0, cr9);
			usleep_range(1000, 2000);
			continue;
		}

		seq = (u32)atomic_inc_return(&net->next_seq) - 1U;
		slot->hdr_len = wmsdr_net_encode_hdr(&net->cfg, seq, cr0, cr9,
						     slot->hdr_va);
		smp_wmb();

		spin_lock_irqsave(&net->ring_lock, flags);
		if (net->ring_count >= net->n_slots) {
			spin_unlock_irqrestore(&net->ring_lock, flags);
			atomic64_inc(&net->frames_dropped);
			atomic_set(&slot->in_use, 0);
			continue;
		}
		net->ready_ring[net->ring_tail] = (u32)slot_idx;
		net->ring_tail = (net->ring_tail + 1U) % net->n_slots;
		net->ring_count++;
		spin_unlock_irqrestore(&net->ring_lock, flags);

		atomic64_inc(&net->frames_captured);
		wmsdr_net_kick_lanes(net);
	}
	return 0;
}

static void wmsdr_net_teardown_lane(struct wmsdr_sock_lane *lane)
{
	struct wmsdr_net_ctx *net = lane->net;
	struct socket *sock = lane->sock;
	struct sock *sk = sock ? sock->sk : NULL;

	if (!lane->active || !sock)
		return;

	atomic_set(&lane->draining, 1);
	smp_mb__after_atomic();
	queue_work(net->tx_wq, &lane->tx_work);
	flush_work(&lane->tx_work);

	if (atomic_read(&lane->cur_busy) && !atomic_read(&lane->dead))
		wait_event_timeout(net->lane_wq,
				   !atomic_read(&lane->cur_busy) ||
				   atomic_read(&lane->dead),
				   msecs_to_jiffies(WMSDR_NET_DRAIN_MS));

	WRITE_ONCE(lane->active, false);
	if (sk) {
		write_lock_bh(&sk->sk_callback_lock);
		if (sk->sk_user_data == lane) {
			sk->sk_user_data = NULL;
			sk->sk_write_space = lane->orig_write_space;
			sk->sk_state_change = lane->orig_state_change;
		}
		write_unlock_bh(&sk->sk_callback_lock);
	}

	cancel_delayed_work_sync(&lane->retry_work);
	cancel_work_sync(&lane->tx_work);

	if (lane->cur_slot >= 0) {
		wmsdr_net_put_slot(net, lane->cur_slot);
		lane->cur_slot = -1;
		atomic_set(&lane->cur_busy, 0);
	}

	atomic64_add(atomic64_read(&lane->frames_sent), &net->closed_frames_sent);
	atomic64_add(atomic64_read(&lane->bytes_sent), &net->closed_bytes_sent);
	atomic64_add(atomic64_read(&lane->send_eagain), &net->closed_send_eagain);
	atomic64_add(atomic64_read(&lane->send_errors), &net->closed_send_errors);

	lane->sock = NULL;
	lane->orig_write_space = NULL;
	lane->orig_state_change = NULL;
	sockfd_put(sock);
}

static void wmsdr_net_fill_stat_locked(struct wmsdr_net_ctx *net,
				       struct wmsdr_net_stat *st)
{
	u32 i, depth;
	unsigned long flags;
	s64 fs = atomic64_read(&net->closed_frames_sent);
	s64 bs = atomic64_read(&net->closed_bytes_sent);
	s64 ea = atomic64_read(&net->closed_send_eagain);
	s64 er = atomic64_read(&net->closed_send_errors);

	memset(st, 0, sizeof(*st));
	st->active = !atomic_read(&net->stop_cap);
	st->next_seq = (u32)atomic_read(&net->next_seq);

	spin_lock_irqsave(&net->ring_lock, flags);
	depth = net->ring_count;
	spin_unlock_irqrestore(&net->ring_lock, flags);
	st->ring_depth = depth;

	for (i = 0; i < WMSDR_NET_MAX_SOCKS; i++) {
		struct wmsdr_sock_lane *lane = &net->lanes[i];

		if (!lane->active)
			continue;
		if (!atomic_read(&lane->dead))
			st->n_socks++;
		fs += atomic64_read(&lane->frames_sent);
		bs += atomic64_read(&lane->bytes_sent);
		ea += atomic64_read(&lane->send_eagain);
		er += atomic64_read(&lane->send_errors);
	}

	st->frames_captured = (u64)atomic64_read(&net->frames_captured);
	st->frames_sent = (u64)fs;
	st->frames_dropped = (u64)atomic64_read(&net->frames_dropped);
	st->bytes_sent = (u64)bs;
	st->send_eagain = (u64)ea;
	st->send_errors = (u64)er;
}

static void wmsdr_net_free_slots(struct wmsdr_net_ctx *net)
{
	u32 i;

	for (i = 0; i < net->n_slots; i++) {
		if (net->slots[i].hdr_page) {
			__free_pages(net->slots[i].hdr_page, 0);
			net->slots[i].hdr_page = NULL;
			net->slots[i].hdr_va = NULL;
		}
		if (net->slots[i].pay_page) {
			__free_pages(net->slots[i].pay_page, net->pay_order);
			net->slots[i].pay_page = NULL;
			net->slots[i].pay_va = NULL;
		}
	}
	net->n_slots = 0;
}

static int wmsdr_net_alloc_slots(struct wmsdr_net_ctx *net, u32 req_slots)
{
	gfp_t pay_gfp = GFP_KERNEL | __GFP_ZERO | __GFP_COMP | __GFP_NOWARN;
	u32 target = req_slots;
	u32 i;

	if (!target)
		target = (net->dma_bytes <= 32768U) ? 64U : 48U;
	target = clamp_t(u32, target, WMSDR_NET_MIN_SLOTS, WMSDR_NET_MAX_SLOTS);

	for (i = 0; i < target; i++) {
		struct page *hp = alloc_page(GFP_KERNEL | __GFP_ZERO | __GFP_NOWARN);
		struct page *pp = hp ? alloc_pages(pay_gfp, net->pay_order) : NULL;

		if (!hp || !pp) {
			if (hp)
				__free_page(hp);
			break;
		}
		net->slots[i].hdr_page = hp;
		net->slots[i].pay_page = pp;
		net->slots[i].hdr_va = page_address(hp);
		net->slots[i].pay_va = page_address(pp);
		atomic_set(&net->slots[i].in_use, 0);
		net->n_slots++;
	}

	if (net->n_slots < WMSDR_NET_MIN_SLOTS) {
		wmsdr_net_free_slots(net);
		return -ENOMEM;
	}
	return 0;
}

static int wmsdr_net_stop_locked(struct wmsdr_dev *w, struct wmsdr_net_stat *stat_out)
{
	struct wmsdr_net_ctx *net = w->net;
	u32 i;

	if (!net) {
		if (stat_out)
			memset(stat_out, 0, sizeof(*stat_out));
		return 0;
	}

	atomic_set(&net->stop_cap, 1);
	wake_up_all(&net->slot_wq);
	if (net->cap_thread) {
		kthread_stop(net->cap_thread);
		net->cap_thread = NULL;
	}

	for (i = 0; i < WMSDR_NET_MAX_SOCKS; i++)
		wmsdr_net_teardown_lane(&net->lanes[i]);

	/* Drain any remaining unsent slots cleanly before collecting stats. */
	for (;;) {
		int idx = wmsdr_net_pop_ready(net);
		if (idx < 0)
			break;
		atomic_set(&net->slots[idx].in_use, 0);
	}

	if (stat_out)
		wmsdr_net_fill_stat_locked(net, stat_out);

	if (net->tx_wq) {
		destroy_workqueue(net->tx_wq);
		net->tx_wq = NULL;
	}
	wmsdr_net_free_slots(net);
	w->net = NULL;
	kfree(net);
	return 0;
}

static int wmsdr_net_start_locked(struct wmsdr_dev *w, struct file *file,
				  struct wmsdr_net_cfg *cfg)
{
	struct wmsdr_net_ctx *net;
	u32 max_words, aligned_words, dma_bytes;
	int rc;

	if (!w->dma_ready || !w->rx_pages)
		return -EOPNOTSUPP;
	if (w->net)
		return -EBUSY;
	if (cfg->mode > 1U || cfg->len_words < 16U)
		return -EINVAL;

	max_words = (cfg->mode == 1U) ? WMIQ_SPECTRUM_MAX_WORDS : 65536U;
	if (cfg->len_words > max_words)
		return -EINVAL;
	aligned_words = ALIGN(cfg->len_words, 4U);
	if (aligned_words > max_words)
		return -EINVAL;
	dma_bytes = aligned_words * 4U;

	net = kzalloc(sizeof(*net), GFP_KERNEL);
	if (!net)
		return -ENOMEM;

	net->w = w;
	net->owner_file = file;
	net->cfg = *cfg;
	net->aligned_words = aligned_words;
	net->pay_bytes = cfg->len_words * 4U;
	net->dma_bytes = dma_bytes;
	net->pay_order = get_order(dma_bytes);

	spin_lock_init(&net->ring_lock);
	init_waitqueue_head(&net->slot_wq);
	init_waitqueue_head(&net->lane_wq);
	atomic_set(&net->next_seq, (int)cfg->seq_start);

	rc = wmsdr_net_alloc_slots(net, cfg->slot_count);
	if (rc) {
		kfree(net);
		return rc;
	}

	net->tx_wq = alloc_workqueue("wmsdr_tx", WQ_UNBOUND | WQ_HIGHPRI,
				     WMSDR_NET_MAX_SOCKS);
	if (!net->tx_wq) {
		wmsdr_net_free_slots(net);
		kfree(net);
		return -ENOMEM;
	}

	w->net = net;
	net->cap_thread = kthread_run(wmsdr_net_cap_fn, net, "wmsdr_cap");
	if (IS_ERR(net->cap_thread)) {
		rc = PTR_ERR(net->cap_thread);
		net->cap_thread = NULL;
		wmsdr_net_stop_locked(w, NULL);
		return rc;
	}

	cfg->slot_count = net->n_slots;
	return 0;
}

static int wmsdr_net_add_sock_locked(struct wmsdr_dev *w,
				     const struct wmsdr_net_sock *req)
{
	struct wmsdr_net_ctx *net = w->net;
	struct socket *sock;
	struct sock *sk;
	struct wmsdr_sock_lane *lane = NULL;
	int err = 0, one = 1, sndbuf;
	u32 i, kb;

	if (!net)
		return -EINVAL;

	sock = sockfd_lookup(req->fd, &err);
	if (!sock)
		return err ? err : -EBADF;

	sk = sock->sk;
	if (!sk || sock->type != SOCK_STREAM ||
	    (sk->sk_family != AF_INET && sk->sk_family != AF_INET6)) {
		sockfd_put(sock);
		return -EINVAL;
	}
	if (sk->sk_state != TCP_ESTABLISHED) {
		sockfd_put(sock);
		return -ENOTCONN;
	}

	for (i = 0; i < WMSDR_NET_MAX_SOCKS; i++) {
		if (!net->lanes[i].active) {
			if (!lane)
				lane = &net->lanes[i];
		} else if (net->lanes[i].sock == sock) {
			sockfd_put(sock);
			return -EBUSY;
		}
	}
	if (!lane) {
		sockfd_put(sock);
		return -ENOSPC;
	}

	kb = net->cfg.sndbuf_kb ? net->cfg.sndbuf_kb : WMSDR_NET_DEF_SNDBUF_KB;
	kb = clamp_t(u32, kb, 32U, 512U);
	sndbuf = (int)(kb * 1024U);
	(void)kernel_setsockopt(sock, SOL_SOCKET, SO_SNDBUF,
				(char *)&sndbuf, sizeof(sndbuf));
	(void)kernel_setsockopt(sock, IPPROTO_TCP, TCP_NODELAY,
				(char *)&one, sizeof(one));

	memset(lane, 0, sizeof(*lane));
	lane->sock = sock;
	lane->net = net;
	lane->cur_slot = -1;
	INIT_WORK(&lane->tx_work, wmsdr_lane_tx_work);
	INIT_DELAYED_WORK(&lane->retry_work, wmsdr_lane_retry_work);

	write_lock_bh(&sk->sk_callback_lock);
	if (sk->sk_user_data) {
		write_unlock_bh(&sk->sk_callback_lock);
		sockfd_put(sock);
		return -EBUSY;
	}
	lane->orig_write_space = sk->sk_write_space;
	lane->orig_state_change = sk->sk_state_change;
	sk->sk_user_data = lane;
	sk->sk_write_space = wmsdr_sk_write_space;
	sk->sk_state_change = wmsdr_sk_state_change;
	write_unlock_bh(&sk->sk_callback_lock);

	WRITE_ONCE(lane->active, true);
	queue_work(net->tx_wq, &lane->tx_work);
	return 0;
}

static int wmsdr_net_del_sock_locked(struct wmsdr_dev *w, s32 fd)
{
	struct wmsdr_net_ctx *net = w->net;
	struct socket *sock;
	struct wmsdr_sock_lane *target = NULL;
	int err = 0;
	u32 i;

	if (!net)
		return 0;

	sock = sockfd_lookup(fd, &err);
	if (!sock)
		return err ? err : -EBADF;

	for (i = 0; i < WMSDR_NET_MAX_SOCKS; i++) {
		if (net->lanes[i].active && net->lanes[i].sock == sock) {
			target = &net->lanes[i];
			break;
		}
	}
	sockfd_put(sock);

	if (!target)
		return -ENOENT;

	wmsdr_net_teardown_lane(target);
	return 0;
}

long wmsdr_net_ioctl(struct wmsdr_dev *w, struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	int rc;

	switch (cmd) {
	case WMSDR_IOC_NET_START: {
		struct wmsdr_net_cfg cfg;

		if (copy_from_user(&cfg, (void __user *)arg, sizeof(cfg)))
			return -EFAULT;
		mutex_lock(&w->net_lock);
		rc = wmsdr_net_start_locked(w, file, &cfg);
		mutex_unlock(&w->net_lock);
		if (!rc && copy_to_user((void __user *)arg, &cfg, sizeof(cfg)))
			return -EFAULT;
		return rc;
	}
	case WMSDR_IOC_NET_ADD_SOCK: {
		struct wmsdr_net_sock req;

		if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
			return -EFAULT;
		mutex_lock(&w->net_lock);
		rc = wmsdr_net_add_sock_locked(w, &req);
		mutex_unlock(&w->net_lock);
		return rc;
	}
	case WMSDR_IOC_NET_DEL_SOCK: {
		s32 fd = -1;

		if (copy_from_user(&fd, (void __user *)arg, sizeof(fd)))
			return -EFAULT;
		mutex_lock(&w->net_lock);
		rc = wmsdr_net_del_sock_locked(w, fd);
		mutex_unlock(&w->net_lock);
		return rc;
	}
	case WMSDR_IOC_NET_STOP: {
		struct wmsdr_net_stat st;

		mutex_lock(&w->net_lock);
		rc = wmsdr_net_stop_locked(w, &st);
		mutex_unlock(&w->net_lock);
		if (!rc && copy_to_user((void __user *)arg, &st, sizeof(st)))
			return -EFAULT;
		return rc;
	}
	case WMSDR_IOC_NET_STAT: {
		struct wmsdr_net_stat st;

		mutex_lock(&w->net_lock);
		if (w->net)
			wmsdr_net_fill_stat_locked(w->net, &st);
		else
			memset(&st, 0, sizeof(st));
		mutex_unlock(&w->net_lock);
		if (copy_to_user((void __user *)arg, &st, sizeof(st)))
			return -EFAULT;
		return 0;
	}
	default:
		return -ENOTTY;
	}
}

void wmsdr_net_release_file(struct wmsdr_dev *w, struct file *file)
{
	mutex_lock(&w->net_lock);
	if (w->net && w->net->owner_file == file)
		wmsdr_net_stop_locked(w, NULL);
	mutex_unlock(&w->net_lock);
}

int wmsdr_net_init(struct wmsdr_dev *w)
{
	mutex_init(&w->net_lock);
	w->net = NULL;
	return 0;
}

void wmsdr_net_exit(struct wmsdr_dev *w)
{
	mutex_lock(&w->net_lock);
	wmsdr_net_stop_locked(w, NULL);
	mutex_unlock(&w->net_lock);
}
