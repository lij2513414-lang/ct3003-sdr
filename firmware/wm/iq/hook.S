/* SPDX-License-Identifier: MIT */
/*
 * Delay-hook trampoline and ICAP IQ capture state machine for CT3003 WM.
 *
 * Runs from extended ALPS patch SRAM at 0x00903000.
 * The 10-byte stub at 0x0090044E replaces the stock delay_us prologue:
 *   push25 r10,0 ; slti r15,r0,0x20 ; mov55 r6,r0 ; beqzs8 0x0090047E
 * with sethi/ori/jr5 to wm_hook.
 *
 * Patch SRAM layout (0x00903800 - 0x009038BF):
 *   0x00903800: u32 raw_tick      (incremented every delay_us call)
 *   0x00903804: u32 in_pump       (re-entrancy lock)
 *   0x00903808: u32 arm_mode      (1 = ARM_API, 4 = ARM_DIRECT, 8 = ARM_KEEP)
 *   0x00903840: struct wm_iq_page (64-byte control/status mailbox)
 *     +0x00 magic        0x51494D57 ("WMIQ")
 *     +0x04 version      1
 *     +0x08 cmd          0=idle, 1=arm_api, 2=stop, 3=reset, 4=arm_direct,
 *                        5=wm_read32, 6=wm_write32, 7=wm_call, 8=arm_keep
 *     +0x0C status       0=idle, 1=armed, 2=done, 3=error
 *     +0x10 node         <39 table index (8 -> 0x10605000) or full 32-bit key
 *     +0x14 length       capture word count (default 0x2000)
 *     +0x18 data_addr    capture buffer WM address (0xE0200000)
 *     +0x1C sample_count captured word count at completion
 *     +0x20 tick         pump tick counter
 *     +0x24 error        last error / function return code
 *     +0x28 cr9          live ICAP write pointer (0x800230B4)
 *     +0x2C cr2          ICAP start address (0x80023098)
 *     +0x30 cr0          ICAP control register (0x80023090)
 *     +0x34 bw           capture bandwidth (default 0)
 *     +0x38 arg0         address/fn for wm_read32/wm_write32/wm_call
 *     +0x3C arg1         value/return for wm_read32/wm_write32/wm_call
 *   0x00903880: u8 req[64]        (IcapConfigRequest 0xE006CC50 buffer)
 */
	.text
	.globl wm_hook
	.type wm_hook,@function
wm_hook:
	push25	r10, 0			/* displaced prologue: frame expected by delay body */
	addi	sp, sp, -32
	swi	r0, [sp+0]
	swi	r1, [sp+4]
	swi	r2, [sp+8]
	swi	r3, [sp+12]
	swi	r4, [sp+16]
	swi	r5, [sp+20]
	swi	lp, [sp+24]
	jal	wm_pump
	lwi	r0, [sp+0]
	lwi	r1, [sp+4]
	lwi	r2, [sp+8]
	lwi	r3, [sp+12]
	lwi	r4, [sp+16]
	lwi	r5, [sp+20]
	lwi	lp, [sp+24]
	addi	sp, sp, 32
	slti	r15, r0, 0x20
	mov55	r6, r0
	beqz	r15, 1f
	sethi	r15, 0x00900
	ori	r15, r15, 0x0458
	jr5	r15
1:
	sethi	r15, 0x00900
	ori	r15, r15, 0x047E
	jr5	r15
	.size wm_hook, .-wm_hook

	.globl wm_pump
	.type wm_pump,@function
wm_pump:
	smw.adm	r6, [sp], r10, 0xe	/* save r6..r10, fp, gp, lp */
	sethi	r6, 0x00900
	ori	r6, r6, 0x3800		/* r6 = 0x00903800 */

	/* 1. Always increment raw_tick at 0x00903800 */
	lwi	r3, [r6+0x00]
	addi	r3, r3, 1
	swi	r3, [r6+0x00]

	/* 2. Initialize wm_iq_page at 0x00903840 if magic != "WMIQ" */
	sethi	r4, 0x51494
	ori	r4, r4, 0x0D57		/* 0x51494D57 = "WMIQ" */
	lwi	r2, [r6+0x40]
	beq	r2, r4, .Lmagic_ok
	swi	r4, [r6+0x40]		/* magic */
	movi	r2, 1
	swi	r2, [r6+0x44]		/* version = 1 */
	movi	r2, 0
	swi	r2, [r6+0x48]		/* cmd = 0 */
	swi	r2, [r6+0x4C]		/* status = 0 */
	movi	r2, 8
	swi	r2, [r6+0x50]		/* node = 8 (0x10605000) */
	movi	r2, 0x2000
	swi	r2, [r6+0x54]		/* length = 8192 words */
	sethi	r2, 0xE0200
	swi	r2, [r6+0x58]		/* data_addr = 0xE0200000 */
	movi	r2, 0
	swi	r2, [r6+0x5C]		/* sample_count = 0 */
	swi	r2, [r6+0x64]		/* error = 0 */
.Lmagic_ok:
	swi	r3, [r6+0x60]		/* page.tick = raw_tick */

	/* 3. Re-entrancy guard (in_pump at 0x00903804) */
	lwi	r2, [r6+0x04]
	bnez	r2, .Lout_nolock
	movi	r2, 1
	swi	r2, [r6+0x04]

	/* Set WM RAM GP (0x02216800) for any C helper calls */
	sethi	gp, 0x02216
	ori	gp, gp, 0x0800

	lwi	r7, [r6+0x48]		/* r7 = cmd */
	lwi	r8, [r6+0x4C]		/* r8 = status */

	/* cmd == 3: RESET */
	movi	r2, 3
	bne	r7, r2, .Lnot_reset
	movi	r2, 0
	swi	r2, [r6+0x4C]		/* status = 0 */
	swi	r2, [r6+0x64]		/* error = 0 */
	swi	r2, [r6+0x48]		/* cmd = 0 */
	j	.Lunlock

.Lnot_reset:
	/* cmd == 5: WM_READ32 */
	movi	r2, 5
	bne	r7, r2, .Lnot_read32
	lwi	r2, [r6+0x78]		/* arg0 = addr */
	lwi	r3, [r2+0]
	swi	r3, [r6+0x7C]		/* arg1 = *(u32 *)arg0 */
	movi	r2, 2
	swi	r2, [r6+0x4C]		/* status = DONE */
	movi	r2, 0
	swi	r2, [r6+0x48]		/* cmd = 0 */
	j	.Lunlock

.Lnot_read32:
	/* cmd == 6: WM_WRITE32 */
	movi	r2, 6
	bne	r7, r2, .Lnot_write32
	lwi	r2, [r6+0x78]		/* arg0 = addr */
	lwi	r3, [r6+0x7C]		/* arg1 = val */
	swi	r3, [r2+0]
	movi	r2, 2
	swi	r2, [r6+0x4C]		/* status = DONE */
	movi	r2, 0
	swi	r2, [r6+0x48]		/* cmd = 0 */
	j	.Lunlock

.Lnot_write32:
	/* cmd == 7: WM_CALL */
	movi	r2, 7
	bne	r7, r2, .Lnot_call
	lwi	r5, [r6+0x78]		/* arg0 = fn */
	lwi	r0, [r6+0x7C]		/* a0 = arg1 */
	lwi	r1, [r6+0x74]		/* a1 = bw */
	lwi	r2, [r6+0x50]		/* a2 = node */
	lwi	r3, [r6+0x54]		/* a3 = length */
	jral5	r5
	swi	r0, [r6+0x7C]		/* ret_val = r0 */
	movi	r2, 2
	swi	r2, [r6+0x4C]		/* status = DONE */
	movi	r2, 0
	swi	r2, [r6+0x48]		/* cmd = 0 */
	j	.Lunlock

.Lnot_call:
	/* cmd == 2: explicit STOP (works both while ARMED and after ARM_KEEP) */
	movi	r2, 2
	beq	r7, r2, .Lpoll_armed

	/* If status == 1 (ARMED), poll active capture */
	movi	r2, 1
	beq	r8, r2, .Lpoll_armed

	/* Check for ARM_API (cmd == 1), ARM_DIRECT (cmd == 4), or ARM_KEEP (cmd == 8) */
	movi	r2, 1
	beq	r7, r2, .Ldo_arm
	movi	r2, 4
	beq	r7, r2, .Ldo_arm
	movi	r2, 8
	beq	r7, r2, .Ldo_arm
	j	.Lunlock

.Ldo_arm:
	movi	r2, 0
	swi	r2, [r6+0x64]		/* error = 0 */
	/* Resolve node key into r9: if node < 39, lookup 0x02238C4C + node * 12 */
	lwi	r2, [r6+0x50]
	slti	r3, r2, 39
	beqz	r3, .Lnode_raw
	slli	r3, r2, 1
	add	r3, r3, r2
	slli	r3, r3, 2		/* r3 = node * 12 */
	sethi	r4, 0x02238
	ori	r4, r4, 0x0C4C
	add	r4, r4, r3
	lwi	r9, [r4+0]
	j	.Lnode_done
.Lnode_raw:
	mov55	r9, r2
.Lnode_done:

	/* Load capture length into r10 (default 0x2000 if 0) */
	lwi	r10, [r6+0x54]
	bnez	r10, .Llen_ok
	movi	r10, 0x2000
	swi	r10, [r6+0x54]
.Llen_ok:

	/* Zero 64-byte req buffer at 0x00903880 (r6 + 0x80) */
	addi	r2, r6, 0x80
	movi	r3, 0
	movi	r4, 16
.Lzero_req:
	swi.bi	r3, [r2], 4
	addi	r4, r4, -1
	bnez	r4, .Lzero_req

	/* Populate req structure for 0xE006CC50 / 0xE006CC02 */
	movi	r2, 1
	swi	r2, [r6+0x80]		/* req[0] = 1 (start), req[1..3] = 0 */
	movi	r2, -1
	swi	r2, [r6+0x88]		/* req[+0x08] = 0xFFFFFFFF */
	swi	r9, [r6+0x8C]		/* req[+0x0C] = node_key */
	swi	r10, [r6+0x90]		/* req[+0x10] = length */
	swi	r10, [r6+0x94]		/* req[+0x14] = stop_cycle */
	lwi	r2, [r6+0x74]
	andi	r2, r2, 0xff
	swi	r2, [r6+0xA8]		/* req[+0x28] = u4BW (bits [7:0]) */
	lwi	r2, [r6+0x74]
	srli	r2, r2, 8
	andi	r2, r2, 1
	swi	r2, [r6+0xA4]		/* req[+0x24] = u4BandIdx (bit [8]) */
	swi	r2, [r6+0xB4]		/* req[+0x34] = u4PhyIdx  (bit [8]) */

	/* Force *(u32 *)0x0231FEB4 = 0 so 0xF00ACB08 selects class 4 (128-bit pack) */
	sethi	r2, 0x0231F
	ori	r2, r2, 0x0EB4
	movi	r3, 0
	swi	r3, [r2+0]

	/* If cmd == 4 (ARM_DIRECT), or *(u32 *)0x0231FD20 == 0, run direct HW arm */
	movi	r2, 4
	beq	r7, r2, .Larm_direct
	sethi	r2, 0x0231F
	ori	r2, r2, 0x0D20
	lwi	r2, [r2+0]
	beqz	r2, .Larm_direct

	/* ARM_API / ARM_KEEP: call IcapConfigRequest 0xE006CC50(req, 0) */
	swi	r7, [r6+0x08]		/* arm_mode = cmd (1 or 8) */
	addi	r0, r6, 0x80
	movi	r1, 0
	sethi	r5, 0xE006C
	ori	r5, r5, 0x0C50
	jral5	r5
	swi	r0, [r6+0x64]		/* page.error = ret */
	beqz	r0, .Larm_ok
	movi	r2, 3
	swi	r2, [r6+0x4C]		/* status = ERROR */
	movi	r2, 0
	swi	r2, [r6+0x48]		/* cmd = 0 */
	j	.Lunlock

.Larm_direct:
	movi	r2, 4
	swi	r2, [r6+0x08]		/* arm_mode = 4 */

	/* 1. Route ICAP0 to UMAC 512 KB pool (0x88009028 |= 0x1) */
	sethi	r4, 0x88009
	lwi	r2, [r4+0x28]
	ori	r2, r2, 0x1
	swi	r2, [r4+0x28]

	/* 2. Configure PHY/DFE node registers via 0xE006CC02(req, 0) */
	addi	r0, r6, 0x80
	movi	r1, 0
	sethi	r5, 0xE006C
	ori	r5, r5, 0x0C02
	jral5	r5
	swi	r0, [r6+0x64]

	/* 3. Program ICAP engine (0x80023000) with w1700k-sdr CR0 gating sequence */
	sethi	r4, 0x80023
	movi	r2, 0
	swi	r2, [r4+0x90]		/* CR0 = 0x000 (HELD: clears CR9 & WRAP) */
	movi	r2, 0x400
	swi	r2, [r4+0x90]		/* CR0 = 0x400 (IDLE: SW_RST_B=1) */
	movi	r2, 0
	swi	r2, [r4+0x98]		/* CR2 = 0 (ADDR0_START) */
	addi	r2, r10, -4
	slti	r3, r2, 4
	beqz	r3, 1f
	movi	r2, 4
1:
	sethi	r3, 0x0001F
	ori	r3, r3, 0x0FFC
	slt	r5, r3, r2
	beqz	r5, 2f
	mov55	r2, r3
2:
	swi	r2, [r4+0x9C]		/* CR3 = min(max(len-4, 4), 0x1FFFC) */
	sethi	r3, 0x00007
	ori	r3, r3, 0x0FFF
	and	r2, r10, r3
	sethi	r3, 0x00010
	or	r2, r2, r3
	swi	r2, [r4+0xB8]		/* CR10 = (length & 0x7FFF) | BIT(16) */
	movi	r2, 0x401
	swi	r2, [r4+0x90]		/* CR0 = 0x401 (ARM: SW_RST_B | EN) */
	movi	r2, 0x403
	swi	r2, [r4+0x90]		/* CR0 = 0x403 (TRIGGER: SW_RST_B | EN | CAPTURE) */

.Larm_ok:
	sethi	r2, 0xE0200
	swi	r2, [r6+0x58]		/* data_addr = 0xE0200000 */
	sethi	r4, 0x80023
	lwi	r10, [r6+0x54]		/* r10 = target length */
	movi	r9, 25000		/* bounded fast-poll (~1.5-2.5 ms max) */
.Lfast_poll:
	lwi	r2, [r4+0x90]		/* r2 = cr0 */
	lwi	r5, [r4+0xB4]		/* r5 = cr9 */
	slt	r3, r5, r10
	beqz	r3, .Lfast_done		/* cr9 >= length */
	andi	r3, r2, 0x2
	beqz	r3, .Lfast_done		/* CR0.CAPTURE cleared */
	addi	r9, r9, -1
	bnez	r9, .Lfast_poll

	/* Capture still running: mark ARMED and yield to RTOS */
	swi	r2, [r6+0x70]		/* cr0 */
	lwi	r3, [r4+0x98]
	swi	r3, [r6+0x6C]		/* cr2 */
	swi	r5, [r6+0x68]		/* cr9 */
	movi	r2, 1
	swi	r2, [r6+0x4C]		/* status = 1 (ARMED) */
	j	.Lunlock

.Lfast_done:
	swi	r2, [r6+0x70]		/* cr0 */
	lwi	r3, [r4+0x98]
	swi	r3, [r6+0x6C]		/* cr2 */
	swi	r5, [r6+0x68]		/* cr9 */
	j	.Lfinish_cap

.Lpoll_armed:
	/* Read live ICAP CR0 (+0x90), CR2 (+0x98), CR9 (+0xB4) */
	sethi	r4, 0x80023
	lwi	r2, [r4+0x90]		/* r2 = cr0 */
	swi	r2, [r6+0x70]
	lwi	r3, [r4+0x98]		/* r3 = cr2 */
	swi	r3, [r6+0x6C]
	lwi	r5, [r4+0xB4]		/* r5 = cr9 */
	swi	r5, [r6+0x68]

	lwi	r10, [r6+0x54]		/* r10 = target length */

	/* Stop if cmd == 2 (STOP) */
	movi	r3, 2
	beq	r7, r3, .Lfinish_cap

	/* Stop if cr9 >= length */
	slt	r3, r5, r10
	beqz	r3, .Lfinish_cap

	/* Stop if hardware auto-cleared CAPTURE (CR0 bit 1 == 0) */
	andi	r3, r2, 0x2
	beqz	r3, .Lfinish_cap
	j	.Lunlock

.Lfinish_cap:
	/* Record sample_count (if cr9 == 0 on auto-stop wrap, use target length) */
	bnez	r5, .Lsave_cnt
	mov55	r5, r10
.Lsave_cnt:
	swi	r5, [r6+0x5C]		/* sample_count */

	/* If armed via ARM_API (1), or ARM_KEEP (8) with STOP (cmd==2), call stock stop 0xE006CBC2(req, 0) */
	lwi	r2, [r6+0x08]
	movi	r3, 1
	beq	r2, r3, .Lapi_stop
	movi	r3, 8
	bne	r2, r3, .Ldirect_stop
	movi	r3, 2
	beq	r7, r3, .Lapi_stop

	/* ARM_KEEP completion: hold PHY/UMAC mux open, stop ICAP engine only */
	sethi	r4, 0x80023
	movi	r2, 0x400
	swi	r2, [r4+0x90]
	lwi	r2, [r4+0x90]
	swi	r2, [r6+0x70]
	j	.Lmark_done

.Lapi_stop:
	movi	r2, 0
	swi	r2, [r6+0x80]		/* req[0..3] = 0 */
	addi	r0, r6, 0x80
	movi	r1, 0
	sethi	r5, 0xE006C
	ori	r5, r5, 0x0BC2
	jral5	r5

.Ldirect_stop:
	/* Clear CR0.EN while keeping SW_RST_B=1 (0x400) & release 0x88009004 grant */
	sethi	r4, 0x80023
	movi	r2, 0x400
	swi	r2, [r4+0x90]
	lwi	r2, [r4+0x90]
	swi	r2, [r6+0x70]
	sethi	r4, 0x88009
	movi	r2, 0
	swi	r2, [r4+0x04]

.Lmark_done:
	sethi	r2, 0xE0200
	swi	r2, [r6+0x58]		/* data_addr = 0xE0200000 */
	movi	r2, 2
	swi	r2, [r6+0x4C]		/* status = 2 (DONE) */
	movi	r2, 0
	swi	r2, [r6+0x48]		/* cmd = 0 (IDLE) */

.Lunlock:
	movi	r2, 0
	swi	r2, [r6+0x04]		/* in_pump = 0 */
.Lout_nolock:
	lmw.bim	r6, [sp], r10, 0xe	/* restore r6..r10, fp, gp, lp */
	ret5
	.size wm_pump, .-wm_pump
