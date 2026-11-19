/* SPDX-License-Identifier: MIT */
/*
 * 10-byte patch for 0x0090044E: jump to the hook at 0x02245800.
 * Covers push25 + slti + mov55 + beqzs8 (10 bytes exactly).
 */
	.text
	.globl wm_patch
wm_patch:
	sethi	r15, 0x00900
	ori	r15, r15, 0x3000
	jr5	r15
