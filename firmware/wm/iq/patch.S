/* SPDX-License-Identifier: MIT */
/*
 * Placeholder for the 6-byte patch at 0x0090044E.  The real first word is
 * computed by build_hook.py (NDS32 'j' takes a PC-relative halfword offset),
 * followed by nop16.  Keep this file as the assembler sanity check.
 */
	.text
	.globl wm_patch
wm_patch:
	j	0
	nop
