//
// Copyright (C) 2015
// Gerald Van Baren, <gvb@unssw.com>
// SPDX-License-Identifier:	LGPL-3.0
//
// To compile:
//   arm-none-eabi-gcc -mcpu=arm1176jzf-s -fpic -ffreestanding -c start.S -o start.o
//
// Place in the boot section
.section ".text.boot"
 
// Life starts here
.globl _start
 
// Entry point
// r15 -> should begin execution at 0x8000.
// r0 -> 0x00000000
// r1 -> 0x00000C42
// r2 -> 0x00000100 - start of ATAGS
// preserve these registers as argument for the kernel
_start:
	// Set up the stack.
	mov sp, #0x8000
 
	// Zero the bss.
	ldr r4, =__bss_start
	ldr r9, =__bss_end
	mov r5, #0
	mov r6, #0
	mov r7, #0
	mov r8, #0
 
1:
	// Clear four words at a time
	stmia r4!, {r5-r8}
 
	// If we are still below bss_end, loop.
	cmp r4, r9
	blo 1b
 
	// Call the kernel
	mov fp, #0
	ldr r3, =kernel
	blx r3
 
	// Halt if the mainline returns
halt:
	wfe
	b halt
