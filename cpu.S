//
// Copyright (C) 2015
// Gerald Van Baren, <gvb@unssw.com>
// SPDX-License-Identifier:	LGPL-3.0
//
// To compile:
//   arm-none-eabi-gcc -mcpu=arm1176jzf-s -fpic -ffreestanding -c cpu.S -o cpu.o
//
// Place in the executable section
.section ".text"
 
// Data Memory Barrier, have to switch to ARM mode to execute it.
.globl dmb
dmb:
	.thumb
	push {r0}
	ldr r0, 2f
	bx r0
	.arm
	.align 2
1:	mov r0, #0
	mcr p15,0,r0,c7,c10,5
	pop {r0}
	bx lr
2:	.word 1b
	.thumb

// Ref: http://infocenter.arm.com/help/topic/com.arm.doc.faqs/4972.html
.globl __aeabi_memcpy
__aeabi_memcpy:
// This function is the same as ANSI C memcpy, except that the return
// value is void.
.globl __aeabi_memcpy4
__aeabi_memcpy4:
// This function is the same as __aeabi_memcpy; but may assume the
// pointers are 4-byte aligned.
.globl __aeabi_memcpy8
__aeabi_memcpy8:
// This function is the same as __aeabi_memcpy but may assume the pointers
// are 8-byte aligned.
	.thumb
	push {r3}
	ldr r3, 2f
	bx r3
	.arm
	.align 2
1:	ldr r3, [r1], #4
	str r3, [r0], #4
	subs r2, r2, #4
	bge 1b
	pop {r3}
	bx lr
2:	.word 1b
	.thumb
