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
// This function is the same as ANSI C memcpy, except that the return
// value is void.
.globl __aeabi_memcpy
__aeabi_memcpy:
	push {r3}
	b 2f
1:	ldrb r3, [r1]
	strb r3, [r0]
	add r0,#1
	add r1,#1
	sub r2, #1
2:	cmp r2,#0
	bgt 1b
	pop {r3}
	bx lr

// This function is the same as __aeabi_memcpy; but may assume the
// pointers are 4-byte aligned.
.globl __aeabi_memcpy4
__aeabi_memcpy4:
// This function is the same as __aeabi_memcpy but may assume the pointers
// are 8-byte aligned.
.globl __aeabi_memcpy8
__aeabi_memcpy8:
	push {r3}
	b 2f
1:	ldr r3, [r1]
	str r3, [r0]
	add r0,#4
	add r1,#4
	sub r2, #4
2:	cmp r2,#0
	bgt 1b
	pop {r3}
	bx lr
