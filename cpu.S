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
